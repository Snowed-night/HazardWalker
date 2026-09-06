"""官方 SimEnv 的 ROS2 合法激光—IMU增量里程计节点。

所属组：感知定位组。
节点只订阅平台适配后的 `/hw/scan`、`/hw/trunk_imu` 和由公开动作确认的楼层编号，
发布 `odom -> base` 与 `/hazardwalker/slam/odometry`。它禁止读取 `/hw/odom`、
`/Odometry_gazebo`、场景布局或危险源真值，为 SLAM Toolbox、Frontier 探索和 RGB-D
三维定位提供同一条可审计坐标链。
"""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32, String
from tf2_ros import TransformBroadcaster

from hazardwalker_perception.scan_imu_localization import (
    ScanMatchResult,
    ScanImuLocalizer,
    ScanImuLocalizerConfig,
    floor_index_to_elevation,
    quaternion_upright_cosine,
    quaternion_to_yaw,
)


class ScanImuLocalizerNode(Node):
    """把公开扫描与 IMU 转成不依赖 Gazebo 真值的二维增量里程计。"""

    def __init__(self):
        super().__init__('hazardwalker_scan_imu_localizer')
        self.declare_parameter('scan_topic', '/hw/scan')
        self.declare_parameter('imu_topic', '/hw/trunk_imu')
        self.declare_parameter('floor_index_topic', '/hazardwalker/navigation/floor_index')
        self.declare_parameter('cmd_vel_topic', '/hw/cmd_vel')
        self.declare_parameter('output_topic', '/hazardwalker/slam/odometry')
        # 运行时发布同一份来源声明，供预检与 rosbag 交叉验证。该节点只能
        # 声明自身实际实现的 scan/IMU 两种来源，不能冒充视觉定位。
        self.declare_parameter('localization_provenance', 'lidar_imu_slam')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base')
        # 作为 SLAM Toolbox 前端时需直接发布 odom→base；作为 Cartographer
        # 合法运动先验时只发布 Odometry，由 Cartographer 独占整条 TF 边。
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('floor_height_m', 2.6)
        self.declare_parameter('initial_floor_index', 0)
        self.declare_parameter('min_floor_index', 0)
        self.declare_parameter('max_floor_index', 31)
        self.declare_parameter('occupancy_resolution_m', 0.08)
        self.declare_parameter('search_radius_m', 0.60)
        self.declare_parameter('search_step_m', 0.05)
        self.declare_parameter('min_match_count', 12)
        self.declare_parameter('laser_offset_x_m', 0.20)
        self.declare_parameter('laser_offset_y_m', 0.0)
        # 该阈值只判断本轮是否允许扫描更新平移，不把命令当作位移真值。
        # 固定距离扫描标定显示命令积分偏大约 1.5 倍，因此默认比例为 0.65；
        # 低于 0.30 m/s 的命令处于官方 A1 控制死区附近，不作为平移先验。
        self.declare_parameter('command_motion_scale', 1.0)
        self.declare_parameter('min_effective_linear_speed_mps', 0.30)
        self.declare_parameter('command_fresh_timeout_s', 0.5)
        self.declare_parameter('max_scan_dt_s', 0.25)
        self.declare_parameter('minimum_command_progress_ratio', 0.85)
        self.declare_parameter('max_degenerate_prior_step_m', 0.25)
        # 与官方控制器安全检查一致：机体倾斜超过 60° 时冻结平移，避免倒地后
        # 的畸变扫描和仍在发布的 cmd_vel 伪造巡检覆盖或危险源位置。
        self.declare_parameter('min_upright_cosine', 0.5)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.floor_height_m = float(self.get_parameter('floor_height_m').value)
        self.min_floor_index = int(self.get_parameter('min_floor_index').value)
        self.max_floor_index = int(self.get_parameter('max_floor_index').value)
        self.floor_index = int(self.get_parameter('initial_floor_index').value)
        self.floor_elevation_m = floor_index_to_elevation(
            self.floor_index,
            floor_height_m=self.floor_height_m,
            min_floor_index=self.min_floor_index,
            max_floor_index=self.max_floor_index,
        )
        self.localizer = ScanImuLocalizer(ScanImuLocalizerConfig(
            occupancy_resolution_m=float(
                self.get_parameter('occupancy_resolution_m').value
            ),
            search_radius_m=float(self.get_parameter('search_radius_m').value),
            search_step_m=float(self.get_parameter('search_step_m').value),
            min_match_count=int(self.get_parameter('min_match_count').value),
            laser_offset_x_m=float(self.get_parameter('laser_offset_x_m').value),
            laser_offset_y_m=float(self.get_parameter('laser_offset_y_m').value),
            minimum_command_progress_ratio=float(self.get_parameter(
                'minimum_command_progress_ratio').value),
            max_degenerate_prior_step_m=float(self.get_parameter(
                'max_degenerate_prior_step_m').value),
        ))
        self.latest_imu_yaw = None
        self.latest_upright_cosine = None
        self.latest_command = Twist()
        self._last_command_monotonic = None
        self._last_scan_time_sec = None
        self.tf_broadcaster = (
            TransformBroadcaster(self)
            if bool(self.get_parameter('publish_tf').value)
            else None
        )
        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter('output_topic').value), 10,
        )
        self.localization_provenance = str(
            self.get_parameter('localization_provenance').value).strip()
        allowed_provenance = {
            'lidar_imu_slam',
            'lidar_imu_slam+public_floor_action',
        }
        if self.localization_provenance not in allowed_provenance:
            raise ValueError(
                'scan/IMU 定位来源不合法：'
                f'{self.localization_provenance!r}')
        provenance_qos = QoSProfile(depth=1)
        provenance_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.provenance_pub = self.create_publisher(
            String, '/hazardwalker/slam/localization_provenance', provenance_qos,
        )
        self.provenance_pub.publish(
            String(data=self.localization_provenance)
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter('imu_topic').value),
            self.on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            self.on_cmd_vel,
            10,
        )
        self.create_subscription(
            Int32,
            str(self.get_parameter('floor_index_topic').value),
            self.on_floor_index,
            10,
        )
        self.get_logger().info(
            'Legal scan/IMU odometry ready: %s + %s -> %s -> %s'
            % (
                self.get_parameter('scan_topic').value,
                self.get_parameter('imu_topic').value,
                self.odom_frame,
                self.base_frame,
            )
        )

    def on_imu(self, message):
        orientation = message.orientation
        self.latest_imu_yaw = quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w,
        )
        self.latest_upright_cosine = quaternion_upright_cosine(
            orientation.x, orientation.y, orientation.z, orientation.w,
        )

    def on_cmd_vel(self, message):
        """保存本系统已下发的合法控制，作为退化走廊中的短时匹配方向先验。"""

        self.latest_command = message
        self._last_command_monotonic = time.monotonic()

    def on_floor_index(self, message):
        try:
            elevation = floor_index_to_elevation(
                message.data,
                floor_height_m=self.floor_height_m,
                min_floor_index=self.min_floor_index,
                max_floor_index=self.max_floor_index,
            )
        except ValueError as error:
            self.get_logger().error('拒绝非法楼层编号：%s' % error)
            return
        new_index = int(message.data)
        if new_index == self.floor_index:
            return
        self.floor_index = new_index
        self.floor_elevation_m = elevation
        self.localizer.reset_matching_map()
        self.get_logger().info(
            '楼层切换到 %d，合法相对高度 %.3f m；已隔离旧楼层扫描地图。'
            % (self.floor_index, self.floor_elevation_m)
        )

    def on_scan(self, message):
        if self.latest_imu_yaw is None:
            self.get_logger().warn(
                '等待 /hw/trunk_imu 后再进行扫描匹配。',
                throttle_duration_sec=5.0,
            )
            return
        if (self.latest_upright_cosine is None
                or self.latest_upright_cosine
                < float(self.get_parameter('min_upright_cosine').value)):
            self.get_logger().error(
                '机体明显倾倒，冻结合法里程计平移；恢复站立后再继续定位。',
                throttle_duration_sec=5.0,
            )
            self.publish_pose(
                ScanMatchResult(
                    self.localizer.pose,
                    'robot_not_upright',
                    0,
                    0.0,
                ),
                message.header.stamp,
            )
            return
        scan_time_sec = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1e-9
        )
        dt_sec = 0.0
        if self._last_scan_time_sec is not None:
            dt_sec = max(
                0.0,
                min(
                    scan_time_sec - self._last_scan_time_sec,
                    float(self.get_parameter('max_scan_dt_s').value),
                ),
            )
        self._last_scan_time_sec = scan_time_sec
        command_fresh = (
            self._last_command_monotonic is not None
            and time.monotonic() - self._last_command_monotonic
            <= float(self.get_parameter('command_fresh_timeout_s').value)
        )
        scale = float(self.get_parameter('command_motion_scale').value)
        motion_prior = (0.0, 0.0)
        translation_expected = False
        if command_fresh and dt_sec > 0.0:
            command_x = float(self.latest_command.linear.x)
            command_y = float(self.latest_command.linear.y)
            min_effective_speed = float(
                self.get_parameter('min_effective_linear_speed_mps').value
            )
            if abs(command_x) < min_effective_speed:
                command_x = 0.0
            if abs(command_y) < min_effective_speed:
                command_y = 0.0
            translation_expected = bool(command_x or command_y)
            motion_prior = (
                command_x * dt_sec * scale,
                command_y * dt_sec * scale,
            )
        result = self.localizer.update_scan(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            self.latest_imu_yaw,
            motion_prior_base=motion_prior,
            allow_translation_update=translation_expected,
        )
        self.publish_pose(result, message.header.stamp)

    def publish_pose(self, result, stamp):
        pose = result.pose
        # scan 与 odom 若使用完全相同时间戳，Cartographer 同步队列会等待
        # “比当前 scan 更新的 odom”。估计由该 scan 计算完成，明确标成其后
        # 1 ms；不能使用经 rosbridge 延迟的 /clock，否则反而落后约 0.2 s。
        source_nanoseconds = (
            int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec))
        publish_stamp = Time(
            nanoseconds=source_nanoseconds + 1_000_000).to_msg()
        half_yaw = pose.yaw * 0.5
        quaternion_z = math.sin(half_yaw)
        quaternion_w = math.cos(half_yaw)

        transform = TransformStamped()
        transform.header.stamp = publish_stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = pose.x
        transform.transform.translation.y = pose.y
        transform.transform.translation.z = self.floor_elevation_m
        transform.transform.rotation.z = quaternion_z
        transform.transform.rotation.w = quaternion_w
        if self.tf_broadcaster is not None:
            self.tf_broadcaster.sendTransform(transform)

        message = Odometry()
        message.header.stamp = publish_stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.position.z = self.floor_elevation_m
        message.pose.pose.orientation.z = quaternion_z
        message.pose.pose.orientation.w = quaternion_w
        variance = 0.04 if result.status in (
            'initialized', 'tracking', 'stationary_command_hold',
        ) else 1.0
        message.pose.covariance[0] = variance
        message.pose.covariance[7] = variance
        message.pose.covariance[14] = 0.04
        message.pose.covariance[35] = variance
        self.odom_pub.publish(message)



def main():
    rclpy.init()
    node = ScanImuLocalizerNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
