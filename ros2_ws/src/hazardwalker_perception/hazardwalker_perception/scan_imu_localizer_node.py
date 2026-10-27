"""官方 SimEnv 的 ROS2 合法激光—IMU—本体增量里程计节点。

所属组：感知定位组。
节点订阅平台适配后的 `/hw/scan`、`/hw/trunk_imu`、宇树 Estimator 的
`/hw/proprio_odom` 和由公开动作确认的楼层编号，发布 `odom -> base` 与
`/hazardwalker/slam/odometry`。它禁止读取 Gazebo 真值 `/hw/odom`、
`/Odometry_gazebo`、场景布局或危险源真值，为 Cartographer、Frontier 探索和
RGB-D 三维定位提供同一条可审计坐标链。
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
    proprioceptive_planar_delta,
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
        self.declare_parameter('proprio_odom_topic', '/hw/proprio_odom')
        self.declare_parameter('output_topic', '/hazardwalker/slam/odometry')
        # 运行时发布同一份来源声明，供预检与 rosbag 交叉验证。该节点只能
        # 声明自身实际实现的 scan/IMU 两种来源，不能冒充视觉定位。
        self.declare_parameter(
            'localization_provenance', 'lidar_imu_proprio_slam')
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
        # cmd_vel 只表示“当前允许发生平移”，绝不能再作为实际位移。真正短时
        # 运动先验来自宇树 Estimator；它由足端接触、关节状态和 IMU 估计，
        # 被门框挡住时不会像命令积分那样凭空累计前进距离。
        self.declare_parameter('proprio_fresh_timeout_s', 0.5)
        self.declare_parameter('proprio_max_interval_s', 0.25)
        self.declare_parameter('proprio_max_step_m', 0.25)
        self.declare_parameter('proprio_motion_gate_m', 0.001)
        self.declare_parameter('proprio_speed_gate_mps', 0.02)
        self.declare_parameter('proprio_motion_hold_s', 0.50)
        self.declare_parameter('use_command_motion_fallback', False)
        # 仅为非正式兼容模式保留命令积分回退；正式入口固定关闭。
        self.declare_parameter('command_motion_scale', 1.0)
        self.declare_parameter('command_lateral_motion_scale', 0.0)
        self.declare_parameter('min_effective_linear_speed_mps', 0.30)
        self.declare_parameter('command_fresh_timeout_s', 0.5)
        self.declare_parameter('max_scan_dt_s', 0.25)
        # 扫描证据必须能把先验完全拉回原位；否则机器狗顶住墙仍会按命令
        # 虚增里程。长直走廊证据退化时才由下游 motion_prior_only 保留先验。
        self.declare_parameter('minimum_command_progress_ratio', 0.0)
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
        self.latest_proprio_pose = None
        self.latest_proprio_speed_mps = 0.0
        self._last_proprio_monotonic = None
        self._last_consumed_proprio_pose = None
        self._last_proprio_motion_monotonic = None
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
            'lidar_imu_proprio_slam',
            'lidar_imu_proprio_slam+public_floor_action',
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
            Odometry,
            str(self.get_parameter('proprio_odom_topic').value),
            self.on_proprio_odom,
            20,
        )
        self.create_subscription(
            Int32,
            str(self.get_parameter('floor_index_topic').value),
            self.on_floor_index,
            10,
        )
        self.get_logger().info(
            'Legal scan/IMU/proprio odometry ready: %s + %s + %s -> %s -> %s'
            % (
                self.get_parameter('scan_topic').value,
                self.get_parameter('imu_topic').value,
                self.get_parameter('proprio_odom_topic').value,
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
        """保存合法控制，只用于判断当前是否允许扫描匹配更新平移。"""

        self.latest_command = message
        self._last_command_monotonic = time.monotonic()

    def on_proprio_odom(self, message):
        """保存宇树 Estimator 的本体位姿；不转发其 TF，也不当累计世界坐标。"""

        stamp = message.header.stamp
        orientation = message.pose.pose.orientation
        self.latest_proprio_pose = (
            float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            quaternion_to_yaw(
                orientation.x, orientation.y, orientation.z, orientation.w),
        )
        linear = message.twist.twist.linear
        self.latest_proprio_speed_mps = math.hypot(
            float(linear.x), float(linear.y))
        self._last_proprio_monotonic = time.monotonic()

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
        # 电梯运动不属于任一楼层的平面扫描匹配；新楼层首帧重新建立短时基线。
        self._last_consumed_proprio_pose = None
        self._last_proprio_motion_monotonic = None
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
        command_x = 0.0
        command_y = 0.0
        command_requests_translation = False
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
            command_requests_translation = bool(command_x or command_y)
        motion_prior = (0.0, 0.0)
        translation_expected = False
        proprio_fresh = (
            self.latest_proprio_pose is not None
            and self._last_proprio_monotonic is not None
            and time.monotonic() - self._last_proprio_monotonic
            <= float(self.get_parameter('proprio_fresh_timeout_s').value)
        )
        if proprio_fresh:
            proprio_delta = (0.0, 0.0)
            if self._last_consumed_proprio_pose is not None:
                proprio_delta = proprioceptive_planar_delta(
                    self._last_consumed_proprio_pose,
                    self.latest_proprio_pose,
                    max_step_m=float(
                        self.get_parameter('proprio_max_step_m').value),
                    max_interval_s=float(
                        self.get_parameter('proprio_max_interval_s').value),
                )
            self._last_consumed_proprio_pose = self.latest_proprio_pose
            # Estimator 的累计位置在步态支撑相位会间歇停滞，但同一消息中的
            # body twist 仍连续反映实际足端运动。任一证据满足即可刷新短保持窗，
            # 避免正常行走每隔几帧被误判为静止而系统性少计里程。
            if (math.hypot(*proprio_delta) >= float(
                    self.get_parameter('proprio_motion_gate_m').value)
                    or self.latest_proprio_speed_mps >= float(
                        self.get_parameter('proprio_speed_gate_mps').value)):
                self._last_proprio_motion_monotonic = time.monotonic()
            proprio_motion_confirmed = (
                self._last_proprio_motion_monotonic is not None
                and time.monotonic() - self._last_proprio_motion_monotonic
                <= float(self.get_parameter('proprio_motion_hold_s').value)
            )
            translation_expected = bool(
                command_requests_translation and proprio_motion_confirmed)
            if translation_expected:
                # Estimator 的累计距离在 A1 仿真中明显低估，但它能证明足端
                # 是否真的产生了位移。距离尺度继续取已标定命令先验，扫描证据
                # 可完整纠正；顶墙时本体运动门禁会在短保持窗后关闭。
                forward_scale = float(
                    self.get_parameter('command_motion_scale').value)
                lateral_scale = float(
                    self.get_parameter('command_lateral_motion_scale').value)
                motion_prior = (
                    command_x * dt_sec * forward_scale,
                    command_y * dt_sec * lateral_scale,
                )
        elif bool(self.get_parameter('use_command_motion_fallback').value):
            # 诊断兼容分支。正式三层入口关闭该分支，缺失本体里程计时宁可
            # 只靠扫描证据，也不能重新把“想走多远”冒充“实际走了多远”。
            forward_scale = float(
                self.get_parameter('command_motion_scale').value)
            lateral_scale = float(
                self.get_parameter('command_lateral_motion_scale').value)
            translation_expected = command_requests_translation
            motion_prior = (
                command_x * dt_sec * forward_scale,
                command_y * dt_sec * lateral_scale,
            ) if translation_expected else (0.0, 0.0)
        elif self.latest_proprio_pose is None:
            self.get_logger().warn(
                '等待 /hw/proprio_odom；正式模式禁止用 cmd_vel 积分替代实际位移。',
                throttle_duration_sec=5.0,
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
