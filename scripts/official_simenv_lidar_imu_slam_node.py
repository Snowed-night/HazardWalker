#!/usr/bin/env python3
"""官方 SimEnv 的合法激光—IMU 增量定位节点。

所属组：感知定位组。
该节点仅订阅官方允许的 ``/scan`` 和 ``/trunk_imu``，以 IMU 朝向约束和激光端点
相关匹配构建本地 ``start`` 坐标系，发布独立的 ``start -> slam_base -> real_sense`` TF 与
``/hazardwalker/slam/odometry``。它绝不订阅 `/Odometry_gazebo`、`/hazardwalker/odom`、
`/ground_truth/*`、场景布局或危险源真值，也不发布任何控制命令。

输出可让 RGB-D 感知节点通过 ``start -> slam_base -> real_sense`` 的公开传感器外参完成
三维反投影；再由启动方明示的公开起点位姿转换到官方要求的 world 坐标。该实现是轻量增量定位，
正式成绩前仍需在随机场景实测漂移与重定位效果。
"""

import math
import sys
import time
from pathlib import Path

import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from std_msgs.msg import Int32, String


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from hazardwalker_perception.scan_imu_localization import (  # noqa: E402
    ScanImuLocalizer,
    ScanImuLocalizerConfig,
    floor_index_to_elevation,
    point_cloud_xyz_to_base_points,
    quaternion_to_yaw,
)


class OfficialScanImuLocalizationNode(object):
    """维护 start 坐标系并把扫描匹配结果发布为可审计的自建定位。"""

    def __init__(self):
        rospy.init_node('hazardwalker_official_lidar_imu_slam', anonymous=False)
        self.start_frame = rospy.get_param('~start_frame', 'start')
        # 绝不复用官方 TF 树已经拥有 parent 的 ``base``：否则同一 child frame 会有
        # odom 与 start 两个父节点。独立 slam_base 由本节点唯一拥有，再显式复刻公开
        # base->real_sense 传感器外参，形成无冲突的定位链。
        self.slam_base_frame = rospy.get_param('~slam_base_frame', 'slam_base')
        self.camera_frame = rospy.get_param('~camera_frame', 'real_sense')
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.imu_topic = rospy.get_param('~imu_topic', '/trunk_imu')
        self.output_topic = rospy.get_param(
            '~output_topic', '/hazardwalker/slam/odometry',
        )
        # 多楼层 z 不得从 Gazebo 里程计或场景布局读取。导航在已接受电梯服务响应、
        # 或通过自主楼梯状态机确认到层后，发布由公开动作推导的楼层编号。
        self.floor_index_topic = rospy.get_param(
            '~floor_index_topic', '/hazardwalker/navigation/floor_index',
        )
        self.floor_height_m = float(rospy.get_param('~floor_height_m', 2.6))
        self.min_floor_index = int(rospy.get_param('~min_floor_index', 0))
        self.max_floor_index = int(rospy.get_param('~max_floor_index', 31))
        self.floor_index = int(rospy.get_param('~initial_floor_index', 0))
        self.floor_elevation_m = floor_index_to_elevation(
            self.floor_index,
            floor_height_m=self.floor_height_m,
            min_floor_index=self.min_floor_index,
            max_floor_index=self.max_floor_index,
        )
        self._latest_imu_yaw = None
        self._last_status = 'waiting_for_imu'
        self.camera_offset_x_m = float(rospy.get_param('~camera_offset_x_m', 0.28))
        self.camera_offset_y_m = float(rospy.get_param('~camera_offset_y_m', 0.0))
        self.camera_offset_z_m = float(rospy.get_param('~camera_offset_z_m', 0.043))
        config = ScanImuLocalizerConfig(
            occupancy_resolution_m=float(rospy.get_param('~occupancy_resolution_m', 0.08)),
            search_radius_m=float(rospy.get_param('~search_radius_m', 0.60)),
            search_step_m=float(rospy.get_param('~search_step_m', 0.05)),
            min_match_count=int(rospy.get_param('~min_match_count', 12)),
            laser_offset_x_m=float(rospy.get_param('~laser_offset_x_m', 0.20)),
            laser_offset_y_m=float(rospy.get_param('~laser_offset_y_m', 0.0)),
            laser_offset_z_m=float(rospy.get_param('~laser_offset_z_m', 0.08)),
            laser_pitch_rad=float(rospy.get_param('~laser_pitch_rad', 0.785)),
            min_endpoint_z_m=float(rospy.get_param('~min_endpoint_z_m', -0.25)),
            max_endpoint_z_m=float(rospy.get_param('~max_endpoint_z_m', 1.50)),
        )
        self.localizer = ScanImuLocalizer(config)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.odom_pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.provenance_pub = rospy.Publisher(
            '/hazardwalker/slam/localization_provenance', String, queue_size=1, latch=True,
        )
        self.provenance_pub.publish(String(data='lidar_imu_slam+public_floor_action'))
        rospy.Subscriber(self.imu_topic, Imu, self._on_imu, queue_size=50)
        rospy.Subscriber(
            self.floor_index_topic, Int32, self._on_floor_index, queue_size=5,
        )
        # 官方仓库文档将 `/scan` 定义为 Livox PointCloud2，但部分平台适配版本
        # 实际发布 LaserScan。启动时读取 ROS master 的真实类型并同时兼容两者；
        # 禁止靠静态文档猜类型，否则定位回调失联会使全部结果坐标为空。
        self.scan_message_type = _resolve_scan_message_type(
            self.scan_topic, rospy.get_param('~scan_message_type', 'auto'),
        )
        if self.scan_message_type == 'sensor_msgs/PointCloud2':
            rospy.Subscriber(
                self.scan_topic, PointCloud2, self._on_point_cloud, queue_size=3,
            )
        else:
            rospy.Subscriber(
                self.scan_topic, LaserScan, self._on_laser_scan, queue_size=10,
            )
        rospy.loginfo(
            'Legal lidar-IMU localization started: %s (%s) + %s + %s -> %s -> %s.',
            self.scan_topic, self.scan_message_type, self.imu_topic,
            self.floor_index_topic, self.start_frame, self.slam_base_frame,
        )

    def _on_imu(self, message):
        orientation = message.orientation
        self._latest_imu_yaw = quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w,
        )

    def _on_floor_index(self, message):
        """接收合法动作链确认的楼层编号，并隔离新旧楼层二维地图。"""

        try:
            elevation = floor_index_to_elevation(
                message.data,
                floor_height_m=self.floor_height_m,
                min_floor_index=self.min_floor_index,
                max_floor_index=self.max_floor_index,
            )
        except ValueError as error:
            rospy.logerr_throttle(3.0, 'Reject invalid floor index: %s', error)
            return
        new_index = int(message.data)
        if new_index == self.floor_index:
            return
        old_index = self.floor_index
        self.floor_index = new_index
        self.floor_elevation_m = elevation
        self.localizer.reset_matching_map()
        rospy.loginfo(
            'Accepted public-action floor transition: %d -> %d, z=%.3f m; '
            'cleared per-floor scan map.',
            old_index, self.floor_index, self.floor_elevation_m,
        )

    def _on_laser_scan(self, message):
        """兼容当前平台实际发布的二维 LaserScan。"""
        if self._latest_imu_yaw is None:
            rospy.logwarn_throttle(5.0, 'Waiting for public IMU before scan matching.')
            return
        result = self.localizer.update_scan(
            message.ranges, message.angle_min, message.angle_increment, self._latest_imu_yaw,
        )
        self._handle_result(result, message.header.stamp)

    def _on_point_cloud(self, message):
        """处理官方文档约定的 Livox PointCloud2。"""
        if self._latest_imu_yaw is None:
            rospy.logwarn_throttle(5.0, 'Waiting for public IMU before scan matching.')
            return
        xyz_points = point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=True,
        )
        base_points = point_cloud_xyz_to_base_points(xyz_points, self.localizer.config)
        result = self.localizer.update_base_points(
            base_points, self._latest_imu_yaw,
        )
        self._handle_result(result, message.header.stamp)

    def _handle_result(self, result, stamp):
        """统一发布两种公开激光输入的匹配结果。"""
        self._last_status = result.status
        self._publish_pose(result, stamp)
        if result.status == 'weak_scan_match':
            rospy.logwarn_throttle(
                3.0, 'Lidar-IMU weak scan match: endpoints=%d score=%.3f; pose is held.',
                result.matched_endpoint_count, result.score,
            )

    def _publish_pose(self, result, stamp):
        pose = result.pose
        half_yaw = pose.yaw * 0.5
        quaternion = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
        self.tf_broadcaster.sendTransform(
            (pose.x, pose.y, self.floor_elevation_m),
            quaternion, stamp, self.slam_base_frame, self.start_frame,
        )
        # 公开传感器标定外参：官方 TF_static 为 base->real_sense (0.28, 0, 0.043, identity)。
        # 该副本仅连接本节点独立的 slam_base，不读取或覆盖官方 base 的动态位姿。
        self.tf_broadcaster.sendTransform(
            (self.camera_offset_x_m, self.camera_offset_y_m, self.camera_offset_z_m),
            (0.0, 0.0, 0.0, 1.0), stamp, self.camera_frame, self.slam_base_frame,
        )
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.start_frame
        message.child_frame_id = self.slam_base_frame
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.position.z = self.floor_elevation_m
        message.pose.pose.orientation.z = quaternion[2]
        message.pose.pose.orientation.w = quaternion[3]
        # 弱匹配明确扩大协方差，供消费方拒绝低质量世界坐标而非盲信。
        variance = 0.04 if result.status in ('initialized', 'tracking') else 1.0
        message.pose.covariance[0] = variance
        message.pose.covariance[7] = variance
        message.pose.covariance[14] = 0.04
        message.pose.covariance[35] = variance
        self.odom_pub.publish(message)


def _resolve_scan_message_type(topic, configured_type):
    """读取 ROS master 的真实类型，避免平台版本差异让正式节点静默失联。"""

    aliases = {
        'laserscan': 'sensor_msgs/LaserScan',
        'sensor_msgs/laserscan': 'sensor_msgs/LaserScan',
        'pointcloud2': 'sensor_msgs/PointCloud2',
        'sensor_msgs/pointcloud2': 'sensor_msgs/PointCloud2',
    }
    normalized = str(configured_type or 'auto').strip().lower()
    if normalized != 'auto':
        resolved = aliases.get(normalized)
        if resolved is None:
            raise rospy.ROSInitException(
                '~scan_message_type 只允许 auto、LaserScan 或 PointCloud2。',
            )
        return resolved

    deadline = time.monotonic() + 10.0
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        for published_topic, message_type in rospy.get_published_topics():
            if published_topic == topic and message_type in aliases.values():
                return message_type
        # 使用 wall clock，避免 Gazebo 暂停时 ROS time 不推进导致初始化永久卡住。
        time.sleep(0.1)
    raise rospy.ROSInitException(
        '10 秒内未发现 %s 的 LaserScan/PointCloud2 发布者。' % topic,
    )


def main():
    OfficialScanImuLocalizationNode()
    rospy.spin()


if __name__ == '__main__':
    main()
