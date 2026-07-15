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
from pathlib import Path

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from hazardwalker_perception.scan_imu_localization import (  # noqa: E402
    ScanImuLocalizer,
    ScanImuLocalizerConfig,
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
        )
        self.localizer = ScanImuLocalizer(config)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.odom_pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.provenance_pub = rospy.Publisher(
            '/hazardwalker/slam/localization_provenance', String, queue_size=1, latch=True,
        )
        self.provenance_pub.publish(String(data='lidar_imu_slam'))
        rospy.Subscriber(self.imu_topic, Imu, self._on_imu, queue_size=50)
        rospy.Subscriber(self.scan_topic, LaserScan, self._on_scan, queue_size=10)
        rospy.loginfo(
            'Legal lidar-IMU localization started: %s + %s -> %s -> %s.',
            self.scan_topic, self.imu_topic, self.start_frame, self.slam_base_frame,
        )

    def _on_imu(self, message):
        orientation = message.orientation
        self._latest_imu_yaw = quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w,
        )

    def _on_scan(self, message):
        if self._latest_imu_yaw is None:
            rospy.logwarn_throttle(5.0, 'Waiting for public IMU before scan matching.')
            return
        result = self.localizer.update_scan(
            message.ranges, message.angle_min, message.angle_increment, self._latest_imu_yaw,
        )
        self._last_status = result.status
        self._publish_pose(result, message.header.stamp)
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
            (pose.x, pose.y, 0.0), quaternion, stamp, self.slam_base_frame, self.start_frame,
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
        message.pose.pose.orientation.z = quaternion[2]
        message.pose.pose.orientation.w = quaternion[3]
        # 弱匹配明确扩大协方差，供消费方拒绝低质量世界坐标而非盲信。
        variance = 0.04 if result.status in ('initialized', 'tracking') else 1.0
        message.pose.covariance[0] = variance
        message.pose.covariance[7] = variance
        message.pose.covariance[35] = variance
        self.odom_pub.publish(message)


def main():
    OfficialScanImuLocalizationNode()
    rospy.spin()


if __name__ == '__main__':
    main()
