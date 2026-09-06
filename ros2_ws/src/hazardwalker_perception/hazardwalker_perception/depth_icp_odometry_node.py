"""使用公开 RealSense 深度图生成负载无关的短时 ICP 里程计。"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from hazardwalker_perception.depth_icp_odometry import (
    base_delta_to_optical_registration,
    integrate_planar_pose,
    optical_registration_to_base_delta,
)


def _stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _depth_to_meters(message):
    encoding = str(message.encoding).lower()
    if encoding == '32fc1':
        dtype, scale, item_size = np.dtype('<f4'), 1.0, 4
    elif encoding in ('16uc1', 'mono16'):
        dtype, scale, item_size = np.dtype('<u2'), 0.001, 2
    else:
        return None
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    expected = int(message.step) * int(message.height)
    if raw.size < expected or message.step < message.width * item_size:
        return None
    rows = raw[:expected].reshape((message.height, message.step))
    depth = rows[:, :message.width * item_size].copy().view(dtype)
    return depth.reshape((message.height, message.width)).astype(
        np.float32, copy=False) * scale


class DepthIcpOdometryNode(Node):
    """深度 ICP 成功时使用几何增量，失败帧只回退已下发控制先验。"""

    def __init__(self):
        super().__init__('hazardwalker_depth_icp_odometry')
        self.declare_parameter('depth_topic', '/hw/camera/depth_image')
        self.declare_parameter('camera_info_topic', '/hw/camera/depth_camera_info')
        self.declare_parameter('cmd_vel_topic', '/hw/cmd_vel')
        self.declare_parameter('output_topic', '/hazardwalker/depth_icp/odometry')
        self.declare_parameter('command_motion_scale', 0.88)
        self.declare_parameter('minimum_frame_interval_s', 0.18)
        # 160×120 在官方 640×480 深度流上可稳定达到数 Hz；320×240 实测
        # 只有约 0.8 Hz，会积压旧帧并失去作为短时里程计的意义。
        self.declare_parameter('downsample_factor', 4)
        self.declare_parameter('min_depth_m', 0.10)
        self.declare_parameter('max_depth_m', 8.0)
        self.declare_parameter('max_depth_difference_m', 0.20)
        self.declare_parameter('maximum_translation_per_frame_m', 0.80)
        self.declare_parameter('maximum_rotation_per_frame_rad', 0.60)
        self.declare_parameter('cmd_fresh_timeout_s', 1.0)

        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError('深度 ICP 需要 python3-opencv-contrib') from exc
        if not hasattr(cv2, 'rgbd'):
            raise RuntimeError('当前 OpenCV 缺少 rgbd/ICPOdometry 模块')
        self.cv2 = cv2
        self.odometry = None
        self.camera_matrix = None
        self.previous_depth = None
        self.previous_stamp = None
        self.pose = (0.0, 0.0, 0.0)
        self.latest_command = (0.0, 0.0, 0.0)
        self.latest_command_stamp = float('-inf')
        self.icp_success_count = 0
        self.fallback_count = 0

        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter('output_topic').value), 20)
        self.status_publisher = self.create_publisher(
            String, '/hazardwalker/depth_icp/status', 10)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self.on_camera_info, 10)
        self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value),
            self.on_depth, 1)
        self.create_subscription(
            Twist, str(self.get_parameter('cmd_vel_topic').value),
            self.on_command, 20)

    def on_camera_info(self, message):
        factor = max(1, int(self.get_parameter('downsample_factor').value))
        values = list(message.k)
        if len(values) != 9 or values[0] <= 0.0 or values[4] <= 0.0:
            return
        self.camera_matrix = np.array([
            [values[0] / factor, 0.0, values[2] / factor],
            [0.0, values[4] / factor, values[5] / factor],
            [0.0, 0.0, 1.0],
        ], dtype=np.float32)
        self.odometry = self.cv2.rgbd.ICPOdometry_create(
            self.camera_matrix,
            float(self.get_parameter('min_depth_m').value),
            float(self.get_parameter('max_depth_m').value),
            float(self.get_parameter('max_depth_difference_m').value),
            0.20,
            [5, 5, 5],
        )

    def on_command(self, message):
        self.latest_command = (
            float(message.linear.x), float(message.linear.y),
            float(message.angular.z))
        self.latest_command_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _command_delta(self, stamp, delta_time):
        now = self.get_clock().now().nanoseconds * 1e-9
        fresh = now - self.latest_command_stamp <= float(
            self.get_parameter('cmd_fresh_timeout_s').value)
        command = self.latest_command if fresh else (0.0, 0.0, 0.0)
        scale = float(self.get_parameter('command_motion_scale').value)
        return (
            command[0] * scale * delta_time,
            command[1] * scale * delta_time,
            command[2] * delta_time,
        )

    def on_depth(self, message):
        if self.odometry is None or self.camera_matrix is None:
            return
        stamp = _stamp_sec(message.header.stamp)
        if self.previous_stamp is not None and stamp <= self.previous_stamp:
            return
        minimum_interval = float(
            self.get_parameter('minimum_frame_interval_s').value)
        if (self.previous_stamp is not None
                and stamp - self.previous_stamp < minimum_interval):
            return
        depth = _depth_to_meters(message)
        if depth is None:
            return
        factor = max(1, int(self.get_parameter('downsample_factor').value))
        if factor > 1:
            depth = self.cv2.resize(
                depth, (depth.shape[1] // factor, depth.shape[0] // factor),
                interpolation=self.cv2.INTER_NEAREST)
        if self.previous_depth is None:
            self.previous_depth = depth
            self.previous_stamp = stamp
            self._publish(message.header.stamp, (0.0, 0.0, 0.0), 'initial')
            return

        delta_time = min(1.0, max(1e-3, stamp - self.previous_stamp))
        command_delta = self._command_delta(stamp, delta_time)
        source_mask = (
            np.isfinite(self.previous_depth)
            & (self.previous_depth >= float(self.get_parameter('min_depth_m').value))
            & (self.previous_depth <= float(self.get_parameter('max_depth_m').value))
        ).astype(np.uint8) * 255
        destination_mask = (
            np.isfinite(depth)
            & (depth >= float(self.get_parameter('min_depth_m').value))
            & (depth <= float(self.get_parameter('max_depth_m').value))
        ).astype(np.uint8) * 255
        empty = np.empty((0, 0), dtype=np.uint8)
        initial = base_delta_to_optical_registration(*command_delta)
        success, transform = self.odometry.compute(
            empty, self.previous_depth, source_mask,
            empty, depth, destination_mask,
            None, initial,
        )
        delta = command_delta
        source = 'command_fallback'
        if success and transform is not None:
            candidate = optical_registration_to_base_delta(transform)
            if (math.hypot(candidate[0], candidate[1]) <= float(
                    self.get_parameter('maximum_translation_per_frame_m').value)
                    and abs(candidate[2]) <= float(self.get_parameter(
                        'maximum_rotation_per_frame_rad').value)):
                delta = candidate
                source = 'depth_icp'
                self.icp_success_count += 1
        if source != 'depth_icp':
            self.fallback_count += 1
        self.pose = integrate_planar_pose(self.pose, delta)
        self.previous_depth = depth
        self.previous_stamp = stamp
        self._publish(message.header.stamp, delta, source, delta_time)

    def _publish(self, stamp, delta, source, delta_time=1.0):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = 'depth_icp_odom'
        message.child_frame_id = 'base'
        message.pose.pose.position.x = self.pose[0]
        message.pose.pose.position.y = self.pose[1]
        message.pose.pose.orientation.z = math.sin(self.pose[2] * 0.5)
        message.pose.pose.orientation.w = math.cos(self.pose[2] * 0.5)
        message.twist.twist.linear.x = float(delta[0]) / max(delta_time, 1e-3)
        message.twist.twist.linear.y = float(delta[1]) / max(delta_time, 1e-3)
        message.twist.twist.angular.z = float(delta[2]) / max(delta_time, 1e-3)
        covariance = 0.02 if source == 'depth_icp' else 0.20
        message.pose.covariance[0] = covariance
        message.pose.covariance[7] = covariance
        message.pose.covariance[35] = covariance
        self.publisher.publish(message)
        status = String()
        status.data = (
            f'source={source} icp={self.icp_success_count} '
            f'fallback={self.fallback_count}')
        self.status_publisher.publish(status)


def main():
    rclpy.init()
    node = DepthIcpOdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
