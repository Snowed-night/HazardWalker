#!/usr/bin/env python3
"""官方 ROS1 rosbridge 到 ROS2 /hw/* 的双向适配节点。

所属组：平台与仿真组。负责人：姜晨。
运行位置：ROS2 主机，不运行在仅安装 ROS1 的官方 Docker 内。通过 rosbridge v2 WebSocket 订阅官方
RGB、深度、内参和里程计，完整重组 fragment 后发布稳定 /hw/*；仅显式开启时才把 /hw/cmd_vel 发回
官方 /cmd_vel。验证：先跑 ROS1 直连控制，再执行 verify_official_simenv_ros1_adapter.sh。
"""

import base64
import json
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from hazardwalker_platform.rosbridge_protocol import FragmentAssembler, decode_packet


def _stamp(target, source):
    source = source or {}
    target.sec = int(source.get('secs', source.get('sec', 0)))
    target.nanosec = int(source.get('nsecs', source.get('nanosec', 0)))


class RosbridgeHwAdapter(Node):
    """保留原始图像字节、限制分片缓存并提供零速度看门狗的 ROS2 适配器。"""

    def __init__(self):
        super().__init__('hazardwalker_official_rosbridge_adapter')
        self.url = self.declare_parameter('rosbridge_url', 'ws://127.0.0.1:9090').value
        self.enable_control = bool(self.declare_parameter('enable_cmd_vel_relay', False).value)
        self.timeout_sec = float(self.declare_parameter('cmd_vel_timeout_sec', 0.5).value)
        self._last_cmd = None
        self._socket = None
        self._send_lock = threading.Lock()
        self._assembler = FragmentAssembler()
        self._counts = {}
        self.odom_pub = self.create_publisher(Odometry, '/hw/odom', 10)
        self.rgb_pub = self.create_publisher(Image, '/hw/camera/image_raw', 1)
        self.depth_pub = self.create_publisher(Image, '/hw/camera/depth_image', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/hw/camera/camera_info', 1)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/hw/camera/depth_camera_info', 1)
        self.status_pub = self.create_publisher(String, '/hw/platform/official_simenv_adapter_status', 10)
        self.cmd_sub = self.create_subscription(Twist, '/hw/cmd_vel', self._on_cmd, 10)
        self.create_timer(0.1, self._watchdog)
        self.create_timer(0.5, self._status)
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()

    def _send(self, packet):
        with self._send_lock:
            if self._socket is not None:
                self._socket.send(json.dumps(packet, separators=(',', ':')))

    def _receive_loop(self):
        try:
            import websocket
        except ImportError:
            self.get_logger().error('缺少 websocket-client；请在 ROS2 主机执行 pip3 install websocket-client。')
            return
        while rclpy.ok():
            try:
                self._socket = websocket.create_connection(self.url, timeout=5)
                self._send({'op': 'advertise', 'topic': '/cmd_vel', 'type': 'geometry_msgs/Twist'})
                for topic, msg_type in (
                    ('/Odometry_gazebo', 'nav_msgs/Odometry'),
                    ('/real_sense/rgb/image_raw', 'sensor_msgs/Image'),
                    ('/real_sense/depth/image_raw', 'sensor_msgs/Image'),
                    ('/real_sense/rgb/camera_info', 'sensor_msgs/CameraInfo'),
                    ('/real_sense/depth/camera_info', 'sensor_msgs/CameraInfo'),
                ):
                    self._send({'op': 'subscribe', 'id': 'hw:' + topic, 'topic': topic, 'type': msg_type,
                                'queue_length': 1, 'fragment_size': 60000, 'compression': 'none'})
                while rclpy.ok():
                    packet = decode_packet(self._socket.recv(), self._assembler)
                    if packet and packet.get('op') == 'publish':
                        self._publish(packet.get('topic'), packet.get('msg', {}))
            except Exception as error:
                self.get_logger().warn('rosbridge 连接中断：%s' % error)
                self._socket = None
                time.sleep(2.0)

    def _publish(self, topic, source):
        header = source.get('header', {})
        if topic == '/Odometry_gazebo':
            message = Odometry(); _stamp(message.header.stamp, header.get('stamp')); message.header.frame_id = header.get('frame_id', '')
            message.child_frame_id = source.get('child_frame_id', '')
            pose = source.get('pose', {}).get('pose', {}); twist = source.get('twist', {}).get('twist', {})
            for name in ('x', 'y', 'z'):
                setattr(message.pose.pose.position, name, float(pose.get('position', {}).get(name, 0.0)))
                setattr(message.twist.twist.linear, name, float(twist.get('linear', {}).get(name, 0.0)))
            for name in ('x', 'y', 'z', 'w'):
                setattr(message.pose.pose.orientation, name, float(pose.get('orientation', {}).get(name, 0.0)))
                setattr(message.twist.twist.angular, name, float(twist.get('angular', {}).get(name, 0.0)))
            self.odom_pub.publish(message)
        elif topic in ('/real_sense/rgb/image_raw', '/real_sense/depth/image_raw'):
            message = Image(); _stamp(message.header.stamp, header.get('stamp')); message.header.frame_id = header.get('frame_id', '')
            message.height = int(source.get('height', 0)); message.width = int(source.get('width', 0))
            message.encoding = source.get('encoding', ''); message.is_bigendian = int(source.get('is_bigendian', 0)); message.step = int(source.get('step', 0))
            message.data = base64.b64decode(source.get('data', ''))
            (self.rgb_pub if topic.endswith('rgb/image_raw') else self.depth_pub).publish(message)
        elif topic in ('/real_sense/rgb/camera_info', '/real_sense/depth/camera_info'):
            message = CameraInfo(); _stamp(message.header.stamp, header.get('stamp')); message.header.frame_id = header.get('frame_id', '')
            message.height = int(source.get('height', 0)); message.width = int(source.get('width', 0))
            message.k = [float(value) for value in source.get('K', source.get('k', []))]
            message.d = [float(value) for value in source.get('D', source.get('d', []))]
            message.r = [float(value) for value in source.get('R', source.get('r', []))]
            message.p = [float(value) for value in source.get('P', source.get('p', []))]
            (self.info_pub if topic.endswith('rgb/camera_info') else self.depth_info_pub).publish(message)
        else:
            return
        self._counts[topic] = self._counts.get(topic, 0) + 1

    def _on_cmd(self, message):
        if not self.enable_control:
            return
        self._send({'op': 'publish', 'topic': '/cmd_vel', 'msg': {
            'linear': {'x': message.linear.x, 'y': message.linear.y, 'z': message.linear.z},
            'angular': {'x': message.angular.x, 'y': message.angular.y, 'z': message.angular.z}}})
        self._last_cmd = time.monotonic()

    def _watchdog(self):
        if self.enable_control and self._last_cmd and time.monotonic() - self._last_cmd > self.timeout_sec:
            self._send({'op': 'publish', 'topic': '/cmd_vel', 'msg': {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}})
            self._last_cmd = None

    def _status(self):
        self.status_pub.publish(String(data=json.dumps({'adapter': 'rosbridge_ros2', 'url': self.url, 'enable_cmd_vel_relay': self.enable_control, 'received': self._counts}, sort_keys=True)))


def main():
    rclpy.init(); node = RosbridgeHwAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
