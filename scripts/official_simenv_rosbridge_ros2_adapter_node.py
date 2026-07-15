#!/usr/bin/env python3
"""官方 ROS1 rosbridge 到 ROS2 /hw/* 的双向适配节点。

所属组：平台与仿真组。负责人：姜晨。
运行位置：ROS2 主机，不运行在仅安装 ROS1 的官方 Docker 内。通过 rosbridge v2 WebSocket 订阅官方
RGB、深度、内参和里程计，完整重组 fragment 后发布稳定 /hw/*；仅显式开启时才把 /hw/cmd_vel 发回
官方 /cmd_vel。验证：先跑 ROS1 直连控制，再执行 verify_official_simenv_ros1_adapter.sh。
"""

import base64
import binascii
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

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
        # Docker 端口映射可使外部 URL 端口不同于容器内 rosbridge 监听端口；部分 rosbridge 会校验 Host。
        self.host_header = self.declare_parameter('rosbridge_host_header', '').value
        # 控制逐段验收可暂时停掉高带宽图像，避免图像重组故障掩盖 /cmd_vel 链路；默认保持完整 RGB-D 转发。
        self.enable_image_relay = bool(self.declare_parameter('enable_image_relay', True).value)
        # 原始 640x480 RGB-D 通过 rosbridge 时，前一帧未收全又开始下一帧会造成分片混杂。
        # 默认节流到 2 Hz；平台带宽验证充足后可按毫秒调小此参数。
        self.image_throttle_rate_ms = int(self.declare_parameter('image_throttle_rate_ms', 500).value)
        # 三维定位需要 tf2 查询相机到 world/odom 的外参；官方 ROS1 /tf 必须显式转入 ROS2。
        self.enable_tf_relay = bool(self.declare_parameter('enable_tf_relay', True).value)
        # 官方 state_from_gazebo 可发布约 1 kHz 位姿。rosbridge 序列化每一条会反向拖慢 Gazebo，
        # 而导航、深度定位并不需要该频率；默认保留 50 Hz 的最新真实里程计。
        self.odom_throttle_rate_ms = int(self.declare_parameter('odom_throttle_rate_ms', 20).value)
        # 官方 /tf 可达 500 Hz；三维定位消费的是低频 RGB-D，保留 50 Hz 已足够插值，
        # 否则 JSON 解码会饿死大图像分片与感知回调。
        self.tf_throttle_rate_ms = int(self.declare_parameter('tf_throttle_rate_ms', 20).value)
        self.tf_odom_consistency_tolerance_m = float(
            self.declare_parameter('tf_odom_consistency_tolerance_m', 0.25).value)
        # 官方 TF 根帧为 map，而赛题结果要求 world。已实测官方 map→odom 为单位变换，
        # 因而默认补一条 world→map 单位静态边；若平台改了地图原点可关掉或改名，绝不静默猜测。
        self.enable_world_frame_alias = bool(self.declare_parameter('enable_world_frame_alias', True).value)
        self.world_frame = self.declare_parameter('world_frame', 'world').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.enable_control = bool(self.declare_parameter('enable_cmd_vel_relay', False).value)
        self.timeout_sec = float(self.declare_parameter('cmd_vel_timeout_sec', 0.5).value)
        # 官方版本相机命名有差异；源话题可配，但稳定的 ROS2 /hw 输出不变。
        self.rgb_topic = self.declare_parameter('rgb_topic', '/real_sense/rgb/image_raw').value
        self.depth_topic = self.declare_parameter('depth_topic', '/real_sense/depth/image_raw').value
        self.rgb_info_topic = self.declare_parameter('rgb_camera_info_topic', '/real_sense/rgb/camera_info').value
        self.depth_info_topic = self.declare_parameter('depth_camera_info_topic', '/real_sense/depth/camera_info').value
        # 修复后的官方启动链路会把 1 kHz 原始里程计压缩为只含最新值的中继，避免
        # rosbridge 向 ROS2 回放启动期旧位姿。旧环境可显式设回 /Odometry_gazebo。
        self.ros1_odom_topic = self.declare_parameter('ros1_odom_topic', '/hazardwalker/odom').value
        self._last_cmd = None
        self._last_forwarded_cmd = None
        self._forwarded_cmd_count = 0
        self._socket = None
        self._send_lock = threading.Lock()
        self._assembler = FragmentAssembler()
        self._counts = {}
        # rosbridge 在高带宽订阅下偶发给出不完整的 base64 图像；只丢弃坏帧，不能让整条
        # WebSocket 接收循环重连，否则 /hw/odom 与 /hw/cmd_vel 也会被一起中断。
        self._dropped_image_frames = {}
        self._dropped_inconsistent_tf = 0
        self._official_odom_xy = None
        # WebSocket 接收线程不能长期直接调用 rclpy Publisher.publish：大图像高负载时
        # Fast DDS 可能出现“已解码计数增长、订阅者无帧”的跨线程投递异常。接收线程只保留
        # 每类最新消息，ROS2 执行器定时统一发布；图像过期帧天然被丢弃，不积压内存。
        self._pending_messages = {}
        self._pending_lock = threading.Lock()
        self.odom_pub = self.create_publisher(Odometry, '/hw/odom', 10)
        self.rgb_pub = self.create_publisher(Image, '/hw/camera/image_raw', 1)
        self.depth_pub = self.create_publisher(Image, '/hw/camera/depth_image', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/hw/camera/camera_info', 1)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/hw/camera/depth_camera_info', 1)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 20)
        self.tf_static_pub = self.create_publisher(
            TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._ros_publishers = {
            'odom': self.odom_pub, 'rgb': self.rgb_pub, 'depth': self.depth_pub,
            'rgb_info': self.info_pub, 'depth_info': self.depth_info_pub,
            'tf': self.tf_pub, 'tf_static': self.tf_static_pub,
        }
        self.status_pub = self.create_publisher(String, '/hw/platform/official_simenv_adapter_status', 10)
        self.cmd_sub = self.create_subscription(Twist, '/hw/cmd_vel', self._on_cmd, 10)
        self.create_timer(0.1, self._watchdog)
        self.create_timer(0.5, self._status)
        self.create_timer(0.01, self._flush_pending_messages)
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
        # 官方镜像内的 rosbridge 会把不同订阅的 fragment 都标成 id=0。若 RGB、深度共用
        # 一个 WebSocket，高带宽帧偶发交错时无法仅靠 id 区分，最终会拼出非法 base64。
        # 因此两个图像源各占一个只接收的连接；主连接保留控制回传、里程计、内参和 TF。
        self._image_threads = []
        if self.enable_image_relay:
            for topic in (self.rgb_topic, self.depth_topic):
                worker = threading.Thread(target=self._receive_image_loop, args=(topic,), daemon=True)
                worker.start()
                self._image_threads.append(worker)

    def _send(self, packet):
        with self._send_lock:
            if self._socket is not None:
                self._socket.send(json.dumps(packet, separators=(',', ':')))
                return True
        return False

    def _queue_message(self, name, message):
        """由 WebSocket 线程覆盖式缓存，避免跨线程直接触发 DDS 发布。"""
        with self._pending_lock:
            self._pending_messages[name] = message

    def _flush_pending_messages(self):
        """在 ROS2 执行器线程实际发布缓存的最新传感器消息。"""
        with self._pending_lock:
            pending = self._pending_messages
            self._pending_messages = {}
        for name, message in pending.items():
            self._ros_publishers[name].publish(message)

    def _receive_loop(self):
        try:
            import websocket
        except ImportError:
            self.get_logger().error('缺少 websocket-client；请在 ROS2 主机执行 pip3 install websocket-client。')
            return
        while rclpy.ok():
            try:
                connection_options = {}
                if self.host_header:
                    connection_options['host'] = self.host_header
                self._socket = websocket.create_connection(self.url, timeout=5, **connection_options)
                self._send({'op': 'advertise', 'topic': '/cmd_vel', 'type': 'geometry_msgs/Twist'})
                subscriptions = [(self.ros1_odom_topic, 'nav_msgs/Odometry'),
                                 (self.rgb_info_topic, 'sensor_msgs/CameraInfo'),
                                 (self.depth_info_topic, 'sensor_msgs/CameraInfo')]
                if self.enable_tf_relay:
                    subscriptions.extend((('/tf', 'tf2_msgs/TFMessage'),
                                          ('/tf_static', 'tf2_msgs/TFMessage')))
                for topic, msg_type in subscriptions:
                    request = {'op': 'subscribe', 'id': 'hw:' + topic, 'topic': topic, 'type': msg_type,
                               'queue_length': 1, 'fragment_size': 60000, 'compression': 'none'}
                    if topic == self.ros1_odom_topic:
                        request['throttle_rate'] = self.odom_throttle_rate_ms
                    elif topic == '/tf':
                        request['throttle_rate'] = self.tf_throttle_rate_ms
                    self._send(request)
                while rclpy.ok():
                    packet = decode_packet(self._socket.recv(), self._assembler)
                    if packet and packet.get('op') == 'publish':
                        self._publish(packet.get('topic'), packet.get('msg', {}))
            except Exception as error:
                self.get_logger().warn('rosbridge 连接中断：%s' % error)
                self._socket = None
                time.sleep(2.0)

    def _receive_image_loop(self, topic):
        """用独立 WebSocket 接收单个图像话题，隔离官方 fragment 的错误 id。"""
        try:
            import websocket
        except ImportError:
            return
        while rclpy.ok():
            connection = None
            try:
                options = {}
                if self.host_header:
                    options['host'] = self.host_header
                connection = websocket.create_connection(self.url, timeout=5, **options)
                connection.send(json.dumps({
                    'op': 'subscribe', 'id': 'hw-image:' + topic, 'topic': topic,
                    'type': 'sensor_msgs/Image', 'queue_length': 1, 'fragment_size': 60000,
                    'compression': 'none', 'throttle_rate': self.image_throttle_rate_ms,
                }, separators=(',', ':')))
                assembler = FragmentAssembler()
                while rclpy.ok():
                    packet = decode_packet(connection.recv(), assembler)
                    if packet and packet.get('op') == 'publish':
                        self._publish(packet.get('topic'), packet.get('msg', {}))
            except Exception as error:
                self.get_logger().warn('图像 rosbridge 连接中断：%s（%s）' % (topic, error))
                time.sleep(2.0)
            finally:
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass

    def _publish(self, topic, source):
        header = source.get('header', {})
        if topic == self.ros1_odom_topic:
            message = Odometry(); _stamp(message.header.stamp, header.get('stamp')); message.header.frame_id = header.get('frame_id', '')
            message.child_frame_id = source.get('child_frame_id', '')
            pose = source.get('pose', {}).get('pose', {}); twist = source.get('twist', {}).get('twist', {})
            for name in ('x', 'y', 'z'):
                setattr(message.pose.pose.position, name, float(pose.get('position', {}).get(name, 0.0)))
                setattr(message.twist.twist.linear, name, float(twist.get('linear', {}).get(name, 0.0)))
            self._official_odom_xy = (message.pose.pose.position.x, message.pose.pose.position.y)
            # Pose 使用四元数 (x/y/z/w)，Twist.angular 是三维 Vector3，不能把 w 写入其中。
            # 这里若混写会在首个 Odometry 包抛 AttributeError，使整个 rosbridge 收帧循环重连。
            for name in ('x', 'y', 'z', 'w'):
                setattr(message.pose.pose.orientation, name, float(pose.get('orientation', {}).get(name, 0.0)))
            for name in ('x', 'y', 'z'):
                setattr(message.twist.twist.angular, name, float(twist.get('angular', {}).get(name, 0.0)))
            self._queue_message('odom', message)
            # /hazardwalker/odom 已是由 Gazebo 真值压缩得到的最新位姿。以它生成唯一的
            # odom→base 动态 TF，避免官方控制器估计 TF 与真值 TF 同名冲突，从而保证
            # world→map→odom→base→real_sense 可供三维定位查询。
            odom_tf = TransformStamped()
            odom_tf.header.stamp = message.header.stamp
            odom_tf.header.frame_id = message.header.frame_id or 'odom'
            odom_tf.child_frame_id = message.child_frame_id or 'base'
            odom_tf.transform.translation.x = message.pose.pose.position.x
            odom_tf.transform.translation.y = message.pose.pose.position.y
            odom_tf.transform.translation.z = message.pose.pose.position.z
            odom_tf.transform.rotation = message.pose.pose.orientation
            self._queue_message('tf', TFMessage(transforms=[odom_tf]))
        elif topic in (self.rgb_topic, self.depth_topic):
            message = Image(); _stamp(message.header.stamp, header.get('stamp')); message.header.frame_id = header.get('frame_id', '')
            message.height = int(source.get('height', 0)); message.width = int(source.get('width', 0))
            message.encoding = source.get('encoding', ''); message.is_bigendian = int(source.get('is_bigendian', 0)); message.step = int(source.get('step', 0))
            try:
                message.data = base64.b64decode(source.get('data', ''), validate=True)
            except (binascii.Error, ValueError) as error:
                self._dropped_image_frames[topic] = self._dropped_image_frames.get(topic, 0) + 1
                self.get_logger().warn('丢弃无效图像帧：%s（%s）' % (topic, error))
                return
            self._queue_message('rgb' if topic == self.rgb_topic else 'depth', message)
        elif topic in (self.rgb_info_topic, self.depth_info_topic):
            message = CameraInfo(); _stamp(message.header.stamp, header.get('stamp')); message.header.frame_id = header.get('frame_id', '')
            message.height = int(source.get('height', 0)); message.width = int(source.get('width', 0))
            message.k = [float(value) for value in source.get('K', source.get('k', []))]
            message.d = [float(value) for value in source.get('D', source.get('d', []))]
            message.r = [float(value) for value in source.get('R', source.get('r', []))]
            message.p = [float(value) for value in source.get('P', source.get('p', []))]
            self._queue_message('rgb_info' if topic == self.rgb_info_topic else 'depth_info', message)
        elif self.enable_tf_relay and topic in ('/tf', '/tf_static'):
            # 官方 rosbridge 会对高频 /tf 回放历史消息。动态位姿已由上面的
            # /hazardwalker/odom 真值提供，map→odom 在官方场景为恒等变换；不再
            # 转发任何 ROS1 /tf，避免旧时间戳污染 tf2 缓存。
            if topic == '/tf':
                self._counts[topic] = self._counts.get(topic, 0) + 1
                return
            message = TFMessage()
            for source_transform in source.get('transforms', []):
                header_value = source_transform.get('header', {})
                transform_value = source_transform.get('transform', {})
                parent = header_value.get('frame_id', '')
                child = source_transform.get('child_frame_id', '')
                translation_value = transform_value.get('translation', {})
                # 所有官方 odom→base 动态边均由上方的 /hazardwalker/odom 真值替代。
                # 即使某一帧数值接近，也不能让 tf2 在两个同名发布者间非确定性选择。
                if topic == '/tf' and parent == 'odom' and child == 'base':
                    self._dropped_inconsistent_tf += 1
                    continue
                transform = TransformStamped()
                _stamp(transform.header.stamp, header_value.get('stamp'))
                transform.header.frame_id = parent
                transform.child_frame_id = child
                for name in ('x', 'y', 'z'):
                    setattr(transform.transform.translation, name,
                            float(translation_value.get(name, 0.0)))
                for name in ('x', 'y', 'z', 'w'):
                    setattr(transform.transform.rotation, name,
                            float(transform_value.get('rotation', {}).get(name, 0.0)))
                message.transforms.append(transform)
            if (topic == '/tf_static' and self.enable_world_frame_alias and self.world_frame != self.map_frame):
                alias = TransformStamped()
                alias.header.frame_id = self.world_frame
                alias.child_frame_id = self.map_frame
                alias.transform.rotation.w = 1.0
                message.transforms.append(alias)
                map_odom = TransformStamped()
                map_odom.header.frame_id = self.map_frame
                map_odom.child_frame_id = 'odom'
                map_odom.transform.rotation.w = 1.0
                message.transforms.append(map_odom)
            # 不发布空 TFMessage，否则会覆盖由真值里程计排队的 odom→base。
            if message.transforms:
                self._queue_message('tf' if topic == '/tf' else 'tf_static', message)
        else:
            return
        self._counts[topic] = self._counts.get(topic, 0) + 1

    def _on_cmd(self, message):
        if not self.enable_control:
            return
        # rosbridge JSON 与 ROS2 Python 绑定均要求数值是浮点；显式转换避免上游整数字面量造成类型断言。
        payload = {'linear': {'x': float(message.linear.x), 'y': float(message.linear.y), 'z': float(message.linear.z)},
                   'angular': {'x': float(message.angular.x), 'y': float(message.angular.y), 'z': float(message.angular.z)}}
        if self._send({'op': 'publish', 'topic': '/cmd_vel', 'msg': payload}):
            self._forwarded_cmd_count += 1
            self._last_forwarded_cmd = payload
            self._last_cmd = time.monotonic()

    def _watchdog(self):
        if self.enable_control and self._last_cmd and time.monotonic() - self._last_cmd > self.timeout_sec:
            self._send({'op': 'publish', 'topic': '/cmd_vel', 'msg': {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}})
            self._last_cmd = None

    def _status(self):
        self.status_pub.publish(String(data=json.dumps({
            'adapter': 'rosbridge_ros2', 'url': self.url,
            'rosbridge_host_header': self.host_header or None,
            'enable_cmd_vel_relay': self.enable_control, 'received': self._counts,
            'dropped_invalid_image_frames': self._dropped_image_frames,
            'forwarded_cmd_count': self._forwarded_cmd_count,
            'last_forwarded_cmd': self._last_forwarded_cmd,
            'sources': {'rgb': self.rgb_topic, 'depth': self.depth_topic,
                        'rgb_camera_info': self.rgb_info_topic,
                        'depth_camera_info': self.depth_info_topic},
            'enable_image_relay': self.enable_image_relay,
            'image_throttle_rate_ms': self.image_throttle_rate_ms,
            'odom_throttle_rate_ms': self.odom_throttle_rate_ms,
            'enable_tf_relay': self.enable_tf_relay,
            'tf_throttle_rate_ms': self.tf_throttle_rate_ms,
            'dropped_inconsistent_tf': self._dropped_inconsistent_tf,
            'world_frame_alias': (self.world_frame + '->' + self.map_frame
                                  if self.enable_world_frame_alias else None),
        }, sort_keys=True)))


def main():
    rclpy.init(); node = RosbridgeHwAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
