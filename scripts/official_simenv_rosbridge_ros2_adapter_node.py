#!/usr/bin/env python3
"""官方 ROS1 rosbridge 到 ROS2 /hw/* 的双向适配节点。

所属组：平台与仿真组。负责人：姜晨。
运行位置：ROS2 主机，不运行在仅安装 ROS1 的官方 Docker 内。通过 rosbridge v2 WebSocket 订阅官方
RGB、深度、内参、激光和 IMU，完整重组 fragment 后发布稳定 /hw/*；仅显式开启时才把
/hw/cmd_vel 发回官方 /cmd_vel。正式模式默认不订阅 Gazebo 派生里程计，由感知定位组
scan/IMU 节点发布合法 odom。验证：先跑 ROS1 直连控制，再执行适配器验收脚本。
"""

import base64
import binascii
import copy
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from hazardwalker_platform.rosbridge_protocol import (
    FragmentAssembler,
    decode_laser_ranges,
    decode_packet,
    decode_ros_time,
    filter_scan_self_returns,
)


def _stamp(target, source):
    target.sec, target.nanosec = decode_ros_time(source)


class RosbridgeHwAdapter(Node):
    """保留原始图像字节、限制分片缓存并提供零速度看门狗的 ROS2 适配器。"""

    def __init__(self):
        super().__init__('hazardwalker_official_rosbridge_adapter')
        self.url = self.declare_parameter('rosbridge_url', 'ws://127.0.0.1:9090').value
        # 正式实例由 auto_docker 宿主侧管理器传入，供业务门禁拒绝旧版手工残留进程。
        self.managed_lifecycle = bool(
            self.declare_parameter('managed_lifecycle', False).value)
        self.lifecycle_container = str(
            self.declare_parameter('lifecycle_container', '').value)
        # 固定 SEED 是正式人工巡检的数据合同，而不是危险源真值。该值由
        # auto_docker.sh 从当前容器 Config.Env 读取，防止录包命令把运行场景
        # 误标成另一个 SEED。
        self.scenario_seed = str(
            self.declare_parameter('scenario_seed', '').value).strip()
        # 适配器本身必须继续使用墙钟，确保首条仿真时钟到达前接收循环与零速
        # 看门狗仍可运行；这里只负责把官方 ROS1 /clock 原样转成 ROS2 /clock。
        self.enable_clock_relay = bool(
            self.declare_parameter('enable_clock_relay', True).value)
        self.clock_topic = self.declare_parameter('clock_topic', '/clock').value
        self.clock_throttle_rate_ms = int(
            self.declare_parameter('clock_throttle_rate_ms', 20).value)
        # Docker 端口映射可使外部 URL 端口不同于容器内 rosbridge 监听端口；部分 rosbridge 会校验 Host。
        self.host_header = self.declare_parameter('rosbridge_host_header', '').value
        # 控制逐段验收可暂时停掉高带宽图像，避免图像重组故障掩盖 /cmd_vel 链路；默认保持完整 RGB-D 转发。
        self.enable_image_relay = bool(self.declare_parameter('enable_image_relay', True).value)
        # 原始 640x480 RGB-D 通过 rosbridge 时，前一帧未收全又开始下一帧会造成分片混杂。
        # 默认节流到 2 Hz；平台带宽验证充足后可按毫秒调小此参数。
        # 默认 5 Hz 支撑动态检测与辅助转向；第一人称视频仍走 ROS1 压缩流，
        # 不通过本适配器搬运高帧率原始图像。
        self.image_throttle_rate_ms = int(
            self.declare_parameter('image_throttle_rate_ms', 200).value)
        # ---- 激光雷达与 IMU 转发 (导航组 SLAM 必需) ----
        self.enable_scan_relay = bool(self.declare_parameter('enable_scan_relay', True).value)
        # 当前 SLAM 只消费 LaserScan 与 trunk IMU。默认关闭高带宽点云和未消费
        # 的 Livox IMU，避免它们在同一 rosbridge WebSocket 上饿死 /clock、/scan。
        self.enable_pointcloud_relay = bool(
            self.declare_parameter('enable_pointcloud_relay', False).value)
        self.enable_livox_imu_relay = bool(
            self.declare_parameter('enable_livox_imu_relay', False).value)
        self.enable_trunk_imu_relay = bool(
            self.declare_parameter('enable_trunk_imu_relay', True).value)
        self.scan_throttle_rate_ms = int(
            self.declare_parameter('scan_throttle_rate_ms', 50).value)
        self.scan_self_filter_range_m = float(
            self.declare_parameter('scan_self_filter_range_m', 0.40).value)
        self.pointcloud_throttle_rate_ms = int(
            self.declare_parameter('pointcloud_throttle_rate_ms', 200).value)
        self.imu_throttle_rate_ms = int(
            self.declare_parameter('imu_throttle_rate_ms', 20).value)
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.livox_cloud_topic = self.declare_parameter('livox_cloud_topic', '/livox/Pointcloud2').value
        self.livox_imu_topic = self.declare_parameter('livox_imu_topic', '/livox/imu').value
        self.trunk_imu_topic = self.declare_parameter('trunk_imu_topic', '/trunk_imu').value
        # 三维定位需要 tf2 查询相机到 world/odom 的外参；官方 ROS1 /tf 必须显式转入 ROS2。
        self.enable_tf_relay = bool(self.declare_parameter('enable_tf_relay', True).value)
        # Gazebo 派生里程计仅供平台诊断，正式算法必须关闭。默认 false，避免把
        # /hazardwalker/odom 或 /Odometry_gazebo 包装成合法 SLAM。
        self.enable_odom_relay = bool(
            self.declare_parameter('enable_odom_relay', False).value
        )
        self.odom_throttle_rate_ms = int(self.declare_parameter('odom_throttle_rate_ms', 20).value)
        # 官方 /tf 可达 500 Hz；三维定位消费的是低频 RGB-D，保留 50 Hz 已足够插值，
        # 否则 JSON 解码会饿死大图像分片与感知回调。
        self.tf_throttle_rate_ms = int(self.declare_parameter('tf_throttle_rate_ms', 20).value)
        self.tf_odom_consistency_tolerance_m = float(
            self.declare_parameter('tf_odom_consistency_tolerance_m', 0.25).value)
        # 合法 SLAM 的 map 原点位于公开出生点；赛题结果要求 Gazebo world。
        # 使用官方 reference.md 的公开起点参数发布 world→map，绝不读取场景 manifest。
        self.enable_world_frame_alias = bool(self.declare_parameter('enable_world_frame_alias', True).value)
        self.world_frame = self.declare_parameter('world_frame', 'world').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.public_start_world_x = float(
            self.declare_parameter('public_start_world_x', 0.0).value
        )
        self.public_start_world_y = float(
            self.declare_parameter('public_start_world_y', -2.2).value
        )
        self.public_start_world_z = float(
            self.declare_parameter('public_start_world_z', 0.6).value
        )
        self.public_start_world_yaw = float(
            self.declare_parameter('public_start_world_yaw', 1.5708).value
        )
        self.enable_control = bool(self.declare_parameter('enable_cmd_vel_relay', False).value)
        self.enable_mission_state_relay = bool(
            self.declare_parameter('enable_mission_state_relay', True).value
        )
        self.ros2_mission_state_topic = self.declare_parameter(
            'ros2_mission_state_topic', '/hw/mission/state',
        ).value
        self.ros1_mission_state_topic = self.declare_parameter(
            'ros1_mission_state_topic', '/hazardwalker/mission/state',
        ).value
        # 第一人称 sidecar 运行在 ROS1 网络。这里只转发低带宽 JSON 状态，
        # 供浏览器叠框和显示动作建议；不改变相机图像，也不新增控制入口。
        self.enable_gui_overlay_relay = bool(
            self.declare_parameter('enable_gui_overlay_relay', True).value
        )
        self.gui_overlay_topics = {
            '/hw/perception/hazard_detections': '/hazardwalker/gui/perception',
            '/hw/control/status': '/hazardwalker/gui/control_status',
            '/hw/control/assist_status': '/hazardwalker/gui/assist_status',
        }
        self.gui_assist_request_topic = self.declare_parameter(
            'gui_assist_request_topic',
            '/hazardwalker/gui/assist_request',
        ).value
        self.timeout_sec = float(self.declare_parameter('cmd_vel_timeout_sec', 0.5).value)
        # 官方版本相机命名有差异；源话题可配，但稳定的 ROS2 /hw 输出不变。
        self.rgb_topic = self.declare_parameter('rgb_topic', '/real_sense/rgb/image_raw').value
        self.depth_topic = self.declare_parameter('depth_topic', '/real_sense/depth/image_raw').value
        self.rgb_info_topic = self.declare_parameter('rgb_camera_info_topic', '/real_sense/rgb/camera_info').value
        self.depth_info_topic = self.declare_parameter('depth_camera_info_topic', '/real_sense/depth/camera_info').value
        # 仅 enable_odom_relay=true 的平台诊断模式会使用该话题。
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
        self._pending_gui_assist_action = ''
        self._pending_lock = threading.Lock()
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, '/hw/odom', 10)
        self.rgb_pub = self.create_publisher(Image, '/hw/camera/image_raw', 1)
        self.depth_pub = self.create_publisher(Image, '/hw/camera/depth_image', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/hw/camera/camera_info', 1)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/hw/camera/depth_camera_info', 1)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 20)
        self.tf_static_pub = self.create_publisher(
            TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scan_pub = self.create_publisher(LaserScan, '/hw/scan', 10)
        self.scan_raw_pub = self.create_publisher(LaserScan, '/hw/scan_raw', 10)
        self.livox_cloud_pub = self.create_publisher(PointCloud2, '/hw/lidar/points', 10)
        self.livox_imu_pub = self.create_publisher(Imu, '/hw/livox/imu', 10)
        self.trunk_imu_pub = self.create_publisher(Imu, '/hw/trunk_imu', 10)
        self._ros_publishers = {
            'clock': self.clock_pub,
            'odom': self.odom_pub, 'rgb': self.rgb_pub, 'depth': self.depth_pub,
            'rgb_info': self.info_pub, 'depth_info': self.depth_info_pub,
            'tf': self.tf_pub, 'tf_static': self.tf_static_pub,
            'scan': self.scan_pub, 'scan_raw': self.scan_raw_pub,
            'livox_cloud': self.livox_cloud_pub,
            'livox_imu': self.livox_imu_pub, 'trunk_imu': self.trunk_imu_pub,
        }
        self.status_pub = self.create_publisher(String, '/hw/platform/official_simenv_adapter_status', 10)
        self.cmd_sub = self.create_subscription(Twist, '/hw/cmd_vel', self._on_cmd, 10)
        self.mission_state_sub = self.create_subscription(
            String, self.ros2_mission_state_topic, self._on_mission_state, 10,
        )
        self.gui_overlay_subscriptions = []
        self.assist_start_client = self.create_client(
            Trigger, '/hw/control/assist_align/start')
        self.assist_cancel_client = self.create_client(
            Trigger, '/hw/control/assist_align/cancel')
        if self.enable_gui_overlay_relay:
            for ros2_topic, ros1_topic in self.gui_overlay_topics.items():
                self.gui_overlay_subscriptions.append(self.create_subscription(
                    String,
                    ros2_topic,
                    lambda message, topic=ros1_topic: self._on_gui_overlay(
                        topic, message),
                    10,
                ))
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
            assist_action = self._pending_gui_assist_action
            self._pending_gui_assist_action = ''
        for name, message in pending.items():
            self._ros_publishers[name].publish(message)
        if assist_action:
            self._dispatch_gui_assist_request(assist_action)

    def _dispatch_gui_assist_request(self, action):
        """在 ROS2 执行器线程调用辅助服务，网页不能直接发布速度。"""

        client = (
            self.assist_start_client if action == 'start'
            else self.assist_cancel_client
        )
        if not client.service_is_ready():
            self.get_logger().warning('辅助对准服务未就绪，拒绝 GUI 请求：%s' % action)
            self._counts['gui_assist_request_rejected'] = (
                self._counts.get('gui_assist_request_rejected', 0) + 1)
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda result, requested=action: self._on_gui_assist_result(
                requested, result))
        self._counts['gui_assist_request:' + action] = (
            self._counts.get('gui_assist_request:' + action, 0) + 1)

    def _on_gui_assist_result(self, action, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(
                'GUI 辅助对准请求失败：%s（%s）' % (action, error))
            return
        if not response.success:
            self.get_logger().warning(
                'GUI 辅助对准请求被拒绝：%s（%s）' % (
                    action, response.message))

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
                if self.enable_mission_state_relay:
                    self._send({
                        'op': 'advertise',
                        'topic': self.ros1_mission_state_topic,
                        'type': 'std_msgs/String',
                    })
                if self.enable_gui_overlay_relay:
                    for topic in self.gui_overlay_topics.values():
                        self._send({
                            'op': 'advertise',
                            'topic': topic,
                            'type': 'std_msgs/String',
                        })
                subscriptions = [(self.rgb_info_topic, 'sensor_msgs/CameraInfo'),
                                 (self.depth_info_topic, 'sensor_msgs/CameraInfo')]
                if self.enable_gui_overlay_relay:
                    subscriptions.append((
                        self.gui_assist_request_topic, 'std_msgs/String'))
                if self.enable_clock_relay:
                    subscriptions.append((self.clock_topic, 'rosgraph_msgs/Clock'))
                if self.enable_odom_relay:
                    subscriptions.append((self.ros1_odom_topic, 'nav_msgs/Odometry'))
                if self.enable_tf_relay:
                    subscriptions.extend((('/tf', 'tf2_msgs/TFMessage'),
                                          ('/tf_static', 'tf2_msgs/TFMessage')))
                if self.enable_scan_relay:
                    subscriptions.append((self.scan_topic, 'sensor_msgs/LaserScan'))
                if self.enable_pointcloud_relay:
                    subscriptions.append(
                        (self.livox_cloud_topic, 'sensor_msgs/PointCloud2'))
                if self.enable_livox_imu_relay:
                    subscriptions.append((self.livox_imu_topic, 'sensor_msgs/Imu'))
                if self.enable_trunk_imu_relay:
                    subscriptions.append((self.trunk_imu_topic, 'sensor_msgs/Imu'))
                for topic, msg_type in subscriptions:
                    request = {'op': 'subscribe', 'id': 'hw:' + topic, 'topic': topic, 'type': msg_type,
                               'queue_length': 1, 'fragment_size': 60000, 'compression': 'none'}
                    if topic == self.ros1_odom_topic:
                        request['throttle_rate'] = self.odom_throttle_rate_ms
                    elif topic == self.clock_topic:
                        request['throttle_rate'] = self.clock_throttle_rate_ms
                    elif topic == '/tf':
                        request['throttle_rate'] = self.tf_throttle_rate_ms
                    elif topic == self.scan_topic:
                        request['throttle_rate'] = self.scan_throttle_rate_ms
                    elif topic == self.livox_cloud_topic:
                        request['throttle_rate'] = self.pointcloud_throttle_rate_ms
                    elif topic in (self.livox_imu_topic, self.trunk_imu_topic):
                        request['throttle_rate'] = self.imu_throttle_rate_ms
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
        if self.enable_clock_relay and topic == self.clock_topic:
            message = Clock()
            message.clock.sec, message.clock.nanosec = decode_ros_time(
                source.get('clock', {}),
            )
            self._queue_message('clock', message)
        elif self.enable_odom_relay and topic == self.ros1_odom_topic:
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
            # 该分支只允许平台诊断。正式模式由 scan_imu_localizer_node 唯一发布
            # odom→base，不能把这里的 Gazebo 派生值用于探索或危险源坐标。
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
            # 动态 /tf 可能包含 Gazebo/控制器派生位姿，正式模式全部丢弃；只保留
            # base→real_sense 等公开静态传感器外参。
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
                # map→odom 由 SLAM Toolbox 唯一拥有，odom→base 由合法 scan/IMU
                # 节点唯一拥有；即使官方把这些边错误放入 /tf_static 也必须过滤。
                if (
                    (parent == self.map_frame and child == 'odom')
                    or (parent == 'odom' and child in ('base', 'base_link'))
                    or (parent == self.world_frame and child == self.map_frame)
                ):
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
                alias.transform.translation.x = self.public_start_world_x
                alias.transform.translation.y = self.public_start_world_y
                alias.transform.translation.z = self.public_start_world_z
                half_yaw = self.public_start_world_yaw * 0.5
                alias.transform.rotation.z = math.sin(half_yaw)
                alias.transform.rotation.w = math.cos(half_yaw)
                message.transforms.append(alias)
            # 不发布空 TFMessage。
            if message.transforms:
                self._queue_message('tf' if topic == '/tf' else 'tf_static', message)
        elif self.enable_scan_relay and topic == self.scan_topic:
            message = LaserScan()
            _stamp(message.header.stamp, header.get('stamp'))
            message.header.frame_id = header.get('frame_id', 'laser_scan')
            message.angle_min = float(source.get('angle_min', 0.0))
            message.angle_max = float(source.get('angle_max', 0.0))
            message.angle_increment = float(source.get('angle_increment', 0.0))
            message.range_min = float(source.get('range_min', 0.0))
            message.range_max = float(source.get('range_max', 0.0))
            # rosbridge 会把 LaserScan 的 +inf 回波编码成 JSON null。必须恢复为
            # +inf，不能让 float(None) 使主 WebSocket 反复断线并拖垮 SLAM。
            message.ranges = decode_laser_ranges(source.get('ranges', []))
            message.intensities = [
                0.0 if value is None else float(value)
                for value in source.get('intensities', [])
            ]
            # 原始合法输入保存在 /hw/scan_raw；导航/SLAM 统一消费仅去除已标定
            # 机身自回波的 /hw/scan，不读取任何 Gazebo 真值或场景元数据。
            self._queue_message('scan_raw', copy.deepcopy(message))
            message.ranges = filter_scan_self_returns(
                message.ranges, self.scan_self_filter_range_m)
            message.range_min = max(
                message.range_min, self.scan_self_filter_range_m)
            self._queue_message('scan', message)
        elif self.enable_pointcloud_relay and topic == self.livox_cloud_topic:
            message = PointCloud2()
            _stamp(message.header.stamp, header.get('stamp'))
            message.header.frame_id = header.get('frame_id', 'laser_livox')
            message.height = int(source.get('height', 1))
            message.width = int(source.get('width', 0))
            message.point_step = int(source.get('point_step', 0))
            message.row_step = int(source.get('row_step', 0))
            message.is_dense = bool(source.get('is_dense', False))
            from sensor_msgs.msg import PointField
            message.fields = [PointField(
                name=f.get('name', ''), offset=int(f.get('offset', 0)),
                datatype=int(f.get('datatype', 0)), count=int(f.get('count', 1))
            ) for f in source.get('fields', [])]
            try:
                message.data = base64.b64decode(source.get('data', ''), validate=True)
            except (binascii.Error, ValueError):
                return
            self._queue_message('livox_cloud', message)
        elif self.enable_livox_imu_relay and topic == self.livox_imu_topic:
            message = Imu()
            _stamp(message.header.stamp, header.get('stamp'))
            message.header.frame_id = header.get('frame_id', 'livox_imu_link')
            orientation = source.get('orientation', {})
            for name in ('x', 'y', 'z', 'w'):
                setattr(
                    message.orientation, name,
                    float(orientation.get(name, 1.0 if name == 'w' else 0.0)),
                )
            acc = source.get('linear_acceleration', {})
            message.linear_acceleration.x = float(acc.get('x', 0.0))
            message.linear_acceleration.y = float(acc.get('y', 0.0))
            message.linear_acceleration.z = float(acc.get('z', 0.0))
            ang = source.get('angular_velocity', {})
            message.angular_velocity.x = float(ang.get('x', 0.0))
            message.angular_velocity.y = float(ang.get('y', 0.0))
            message.angular_velocity.z = float(ang.get('z', 0.0))
            self._queue_message('livox_imu', message)
        elif self.enable_trunk_imu_relay and topic == self.trunk_imu_topic:
            message = Imu()
            _stamp(message.header.stamp, header.get('stamp'))
            message.header.frame_id = header.get('frame_id', 'imu_link')
            # 合法 scan/IMU 里程计用 orientation 锁定 yaw；旧适配器漏掉该字段会
            # 把所有转向都伪装成零偏航，导致扫描匹配和地图立即失真。
            orientation = source.get('orientation', {})
            for name in ('x', 'y', 'z', 'w'):
                setattr(
                    message.orientation, name,
                    float(orientation.get(name, 1.0 if name == 'w' else 0.0)),
                )
            acc = source.get('linear_acceleration', {})
            message.linear_acceleration.x = float(acc.get('x', 0.0))
            message.linear_acceleration.y = float(acc.get('y', 0.0))
            message.linear_acceleration.z = float(acc.get('z', 0.0))
            ang = source.get('angular_velocity', {})
            message.angular_velocity.x = float(ang.get('x', 0.0))
            message.angular_velocity.y = float(ang.get('y', 0.0))
            message.angular_velocity.z = float(ang.get('z', 0.0))
            self._queue_message('trunk_imu', message)
        elif self.enable_gui_overlay_relay and topic == self.gui_assist_request_topic:
            action = str(source.get('data', '')).strip().lower()
            if action not in ('start', 'cancel'):
                self._counts['gui_assist_request_invalid'] = (
                    self._counts.get('gui_assist_request_invalid', 0) + 1)
                return
            # WebSocket 接收线程只保存最后一次明确请求；服务调用由 ROS2 定时
            # 回调执行，避免从网络线程直接操作 rclpy client。
            with self._pending_lock:
                self._pending_gui_assist_action = action
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

    def _on_mission_state(self, message):
        """把 ROS2 任务完成状态回传给 ROS1 证据记录器。"""

        if not self.enable_mission_state_relay:
            return
        if self._send({
            'op': 'publish',
            'topic': self.ros1_mission_state_topic,
            'msg': {'data': str(message.data)},
        }):
            self._counts['mission_state_forwarded'] = (
                self._counts.get('mission_state_forwarded', 0) + 1
            )

    def _on_gui_overlay(self, ros1_topic, message):
        """把 ROS2 感知/控制状态转成 ROS1 String，供只读 GUI 使用。"""

        if not self.enable_gui_overlay_relay:
            return
        if self._send({
            'op': 'publish',
            'topic': ros1_topic,
            'msg': {'data': str(message.data)},
        }):
            key = 'gui_overlay:' + ros1_topic.rsplit('/', 1)[-1]
            self._counts[key] = self._counts.get(key, 0) + 1

    def _watchdog(self):
        if self.enable_control and self._last_cmd and time.monotonic() - self._last_cmd > self.timeout_sec:
            self._send({'op': 'publish', 'topic': '/cmd_vel', 'msg': {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}})
            self._last_cmd = None

    def _status(self):
        self.status_pub.publish(String(data=json.dumps({
            'adapter': 'rosbridge_ros2', 'url': self.url,
            'managed_lifecycle': self.managed_lifecycle,
            'lifecycle_container': self.lifecycle_container or None,
            'scenario_seed': self.scenario_seed or None,
            'rosbridge_host_header': self.host_header or None,
            'enable_cmd_vel_relay': self.enable_control, 'received': self._counts,
            'enable_gui_overlay_relay': self.enable_gui_overlay_relay,
            'gui_assist_request_topic': self.gui_assist_request_topic,
            'dropped_invalid_image_frames': self._dropped_image_frames,
            'forwarded_cmd_count': self._forwarded_cmd_count,
            'last_forwarded_cmd': self._last_forwarded_cmd,
            'sources': {'rgb': self.rgb_topic, 'depth': self.depth_topic,
                        'rgb_camera_info': self.rgb_info_topic,
                        'depth_camera_info': self.depth_info_topic},
            'enable_image_relay': self.enable_image_relay,
            'enable_clock_relay': self.enable_clock_relay,
            'clock_topic': self.clock_topic,
            'clock_throttle_rate_ms': self.clock_throttle_rate_ms,
            'scan_throttle_rate_ms': self.scan_throttle_rate_ms,
            'scan_self_filter_range_m': self.scan_self_filter_range_m,
            'enable_pointcloud_relay': self.enable_pointcloud_relay,
            'enable_livox_imu_relay': self.enable_livox_imu_relay,
            'enable_trunk_imu_relay': self.enable_trunk_imu_relay,
            'enable_mission_state_relay': self.enable_mission_state_relay,
            'image_throttle_rate_ms': self.image_throttle_rate_ms,
            'enable_odom_relay': self.enable_odom_relay,
            'odom_throttle_rate_ms': self.odom_throttle_rate_ms,
            'enable_tf_relay': self.enable_tf_relay,
            'tf_throttle_rate_ms': self.tf_throttle_rate_ms,
            'dropped_inconsistent_tf': self._dropped_inconsistent_tf,
            'world_frame_alias': (self.world_frame + '->' + self.map_frame
                                  if self.enable_world_frame_alias else None),
        }, sort_keys=True)))


def main():
    rclpy.init()
    node = RosbridgeHwAdapter()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        # 栈脚本会向适配器进程组发 SIGTERM；上下文已被 rclpy 关闭时属于正常收尾，
        # 不应把可预期的回收过程打印成适配器故障。
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
