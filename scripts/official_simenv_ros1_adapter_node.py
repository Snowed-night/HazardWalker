#!/usr/bin/env python3
"""官方 SimEnv ROS1 话题中继与控制审计节点。

所属组：平台与仿真组。负责人：姜晨。
文件作用：运行在官方 ROS1 Noetic Docker 容器内，将 RGB、深度、内参、点云和里程计转换为
HazardWalker 稳定 ``/hw/*`` 名称；接收 ROS2 经 ``ros1_bridge`` 送回的 ``/hw/cmd_vel``，在
控制器身份可验证时才转发到官方 ``/cmd_vel``。
当前边界：本节点不替代官方步态/控制器，也不把“收到 Twist”写成“机器人已运动”。真正运动必须
由验证脚本同步检查 ROS1 回显、控制器订阅和至少 1m 的里程计变化。
验证方式：scripts/verify_official_simenv_ros1_adapter.sh；可在无 ROS 环境的机器上运行离线测试。
"""

from __future__ import print_function

import json
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String


class OfficialSimEnvAdapter(object):
    """ROS1 内部官方话题到 /hw 话题的无损转发与安全控制门。"""

    SENSOR_MAPPINGS = (
        ('/Odometry_gazebo', '/hw/odom', Odometry),
        ('/real_sense/rgb/image_raw', '/hw/camera/image_raw', Image),
        ('/real_sense/depth/image_raw', '/hw/camera/depth_image', Image),
        ('/real_sense/rgb/camera_info', '/hw/camera/camera_info', CameraInfo),
        ('/real_sense/depth/camera_info', '/hw/camera/depth_camera_info', CameraInfo),
        ('/real_sense/depth/points', '/hw/camera/depth_points', PointCloud2),
        ('/livox/Pointcloud2', '/hw/lidar/points', PointCloud2),
    )

    def __init__(self):
        self._lock = threading.Lock()
        self._enable_cmd_vel = bool(rospy.get_param('~enable_cmd_vel_relay', False))
        self._allow_unknown_controller = bool(rospy.get_param('~allow_unknown_controller', False))
        self._timeout_sec = float(rospy.get_param('~cmd_vel_timeout_sec', 0.5))
        self._max_linear = float(rospy.get_param('~max_linear_mps', 0.45))
        self._max_angular = float(rospy.get_param('~max_angular_rps', 0.8))
        self._controller_patterns = tuple(rospy.get_param(
            '~controller_node_patterns', ['junior_ctrl', 'unitree_gazebo_servo']))
        # 不能使用 rospy.Time：官方场景暂停或 /clock 停止时，ROS 时间不会前进，零速看门狗会失效。
        self._last_cmd_monotonic = None
        self._last_cmd = Twist()
        self._forwarded_count = 0
        self._blocked_count = 0
        self._sensor_counts = {}
        self._source_overrides = {
            '/Odometry_gazebo': rospy.get_param('~odom_topic', '/Odometry_gazebo'),
            '/real_sense/rgb/image_raw': rospy.get_param(
                '~rgb_topic', '/real_sense/rgb/image_raw'),
            '/real_sense/depth/image_raw': rospy.get_param(
                '~depth_topic', '/real_sense/depth/image_raw'),
            '/real_sense/rgb/camera_info': rospy.get_param(
                '~rgb_camera_info_topic', '/real_sense/rgb/camera_info'),
            '/real_sense/depth/camera_info': rospy.get_param(
                '~depth_camera_info_topic', '/real_sense/depth/camera_info'),
            '/real_sense/depth/points': rospy.get_param(
                '~depth_points_topic', '/real_sense/depth/points'),
            '/livox/Pointcloud2': rospy.get_param(
                '~lidar_points_topic', '/livox/Pointcloud2'),
        }

        self._status_pub = rospy.Publisher(
            '/hw/platform/official_simenv_adapter_status', String, queue_size=10, latch=True)
        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._sensor_publishers = {}
        for default_source, destination, msg_type in self.SENSOR_MAPPINGS:
            source = self._source_overrides[default_source]
            if source in self._sensor_publishers:
                raise ValueError('多个 /hw 输出不能复用同一 ROS1 源话题：%s' % source)
            self._sensor_publishers[source] = rospy.Publisher(destination, msg_type, queue_size=1)
            rospy.Subscriber(source, msg_type, self._make_sensor_callback(source), queue_size=1)

        rospy.Subscriber('/hw/cmd_vel', Twist, self._on_hw_cmd_vel, queue_size=10)
        self._publish_status(None)
        self._safety_thread = threading.Thread(target=self._safety_loop)
        self._safety_thread.daemon = True
        self._safety_thread.start()

    def _make_sensor_callback(self, source):
        """构造保留原始 header/数据的传感器回调，避免序列化导致时序错位。"""

        def callback(message):
            self._sensor_publishers[source].publish(message)
            with self._lock:
                self._sensor_counts[source] = self._sensor_counts.get(source, 0) + 1
        return callback

    def _controller_subscribers(self):
        """从 ROS master 获取 /cmd_vel 的真实订阅节点，供安全门和验收记录使用。"""

        try:
            _, subscriptions, _ = rospy.get_master().getSystemState()
            for topic, nodes in subscriptions:
                if topic == '/cmd_vel':
                    return sorted(set(nodes))
        except Exception as error:  # ROS master 短暂不可用时保守拒绝控制
            rospy.logwarn_throttle(5.0, '无法读取 /cmd_vel 订阅者：%s' % error)
        return []

    def _has_known_controller(self, subscribers):
        """避免仅有 ros1_bridge 自己订阅时仍把命令当作已进入四足控制器。"""

        return any(any(pattern in node for pattern in self._controller_patterns) for node in subscribers)

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def _on_hw_cmd_vel(self, message):
        """将 ROS2 业务速度命令安全转发到官方 ROS1，未知控制器时默认拒绝。"""

        subscribers = self._controller_subscribers()
        controller_ready = self._has_known_controller(subscribers)
        if not self._enable_cmd_vel or (not controller_ready and not self._allow_unknown_controller):
            with self._lock:
                self._blocked_count += 1
            rospy.logwarn_throttle(
                2.0,
                '拒绝 /hw/cmd_vel：enable=%s，controller_ready=%s，subscribers=%s' %
                (self._enable_cmd_vel, controller_ready, subscribers))
            return

        safe = Twist()
        safe.linear.x = self._clamp(message.linear.x, self._max_linear)
        safe.linear.y = self._clamp(message.linear.y, self._max_linear)
        safe.angular.z = self._clamp(message.angular.z, self._max_angular)
        self._cmd_pub.publish(safe)
        with self._lock:
            self._last_cmd = safe
            self._last_cmd_monotonic = time.monotonic()
            self._forwarded_count += 1

    def _watchdog(self, _event):
        """业务端断流超过超时时间后主动发送零速度，防止桥断开后持续运动。"""

        with self._lock:
            last_time = self._last_cmd_monotonic
        if last_time is None:
            return
        if time.monotonic() - last_time > self._timeout_sec:
            self._cmd_pub.publish(Twist())
            with self._lock:
                self._last_cmd_monotonic = None

    def _safety_loop(self):
        """用墙钟驱动看门狗和状态，即使 Gazebo /clock 暂停也保持安全行为。"""

        next_status = 0.0
        while not rospy.is_shutdown():
            self._watchdog(None)
            now = time.monotonic()
            if now >= next_status:
                self._publish_status(None)
                next_status = now + 0.5
            time.sleep(0.05)

    def _publish_status(self, _event):
        """发布结构化状态，供 ROS2 侧日志和验证脚本判定，不冒充运动成功。"""

        subscribers = self._controller_subscribers()
        with self._lock:
            payload = {
                'adapter': 'official_simenv_ros1_adapter',
                'enable_cmd_vel_relay': self._enable_cmd_vel,
                'allow_unknown_controller': self._allow_unknown_controller,
                'controller_subscribers': subscribers,
                'known_controller_ready': self._has_known_controller(subscribers),
                'forwarded_cmd_count': self._forwarded_count,
                'blocked_cmd_count': self._blocked_count,
                'sensor_message_counts': dict(self._sensor_counts),
                'note': 'forwarded_cmd_count only proves ROS1 relay publication; check odometry for real motion.',
            }
        self._status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main():
    rospy.init_node('hazardwalker_official_simenv_adapter', anonymous=False)
    OfficialSimEnvAdapter()
    rospy.loginfo('Official SimEnv ROS1 adapter started; /hw/cmd_vel relay defaults to disabled.')
    rospy.spin()


if __name__ == '__main__':
    main()
