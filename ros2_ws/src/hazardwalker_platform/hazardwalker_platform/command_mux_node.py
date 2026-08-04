"""键盘、导航和辅助对准的 ROS2 速度仲裁节点。

所有控制源先发布到独立输入话题，本节点是业务层唯一的 ``/hw/cmd_vel``
发布者。模式切换、源超时和节点退出均主动发送零速度，防止多个模块抢占底盘。
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .control_arbitration import ControlArbitrator, should_publish_status


class CommandMuxNode(Node):
    """把显式选中的新鲜控制源转发到统一底盘话题。"""

    def __init__(self) -> None:
        super().__init__('hazardwalker_command_mux')
        self.declare_parameter('keyboard_topic', '/hw/control/keyboard_cmd_vel')
        self.declare_parameter('navigation_topic', '/hw/control/navigation_cmd_vel')
        self.declare_parameter('assist_topic', '/hw/control/assist_cmd_vel')
        self.declare_parameter('output_topic', '/hw/cmd_vel')
        self.declare_parameter('mode_request_topic', '/hw/control/mode_request')
        self.declare_parameter('status_topic', '/hw/control/status')
        self.declare_parameter('default_mode', 'keyboard')
        self.declare_parameter('keyboard_timeout_sec', 0.50)
        self.declare_parameter('navigation_timeout_sec', 0.50)
        self.declare_parameter('assist_timeout_sec', 0.30)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('status_heartbeat_sec', 1.0)
        self.declare_parameter('max_abs_linear_x', 0.80)
        self.declare_parameter('max_abs_linear_y', 0.50)
        self.declare_parameter('max_abs_angular_z', 1.80)

        self.arbitrator = ControlArbitrator(
            default_mode=str(self.get_parameter('default_mode').value),
            source_timeouts_sec={
                'keyboard': float(
                    self.get_parameter('keyboard_timeout_sec').value),
                'navigation': float(
                    self.get_parameter('navigation_timeout_sec').value),
                'assist': float(
                    self.get_parameter('assist_timeout_sec').value),
            },
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('publish_rate_hz 必须为正数')
        self.status_heartbeat_sec = self._positive_parameter(
            'status_heartbeat_sec')
        self.speed_limits = {
            'linear_x': self._positive_parameter('max_abs_linear_x'),
            'linear_y': self._positive_parameter('max_abs_linear_y'),
            'angular_z': self._positive_parameter('max_abs_angular_z'),
        }

        self.output_pub = self.create_publisher(
            Twist, str(self.get_parameter('output_topic').value), 10)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self.mode_sub = self.create_subscription(
            String,
            str(self.get_parameter('mode_request_topic').value),
            self.on_mode_request,
            10,
        )
        self.source_subscriptions = []
        for source, parameter in (
            ('keyboard', 'keyboard_topic'),
            ('navigation', 'navigation_topic'),
            ('assist', 'assist_topic'),
        ):
            self.source_subscriptions.append(self.create_subscription(
                Twist,
                str(self.get_parameter(parameter).value),
                lambda message, source=source: self.on_source(source, message),
                10,
            ))
        self._last_status = None
        self._last_status_publish_monotonic_sec = 0.0
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            '统一控制仲裁已启动：mode=%s output=%s' % (
                self.arbitrator.mode,
                self.get_parameter('output_topic').value,
            )
        )

    def on_source(self, source: str, message: Twist) -> None:
        """缓存控制源最新命令；NaN/Inf 会被拒绝并保持安全停车。"""

        try:
            self.arbitrator.update_source(
                source,
                linear_x=self._bounded(
                    message.linear.x, self.speed_limits['linear_x']),
                linear_y=self._bounded(
                    message.linear.y, self.speed_limits['linear_y']),
                angular_z=self._bounded(
                    message.angular.z, self.speed_limits['angular_z']),
                received_monotonic_sec=time.monotonic(),
            )
        except ValueError as exc:
            # 非法包不能只被忽略，否则同一源上一条合法速度还会继续到超时。
            # 清除缓存后，当前源在下一个 20 Hz 周期立即归零。
            self.arbitrator.clear_source(source)
            self.get_logger().error(
                f'拒绝 {source} 非法速度：{exc}', throttle_duration_sec=2.0)

    def on_mode_request(self, message: String) -> None:
        """接受 keyboard/navigation/assist/stopped 模式请求。"""

        requested = message.data.strip().lower()
        try:
            self.arbitrator.select_mode(requested)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return
        # 切换源时先归零，避免沿用上一个源的剩余速度。
        self.output_pub.publish(Twist())
        self.get_logger().info(f'控制模式切换为 {requested}')

    def on_timer(self) -> None:
        now = time.monotonic()
        result = self.arbitrator.resolve(now)
        output = Twist()
        output.linear.x = result.linear_x
        output.linear.y = result.linear_y
        output.angular.z = result.angular_z
        self.output_pub.publish(output)
        status = {
            'mode': result.mode,
            'source_fresh': result.source_fresh,
            'reason': result.reason,
            'linear_x': result.linear_x,
            'linear_y': result.linear_y,
            'angular_z': result.angular_z,
        }
        encoded = json.dumps(status, ensure_ascii=False, sort_keys=True)
        if should_publish_status(
            encoded,
            self._last_status,
            now_monotonic_sec=now,
            previous_publish_monotonic_sec=(
                self._last_status_publish_monotonic_sec
            ),
            heartbeat_sec=self.status_heartbeat_sec,
        ):
            message = String()
            message.data = encoded
            self.status_pub.publish(message)
            self._last_status = encoded
            self._last_status_publish_monotonic_sec = now

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} 必须是有限正数')
        return value

    @staticmethod
    def _bounded(value: float, limit: float) -> float:
        """拒绝非有限输入，并将正常速度限制到验收过的安全范围。"""

        number = float(value)
        if not math.isfinite(number):
            raise ValueError('速度必须是有限数值')
        return max(-limit, min(limit, number))

    def publish_stop(self) -> None:
        if rclpy.ok(context=self.context):
            self.output_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandMuxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
