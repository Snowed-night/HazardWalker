"""负责人维护的官方 SimEnv ROS2 安全键盘控制节点。

文件作用：
- 仅向可配置的业务控制话题（默认 ``/hw/cmd_vel``）发布速度。
- 支持 W/S 前后、A/D 左右转、K 立即停止。
- 使用短时命令保持和退出零速度，避免终端失焦后机器人持续运动。

安全边界：
- 只能在已获独占控制时段且平台控制验收通过后运行。
- 本节点不绕过 ROS2 适配器，也不负责启动、重启或切换容器控制器。
"""

import select
import sys
import termios
import time
import tty
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .keyboard_control import KeyboardCommand, command_for_key


class KeyboardControlNode(Node):
    """以安全超时方式发布键盘速度命令。"""

    def __init__(self) -> None:
        super().__init__('hazardwalker_keyboard_control')
        self.declare_parameter('cmd_vel_topic', '/hw/cmd_vel')
        self.declare_parameter('linear_speed', 0.30)
        self.declare_parameter('angular_speed', 0.60)
        self.declare_parameter('command_hold_sec', 0.35)
        self.declare_parameter('publish_rate_hz', 20.0)

        topic = str(self.get_parameter('cmd_vel_topic').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.command_hold_sec = float(
            self.get_parameter('command_hold_sec').value)
        publish_rate_hz = float(
            self.get_parameter('publish_rate_hz').value)
        if publish_rate_hz <= 0.0 or self.command_hold_sec <= 0.0:
            raise ValueError('publish_rate_hz 和 command_hold_sec 必须为正数')

        self.publisher = self.create_publisher(Twist, topic, 10)
        self.active_command: Optional[KeyboardCommand] = None
        self.command_deadline = 0.0
        self.stop_sent_after_timeout = True
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.on_timer)
        self.get_logger().info(
            f'负责人键盘控制已启动：topic={topic}，'
            'W前进/S后退/A左转/D右转/K立即停止；按住按键持续运动。')

    @staticmethod
    def _twist(command: KeyboardCommand) -> Twist:
        message = Twist()
        message.linear.x = command.linear_x
        message.angular.z = command.angular_z
        return message

    def publish_stop(self) -> None:
        """立即发布零速度，并清除任何尚未到期的运动命令。"""

        self.active_command = None
        self.command_deadline = 0.0
        self.stop_sent_after_timeout = True
        stop = KeyboardCommand(0.0, 0.0, '立即停止', is_stop=True)
        # 连发三次降低单包丢失风险；下游控制器仍保留自己的命令看门狗。
        for _ in range(3):
            self.publisher.publish(self._twist(stop))

    def accept_key(self, key: str) -> None:
        command = command_for_key(
            key,
            linear_speed=self.linear_speed,
            angular_speed=self.angular_speed,
        )
        if command is None:
            return
        if command.is_stop:
            self.publish_stop()
            self.get_logger().warning('K 急停：已发布零速度。')
            return

        self.active_command = command
        self.command_deadline = time.monotonic() + self.command_hold_sec
        self.stop_sent_after_timeout = False
        self.publisher.publish(self._twist(command))
        self.get_logger().info(
            f'{command.label}: linear.x={command.linear_x:.2f}, '
            f'angular.z={command.angular_z:.2f}')

    def on_timer(self) -> None:
        if self.active_command is None:
            return
        if time.monotonic() <= self.command_deadline:
            self.publisher.publish(self._twist(self.active_command))
            return
        if not self.stop_sent_after_timeout:
            self.publish_stop()
            self.get_logger().info('按键超时，已自动停止。')


def main(args=None) -> None:
    """ROS2 入口；退出和异常路径都先发布零速度并恢复终端。"""

    if not sys.stdin.isatty():
        raise RuntimeError('keyboard_control_node 必须在交互式终端中运行')

    rclpy.init(args=args)
    node = KeyboardControlNode()
    old_terminal = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not readable:
                continue
            key = sys.stdin.read(1)
            if key.lower() == 'q':
                break
            node.accept_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal)
        node.destroy_node()
        rclpy.shutdown()
