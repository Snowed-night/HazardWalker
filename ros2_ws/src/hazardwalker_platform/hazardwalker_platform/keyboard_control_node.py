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
from rcl_interfaces.msg import SetParametersResult

from .keyboard_control import KeyboardCommand, command_for_key


class KeyboardControlNode(Node):
    """以安全超时方式发布键盘速度命令。"""

    def __init__(self) -> None:
        super().__init__('hazardwalker_keyboard_control')
        self.declare_parameter('cmd_vel_topic', '/hw/cmd_vel')
        # 以稳定优先：相较最初 0.30/0.60 提高响应，但不把 RL 步态推到
        # 急转易翻倒的区间。更高速度须先在独占场景中按 ROS 参数逐步验收。
        self.declare_parameter('linear_speed', 0.45)
        self.declare_parameter('angular_speed', 0.80)
        self.declare_parameter('command_hold_sec', 0.8)
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
        # 参数服务默认只修改参数表，不会自动更新上面缓存的速度成员。
        # 注册回调后，ros2 param set 可立即影响后续按键，不必重启控制节点。
        self.add_on_set_parameters_callback(self._update_motion_parameters)
        self.active_command: Optional[KeyboardCommand] = None
        self.command_deadline = 0.0
        self.stop_sent_after_timeout = True
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.on_timer)
        self.get_logger().info(
            f'负责人键盘控制已启动：topic={topic}，'
            f'W前进/S后退/A左转/D右转/K立即停止；单次按键保持约 {self.command_hold_sec:.1f} 秒，按住按键持续运动。')

    def _update_motion_parameters(self, parameters):
        """安全地热更新键盘控制参数，拒绝会导致失控或失去看门狗的取值。"""

        values = {
            'linear_speed': self.linear_speed,
            'angular_speed': self.angular_speed,
            'command_hold_sec': self.command_hold_sec,
        }
        limits = {
            'linear_speed': (0.05, 0.65),
            'angular_speed': (0.10, 1.20),
            'command_hold_sec': (0.40, 1.20),
        }
        for parameter in parameters:
            if parameter.name not in values:
                continue
            try:
                value = float(parameter.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} 必须为数值')
            lower, upper = limits[parameter.name]
            if not lower <= value <= upper:
                return SetParametersResult(
                    successful=False,
                    reason=(f'{parameter.name} 必须在 {lower} 到 {upper} 之间'))
            values[parameter.name] = value

        self.linear_speed = values['linear_speed']
        self.angular_speed = values['angular_speed']
        self.command_hold_sec = values['command_hold_sec']
        self.get_logger().info(
            '键盘控制参数已更新：'
            f'linear={self.linear_speed:.2f} m/s，'
            f'angular={self.angular_speed:.2f} rad/s，'
            f'hold={self.command_hold_sec:.2f} s。')
        return SetParametersResult(successful=True)

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
        # Ctrl+C 会先使 rclpy context 失效。此时继续 publish 会触发 RCLError，
        # 而控制器自己的命令看门狗会将已过期的速度归零，所以只恢复终端并退出。
        if not rclpy.ok(context=self.context):
            return
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
        # SIGINT 的 ROS2 信号处理器可能已经执行 shutdown；仅在 context 尚可用时
        # 关闭，且容忍检查与关闭之间发生的退出竞态，避免 Ctrl+C 输出伪异常。
        if rclpy.ok(context=node.context):
            try:
                rclpy.shutdown(context=node.context)
            except Exception:
                if rclpy.ok(context=node.context):
                    raise


# 同时支持 ``ros2 run`` 与 ``python -m``。后者用于已装有 ROS2、
# 但尚未构建本工作区的官方共享环境，不需要修改或重启平台容器。
if __name__ == '__main__':
    main()
