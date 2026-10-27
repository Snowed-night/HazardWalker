"""负责人维护的官方 SimEnv 控制备用中继。

文件作用：
- 只把 ROS2 ``/hw/cmd_vel`` 经 rosbridge 转发为官方 ROS1 ``/cmd_vel``。
- 当完整 RGB-D 适配器的执行器失活、但容器和 rosbridge 仍健康时，恢复受控键盘或导航验证。
- 不转发传感器，不读取真值，不启动、停止或改写官方容器进程。

使用边界：
- 仅在平台管理员确认完整适配器失活且共享控制时段独占时启动。
- 完整适配器恢复后必须停止本节点，避免两个速度中继并行发送同一条命令。
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class OfficialSimEnvCmdVelRelay(Node):
    """仅负责速度桥接，断连或超时都以零速度失败关闭。"""

    def __init__(self) -> None:
        super().__init__('hazardwalker_official_cmd_vel_relay')
        self.declare_parameter('rosbridge_url', 'ws://127.0.0.1:9090')
        self.declare_parameter('rosbridge_host_header', '')
        self.declare_parameter('ros2_cmd_vel_topic', '/hw/cmd_vel')
        self.declare_parameter('ros1_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('reconnect_interval_sec', 1.0)

        self.url = str(self.get_parameter('rosbridge_url').value)
        self.host_header = str(self.get_parameter('rosbridge_host_header').value)
        self.ros1_topic = str(self.get_parameter('ros1_cmd_vel_topic').value)
        self.timeout_sec = float(self.get_parameter('cmd_vel_timeout_sec').value)
        self.reconnect_interval_sec = float(
            self.get_parameter('reconnect_interval_sec').value)
        if self.timeout_sec <= 0.0 or self.reconnect_interval_sec <= 0.0:
            raise ValueError('速度超时和重连间隔必须为正数')

        self._socket: Optional[Any] = None
        self._socket_lock = threading.Lock()
        self._last_command_at: Optional[float] = None
        self._forwarded_count = 0
        self._last_error = ''
        self._stop_requested = False

        ros2_topic = str(self.get_parameter('ros2_cmd_vel_topic').value)
        self.create_subscription(Twist, ros2_topic, self._on_command, 10)
        self.status_pub = self.create_publisher(
            String, '/hw/platform/cmd_vel_relay_status', 10)
        self.create_timer(0.05, self._watchdog)
        self.create_timer(0.5, self._publish_status)
        self._worker = threading.Thread(target=self._connection_loop, daemon=True)
        self._worker.start()
        self.get_logger().warning(
            f'负责人控制备用中继已启动：仅 {ros2_topic} -> {self.ros1_topic}；'
            '完整适配器恢复后必须停止本节点。')

    @staticmethod
    def _payload(message: Twist) -> dict[str, dict[str, float]]:
        """显式转换 ROS2 数值，保证 rosbridge 获得 JSON 浮点字段。"""

        return {
            'linear': {
                'x': float(message.linear.x), 'y': float(message.linear.y),
                'z': float(message.linear.z),
            },
            'angular': {
                'x': float(message.angular.x), 'y': float(message.angular.y),
                'z': float(message.angular.z),
            },
        }

    def _connection_loop(self) -> None:
        """后台维护单条 WebSocket；连接失败不阻塞 ROS2 控制回调。"""

        try:
            import websocket
        except ImportError:
            self._last_error = '缺少 websocket-client'
            self.get_logger().error(f'{self._last_error}；请修复 ROS2 主机依赖。')
            return

        while rclpy.ok() and not self._stop_requested:
            with self._socket_lock:
                connected = self._socket is not None
            if connected:
                time.sleep(0.2)
                continue
            try:
                options = {'host': self.host_header} if self.host_header else {}
                connection = websocket.create_connection(
                    self.url, timeout=5, **options)
                connection.send(json.dumps({
                    'op': 'advertise', 'topic': self.ros1_topic,
                    'type': 'geometry_msgs/Twist',
                }, separators=(',', ':')))
                with self._socket_lock:
                    self._socket = connection
                self._last_error = ''
                self.get_logger().info(f'控制备用中继已连接 rosbridge：{self.url}')
            except Exception as error:  # 网络或 rosbridge 尚未就绪时持续失败关闭。
                self._last_error = str(error)
                self.get_logger().warning(f'控制备用中继连接失败：{error}')
                time.sleep(self.reconnect_interval_sec)

    def _send(self, payload: dict[str, dict[str, float]]) -> bool:
        """发送一条速度；错误时断开连接，绝不保留旧速度。"""

        packet = {
            'op': 'publish', 'topic': self.ros1_topic, 'msg': payload,
        }
        with self._socket_lock:
            connection = self._socket
            if connection is None:
                return False
            try:
                connection.send(json.dumps(packet, separators=(',', ':')))
                return True
            except Exception as error:
                self._last_error = str(error)
                self._socket = None
                try:
                    connection.close()
                except Exception:
                    pass
                self.get_logger().error(f'控制备用中继发送失败，已断开：{error}')
                return False

    def _on_command(self, message: Twist) -> None:
        """上游只要给出速度就立即桥接；连接未就绪时失败关闭。"""

        payload = self._payload(message)
        if self._send(payload):
            self._last_command_at = time.monotonic()
            self._forwarded_count += 1

    def _watchdog(self) -> None:
        """没有新命令超过阈值时发送一次零速度，防止断键后滑行。"""

        if self._last_command_at is None:
            return
        if time.monotonic() - self._last_command_at <= self.timeout_sec:
            return
        zero = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
        self._send(zero)
        self._last_command_at = None

    def _publish_status(self) -> None:
        """向测试人员公开连接、转发计数和故障原因，禁止静默失败。"""

        with self._socket_lock:
            connected = self._socket is not None
        self.status_pub.publish(String(data=json.dumps({
            'relay': 'official_simenv_cmd_vel_backup',
            'connected': connected,
            'rosbridge_url': self.url,
            'ros1_cmd_vel_topic': self.ros1_topic,
            'forwarded_count': self._forwarded_count,
            'last_error': self._last_error or None,
        }, sort_keys=True)))

    def stop(self) -> None:
        """退出前主动发零速度并关闭 WebSocket。"""

        self._stop_requested = True
        zero = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
        self._send(zero)
        with self._socket_lock:
            connection = self._socket
            self._socket = None
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass


def main(args: Optional[list[str]] = None) -> None:
    """备用入口；无论异常或中断均执行零速度收尾。"""

    # 依赖缺失时必须在创建 ROS2 节点前失败，不能留下“图中可见但不转发”的空壳节点。
    try:
        import websocket  # noqa: F401
    except ImportError as error:
        raise RuntimeError(
            '缺少 websocket-client；请使用平台组提供的 ROS2 Python 环境。') from error

    rclpy.init(args=args)
    node = OfficialSimEnvCmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
