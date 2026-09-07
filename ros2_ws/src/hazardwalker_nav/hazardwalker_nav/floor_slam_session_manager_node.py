"""逐楼层管理独立 Cartographer 2D 会话。

所属组：SLAM 与导航组。负责人：姜晨。
每次公开电梯动作切换楼层后，结束当前 Cartographer 进程并启动全新会话，
避免结构相似的不同楼层进入同一个位姿图。节点只消费公开楼层编号和 `/map`，
不读取 Gazebo 真值、场景布局或危险源真值。
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String


def build_cartographer_session_commands(
        configuration_directory: str,
        configuration_basename: str = 'cartographer_official_2d.lua',
        use_sim_time: bool = True) -> tuple[list[str], list[str]]:
    """构造一个独立 2D Cartographer 会话及其占据图进程命令。"""

    if not configuration_directory or not configuration_basename:
        raise ValueError('Cartographer 配置目录和文件名不能为空')
    sim_value = 'true' if bool(use_sim_time) else 'false'
    cartographer = [
        'ros2', 'run', 'cartographer_ros', 'cartographer_node',
        '-configuration_directory', str(configuration_directory),
        '-configuration_basename', str(configuration_basename),
        '--ros-args',
        '-r', '__node:=hazardwalker_cartographer',
        '-r', 'scan:=/hw/scan',
        '-r', 'imu:=/hw/trunk_imu',
        '-r', 'odom:=/hazardwalker/slam/odometry',
        '-p', f'use_sim_time:={sim_value}',
    ]
    occupancy = [
        'ros2', 'run', 'cartographer_ros',
        'cartographer_occupancy_grid_node',
        '-resolution', '0.05', '-publish_period_sec', '1.0',
        '--ros-args',
        '-r', '__node:=hazardwalker_cartographer_occupancy_grid',
        '-p', f'use_sim_time:={sim_value}',
    ]
    return cartographer, occupancy


def _stamp_seconds(message: OccupancyGrid) -> float:
    return (
        float(message.header.stamp.sec)
        + float(message.header.stamp.nanosec) * 1e-9
    )


class FloorSlamSessionManagerNode(Node):
    """保证每次楼层切换都使用新的 Cartographer 位姿图。"""

    def __init__(self) -> None:
        super().__init__('hazardwalker_floor_slam_session_manager')
        self.declare_parameter(
            'floor_index_topic', '/hazardwalker/navigation/floor_index')
        self.declare_parameter(
            'session_topic', '/hazardwalker/slam/floor_session')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_archive_dir', '')
        self.declare_parameter('initial_floor_index', 0)
        self.declare_parameter('configuration_directory', '')
        self.declare_parameter(
            'configuration_basename', 'cartographer_official_2d.lua')
        # 给锚点节点留出时间，用旧会话在电梯内冻结刚完成楼层坐标。
        self.declare_parameter('restart_delay_s', 2.0)
        self.declare_parameter('process_stop_timeout_s', 20.0)

        self.current_floor = int(
            self.get_parameter('initial_floor_index').value)
        self.generation = 0
        self.children: list[subprocess.Popen] = []
        self.pending_floor = None
        self.restart_due_monotonic = None
        self.waiting_for_map = True
        self.map_stamp_before_restart = -1.0
        self.latest_map = None
        self.archived_floors = set()

        qos = QoSProfile(depth=8)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.session_pub = self.create_publisher(
            String, str(self.get_parameter('session_topic').value), qos)
        self.create_subscription(
            Int32,
            str(self.get_parameter('floor_index_topic').value),
            self.on_floor_index,
            qos,
        )
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self.on_map,
            map_qos,
        )
        self.create_timer(0.1, self.on_timer)
        self._start_session()

    def _commands(self):
        return build_cartographer_session_commands(
            str(self.get_parameter('configuration_directory').value),
            str(self.get_parameter('configuration_basename').value),
            bool(self.get_parameter('use_sim_time').value),
        )

    def _publish_session(self, state: str, **extra) -> None:
        payload = {
            'schema': 'hazardwalker_floor_slam_session_v1',
            'state': str(state),
            'floor_index': int(self.current_floor),
            'generation': int(self.generation),
        }
        payload.update(extra)
        self.session_pub.publish(String(data=json.dumps(payload)))

    def _start_session(self) -> None:
        cartographer, occupancy = self._commands()
        self.children = [
            subprocess.Popen(command, start_new_session=True)
            for command in (cartographer, occupancy)
        ]
        self.waiting_for_map = True
        self._publish_session('starting')
        self.get_logger().info(
            f'Cartographer floor session started: floor={self.current_floor}, '
            f'generation={self.generation}.')

    def _stop_session(self) -> None:
        timeout = max(
            1.0,
            float(self.get_parameter('process_stop_timeout_s').value),
        )
        for child in self.children:
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + timeout
        for child in self.children:
            remaining = max(0.0, deadline - time.monotonic())
            try:
                child.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=3.0)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    if child.poll() is None:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait(timeout=3.0)
        self.children = []

    def _archive_current_map(self) -> None:
        """在离层前保存该层最后一张地图，后续会话不得覆盖。"""

        directory = str(self.get_parameter('map_archive_dir').value).strip()
        message = self.latest_map
        if (not directory or message is None
                or self.current_floor in self.archived_floors):
            return
        output = Path(directory)
        output.mkdir(parents=True, exist_ok=True)
        width = int(message.info.width)
        height = int(message.info.height)
        values = list(message.data)
        if width <= 0 or height <= 0 or len(values) != width * height:
            raise RuntimeError('当前楼层 OccupancyGrid 尺寸无效')
        pixels = bytearray()
        # ROS OccupancyGrid 原点在左下；PGM 从左上写入，因此逐行翻转。
        for row in range(height - 1, -1, -1):
            for value in values[row * width:(row + 1) * width]:
                pixels.append(205 if value < 0 else 0 if value >= 65 else 254)
        stem = f'floor_{self.current_floor}'
        pgm = output / f'{stem}.pgm'
        pgm.write_bytes(
            f'P5\n{width} {height}\n255\n'.encode('ascii') + bytes(pixels))
        yaw = 0.0
        q = message.info.origin.orientation
        yaw = 2.0 * math.atan2(float(q.z), float(q.w))
        yaml_text = (
            f'image: {pgm.name}\n'
            f'resolution: {float(message.info.resolution):.9f}\n'
            'origin: '
            f'[{float(message.info.origin.position.x):.9f}, '
            f'{float(message.info.origin.position.y):.9f}, {yaw:.9f}]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
        )
        (output / f'{stem}.yaml').write_text(yaml_text, encoding='utf-8')
        self.archived_floors.add(self.current_floor)
        self.get_logger().info(
            f'Frozen floor map archived: floor={self.current_floor}, '
            f'path={pgm}.')

    def on_floor_index(self, message: Int32) -> None:
        floor = int(message.data)
        if floor == self.current_floor or floor == self.pending_floor:
            return
        self.pending_floor = floor
        self.restart_due_monotonic = (
            time.monotonic()
            + max(0.2, float(self.get_parameter('restart_delay_s').value))
        )
        self._publish_session(
            'restart_pending', next_floor_index=floor)
        self.get_logger().info(
            f'Floor change scheduled: {self.current_floor} -> {floor}.')

    def on_map(self, message: OccupancyGrid) -> None:
        self.latest_map = message
        stamp = _stamp_seconds(message)
        if not self.waiting_for_map or stamp <= self.map_stamp_before_restart:
            return
        if not self.children or any(child.poll() is not None for child in self.children):
            return
        self.waiting_for_map = False
        self.map_stamp_before_restart = stamp
        self._publish_session('ready', map_stamp_sec=round(stamp, 6))
        self.get_logger().info(
            f'Cartographer floor session ready: floor={self.current_floor}, '
            f'generation={self.generation}, map_stamp={stamp:.3f}.')

    def on_timer(self) -> None:
        if self.pending_floor is not None:
            if time.monotonic() < float(self.restart_due_monotonic):
                return
            next_floor = int(self.pending_floor)
            self.pending_floor = None
            self.restart_due_monotonic = None
            self._publish_session('stopping', next_floor_index=next_floor)
            self._archive_current_map()
            self._stop_session()
            self.current_floor = next_floor
            self.generation += 1
            self.latest_map = None
            self._start_session()
            return
        failed = [
            child.returncode for child in self.children
            if child.poll() is not None
        ]
        if failed:
            self._publish_session('failed', return_codes=failed)
            raise RuntimeError(
                f'Cartographer floor session exited unexpectedly: {failed}')

    def destroy_node(self):
        self._archive_current_map()
        self._stop_session()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FloorSlamSessionManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
