"""SLAM 监测节点：实时检测跳变、漂移与地图质量。

所属组：导航组。
文件作用：
- 独立观测器，只读 TF 与 /map，不改写任何控制输出。
- 实时打印跳变告警 + 周期打印漂移/地图摘要，并落盘 JSONL 留档。

用法：
    ros2 run hazardwalker_nav slam_monitor
    # 或 python -m hazardwalker_nav.slam_monitor_node
    # 仿真环境需显式启用 sim time：
    #   ros2 run hazardwalker_nav slam_monitor --ros-args -p use_sim_time:=true
"""

from __future__ import annotations

import json
import math
import os
import time
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from .slam_metrics import (
    detect_pose_jump,
    drift_magnitude,
    map_occupancy_stats,
)


def _ensure_dir(path: str) -> str:
    os.makedirs(path, exist_ok=True)
    return path


def _ros_time_str(ros_sec: Optional[float]) -> str:
    """ISO 8601 时间戳字符串（UTC），精确到毫秒；无效时间返回占位符。"""
    if ros_sec is None or ros_sec <= 0.0:
        return '0000-00-00T00:00:00.000Z'
    import datetime as _dt
    dt = _dt.datetime.fromtimestamp(ros_sec, tz=_dt.timezone.utc)
    return dt.strftime('%Y-%m-%dT%H:%M:%S.') + f'{dt.microsecond // 1000:03d}Z'


class SlamMonitorNode(Node):
    """订阅 TF 与 /map，实时检测跳变/漂移/地图质量并落盘。"""

    def __init__(self) -> None:
        super().__init__('hazardwalker_slam_monitor')

        # ---- 参数 ----
        self.declare_parameter('max_speed_m_s', 1.0)
        self.declare_parameter('min_distance_m', 0.5)
        self.declare_parameter('drift_warn_m', 2.0)
        self.declare_parameter('monitor_rate_hz', 10.0)
        self.declare_parameter('map_report_period_s', 1.0)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('odom_frame', 'odom')

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- 订阅 /map ----
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.on_map, 10)

        # ---- 跳变检测状态 ----
        self._last_x: Optional[float] = None
        self._last_y: Optional[float] = None
        self._last_monotonic: Optional[float] = None
        self._jump_count = 0
        self._max_jump_m = 0.0

        # ---- 漂移告警状态 ----
        self._drift_warn_emitted = False

        # ---- 地图降采样状态 ----
        self._last_map_report_monotonic = 0.0

        # ---- 落盘 ----
        self._init_output()

        # ---- 定时器 ----
        rate = float(self.get_parameter('monitor_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(0.5, rate), self.on_timer)

        self.get_logger().info(
            'SLAM 监测已启动：跳变阈值 %.2f m/s + %.2f m 容差，漂移告警 %.1f m；'
            '输出目录 %s'
            % (
                float(self.get_parameter('max_speed_m_s').value),
                float(self.get_parameter('min_distance_m').value),
                float(self.get_parameter('drift_warn_m').value),
                self._dir,
            )
        )

    # ---- 输出目录 ----

    def _init_output(self) -> None:
        output_dir = str(self.get_parameter('output_dir').value)
        if not output_dir:
            ts = time.strftime('%Y%m%d_%H%M%S', time.localtime())
            repo_root = os.environ.get(
                'HAZARDWALKER_ROOT',
                os.path.join(os.path.expanduser('~'), 'HazardWalker'),
            )
            output_dir = os.path.join(
                repo_root, 'reports', 'slam_monitor', f'run_{ts}')
        self._dir = _ensure_dir(output_dir)

        self._jumps_fp = open(
            os.path.join(self._dir, 'jumps.jsonl'), 'w', encoding='utf-8')
        self._drift_fp = open(
            os.path.join(self._dir, 'drift.jsonl'), 'w', encoding='utf-8')
        self._map_fp = open(
            os.path.join(self._dir, 'map_metrics.jsonl'), 'w', encoding='utf-8')

        meta = {
            'start_time': _ros_time_str(self._ros_sec()),
            'output_dir': self._dir,
            'params': {
                'max_speed_m_s': float(self.get_parameter('max_speed_m_s').value),
                'min_distance_m': float(self.get_parameter('min_distance_m').value),
                'drift_warn_m': float(self.get_parameter('drift_warn_m').value),
                'monitor_rate_hz': float(self.get_parameter('monitor_rate_hz').value),
            },
        }
        self._meta_path = os.path.join(self._dir, 'run_meta.json')
        with open(self._meta_path, 'w', encoding='utf-8') as fp:
            json.dump(meta, fp, ensure_ascii=False, indent=2)

    def _ros_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _write_jsonl(self, fp, record: dict) -> None:
        try:
            fp.write(json.dumps(record, ensure_ascii=False) + '\n')
            fp.flush()
        except Exception:
            pass

    def _update_meta(self, key: str, value) -> None:
        try:
            meta = {}
            if os.path.exists(self._meta_path):
                with open(self._meta_path, 'r', encoding='utf-8') as fp:
                    meta = json.load(fp)
            meta[key] = value
            with open(self._meta_path, 'w', encoding='utf-8') as fp:
                json.dump(meta, fp, ensure_ascii=False, indent=2)
        except Exception:
            pass

    # ---- 数据源 ----

    def _lookup(self, target: str, source: str):
        try:
            return self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5))
        except Exception:
            return None

    def on_map(self, msg: OccupancyGrid) -> None:
        now_mono = time.monotonic()
        period = float(self.get_parameter('map_report_period_s').value)
        if now_mono - self._last_map_report_monotonic < period:
            return
        self._last_map_report_monotonic = now_mono

        try:
            grid = np.array(msg.data, dtype=np.int8).reshape(
                msg.info.height, msg.info.width)
        except Exception:
            return
        stats = map_occupancy_stats(grid)
        ros_sec = self._ros_sec()
        record = {
            'time': _ros_time_str(ros_sec),
            'ros_sec': round(ros_sec, 4),
            'occupied_ratio': round(stats['occupied_ratio'], 6),
            'free_ratio': round(stats['free_ratio'], 6),
            'unknown_ratio': round(stats['unknown_ratio'], 6),
            'known_cells': stats['known_cells'],
            'total_cells': stats['total_cells'],
            'width': msg.info.width,
            'height': msg.info.height,
        }
        self._write_jsonl(self._map_fp, record)
        self.get_logger().info(
            f'[MAP] occupied={stats["occupied_ratio"] * 100:.2f}% '
            f'free={stats["free_ratio"] * 100:.2f}% '
            f'unknown={stats["unknown_ratio"] * 100:.2f}% '
            f'{msg.info.width}x{msg.info.height}')

    # ---- 主循环 ----

    def on_timer(self) -> None:
        self._check_pose_jump()
        self._check_drift()

    def _check_pose_jump(self) -> None:
        tf = self._lookup(self.map_frame, self.base_frame)
        if tf is None:
            return
        new_x = tf.transform.translation.x
        new_y = tf.transform.translation.y
        now_mono = time.monotonic()

        if self._last_x is not None and self._last_monotonic is not None:
            displacement = math.hypot(
                new_x - self._last_x, new_y - self._last_y)
            elapsed = now_mono - self._last_monotonic
            if detect_pose_jump(
                displacement, elapsed,
                float(self.get_parameter('max_speed_m_s').value),
                float(self.get_parameter('min_distance_m').value),
            ):
                self._jump_count += 1
                self._max_jump_m = max(self._max_jump_m, displacement)
                ros_sec = self._ros_sec()
                record = {
                    'time': _ros_time_str(ros_sec),
                    'ros_sec': round(ros_sec, 4),
                    'displacement_m': round(displacement, 4),
                    'x': round(new_x, 4),
                    'y': round(new_y, 4),
                    'cumulative': self._jump_count,
                }
                self._write_jsonl(self._jumps_fp, record)
                self._update_meta('jump_count', self._jump_count)
                self._update_meta('max_jump_m', round(self._max_jump_m, 4))
                self.get_logger().error(
                    f'[JUMP] +{displacement:.2f}m at '
                    f'({new_x:.2f},{new_y:.2f}) '
                    f'(累计 {self._jump_count} 次)')

        self._last_x = new_x
        self._last_y = new_y
        self._last_monotonic = now_mono

    def _check_drift(self) -> None:
        tf = self._lookup(self.map_frame, self.odom_frame)
        if tf is None:
            return
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        drift = drift_magnitude(x, y)
        warn_m = float(self.get_parameter('drift_warn_m').value)
        ros_sec = self._ros_sec()

        record = {
            'time': _ros_time_str(ros_sec),
            'ros_sec': round(ros_sec, 4),
            'drift_m': round(drift, 4),
            'x': round(x, 4),
            'y': round(y, 4),
        }
        self._write_jsonl(self._drift_fp, record)

        # 只在首次超阈值时告警一次，回落后重置，避免高频刷屏。
        if drift > warn_m:
            if not self._drift_warn_emitted:
                self.get_logger().warning(
                    f'[DRIFT] map→odom 偏移 {drift:.2f}m '
                    f'超过告警阈值 {warn_m:.1f}m')
                self._drift_warn_emitted = True
        else:
            self._drift_warn_emitted = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(context=node.context):
            rclpy.shutdown(context=node.context)


# 同时支持 ``ros2 run`` 与 ``python -m``。后者用于已装有 ROS2、
# 但尚未重新构建本工作区的环境，便于快速跑起来观测。
if __name__ == '__main__':
    main()
