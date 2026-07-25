"""导航数据记录器：轨迹、cmd_vel、状态变迁、失败日志、地图保存。

所属组：导航组。
文件作用：
- 提供 ROS-independent 的 NavRecorder 类，供 frontier_explorer_node 集成。
- 记录完整导航闭环证据，支持离线回放和故障分析。
- 所有文件写入立即 flush，崩溃也不丢失已记录数据。

记录内容：
- trajectory.jsonl: 时间戳、合法 SLAM 位姿 (x, y, yaw)、当前状态、目标前沿
- cmd_vel.jsonl: 时间戳、发布的速度指令 (linear_x, angular_z)
- state_transitions.jsonl: 时间戳、状态变迁 (from → to)、触发原因
- failures.jsonl: 时间戳、失败类型、位姿、详情
- reobservations.jsonl: 时间戳、感知复查请求、执行结果
- map.pgm + map.yaml: 最终 SLAM 地图（FINISHED 时保存）

验证方式：
- 每个 JSONL 文件每行一条完整 JSON 记录，可用 jq 或 Python 解析。
- 输出目录按时间戳命名，多轮测试不互相覆盖。
"""

from __future__ import annotations

import json
import math
import os
import time
from typing import Optional, Tuple

import numpy as np


def _ensure_dir(path: str) -> str:
    os.makedirs(path, exist_ok=True)
    return path


def _ros_time_str(ros_sec: Optional[float] = None) -> str:
    """ISO 8601 时间戳字符串，精确到毫秒。"""
    if ros_sec is not None and ros_sec > 0.0:
        t = ros_sec
    else:
        t = time.time()
    # 用 UTC 避免时区歧义
    import datetime as _dt
    dt = _dt.datetime.fromtimestamp(t, tz=_dt.timezone.utc)
    return dt.strftime('%Y-%m-%dT%H:%M:%S.') + f'{dt.microsecond // 1000:03d}Z'


class NavRecorder:
    """导航数据记录器——轨迹、指令、状态、失败事件、地图。"""

    def __init__(
        self,
        output_dir: str = '',
        enabled: bool = True,
        record_trajectory_hz: float = 10.0,
        record_cmd_vel_hz: float = 10.0,
    ):
        """初始化记录器。

        Args:
            output_dir: 输出根目录，空字符串则自动在 reports/nav/ 下创建。
            enabled: 是否启用记录。
            record_trajectory_hz: 轨迹记录降采样频率。
            record_cmd_vel_hz: 速度指令记录降采样频率。
        """
        self._enabled = enabled
        self._trajectory_interval = 1.0 / max(0.5, record_trajectory_hz)
        self._cmd_vel_interval = 1.0 / max(0.5, record_cmd_vel_hz)

        if not self._enabled:
            self._dir = ''
            return

        if not output_dir:
            ts = time.strftime('%Y%m%d_%H%M%S', time.localtime())
            # 仓库根目录 → reports/nav/
            repo_root = os.environ.get(
                'HAZARDWALKER_ROOT',
                os.path.join(os.path.expanduser('~'), 'HazardWalker'),
            )
            output_dir = os.path.join(repo_root, 'reports', 'nav', f'run_{ts}')

        self._dir = _ensure_dir(output_dir)

        # 打开 JSONL 文件
        self._trajectory_fp = open(
            os.path.join(self._dir, 'trajectory.jsonl'), 'w', encoding='utf-8')
        self._cmd_vel_fp = open(
            os.path.join(self._dir, 'cmd_vel.jsonl'), 'w', encoding='utf-8')
        self._transitions_fp = open(
            os.path.join(self._dir, 'state_transitions.jsonl'), 'w', encoding='utf-8')
        self._failures_fp = open(
            os.path.join(self._dir, 'failures.jsonl'), 'w', encoding='utf-8')
        self._reobservations_fp = open(
            os.path.join(self._dir, 'reobservations.jsonl'), 'w', encoding='utf-8')

        # 降采样计时
        self._last_trajectory_time: float = 0.0
        self._last_cmd_vel_time: float = 0.0

        # 统计
        self._sequence: int = 0
        self._map_saved: bool = False

        self._write_summary_header()

    # ---- 公开记录方法 ----

    def record_pose(
        self,
        ros_sec: float,
        x: float,
        y: float,
        yaw: float,
        state: str,
        target_frontier: Optional[Tuple[float, float]] = None,
    ):
        """记录一帧合法 SLAM 位姿（内部降采样）。"""
        if not self._enabled:
            return
        now = time.monotonic()
        if now - self._last_trajectory_time < self._trajectory_interval:
            return
        self._last_trajectory_time = now

        self._sequence += 1
        record = {
            'seq': self._sequence,
            'time': _ros_time_str(ros_sec),
            'ros_sec': round(ros_sec, 4),
            'x': round(x, 4),
            'y': round(y, 4),
            'yaw_deg': round(math.degrees(yaw), 2),
            'state': state,
        }
        if target_frontier is not None:
            record['target_frontier'] = [
                round(target_frontier[0], 4),
                round(target_frontier[1], 4),
            ]
        self._write_jsonl(self._trajectory_fp, record)

    def record_cmd_vel(
        self,
        ros_sec: float,
        linear_x: float,
        angular_z: float,
        linear_y: float = 0.0,
    ):
        """记录一帧速度指令（内部降采样）。"""
        if not self._enabled:
            return
        now = time.monotonic()
        if now - self._last_cmd_vel_time < self._cmd_vel_interval:
            return
        self._last_cmd_vel_time = now

        record = {
            'seq': self._sequence,  # 复用轨迹序号以便对齐
            'time': _ros_time_str(ros_sec),
            'ros_sec': round(ros_sec, 4),
            'linear_x': round(linear_x, 4),
            'linear_y': round(linear_y, 4),
            'angular_z': round(angular_z, 4),
            'moving': abs(linear_x) > 0.02 or abs(angular_z) > 0.05,
        }
        self._write_jsonl(self._cmd_vel_fp, record)

    def record_state_transition(
        self,
        ros_sec: float,
        prev_state: str,
        new_state: str,
        reason: str = '',
    ):
        """记录状态变迁事件。"""
        if not self._enabled:
            return
        record = {
            'time': _ros_time_str(ros_sec),
            'ros_sec': round(ros_sec, 4),
            'transition': f'{prev_state} → {new_state}',
            'prev_state': prev_state,
            'new_state': new_state,
        }
        if reason:
            record['reason'] = reason
        self._write_jsonl(self._transitions_fp, record)

    def record_failure(
        self,
        ros_sec: float,
        failure_type: str,
        x: float = 0.0,
        y: float = 0.0,
        detail: str = '',
    ):
        """记录一次故障/异常事件。"""
        if not self._enabled:
            return
        record = {
            'time': _ros_time_str(ros_sec),
            'ros_sec': round(ros_sec, 4),
            'type': failure_type,
            'pose': [round(x, 4), round(y, 4)],
        }
        if detail:
            record['detail'] = detail
        self._write_jsonl(self._failures_fp, record)

    def record_reobservation(
        self,
        ros_sec: float,
        target_id: str,
        action: str,
        phase: str,  # 'started' | 'completed' | 'aborted'
        reason: str = '',
        bearing_change_deg: Optional[float] = None,
    ):
        """记录感知复查交互。"""
        if not self._enabled:
            return
        record = {
            'time': _ros_time_str(ros_sec),
            'ros_sec': round(ros_sec, 4),
            'target_id': target_id,
            'action': action,
            'phase': phase,
        }
        if reason:
            record['reason'] = reason
        if bearing_change_deg is not None:
            record['bearing_change_deg'] = round(bearing_change_deg, 2)
        self._write_jsonl(self._reobservations_fp, record)

    def save_map(
        self,
        grid: np.ndarray,
        map_msg,
        ros_sec: float,
        resolution: float = 0.05,
    ):
        """保存 SLAM 地图为 PGM + YAML。"""
        if not self._enabled or self._map_saved:
            return

        try:
            height, width = grid.shape
            # 转换为 PGM 格式：0=占用(黑), 254=自由(白), 205=未知(灰)
            pgm = np.full((height, width), 205, dtype=np.uint8)
            pgm[(grid >= 0) & (grid <= 49)] = 254   # 自由空间
            pgm[grid >= 65] = 0                       # 占用

            # 保存 PGM
            pgm_path = os.path.join(self._dir, 'map.pgm')
            with open(pgm_path, 'wb') as fp:
                fp.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
                fp.write(pgm.tobytes())

            # 保存 YAML
            origin_x = float(map_msg.info.origin.position.x) if map_msg is not None else 0.0
            origin_y = float(map_msg.info.origin.position.y) if map_msg is not None else 0.0
            yaml_content = (
                f'image: map.pgm\n'
                f'mode: trinary\n'
                f'resolution: {resolution:.4f}\n'
                f'origin: [{origin_x:.4f}, {origin_y:.4f}, 0.0000]\n'
                f'negate: 0\n'
                f'occupied_thresh: 0.65\n'
                f'free_thresh: 0.196\n'
            )
            yaml_path = os.path.join(self._dir, 'map.yaml')
            with open(yaml_path, 'w', encoding='utf-8') as fp:
                fp.write(yaml_content)

            self._map_saved = True
            self._write_summary_field('map_saved_ros_sec', round(ros_sec, 4))
        except Exception as exc:
            # 地图保存失败不能中断导航
            self._write_summary_field('map_save_error', str(exc))

    def close(
        self,
        ros_sec: float = 0.0,
        final_state: str = '',
        total_hazards_confirmed: int = 0,
        total_frontiers_visited: int = 0,
    ):
        """关闭所有文件，写入汇总元数据。"""
        if not self._enabled:
            return
        self._write_summary_field('end_ros_sec', round(ros_sec, 4))
        self._write_summary_field('final_state', final_state)
        self._write_summary_field('total_records', self._sequence)
        if total_hazards_confirmed:
            self._write_summary_field('total_hazards_confirmed', total_hazards_confirmed)
        if total_frontiers_visited:
            self._write_summary_field('total_frontiers_visited', total_frontiers_visited)

        for fp in (
            self._trajectory_fp,
            self._cmd_vel_fp,
            self._transitions_fp,
            self._failures_fp,
            self._reobservations_fp,
        ):
            try:
                fp.close()
            except Exception:
                pass

    # ---- 内部方法 ----

    def _write_jsonl(self, fp, record: dict):
        """写入一行 JSON 并立即 flush。"""
        try:
            fp.write(json.dumps(record, ensure_ascii=False) + '\n')
            fp.flush()
        except Exception:
            pass

    def _write_summary_header(self):
        """写入 run_meta.json 初始字段。"""
        self._summary_path = os.path.join(self._dir, 'run_meta.json')
        meta = {
            'start_time': _ros_time_str(),
            'start_unix': time.time(),
            'output_dir': self._dir,
        }
        try:
            with open(self._summary_path, 'w', encoding='utf-8') as fp:
                json.dump(meta, fp, ensure_ascii=False, indent=2)
        except Exception:
            pass

    def _write_summary_field(self, key: str, value):
        """更新 run_meta.json 的一个字段。"""
        try:
            meta = {}
            if os.path.exists(self._summary_path):
                with open(self._summary_path, 'r', encoding='utf-8') as fp:
                    meta = json.load(fp)
            meta[key] = value
            with open(self._summary_path, 'w', encoding='utf-8') as fp:
                json.dump(meta, fp, ensure_ascii=False, indent=2)
        except Exception:
            pass
