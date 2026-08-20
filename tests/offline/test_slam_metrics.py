"""SLAM 监测纯函数的离线回归测试。

验证 detect_pose_jump / drift_magnitude / yaw_from_quaternion /
map_occupancy_stats 的边界行为，不依赖 ROS2。
"""

import math
import sys
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
NAV_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav'
if str(NAV_SRC) not in sys.path:
    sys.path.insert(0, str(NAV_SRC))

from hazardwalker_nav.slam_metrics import (  # noqa: E402
    detect_pose_jump,
    drift_magnitude,
    map_occupancy_stats,
    yaw_from_quaternion,
)


def _assert_close(actual, expected, tol=1e-9):
    assert abs(actual - expected) < tol, f'{actual} != {expected}'


def test_pose_jump_rejects_instantaneous_teleport_but_keeps_normal_motion():
    # 正常底盘 0.35 m/s，一帧 0.1s 位移约 0.035m，远小于 1.0*0.1+0.5。
    assert not detect_pose_jump(0.035, 0.1, 1.0, 0.5)
    # 实测 Cartographer 相似走廊多解瞬移 7.5m 与 31m。
    assert detect_pose_jump(7.5, 0.1, 1.0, 0.5)
    assert detect_pose_jump(31.0, 0.1, 1.0, 0.5)


def test_pose_jump_allows_recovery_after_tf_gap_without_false_positive():
    # TF 短暂丢失后恢复，1s 内正常位移 0.35m 不应误判为跳变。
    assert not detect_pose_jump(0.35, 1.0, 1.0, 0.5)
    # 但同样 1s 内瞬移 5m 仍是跳变。
    assert detect_pose_jump(5.0, 1.0, 1.0, 0.5)


def test_pose_jump_handles_invalid_inputs_and_negative_elapsed():
    assert not detect_pose_jump(float('nan'), 0.1)
    assert not detect_pose_jump(1.0, float('nan'))
    assert not detect_pose_jump('bad', 0.1)
    assert not detect_pose_jump(1.0, None)
    # 负 elapsed 按 0 处理：位移超过固定容差仍判跳变，低于容差则放行。
    assert detect_pose_jump(1.0, -0.5)
    assert not detect_pose_jump(0.3, -0.5)


def test_drift_magnitude_computes_euclidean_distance():
    _assert_close(drift_magnitude(3.0, 4.0), 5.0)
    _assert_close(drift_magnitude(0.0, 0.0), 0.0)
    _assert_close(drift_magnitude(-1.0, -1.0), math.sqrt(2.0))


def test_drift_magnitude_handles_invalid_inputs():
    assert drift_magnitude('bad', 1.0) == 0.0
    assert drift_magnitude(1.0, None) == 0.0


def test_yaw_from_quaternion_identity_and_quarter_turn():
    _assert_close(yaw_from_quaternion(1.0, 0.0, 0.0, 0.0), 0.0)
    # 绕 z 轴转 90°：w=cos(π/4), z=sin(π/4)。
    s = math.sqrt(2.0) / 2.0
    _assert_close(yaw_from_quaternion(s, 0.0, 0.0, s), math.pi / 2.0)


def test_map_occupancy_stats_counts_known_and_unknown():
    grid = np.array([
        [-1, 0, 100],
        [0, 0, 100],
    ], dtype=np.int8)
    stats = map_occupancy_stats(grid)
    assert stats['total_cells'] == 6
    assert stats['known_cells'] == 5
    # 自由 3、占用 2、未知 1。
    _assert_close(stats['occupied_ratio'], 2.0 / 5.0)
    _assert_close(stats['free_ratio'], 3.0 / 5.0)
    _assert_close(stats['unknown_ratio'], 1.0 / 6.0)


def test_map_occupancy_stats_ignores_ambiguous_middle_band():
    # 50~64 的模糊区不计入自由也不计入占用。
    grid = np.array([0, 50, 100], dtype=np.int8)
    stats = map_occupancy_stats(grid)
    assert stats['known_cells'] == 2  # 0（自由） + 100（占用）
    _assert_close(stats['occupied_ratio'], 0.5)
    _assert_close(stats['free_ratio'], 0.5)


def test_map_occupancy_stats_empty_and_all_unknown_defenses():
    empty = map_occupancy_stats(np.array([], dtype=np.int8))
    assert empty['total_cells'] == 0
    assert empty['occupied_ratio'] == 0.0
    assert empty['free_ratio'] == 0.0
    assert empty['unknown_ratio'] == 0.0

    all_unknown = map_occupancy_stats(np.full((3, 3), -1, dtype=np.int8))
    assert all_unknown['known_cells'] == 0
    assert all_unknown['occupied_ratio'] == 0.0
    assert all_unknown['free_ratio'] == 0.0
    _assert_close(all_unknown['unknown_ratio'], 1.0)
