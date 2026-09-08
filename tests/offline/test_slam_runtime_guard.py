"""SLAM 实时连续性熔断的离线测试。"""

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
NAV_SRC = ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav'
sys.path.insert(0, str(NAV_SRC))

from hazardwalker_nav.slam_metrics import (  # noqa: E402
    fatal_map_jump_without_odom_motion,
)


def test_false_loop_closure_is_fatal_during_exploration():
    assert fatal_map_jump_without_odom_motion(
        2.58, 0.001, 0.1, 'EXPLORING', 60.0)


def test_real_robot_motion_is_not_a_map_only_jump():
    assert not fatal_map_jump_without_odom_motion(
        1.2, 1.1, 0.1, 'EXPLORING', 60.0)


def test_floor_session_start_has_a_jump_grace_period():
    assert not fatal_map_jump_without_odom_motion(
        5.1, 0.0, 0.1, 'EXPLORING', 2.0)


def test_floor_transition_never_trips_the_active_map_guard():
    assert not fatal_map_jump_without_odom_motion(
        22.8, 0.0, 0.1, 'FLOOR_TRANSITION', 60.0)


def test_runner_subscribes_to_slam_health_and_stops_early():
    source = (
        ROOT / 'scripts' / 'run_official_slam_exploration.py'
    ).read_text(encoding='utf-8')
    assert "'/hazardwalker/slam/health'" in source
    assert 'if observer.slam_failure:' in source
    assert 'SLAM 实时连续性门禁失败' in source
