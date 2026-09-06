"""SLAM 与物理轨迹赛后对齐门禁测试。"""

import math
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
NAV_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav'
sys.path.insert(0, str(NAV_SRC))

from hazardwalker_nav.slam_alignment import (  # noqa: E402
    alignment_is_acceptable,
    evaluate_map_physical_alignment,
)


def _sample(index, drift_x=0.0, drift_y=0.0):
    map_x = index * 0.2
    map_y = math.sin(index * 0.1)
    yaw_deg = 10.0
    rotation = math.radians(80.0)
    physical_x = 2.0 + math.cos(rotation) * map_x - math.sin(rotation) * map_y
    physical_y = -1.0 + math.sin(rotation) * map_x + math.cos(rotation) * map_y
    return {
        'ros_sec': float(index),
        'x': map_x + drift_x,
        'y': map_y + drift_y,
        'yaw_deg': yaw_deg,
        'official_x': physical_x,
        'official_y': physical_y,
        'official_yaw_deg': 90.0,
    }


def test_rigidly_equivalent_trajectory_passes_alignment_gate():
    metrics = evaluate_map_physical_alignment(
        [_sample(index) for index in range(100)])
    assert metrics['p95_error_m'] < 1e-9
    assert metrics['max_error_m'] < 1e-9
    assert alignment_is_acceptable(metrics)


def test_late_slam_drift_fails_alignment_gate():
    samples = []
    for index in range(100):
        drift = 0.0 if index < 20 else (index - 20) * 0.08
        samples.append(_sample(index, drift_x=drift, drift_y=-0.5 * drift))
    metrics = evaluate_map_physical_alignment(samples)
    assert metrics['p95_error_m'] > 2.0
    assert not alignment_is_acceptable(metrics)


def test_global_alignment_cannot_hide_metric_scale_error():
    samples = []
    for index in range(100):
        sample = _sample(index)
        sample['x'] *= 1.25
        sample['y'] *= 1.25
        samples.append(sample)
    metrics = evaluate_map_physical_alignment(samples)
    assert metrics['p95_error_m'] > 1.0
    assert not alignment_is_acceptable(metrics)


def test_alignment_uses_position_motion_instead_of_noisy_startup_yaw():
    samples = []
    for index in range(100):
        sample = _sample(index)
        sample['yaw_deg'] = 10.0 + (12.0 if index % 2 else -9.0)
        sample['official_yaw_deg'] = 90.0 + (7.0 if index % 3 else -11.0)
        samples.append(sample)
    metrics = evaluate_map_physical_alignment(samples)
    assert abs(metrics['rotation_deg'] - 80.0) < 1e-9
    assert metrics['max_error_m'] < 1e-9
    assert alignment_is_acceptable(metrics)
