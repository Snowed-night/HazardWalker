"""赛事公开里程计起点归一化回归。"""

import math
import os
import sys
from pathlib import Path


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(
    0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_perception'))

from hazardwalker_perception.official_odometry import (  # noqa: E402
    PlanarOdomPose,
    relative_planar_pose,
)


def test_first_official_pose_becomes_zero_without_changing_metric_scale():
    anchor = PlanarOdomPose(0.0, -2.2, math.pi / 2.0)

    relative = relative_planar_pose(
        anchor, PlanarOdomPose(0.0, 22.8, math.pi / 2.0))

    assert abs(relative.x - 25.0) < 1e-9
    assert abs(relative.y) < 1e-9
    assert abs(relative.yaw) < 1e-9


def test_relative_pose_handles_turn_without_building_coordinates():
    relative = relative_planar_pose(
        (10.0, 20.0, math.pi / 2.0),
        (8.0, 23.0, math.pi),
    )

    assert abs(relative.x - 3.0) < 1e-9
    assert abs(relative.y - 2.0) < 1e-9
    assert abs(relative.yaw - math.pi / 2.0) < 1e-9


def test_nonfinite_official_pose_is_rejected():
    try:
        relative_planar_pose((0.0, 0.0, 0.0), (float('nan'), 0.0, 0.0))
    except ValueError:
        return
    raise AssertionError('非有限里程计不得进入正式坐标链')


def test_official_localizer_never_reads_referee_truth_or_layout():
    source = (
        Path(REPO_ROOT) / 'ros2_ws' / 'src' / 'hazardwalker_perception' /
        'hazardwalker_perception' / 'official_odometry_localizer_node.py'
    ).read_text(encoding='utf-8')
    assert "declare_parameter('input_topic', '/hw/odom')" in source
    assert "TransformBroadcaster(self)" in source
    assert 'relative_planar_pose(self.anchor, current)' in source
    assert 'danger_truth.json' not in source
    assert 'layout_metadata.json' not in source
