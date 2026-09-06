"""每层 SLAM map 公共锚点纯函数测试。"""

import math
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
PERCEPTION_SRC = ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
sys.path.insert(0, str(PERCEPTION_SRC))

from hazardwalker_perception.floor_frame_alignment import (  # noqa: E402
    world_from_map_at_robot_anchor,
)


def test_anchor_transform_maps_robot_pose_to_public_world_pose():
    transform = world_from_map_at_robot_anchor(
        map_x=5.0,
        map_y=2.0,
        map_yaw=0.2,
        world_x=0.8,
        world_y=2.6,
        world_yaw=math.pi / 2.0,
    )
    tx, ty, yaw = transform
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    assert tx + cosine * 5.0 - sine * 2.0 == pytest.approx(0.8)
    assert ty + sine * 5.0 + cosine * 2.0 == pytest.approx(2.6)
    assert yaw + 0.2 == pytest.approx(math.pi / 2.0)


def test_anchor_rejects_nonfinite_input():
    with pytest.raises(ValueError):
        world_from_map_at_robot_anchor(0.0, 0.0, 0.0, math.nan, 0.0, 0.0)
