"""深度 ICP 里程计坐标合同测试。"""

import math
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'))

from hazardwalker_perception.depth_icp_odometry import (  # noqa: E402
    base_delta_to_optical_registration,
    integrate_planar_pose,
    optical_registration_to_base_delta,
)


def test_optical_registration_round_trips_planar_base_motion():
    expected = (0.12, -0.03, math.radians(4.0))
    registration = base_delta_to_optical_registration(*expected)
    actual = optical_registration_to_base_delta(registration)
    assert np.allclose(actual, expected, atol=1e-9)


def test_integrate_planar_pose_rotates_body_translation_into_world():
    pose = integrate_planar_pose(
        (1.0, 2.0, math.pi / 2.0),
        (0.5, 0.0, math.pi / 2.0),
    )
    assert np.allclose(pose, (1.0, 2.5, math.pi), atol=1e-9)
