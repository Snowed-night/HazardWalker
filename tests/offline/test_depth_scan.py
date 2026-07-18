"""官方 RGB-D 深度转前向 LaserScan 的离线回归。"""

from pathlib import Path
import math
import sys

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
PERCEPTION_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
if str(PERCEPTION_SRC) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SRC))

from hazardwalker_perception.depth_scan import (  # noqa: E402
    depth_image_to_planar_scan,
)


def test_constant_depth_becomes_monotonic_forward_scan():
    depth = np.full((6, 5), 4.0, dtype=np.float32)

    angle_min, angle_max, increment, ranges = depth_image_to_planar_scan(
        depth, fx=4.0, cx=2.0,
        row_min_fraction=0.0, row_max_fraction=1.0,
        column_stride=1,
    )

    assert angle_min < 0.0 < angle_max
    assert increment > 0.0
    assert len(ranges) == 5
    assert math.isclose(ranges[2], 4.0, rel_tol=1e-6)
    assert ranges[0] > ranges[2] and ranges[-1] > ranges[2]


def test_low_percentile_keeps_near_obstacle_and_invalid_column_is_inf():
    depth = np.full((10, 4), 6.0, dtype=np.float32)
    depth[3:7, 2] = 2.0
    depth[:, 3] = np.nan

    _, _, _, ranges = depth_image_to_planar_scan(
        depth, fx=6.0, cx=1.5,
        row_min_fraction=0.2, row_max_fraction=0.8,
        column_stride=1, depth_percentile=10.0,
    )

    # 输出列顺序为 3,2,1,0。
    assert math.isinf(ranges[0])
    assert ranges[1] < 3.0


def test_depth_scan_rejects_invalid_intrinsics_and_shape():
    for depth, fx in (
            (np.zeros((1, 2), dtype=np.float32), 2.0),
            (np.zeros((2, 2), dtype=np.float32), 0.0)):
        try:
            depth_image_to_planar_scan(depth, fx=fx, cx=1.0)
        except ValueError:
            continue
        raise AssertionError('损坏深度图或内参必须拒绝')
