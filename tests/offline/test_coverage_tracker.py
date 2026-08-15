"""覆盖网格追踪 + 楼梯/电梯候选检测的离线回归测试。

所属组：导航组。
文件作用：验证 CoverageGrid 的访问标记、覆盖率计算，以及楼梯/电梯候选检测的
纯 numpy 逻辑，不依赖 ROS2。

对应实现：ros2_ws/src/hazardwalker_nav/hazardwalker_nav/coverage_tracker.py
"""

from pathlib import Path
import sys

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
NAV_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav'
if str(NAV_SRC) not in sys.path:
    sys.path.insert(0, str(NAV_SRC))

from hazardwalker_nav.coverage_tracker import (  # noqa: E402
    CoverageGrid,
    detect_stair_candidates,
    find_elevator_approach_candidates,
)


def test_coverage_update_marks_free_cells_in_radius():
    grid = np.zeros((9, 9), dtype=np.int8)
    coverage = CoverageGrid(9, 9)
    coverage.update(4, 4, grid, radius_cells=2)

    assert coverage.grid[4, 4]
    assert coverage.grid[4, 3]
    assert coverage.grid[3, 4]
    assert coverage.grid[4, 2]
    assert not coverage.grid[0, 0]
    assert not coverage.grid[8, 8]


def test_coverage_update_skips_occupied_and_unknown():
    grid = np.zeros((9, 9), dtype=np.int8)
    grid[4, 5] = 100  # 占用
    grid[5, 4] = -1   # 未知
    coverage = CoverageGrid(9, 9)
    coverage.update(4, 4, grid, radius_cells=2)

    assert coverage.grid[4, 4]
    assert not coverage.grid[4, 5]
    assert not coverage.grid[5, 4]


def test_coverage_update_clips_at_boundary():
    grid = np.zeros((3, 3), dtype=np.int8)
    coverage = CoverageGrid(3, 3)
    coverage.update(0, 0, grid, radius_cells=10)

    assert coverage.grid[0, 0]
    assert coverage.grid[2, 2]


def test_coverage_accumulates_across_updates():
    grid = np.zeros((5, 5), dtype=np.int8)
    coverage = CoverageGrid(5, 5)
    coverage.update(0, 0, grid, radius_cells=0)
    coverage.update(4, 4, grid, radius_cells=0)

    assert coverage.grid[0, 0]
    assert coverage.grid[4, 4]
    assert not coverage.grid[2, 2]


def test_floor_coverage_ratio_full_coverage():
    grid = np.zeros((5, 5), dtype=np.int8)
    coverage = CoverageGrid(5, 5)
    coverage.update(2, 2, grid, radius_cells=10)

    assert coverage.floor_coverage_ratio(grid) == 1.0


def test_floor_coverage_ratio_partial():
    grid = np.zeros((1, 4), dtype=np.int8)
    coverage = CoverageGrid(1, 4)
    coverage.update(0, 0, grid, radius_cells=0)  # 标记 (gy=0, gx=0)
    coverage.update(1, 0, grid, radius_cells=0)  # 标记 (gy=0, gx=1)

    assert coverage.floor_coverage_ratio(grid) == 0.5


def test_floor_coverage_ratio_zero_when_no_free():
    grid = np.full((4, 4), -1, dtype=np.int8)
    grid[0, 0] = 100
    coverage = CoverageGrid(4, 4)

    assert coverage.floor_coverage_ratio(grid) == 0.0


def test_floor_coverage_ratio_ignores_occupied_when_counting_free():
    # 占用格不算自由空间，因此不拉低覆盖率分母。
    grid = np.full((1, 3), 100, dtype=np.int8)
    grid[0, 0] = 0
    grid[0, 1] = 0
    coverage = CoverageGrid(1, 3)
    coverage.update(0, 0, grid, radius_cells=0)  # 标记 (gy=0, gx=0)
    coverage.update(1, 0, grid, radius_cells=0)  # 标记 (gy=0, gx=1)

    assert coverage.floor_coverage_ratio(grid) == 1.0


def test_detect_stair_candidates_empty_grid():
    grid = np.full((10, 10), -1, dtype=np.int8)

    assert detect_stair_candidates(grid, 0.5) == []


def test_detect_stair_candidates_narrow_horizontal():
    grid = np.full((24, 24), -1, dtype=np.int8)
    grid[8:14, 4:18] = 0  # 高 6 宽 14 的水平窄廊

    candidates = detect_stair_candidates(grid, 0.5)

    assert candidates
    assert candidates[0].orientation_deg == 90.0


def test_detect_stair_candidates_narrow_vertical():
    grid = np.full((24, 24), -1, dtype=np.int8)
    grid[4:18, 8:14] = 0  # 高 14 宽 6 的垂直窄廊

    candidates = detect_stair_candidates(grid, 0.5)

    assert candidates
    assert candidates[0].orientation_deg == 0.0


def test_find_elevator_approach_candidates_empty():
    grid = np.zeros((20, 20), dtype=np.int8)
    visited = np.ones((20, 20), dtype=bool)

    assert find_elevator_approach_candidates(grid, visited) == []


def test_find_elevator_approach_candidates_returns_unvisited_free_patch():
    grid = np.zeros((20, 20), dtype=np.int8)
    visited = np.zeros((20, 20), dtype=bool)

    result = find_elevator_approach_candidates(grid, visited)

    assert len(result) == 1
    assert abs(result[0][0] - 0.475) < 1e-9
    assert abs(result[0][1] - 0.475) < 1e-9
