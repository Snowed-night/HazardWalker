"""Frontier A* 安全约束的离线回归测试。

负责人：姜晨。验证未知区不可穿越、障碍膨胀和对角防穿角，不依赖 ROS2。
"""

from types import SimpleNamespace
import sys
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
NAV_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav'
if str(NAV_SRC) not in sys.path:
    sys.path.insert(0, str(NAV_SRC))

from hazardwalker_nav.frontier_detector import (  # noqa: E402
    a_star_path,
    cluster_frontiers,
    find_frontiers,
    Frontier,
    select_best_frontier,
    world_to_grid,
)


def _grid_message(grid, resolution=1.0):
    origin = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0),
    )
    info = SimpleNamespace(
        width=grid.shape[1],
        height=grid.shape[0],
        resolution=resolution,
        origin=origin,
    )
    return SimpleNamespace(info=info, data=grid.reshape(-1).tolist())


def _path_cells(path, message):
    return [world_to_grid(x, y, message) for x, y in path]


def test_a_star_never_uses_unknown_cells_as_a_shortcut():
    grid = np.zeros((5, 5), dtype=np.int8)
    grid[2, 1:4] = -1
    message = _grid_message(grid)

    path = a_star_path(
        grid, message, 0.5, 2.5, 4.5, 2.5,
        inflation_radius_m=0.0,
    )

    assert path
    assert all(grid[gy, gx] == 0 for gx, gy in _path_cells(path, message))


def test_cartographer_probabilistic_free_cells_form_frontiers_and_paths():
    """实时 Cartographer 自由边缘可到 49，不能沿用静态地图 25 阈值。"""

    grid = np.full((7, 7), -1, dtype=np.int8)
    grid[2:5, 1:6] = 49
    grid[3, 1:5] = 0
    message = _grid_message(grid)

    mask = find_frontiers(grid)
    path = a_star_path(
        grid, message, 1.5, 3.5, 4.5, 3.5,
        inflation_radius_m=0.0,
    )

    assert mask.any()
    assert path
    assert all(0 <= grid[gy, gx] <= 49
               for gx, gy in _path_cells(path, message))


def test_a_star_rejects_diagonal_corner_cutting():
    grid = np.zeros((3, 3), dtype=np.int8)
    grid[0, 1] = 100
    grid[1, 0] = 100
    message = _grid_message(grid)

    path = a_star_path(
        grid, message, 0.5, 0.5, 1.5, 1.5,
        inflation_radius_m=0.0,
    )

    assert path == []


def test_a_star_inflates_obstacles_by_robot_safety_radius():
    grid = np.zeros((7, 7), dtype=np.int8)
    grid[3, 3] = 100
    message = _grid_message(grid)

    path = a_star_path(
        grid, message, 0.5, 3.5, 6.5, 3.5,
        inflation_radius_m=1.0,
    )
    cells = _path_cells(path, message)

    assert path
    assert all((gx - 3) ** 2 + (gy - 3) ** 2 > 1 for gx, gy in cells)


def test_a_star_expansion_budget_fails_closed_on_large_unreachable_region():
    grid = np.zeros((80, 80), dtype=np.int8)
    grid[:, 40] = 100
    message = _grid_message(grid)

    path = a_star_path(
        grid, message, 1.5, 1.5, 70.5, 70.5,
        inflation_radius_m=0.0,
        max_expansions=100,
    )

    assert path == []


def test_a_star_snaps_a_locally_occupied_robot_cell_to_nearest_safe_cell():
    grid = np.zeros((11, 11), dtype=np.int8)
    grid[5, 5] = 100
    message = _grid_message(grid, resolution=0.1)

    path = a_star_path(
        grid, message, 0.55, 0.55, 0.95, 0.55,
        inflation_radius_m=0.0,
        endpoint_search_radius_m=0.2,
    )

    assert path
    assert _path_cells(path, message)[0] != (5, 5)


def test_concave_frontier_uses_an_actual_free_frontier_cell_as_goal():
    grid = np.zeros((5, 5), dtype=np.int8)
    mask = np.zeros_like(grid, dtype=bool)
    mask[1, 1:4] = True
    mask[2, 1] = True
    mask[2, 3] = True
    mask[3, 1:4] = True
    message = _grid_message(grid)

    frontiers = cluster_frontiers(mask, grid, message)

    assert len(frontiers) == 1
    goal_cell = world_to_grid(*frontiers[0].centroid, message)
    assert goal_cell in frontiers[0].points
    assert grid[goal_cell[1], goal_cell[0]] == 0


def test_frontier_selection_prefers_current_forward_half_plane():
    """入口外不能被身后巨大开放区吸走，应优先进入当前朝向的建筑前沿。"""

    behind = Frontier(
        centroid=(-8.0, 3.0),
        size=3000,
        points=[(0, 0)],
        info_gain=3000.0,
    )
    ahead = Frontier(
        centroid=(4.0, 0.5),
        size=80,
        points=[(0, 0)],
        info_gain=80.0,
    )

    selected = select_best_frontier(
        [behind, ahead],
        0.0,
        0.0,
        min_frontier_size=10,
        robot_yaw=0.0,
    )

    assert selected is ahead


def test_frontier_selection_restores_all_directions_after_entry():
    """首次入口保护结束后，近处侧后方房间应重新参与正常评分。"""

    corridor_ahead = Frontier(
        centroid=(20.0, 0.0),
        size=40,
        points=[(0, 0)],
        info_gain=40.0,
    )
    nearby_room = Frontier(
        centroid=(-2.0, 1.0),
        size=200,
        points=[(0, 0)],
        info_gain=200.0,
    )

    selected = select_best_frontier(
        [corridor_ahead, nearby_room],
        0.0,
        0.0,
        min_frontier_size=10,
        robot_yaw=None,
    )

    assert selected is nearby_room


def test_entry_axis_excludes_outside_but_keeps_side_rooms():
    """入楼轴只屏蔽起点背面，入口前方的侧向房间仍可正常竞争。"""

    outside = Frontier(
        centroid=(0.0, -16.0),
        size=3000,
        points=[(0, 0)],
        info_gain=3000.0,
    )
    side_room = Frontier(
        centroid=(5.0, 2.0),
        size=120,
        points=[(0, 0)],
        info_gain=120.0,
    )

    selected = select_best_frontier(
        [outside, side_room],
        0.0,
        5.0,
        min_frontier_size=10,
        entry_origin=(0.0, 0.0),
        entry_axis=(0.0, 1.0),
    )

    assert selected is side_room


def test_entry_axis_finishes_when_only_outside_frontiers_remain():
    outside = Frontier(
        centroid=(0.0, -4.0),
        size=100,
        points=[(0, 0)],
        info_gain=100.0,
    )

    selected = select_best_frontier(
        [outside],
        0.0,
        2.0,
        entry_origin=(0.0, 0.0),
        entry_axis=(0.0, 1.0),
    )

    assert selected is None


def test_frontier_node_fails_closed_without_pose_scan_or_safe_return_path():
    source = (
        NAV_SRC / 'hazardwalker_nav' / 'frontier_explorer_node.py'
    ).read_text(encoding='utf-8')

    assert "LaserScan, '/hw/scan'" in source
    assert 'if not self._has_fresh_pose():' in source
    assert 'action_has_scan_clearance(' in source
    assert 'No safe path home found' in source
    assert 'attempting blind return' not in source
    assert "declare_parameter('unreachable_frontier_ttl_s'" in source
    assert "declare_parameter('frontier_completion_grace_s', 30.0)" in source
    assert "declare_parameter('frontier_recovery_turn_speed', 0.60)" in source
    assert "declare_parameter('minimum_linear_speed', 0.30)" in source
    assert "declare_parameter('minimum_turn_speed', 0.45)" in source
    assert 'math.copysign(' in source
    assert 'except ExternalShutdownException:' in source
    assert 'if rclpy.ok():' in source
    assert "declare_parameter('goal_tolerance_m', 0.25)" in source
    assert "stamp != (0, 0) and stamp != self._last_pose_stamp" in source
    assert "stamp != (0, 0) and stamp != self._last_scan_stamp" in source
    assert "if self._scan_allows_action(" in source
    assert "'turn_left'," in source
    assert 'Clock(clock_type=ClockType.STEADY_TIME)' in source
    assert 'clock=self._control_clock' in source
    assert 'if not new_frontiers and unvisited_frontiers:' in source
    assert 'trying %d largest ' in source
    assert 'unvisited fragments safely.' in source
    assert 'self._initial_heading_yaw = self.robot_yaw' in source
    assert 'self._initial_heading_yaw' in source
    assert 'if self._entry_axis is None' in source
    assert 'self._entry_axis = (' in source
    compile(source, 'frontier_explorer_node.py', 'exec')
