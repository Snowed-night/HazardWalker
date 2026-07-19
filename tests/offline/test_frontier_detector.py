"""Frontier A* 安全约束的离线回归测试。

负责人：姜晨。验证未知区不可穿越、障碍膨胀和对角防穿角，不依赖 ROS2。
"""

from types import SimpleNamespace
import math
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
    compute_frontier_backoff_ttl_s,
    compute_exploration_time_limit_s,
    entry_axis_progress_m,
    entry_ingress_constraint_active,
    entry_ingress_half_angles_deg,
    find_frontiers,
    Frontier,
    nearest_frontier_basin_key,
    return_pose_has_progress,
    select_best_frontier,
    should_switch_frontier,
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


def test_default_a_star_clearance_matches_navigation_safety_gate():
    source = (
        NAV_SRC / 'hazardwalker_nav' / 'frontier_detector.py'
    ).read_text(encoding='utf-8')

    assert 'inflation_radius_m: float = 0.45' in source


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


def test_a_star_uses_independent_start_and_goal_snap_radii():
    grid = np.zeros((11, 11), dtype=np.int8)
    grid[5, 1] = 100
    grid[5, 9] = 100
    message = _grid_message(grid, resolution=0.1)

    # 机器人当前格允许越过 0.1 m 的近场污染，但返航目标不能吸附到
    # 0.25 m 到家容差之外。
    strict_goal_path = a_star_path(
        grid, message, 0.15, 0.55, 0.95, 0.55,
        inflation_radius_m=0.0,
        start_search_radius_m=0.2,
        goal_search_radius_m=0.05,
    )
    compatible_common_radius_path = a_star_path(
        grid, message, 0.15, 0.55, 0.95, 0.55,
        inflation_radius_m=0.0,
        endpoint_search_radius_m=0.2,
    )

    assert strict_goal_path == []
    assert compatible_common_radius_path


def test_return_path_appends_exact_home_without_goal_cell_snapping():
    grid = np.zeros((11, 11), dtype=np.int8)
    message = _grid_message(grid, resolution=0.1)
    exact_home = (0.93, 0.57)

    path = a_star_path(
        grid, message, 0.15, 0.55, *exact_home,
        inflation_radius_m=0.0,
        start_search_radius_m=0.2,
        goal_search_radius_m=0.0,
        append_exact_goal=True,
    )

    assert path
    assert path[-1] == exact_home


def test_return_path_rejects_exact_home_outside_current_map():
    grid = np.zeros((11, 11), dtype=np.int8)
    message = _grid_message(grid, resolution=0.1)

    path = a_star_path(
        grid, message, 0.15, 0.55, 2.0, 0.55,
        inflation_radius_m=0.0,
        start_search_radius_m=0.2,
        goal_search_radius_m=0.0,
        append_exact_goal=True,
    )

    assert path == []


def test_unreachable_frontier_basin_merges_centroid_jitter_and_backs_off():
    basin_keys = [(3.86, -3.42), (8.0, 1.0)]

    assert nearest_frontier_basin_key(
        basin_keys, 3.88, -3.47, radius_m=0.45,
    ) == (3.86, -3.42)
    assert nearest_frontier_basin_key(
        basin_keys, 4.40, -3.47, radius_m=0.45,
    ) is None
    assert compute_frontier_backoff_ttl_s(45.0, 180.0, 1) == 45.0
    assert compute_frontier_backoff_ttl_s(45.0, 180.0, 2) == 90.0
    assert compute_frontier_backoff_ttl_s(45.0, 180.0, 4) == 180.0


def test_exploration_limit_protects_minimum_and_distance_based_return_time():
    near_home_limit = compute_exploration_time_limit_s(
        configured_timeout_s=540.0,
        mission_budget_s=600.0,
        distance_home_m=0.0,
        return_speed_mps=0.30,
    )
    far_from_home_limit = compute_exploration_time_limit_s(
        configured_timeout_s=540.0,
        mission_budget_s=600.0,
        distance_home_m=40.0,
        return_speed_mps=0.25,
    )
    diagnostic_limit = compute_exploration_time_limit_s(
        configured_timeout_s=300.0,
        mission_budget_s=600.0,
        distance_home_m=0.0,
        return_speed_mps=0.30,
    )

    assert near_home_limit == 480.0
    assert far_from_home_limit == 250.0
    assert far_from_home_limit < near_home_limit
    assert diagnostic_limit == 300.0


def test_return_watchdog_counts_real_motion_even_when_it_moves_away_from_home():
    # 家在 x=0，机器人从 x=1.0 绕障到 x=1.2，虽然距家增大仍是合法进展。
    assert return_pose_has_progress(
        previous_x=1.0,
        previous_y=0.0,
        current_x=1.2,
        current_y=0.0,
        minimum_distance_m=0.10,
    ) is True
    assert return_pose_has_progress(
        previous_x=1.0,
        previous_y=0.0,
        current_x=1.04,
        current_y=0.02,
        minimum_distance_m=0.10,
    ) is False


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


def test_frontier_selection_requires_narrow_official_entry_cone():
    diagonal_outside = Frontier(
        centroid=(3.0, 5.5),
        size=1000,
        points=[(0, 0)],
        info_gain=1000.0,
    )
    straight_entry = Frontier(
        centroid=(5.0, 1.0),
        size=60,
        points=[(0, 0)],
        info_gain=60.0,
    )

    selected = select_best_frontier(
        [diagonal_outside, straight_entry],
        0.0,
        0.0,
        robot_yaw=0.0,
        robot_yaw_half_angle_rad=math.radians(35.0),
        require_robot_yaw_candidate=True,
    )

    assert selected is straight_entry


def test_required_entry_cone_fails_closed_without_candidate():
    outside = Frontier(
        centroid=(0.0, 5.0),
        size=100,
        points=[(0, 0)],
        info_gain=100.0,
    )

    assert select_best_frontier(
        [outside],
        0.0,
        0.0,
        robot_yaw=0.0,
        robot_yaw_half_angle_rad=math.radians(35.0),
        require_robot_yaw_candidate=True,
    ) is None


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


def test_frontier_selection_keeps_near_small_room_over_far_huge_ray():
    near_room = Frontier(
        centroid=(2.0, 0.5), size=6, points=[], info_gain=8,
    )
    far_ray = Frontier(
        centroid=(18.0, 0.0), size=300, points=[], info_gain=900,
    )

    selected = select_best_frontier(
        [far_ray, near_room],
        robot_wx=0.0,
        robot_wy=0.0,
        min_frontier_size=10,
        locality_slack_m=3.0,
    )

    # 近场带内全是小簇时必须保留门口前沿，不能回退到远端巨大伪前沿。
    assert selected is near_room


def test_frontier_switch_uses_hold_time_and_distance_hysteresis():
    assert not should_switch_frontier(
        12.0, 2.0, held_duration_s=3.0,
        switch_margin_m=1.0, minimum_hold_s=8.0,
    )
    assert not should_switch_frontier(
        5.0, 4.2, held_duration_s=10.0,
        switch_margin_m=1.0, minimum_hold_s=8.0,
    )
    assert should_switch_frontier(
        12.0, 3.0, held_duration_s=10.0,
        switch_margin_m=1.0, minimum_hold_s=8.0,
    )


def test_frontier_switch_protects_a_target_with_recent_net_progress():
    assert not should_switch_frontier(
        12.0, 3.0, held_duration_s=20.0,
        switch_margin_m=1.0, minimum_hold_s=8.0,
        recent_progress_age_s=2.0, progress_protection_s=12.0,
    )
    assert should_switch_frontier(
        12.0, 3.0, held_duration_s=20.0,
        switch_margin_m=1.0, minimum_hold_s=8.0,
        recent_progress_age_s=13.0, progress_protection_s=12.0,
    )
    assert should_switch_frontier(
        12.0, 3.0, held_duration_s=50.0,
        switch_margin_m=1.0, minimum_hold_s=8.0,
        recent_progress_age_s=2.0, progress_protection_s=12.0,
        progress_protection_max_hold_s=45.0,
    )


def test_entry_axis_progress_uses_only_public_start_axis_geometry():
    assert entry_axis_progress_m(
        6.0, 2.0, entry_origin=(1.0, 2.0), entry_axis=(2.0, 0.0),
    ) == 5.0
    assert entry_axis_progress_m(
        1.0, 2.0, entry_origin=None, entry_axis=(1.0, 0.0),
    ) is None
    assert entry_axis_progress_m(
        1.0, 2.0, entry_origin=(0.0, 0.0), entry_axis=(0.0, 0.0),
    ) is None


def test_entry_ingress_constraint_stays_active_until_required_depth():
    assert entry_ingress_constraint_active(
        None, None, ingress_depth_m=6.0,
    )
    assert entry_ingress_constraint_active(
        (1.0, 0.0), 5.99, ingress_depth_m=6.0,
    )
    assert not entry_ingress_constraint_active(
        (1.0, 0.0), 6.0, ingress_depth_m=6.0,
    )
    assert not entry_ingress_constraint_active(
        (1.0, 0.0), None, ingress_depth_m=0.0,
    )


def test_entry_ingress_angles_relax_deterministically_without_duplicates():
    assert entry_ingress_half_angles_deg(
        35.0, 55.0, 90.0, constraint_active=True,
    ) == (35.0, 55.0, 90.0)
    assert entry_ingress_half_angles_deg(
        35.0, 35.0, 35.0, constraint_active=True,
    ) == (35.0,)
    assert entry_ingress_half_angles_deg(
        35.0, 55.0, 90.0, constraint_active=False,
    ) == (None,)


def test_public_building_width_band_excludes_far_side_exterior():
    far_side_exterior = Frontier(
        centroid=(2.0, 18.0),
        size=1000,
        points=[(0, 0)],
        info_gain=1000.0,
    )
    interior_corridor = Frontier(
        centroid=(18.0, -2.0),
        size=80,
        points=[(0, 0)],
        info_gain=80.0,
    )

    selected = select_best_frontier(
        [far_side_exterior, interior_corridor],
        1.0,
        0.0,
        entry_origin=(0.0, 0.0),
        entry_axis=(1.0, 0.0),
        entry_lateral_limit_m=12.0,
    )

    assert selected is interior_corridor


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
    assert "declare_parameter('frontier_completion_grace_s', 60.0)" in source
    assert "declare_parameter('frontier_recovery_turn_speed', 0.60)" in source
    assert "declare_parameter('minimum_linear_speed', 0.30)" in source
    assert "declare_parameter('minimum_turn_speed', 0.45)" in source
    assert 'math.copysign(' in source
    assert 'except ExternalShutdownException:' in source
    assert 'if rclpy.ok():' in source
    assert "declare_parameter('goal_tolerance_m', 0.25)" in source
    assert "declare_parameter('exploration_timeout_s', 480.0)" in source
    assert "declare_parameter('mission_time_budget_s', 600.0)" in source
    assert "declare_parameter('minimum_return_reserve_s', 120.0)" in source
    assert "declare_parameter('unreachable_frontier_radius_m', 0.45)" in source
    assert "declare_parameter('unreachable_frontier_ttl_s', 45.0)" in source
    assert (
        "declare_parameter('max_frontier_plan_failures_per_replan', 4)"
        in source
    )
    assert (
        'plan_failures_this_cycle < plan_failure_budget'
        in source
    )
    assert (
        'Frontier planning failure budget exhausted: '
        in source
    )
    assert 'nearest_frontier_basin_key(' in source
    assert 'for record in self._unreachable_frontiers.values()' in source
    assert 'record[0] > now_ros' in source
    assert 'compute_exploration_time_limit_s(' in source
    assert "stamp != (0, 0) and stamp != self._last_pose_stamp" in source
    assert "stamp != (0, 0) and stamp != self._last_scan_stamp" in source
    assert "if self._scan_allows_action(" in source
    assert "'turn_left'," in source
    assert 'Clock(clock_type=ClockType.STEADY_TIME)' in source
    assert 'clock=self._control_clock' in source
    assert "declare_parameter('frontier_locality_slack_m', 3.0)" in source
    assert (
        "declare_parameter('frontier_recent_progress_protection_s', 12.0)"
        in source
    )
    assert (
        "'frontier_progress_protection_max_hold_s', 45.0"
        in source
    )
    assert "declare_parameter('frontier_net_progress_timeout_s', 30.0)" in source
    assert "declare_parameter('safety_blocked_timeout_s', 8.0)" in source
    assert "declare_parameter('frontier_observation_sweep_speed', 0.60)" in source
    assert "declare_parameter('frontier_observation_sweep_timeout_s', 18.0)" in source
    assert 'Starting frontier RGB-D observation sweep:' in source
    assert 'self._handle_frontier_observation_sweep(now_ros)' in source
    assert 'self._frontier_observation_remaining_rad - delta' in source
    assert 'candidates = list(unvisited_frontiers)' in source
    assert 'should_switch_frontier(' in source
    assert 'Safety gate blocked all requested motion' in source
    assert 'Frontier net-progress watchdog expired' in source
    assert 'self._initial_heading_yaw = self.robot_yaw' in source
    assert 'self._initial_heading_yaw' in source
    assert 'if self._entry_axis is None' in source
    assert "declare_parameter('entry_heading_yaw', float('nan'))" in source
    assert "declare_parameter('entry_ingress_depth_m', 0.0)" in source
    assert 'entry_axis_progress_m(' in source
    assert 'ingress_constraint_active' in source
    assert "'ingress-cone='" in source
    assert "declare_parameter('entry_lateral_limit_m', 0.0)" in source
    assert 'require_robot_yaw_candidate=half_angle is not None' in source
    assert 'self._entry_heading() - self.robot_yaw' in source
    assert 'if self._entry_axis is None:' in source
    assert 'self._entry_axis = (' in source
    compile(source, 'frontier_explorer_node.py', 'exec')
