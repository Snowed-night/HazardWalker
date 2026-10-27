"""Frontier A* 安全约束的离线回归测试。

负责人：姜晨。验证未知区不可穿越、障碍膨胀和对角防穿角，不依赖 ROS2。
"""

from types import SimpleNamespace
from collections import deque
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
    append_loop_erased_history,
    build_reverse_history_path,
    body_tilt_degrees_from_quaternion,
    build_counterclockwise_room_loop,
    physical_room_loop_is_valid,
    scaled_room_waypoint_candidates,
    cluster_frontiers,
    compute_frontier_backoff_ttl_s,
    compute_exploration_time_limit_s,
    corridor_room_sector,
    detect_opened_door_from_scans,
    entry_axis_progress_m,
    entry_ingress_constraint_active,
    entry_ingress_half_angles_deg,
    find_frontiers,
    Frontier,
    frontier_route_is_excessive_detour,
    interpolate_path_lookahead,
    nearest_frontier_basin_key,
    prioritize_unvisited_room_frontiers,
    polygon_signed_area,
    return_pose_has_progress,
    return_recovery_turn_command,
    select_best_frontier,
    select_cached_missing_room_doorway,
    select_symmetric_doorway_stations,
    simulation_period_elapsed,
    should_switch_frontier,
    transform_planar_point,
    transform_planar_goal_to_robot_frame,
    world_to_grid,
)


def test_body_tilt_from_imu_quaternion_detects_upright_and_side_fall():
    assert body_tilt_degrees_from_quaternion(0.0, 0.0, 0.0, 1.0) == 0.0
    half = math.sqrt(0.5)
    assert math.isclose(
        body_tilt_degrees_from_quaternion(half, 0.0, 0.0, half),
        90.0,
        abs_tol=1e-6,
    )
    assert body_tilt_degrees_from_quaternion(
        float('nan'), 0.0, 0.0, 1.0,
    ) is None


def test_sparse_path_lookahead_interpolates_instead_of_jumping_to_endpoint():
    goal = interpolate_path_lookahead(
        [(5.0, 0.0)], 0, 0.0, 0.0, 1.25)
    assert goal is not None
    assert math.isclose(goal[0], 1.25, abs_tol=1e-9)
    assert math.isclose(goal[1], 0.0, abs_tol=1e-9)
    assert math.isclose(goal[2], 0.0, abs_tol=1e-9)


def test_map_goal_converts_to_robot_relative_frame_without_shared_odom_origin():
    goal = transform_planar_goal_to_robot_frame(
        10.0, 7.0, math.pi / 2.0,
        10.0, 5.0, math.pi / 2.0)
    assert math.isclose(goal[0], 2.0, abs_tol=1e-9)
    assert math.isclose(goal[1], 0.0, abs_tol=1e-9)
    assert math.isclose(goal[2], 0.0, abs_tol=1e-9)


def test_path_lookahead_follows_polyline_corner_and_reports_tangent():
    goal = interpolate_path_lookahead(
        [(1.0, 0.0), (1.0, 2.0)], 0, 0.0, 0.0, 1.5)
    assert goal is not None
    assert math.isclose(goal[0], 1.0, abs_tol=1e-9)
    assert math.isclose(goal[1], 0.5, abs_tol=1e-9)
    assert math.isclose(goal[2], math.pi / 2.0, abs_tol=1e-9)


def test_path_lookahead_projects_past_old_anchor_and_never_points_backward():
    goal = interpolate_path_lookahead(
        [(0.0, 0.0), (10.0, 0.0)], 0,
        6.0, 0.0, 1.25)
    assert goal is not None
    assert math.isclose(goal[0], 7.25, abs_tol=1e-9)
    assert math.isclose(goal[1], 0.0, abs_tol=1e-9)
    assert math.isclose(goal[2], 0.0, abs_tol=1e-9)


def test_loop_erased_history_removes_completed_room_excursion():
    history = deque()
    points = [
        (0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0),
        (2.0, 0.0), (2.5, 0.0), (3.0, 0.0), (3.0, 0.5),
        (3.0, 1.0), (2.5, 1.0), (2.0, 1.0), (1.5, 1.0),
        (1.0, 1.0), (1.0, 0.55),
    ]
    for point in points:
        append_loop_erased_history(
            history, point, spacing_m=0.10,
            loop_radius_m=0.60, min_index_gap=6)
    assert history[-1] == (1.0, 0.55)
    assert len(history) < len(points)
    assert (3.0, 1.0) not in history


def test_door_scan_difference_returns_largest_contiguous_opening():
    closed = [5.0] * 360
    opened = list(closed)
    for index in range(170, 181):
        closed[index] = 2.5
        opened[index] = 6.0
    # 离散噪声不能覆盖真实连续门洞。
    opened[40] = 9.0
    result = detect_opened_door_from_scans(
        closed, opened, -math.pi, 2.0 * math.pi / 360.0)
    assert result is not None
    bearing, distance, bins = result
    assert abs(bearing) < math.radians(6.0)
    assert distance == 2.5
    assert bins == 11


def test_door_scan_difference_rejects_sparse_noise():
    closed = [2.0] * 360
    opened = list(closed)
    opened[10] = 4.0
    opened[100] = 5.0
    assert detect_opened_door_from_scans(
        closed, opened, -math.pi,
        2.0 * math.pi / 360.0) is None

    distant_closed = [30.0] * 360
    distant_opened = [40.0] * 360
    assert detect_opened_door_from_scans(
        distant_closed, distant_opened, -math.pi,
        2.0 * math.pi / 360.0) is None

    near_closed = [1.0] * 360
    near_opened = [4.0] * 360
    assert detect_opened_door_from_scans(
        near_closed, near_opened, -math.pi,
        2.0 * math.pi / 360.0) is None


def test_reverse_history_path_replays_verified_route_with_spacing():
    history = [(index * 0.1, 0.0) for index in range(51)]
    path = build_reverse_history_path(
        history, current_x=5.0, current_y=0.0,
        home_x=0.0, home_y=0.0, spacing_m=0.45)

    assert path
    assert path[-1] == (0.0, 0.0)
    assert path[0][0] < 5.0
    assert all(
        math.hypot(b[0] - a[0], b[1] - a[1]) >= 0.45 - 1e-9
        for a, b in zip(path, path[1:-1]))


def test_simulation_period_throttles_planning_independent_of_wall_rate():
    assert not simulation_period_elapsed(0.0, None, 3.0)
    assert simulation_period_elapsed(10.0, None, 3.0)
    assert not simulation_period_elapsed(10.39, 10.0, 3.0)
    assert simulation_period_elapsed(13.0, 10.0, 3.0)
    # 仿真重置或 rosbag 回跳后立即允许重新建立周期锚点。
    assert simulation_period_elapsed(2.0, 13.0, 3.0)
    assert not simulation_period_elapsed(float('nan'), 1.0, 3.0)


def test_revisit_density_penalty_prefers_equally_good_unvisited_frontier():
    repeated = Frontier((2.0, 0.0), 20, [], 20.0)
    fresh = Frontier((0.0, 2.0), 20, [], 20.0)
    visited = [(2.0, 0.0)] * 16

    selected = select_best_frontier(
        [repeated, fresh], 0.0, 0.0,
        min_frontier_size=1,
        locality_slack_m=3.0,
        visited_positions=visited,
        revisit_penalty_radius_m=1.0,
        revisit_penalty_strength=0.8,
        revisit_free_samples=4,
        revisit_full_penalty_samples=12,
    )

    assert selected is fresh


def test_legal_odom_home_point_reprojects_after_map_loop_closure():
    x, y = transform_planar_point(
        2.0, 1.0,
        translation_x=10.0,
        translation_y=-3.0,
        yaw_rad=math.pi / 2.0,
    )
    assert abs(x - 9.0) < 1e-9
    assert abs(y + 1.0) < 1e-9


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


def test_a_star_simplifies_open_path_to_endpoints():
    grid = np.zeros((12, 12), dtype=np.int8)
    message = _grid_message(grid)

    path = a_star_path(
        grid, message, 1.5, 1.5, 10.5, 9.5,
        inflation_radius_m=0.0,
    )

    assert len(path) == 2
    assert _path_cells(path, message) == [(1, 1), (10, 9)]


def test_a_star_simplification_keeps_safe_corner_waypoint():
    grid = np.zeros((12, 12), dtype=np.int8)
    grid[0:9, 6] = 100
    message = _grid_message(grid)

    path = a_star_path(
        grid, message, 2.5, 2.5, 9.5, 2.5,
        inflation_radius_m=0.0,
    )

    cells = _path_cells(path, message)
    assert len(path) >= 3
    assert any(y >= 9 for _, y in cells)
    assert all(grid[y, x] == 0 for x, y in cells)


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


def test_frontier_vectorization_preserves_four_neighbor_and_boundary_rules():
    grid = np.zeros((5, 5), dtype=np.int8)
    grid[2, 2] = -1
    grid[0, 2] = -1
    mask = find_frontiers(grid)

    assert mask[1, 2]
    assert mask[2, 1]
    assert mask[2, 3]
    assert mask[3, 2]
    assert not mask[1, 1]  # 对角未知不构成四邻域前沿。
    assert not mask[0, 1]  # 边界格永远不选作前沿。

    with np.testing.assert_raises(ValueError):
        find_frontiers(np.zeros((2, 2, 2), dtype=np.int8))


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


def test_return_recovery_turn_command_alternates_without_changing_normal_turns():
    assert return_recovery_turn_command(1, 0.8) == 0.8
    assert return_recovery_turn_command(2, 0.8) == -0.8
    assert return_recovery_turn_command(3, 0.8) == 0.8
    assert return_recovery_turn_command(0, 0.8) == 0.0
    assert return_recovery_turn_command(1, float('nan')) == 0.0


def test_frontier_detour_gate_rejects_only_large_relative_and_absolute_detours():
    # 第 23 轮典型隔墙前沿：约 2m 直线距离，却要走 12.5m。
    assert frontier_route_is_excessive_detour(12.5, 2.0, 2.8, 5.0)
    # 第 23 轮第二个隔墙目标也应被识别为低效路线。
    assert frontier_route_is_excessive_detour(14.3, 4.74, 2.8, 5.0)
    # 合法房间绕门：路径虽长，但相对直线距离没有达到 2.8 倍。
    assert not frontier_route_is_excessive_detour(14.3, 5.9, 2.8, 5.0)
    # 短距离离散误差不能仅凭比例被过滤。
    assert not frontier_route_is_excessive_detour(1.2, 0.25, 2.8, 5.0)
    assert not frontier_route_is_excessive_detour(float('nan'), 2.0)


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


def test_ingress_progress_priority_builds_far_corridor_backbone_first():
    """骨架阶段优先纵深，不被近处侧门的大信息增益前沿截断。"""

    near_room = Frontier(
        centroid=(5.0, 1.0), size=500, points=[], info_gain=1000.0,
    )
    far_corridor = Frontier(
        centroid=(16.0, 0.5), size=20, points=[], info_gain=20.0,
    )
    selected = select_best_frontier(
        [near_room, far_corridor],
        robot_wx=4.0,
        robot_wy=0.0,
        min_frontier_size=3,
        entry_origin=(0.0, 0.0),
        entry_axis=(1.0, 0.0),
        entry_progress_priority_slack_m=2.0,
    )

    assert selected is far_corridor


def test_room_sector_requires_actual_lateral_entry_and_splits_depth():
    assert corridor_room_sector(
        5.0, 2.5, (0.0, 0.0), (1.0, 0.0), 14.0, 2.0,
    ) == 'near_left'
    assert corridor_room_sector(
        18.0, -3.0, (0.0, 0.0), (1.0, 0.0), 14.0, 2.0,
    ) == 'far_right'
    assert corridor_room_sector(
        18.0, 1.5, (0.0, 0.0), (1.0, 0.0), 14.0, 2.0,
    ) is None


def test_doorway_stations_reject_unpaired_corridor_end_and_wall_candidates():
    observations = [
        # v45 式伪候选：走廊尽头单侧点和接近分界线的墙边点。
        (20.0, -2.0),
        (13.0, 2.0),
        # v41 式真实同排门：远端约 15m、近端约 4m，左右成对。
        (15.0, 2.1),
        (15.2, -2.0),
        (4.2, 2.0),
        (4.0, -2.2),
    ]
    selected = select_symmetric_doorway_stations(
        observations,
        (0.0, 0.0),
        (1.0, 0.0),
        preferred_lateral_m=1.5,
        pair_progress_gap_m=2.0,
        station_cluster_m=2.0,
        minimum_station_separation_m=6.0,
    )

    assert set(selected) == {
        'far_left', 'far_right', 'near_left', 'near_right'}
    assert math.isclose(selected['far_left'][0], 15.1, abs_tol=0.01)
    assert math.isclose(selected['far_right'][0], 15.1, abs_tol=0.01)
    assert math.isclose(selected['near_left'][0], 4.1, abs_tol=0.01)
    assert math.isclose(selected['near_right'][0], 4.1, abs_tol=0.01)
    assert selected['far_right'][0] != 20.0
    assert selected['near_left'][0] < 8.0


def test_doorway_stations_choose_two_farthest_pairs_and_ignore_entry_empty():
    selected = select_symmetric_doorway_stations(
        [
            # 入口左右空地也可能形成一组前沿，但不是房间。
            (4.0, 1.5), (4.1, -1.5),
            # 人工标定确认的两排真实房门近似纵深。
            (18.9, 1.5), (19.0, -1.5),
            (32.8, 1.5), (32.6, -1.5),
        ],
        (0.0, 0.0),
        (1.0, 0.0),
        pair_progress_gap_m=1.0,
        station_cluster_m=1.0,
        minimum_station_separation_m=6.0,
    )

    assert math.isclose(selected['far_left'][0], 32.7, abs_tol=0.01)
    assert math.isclose(selected['far_right'][0], 32.7, abs_tol=0.01)
    assert math.isclose(selected['near_left'][0], 18.95, abs_tol=0.01)
    assert math.isclose(selected['near_right'][0], 18.95, abs_tol=0.01)
    assert selected['far_left'][1] == -selected['far_right'][1]
    assert selected['near_left'][1] == -selected['near_right'][1]
    assert all(
        not math.isclose(pose[0], 4.05, abs_tol=0.5)
        for pose in selected.values())


def test_doorway_stations_require_two_left_right_pairs_without_synthesis():
    assert select_symmetric_doorway_stations(
        [(32.7, 1.5), (32.8, -1.5)],
        (0.0, 0.0),
        (1.0, 0.0),
    ) == {}


def test_room_priority_finishes_far_sides_before_near_rooms():
    near_left = Frontier((7.0, 3.0), 30, [], 30.0)
    far_left = Frontier((18.0, 3.0), 20, [], 20.0)
    far_right = Frontier((19.0, -3.0), 20, [], 20.0)
    pool = [near_left, far_left, far_right]

    selected, mode = prioritize_unvisited_room_frontiers(
        pool, (0.0, 0.0), (1.0, 0.0), set(),
    )
    assert selected == [far_left, far_right]
    assert mode == 'unvisited_far_room'

    selected, mode = prioritize_unvisited_room_frontiers(
        pool, (0.0, 0.0), (1.0, 0.0),
        {'far_left', 'far_right'},
    )
    assert selected == [near_left]
    assert mode == 'unvisited_near_room'


def test_unvisited_room_priority_stays_at_door_band_before_deep_room():
    door = Frontier((18.0, 3.0), 10, [], 10.0)
    deep_behind_wall = Frontier((20.0, 9.0), 1000, [], 1000.0)
    opposite_room = Frontier((18.0, -3.0), 5000, [], 5000.0)
    selected, mode = prioritize_unvisited_room_frontiers(
        [deep_behind_wall, door],
        (0.0, 0.0),
        (1.0, 0.0),
        set(),
        candidate_lateral_m=1.2,
        candidate_max_lateral_m=4.0,
    )
    assert selected == [door]
    assert mode == 'unvisited_far_room'

    selected, mode = prioritize_unvisited_room_frontiers(
        [deep_behind_wall, door, opposite_room],
        (0.0, 0.0),
        (1.0, 0.0),
        set(),
        {'far_left'},
        candidate_lateral_m=1.2,
        candidate_max_lateral_m=4.0,
    )
    assert selected == [deep_behind_wall]
    assert mode == 'attempted_room_deep_after_door'


def test_active_room_priority_rejects_other_rooms_until_perimeter_finishes():
    far_left_door = Frontier((18.0, 2.0), 10, [], 10.0)
    far_left_deep = Frontier((19.0, 8.0), 50, [], 50.0)
    far_right = Frontier((18.0, -3.0), 100, [], 100.0)

    selected, mode = prioritize_unvisited_room_frontiers(
        [far_right, far_left_door, far_left_deep],
        (0.0, 0.0),
        (1.0, 0.0),
        set(),
        {'far_left'},
        active_sector='far_left',
        candidate_lateral_m=1.2,
    )

    assert selected == [far_left_door, far_left_deep]
    assert mode == 'active_room_perimeter'


def test_active_room_priority_reports_when_room_frontiers_are_exhausted():
    selected, mode = prioritize_unvisited_room_frontiers(
        [Frontier((18.0, -3.0), 20, [], 20.0)],
        (0.0, 0.0),
        (1.0, 0.0),
        set(),
        {'far_left'},
        active_sector='far_left',
    )

    assert selected == []
    assert mode == 'active_room_frontiers_exhausted'


def test_cached_doorway_recovers_near_room_after_frontier_disappears():
    cached = {
        'far_right': (18.0, -2.0),
        'near_left': (7.0, 2.0),
        'near_right': (7.0, -2.0),
    }
    assert select_cached_missing_room_doorway(
        cached, {'far_right'}, set()) == ('near_left', (7.0, 2.0))
    assert select_cached_missing_room_doorway(
        cached, {'far_right'}, {'near_left'}) is None
    assert select_cached_missing_room_doorway(
        cached, {'far_right', 'near_left'}, {'near_left'}) == (
            'near_right', (7.0, -2.0))


def test_counterclockwise_loop_is_ccw_on_both_corridor_sides():
    left = build_counterclockwise_room_loop(
        (10.0, 0.0), (1.0, 0.0), 'left', 2.0, 5.0, 2.0)
    right = build_counterclockwise_room_loop(
        (10.0, 0.0), (1.0, 0.0), 'right', 2.0, 5.0, 2.0)
    assert left == [(12.0, 2.0), (12.0, 5.0), (8.0, 5.0), (8.0, 2.0)]
    assert right == [(8.0, -2.0), (8.0, -5.0), (12.0, -5.0), (12.0, -2.0)]
    assert polygon_signed_area(left) > 0.0
    assert polygon_signed_area(right) > 0.0

    rounded = build_counterclockwise_room_loop(
        (10.0, 0.0), (1.0, 0.0), 'left',
        shallow_depth_m=1.1,
        deep_depth_m=2.1,
        half_length_m=1.2,
        corner_radius_m=0.35,
    )
    assert len(rounded) == 6
    assert polygon_signed_area(rounded) > 0.0
    assert all(
        math.hypot(
            rounded[(index + 1) % len(rounded)][0] - point[0],
            rounded[(index + 1) % len(rounded)][1] - point[1],
        ) > 0.0
        for index, point in enumerate(rounded)
    )

    # 四个真实物理角点、足够轨迹长度和闭环面积必须同时成立。
    assert physical_room_loop_is_valid(
        [(1.0, 1.0), (1.0, 3.0), (-1.0, 3.0), (-1.0, 1.0)],
        reached_count=4,
        expected_count=4,
        physical_path_m=7.5,
        min_path_m=5.0,
        min_area_m2=2.0,
    )
    assert not physical_room_loop_is_valid(
        [(1.0, 1.0), (1.0, 1.1), (1.0, 1.0), (1.0, 1.1)],
        reached_count=4,
        expected_count=4,
        physical_path_m=8.0,
        min_path_m=5.0,
        min_area_m2=2.0,
    )
    assert not physical_room_loop_is_valid(
        [(1.0, 1.0), (1.0, 3.0)],
        reached_count=2,
        expected_count=4,
        physical_path_m=8.0,
        min_path_m=5.0,
        min_area_m2=2.0,
    )

    assert scaled_room_waypoint_candidates(
        (6.55, 1.45), (8.05, 2.65)) == [
            (7.75, 2.41),
            (7.45, 2.17),
            (7.15, 1.93),
        ]

    selected, mode = prioritize_unvisited_room_frontiers(
        [], (0.0, 0.0), (1.0, 0.0), set(),
        active_sector='far_left',
    )
    assert selected == []
    assert mode == 'active_room_frontiers_exhausted'


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
    assert "declare_parameter('minimum_linear_speed', 0.45)" in source
    assert "declare_parameter('minimum_turn_speed', 0.60)" in source
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
        "declare_parameter('max_frontier_plan_failures_per_replan', 12)"
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
    assert "declare_parameter('odom_frame', 'odom')" in source
    assert 'self._home_odom' in source
    assert 'self._home_map_position()' in source
    # 官方 odom 只允许用于 ROS1 DWA 的物理走廊居中；SLAM/返航仍必须使用
    # scan+IMU 合法 odom。源码中显式的开关与独立缓存防止混用。
    assert "'official_odom_topic', '/hw/odom'" in source
    assert "'use_official_odom_for_corridor_control', False" in source
    assert 'self._official_control_odom' in source
    assert 'self._home_odom = self._official_control_odom' not in source
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
    assert "declare_parameter('frontier_max_detour_ratio', 2.8)" in source
    assert "declare_parameter('frontier_min_detour_excess_m', 5.0)" in source
    assert "declare_parameter('frontier_detour_defer_ttl_s', 30.0)" in source
    assert "declare_parameter('frontier_detour_evaluation_limit', 2)" in source
    assert 'frontier_route_is_excessive_detour(' in source
    assert 'Deferring inefficient frontier basin' in source
    assert 'accepting the first safe fallback now' in source
    assert '_clear_detour_deferred_frontier(best)' in source
    assert 'preferred_candidates or candidates' in source
    assert "declare_parameter('safety_blocked_timeout_s', 8.0)" in source
    assert "declare_parameter('frontier_observation_sweep_speed', 1.00)" in source
    assert "declare_parameter('entry_min_frontier_size', 3)" in source
    assert "declare_parameter('max_frontier_plan_failures_per_replan', 12)" in source
    assert "declare_parameter('frontier_observation_sweep_timeout_s', 10.0)" in source
    assert 'Starting frontier RGB-D observation sweep:' in source
    assert 'self._handle_frontier_observation_sweep(now_ros)' in source
    assert 'and not self._entry_backbone_active()' in source
    assert 'self._frontier_observation_remaining_rad - delta' in source
    assert 'candidates = apply_room_priority(unvisited_frontiers)' in source
    assert 'should_switch_frontier(' in source
    assert 'Safety gate blocked all requested motion' in source
    assert "declare_parameter('exploration_recovery_turn_speed', 1.0)" in source
    assert "self._start_exploration_recovery('safety_blocked')" in source
    assert "self._start_exploration_recovery('stuck')" in source
    assert 'Starting bounded exploration recovery turn:' in source
    assert 'Frontier net-progress watchdog expired' in source
    assert 'self._initial_heading_yaw = self.robot_yaw' in source
    assert 'self._initial_heading_yaw' in source
    assert 'if self._entry_axis is None' in source
    assert "declare_parameter('entry_heading_yaw', float('nan'))" in source
    assert "declare_parameter('entry_ingress_depth_m', 0.0)" in source
    assert "declare_parameter('entry_ingress_progress_slack_m', 2.0)" in source
    assert "declare_parameter('entry_ingress_time_limit_s', 0.0)" in source
    assert 'Corridor backbone phase completed' in source
    assert "declare_parameter('room_sector_split_depth_m', 14.0)" in source
    assert "declare_parameter('room_sector_candidate_max_lateral_m', 4.0)" in source
    assert 'Room doorway approach selected:' in source
    assert 'Room doorway target physically reached' in source
    assert 'self._attempted_room_sectors.add(reached_room_sector)' in source
    assert 'Using observed doorway symmetry to visit the opposite room:' in source
    assert "self._room_selection_mode = 'mirrored_room_doorway'" in source
    assert "'room_mirrored_doorway_extra_lateral_m', 0.8" in source
    assert 'Room sector entered; starting perimeter coverage' in source
    assert 'Room perimeter coverage complete:' in source
    assert "declare_parameter('room_sector_early_finish_min_s', 60.0)" in source
    assert 'all four room sectors' in source
    assert 'hard time limit' in source
    assert 'prioritize_unvisited_room_frontiers(' in source
    assert "declare_parameter('room_perimeter_min_path_m', 6.0)" in source
    assert "declare_parameter('room_perimeter_linear_speed', 1.80)" in source
    assert "declare_parameter('room_perimeter_probe_count', 4)" in source
    assert 'active_sector=self._active_room_sector' in source
    assert 'frontiers_exhausted_after_360_scan' in source
    assert 'active_room_free_space_probe' in source
    assert 'active_room_loop_return' in source
    assert 'coverage_distance_met' in source
    assert 'and self._active_room_return_attempted' in source
    assert 'if self.current_target is not None and self.current_path:' in source
    assert '活动房间中的候选均不可规划时' in source
    assert 'Deterministic room entered:' in source
    assert 'Deterministic room complete:' in source
    assert "'corridor_outbound'" in source
    assert 'build_counterclockwise_room_loop(' in source
    assert "'deterministic_waypoint_stall_s', 30.0" in source
    assert "'deterministic_min_scale_tolerance_m', 1.5" in source
    assert "'deterministic_blocked_corner_accept_m', 1.0" in source
    assert "float(self._deterministic_waypoint_scales.get(" in source
    assert 'Blocked room loop corner' in source
    assert 'nearest scaled corner distance=' in source
    assert 'def _recover_stalled_deterministic_waypoint(' in source
    assert 'shrinking relative ' in source
    assert 'room target to' in source
    assert "completed all '" in source
    assert "'four counterclockwise room loops.'" in source
    assert 'def _plan_reachable_corridor_goal(' in source
    assert "'corridor_outbound'" in source
    assert 'Corridor end reached by' in source
    assert "declare_parameter('room_entry_inflation_radius_m', 0.45)" in source
    assert "declare_parameter('room_entry_navigation_clearance_m', 0.35)" in source
    assert "declare_parameter('fall_tilt_threshold_deg', 55.0)" in source
    assert "self._transition('FAILED')" in source
    assert "Imu, str(self.get_parameter('imu_topic').value)" in source
    assert "'fall_detected'" in source
    assert "'invalidated'" in source
    assert 'self._visited_room_sectors.discard(sector)' in source
    assert 'inflation_radius_m=self._frontier_planning_inflation_m(' in source
    assert "declare_parameter('return_linear_speed', 2.00)" in source
    assert "declare_parameter('return_minimum_linear_speed', 1.20)" in source
    assert "declare_parameter('return_final_linear_speed', 0.80)" in source
    assert "declare_parameter('return_final_minimum_linear_speed', 0.30)" in source
    assert "declare_parameter('return_waypoint_spacing_m', 1.50)" in source
    assert 'return self._follow_return_path()' in source
    assert 'entry_axis_progress_m(' in source
    assert 'ingress_constraint_active' in source
    assert "'ingress-cone='" in source
    assert "declare_parameter('entry_lateral_limit_m', 0.0)" in source
    assert 'require_robot_yaw_candidate=half_angle is not None' in source
    assert 'self._entry_heading() - self.robot_yaw' in source
    assert 'if self._entry_axis is None:' in source
    assert 'self._entry_axis = (' in source
    compile(source, 'frontier_explorer_node.py', 'exec')
