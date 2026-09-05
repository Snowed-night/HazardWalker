"""通用房间视线覆盖规划器的行为测试，不依赖 ROS 或官方楼宇坐标。"""

import math
from pathlib import Path
import sys

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
NAV_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav'
if str(NAV_SRC) not in sys.path:
    sys.path.insert(0, str(NAV_SRC))

from hazardwalker_nav.room_coverage import (  # noqa: E402
    GridFrame,
    coverage_candidate_utility,
    grid_shortest_path_distances,
    plan_room_visibility_coverage,
    visible_room_cells,
)


def test_grid_shortest_path_cost_respects_wall_detour_and_corner_cutting():
    traversable = np.ones((9, 9), dtype=bool)
    traversable[1:8, 4] = False
    distances = grid_shortest_path_distances(
        traversable, start_cell=(2, 4), resolution_m=0.5)

    # 直线仅 2m，但墙迫使路线从上下开口绕行；代价必须反映真实绕路。
    assert distances[4, 6] > 4.0
    blocked = np.ones((3, 3), dtype=bool)
    blocked[0, 1] = False
    blocked[1, 0] = False
    corner_distances = grid_shortest_path_distances(
        blocked, start_cell=(0, 0), resolution_m=1.0)
    assert not math.isfinite(corner_distances[1, 1])


def test_normalized_utility_can_prefer_near_high_information_view():
    near = coverage_candidate_utility(
        80, travel_m=1.0, turn_rad=0.2,
        travel_cost_weight=0.5, turn_cost_weight=0.2)
    far = coverage_candidate_utility(
        100, travel_m=8.0, turn_rad=math.pi,
        travel_cost_weight=0.5, turn_cost_weight=0.2)
    assert near > far


def _empty_room(width=32, height=24):
    grid = np.full((height, width), 100, dtype=np.int16)
    room = np.zeros_like(grid, dtype=bool)
    room[2:-2, 2:-2] = True
    grid[room] = 0
    return grid, room


def test_open_room_reaches_requested_visibility_without_building_coordinates():
    grid, room = _empty_room()
    plan = plan_room_visibility_coverage(
        grid, room, GridFrame(0.25), entrance_world=(1.0, 3.0),
        camera_fov_rad=math.radians(90.0), camera_range_m=6.0,
        robot_clearance_m=0.25, desired_coverage_ratio=0.92,
    )

    assert plan.coverage_ratio >= 0.92
    assert 1 <= len(plan.observation_poses) <= 12
    assert all(pose.newly_visible_cells > 0 for pose in plan.observation_poses)
    assert all(
        len(pose.newly_visible_world_points) == pose.newly_visible_cells
        for pose in plan.observation_poses)


def test_central_obstacle_requires_observations_from_multiple_sides():
    grid, room = _empty_room(width=36, height=28)
    grid[10:18, 15:21] = 100
    plan = plan_room_visibility_coverage(
        grid, room, GridFrame(0.25), entrance_world=(1.0, 3.5),
        camera_fov_rad=math.radians(80.0), camera_range_m=5.0,
        robot_clearance_m=0.25, desired_coverage_ratio=0.90,
        maximum_viewpoints=14,
    )

    assert plan.coverage_ratio >= 0.90
    assert len(plan.observation_poses) >= 2
    x_values = [pose.x_m for pose in plan.observation_poses]
    obstacle_center_x = 18.0 * 0.25
    assert min(x_values) < obstacle_center_x < max(x_values)


def test_visibility_ray_does_not_pass_through_furniture():
    grid, room = _empty_room(width=20, height=14)
    grid[3:11, 9] = 100
    targets = [(6, 7), (13, 7)]
    visible = visible_room_cells(
        grid, room, GridFrame(0.5), viewpoint=(5, 7), yaw_rad=0.0,
        target_cells=targets, camera_fov_rad=math.pi,
        camera_range_m=10.0,
    )

    assert (6, 7) in visible
    assert (13, 7) not in visible


def test_plan_is_translation_equivariant_instead_of_using_fixed_room_positions():
    base_grid, base_room = _empty_room(width=24, height=20)
    shifted_grid = np.full((32, 38), 100, dtype=np.int16)
    shifted_room = np.zeros_like(shifted_grid, dtype=bool)
    offset_x, offset_y = 7, 5
    shifted_grid[
        offset_y:offset_y + base_grid.shape[0],
        offset_x:offset_x + base_grid.shape[1],
    ] = base_grid
    shifted_room[
        offset_y:offset_y + base_room.shape[0],
        offset_x:offset_x + base_room.shape[1],
    ] = base_room
    frame = GridFrame(0.25)
    base = plan_room_visibility_coverage(
        base_grid, base_room, frame, entrance_world=(1.0, 2.5),
        camera_fov_rad=math.radians(90), camera_range_m=5.0,
        robot_clearance_m=0.25, desired_coverage_ratio=0.90,
    )
    shifted = plan_room_visibility_coverage(
        shifted_grid, shifted_room, frame,
        entrance_world=(1.0 + offset_x * 0.25, 2.5 + offset_y * 0.25),
        camera_fov_rad=math.radians(90), camera_range_m=5.0,
        robot_clearance_m=0.25, desired_coverage_ratio=0.90,
    )

    assert math.isclose(base.coverage_ratio, shifted.coverage_ratio)
    assert len(base.observation_poses) == len(shifted.observation_poses)
    for original, moved in zip(base.observation_poses, shifted.observation_poses):
        assert math.isclose(moved.x_m - original.x_m, offset_x * 0.25)
        assert math.isclose(moved.y_m - original.y_m, offset_y * 0.25)
        assert math.isclose(moved.yaw_rad, original.yaw_rad)


def test_no_traversable_room_returns_explicit_empty_plan():
    grid = np.full((10, 10), 100, dtype=np.int16)
    room = np.ones_like(grid, dtype=bool)
    plan = plan_room_visibility_coverage(
        grid, room, GridFrame(0.1), entrance_world=(0.0, 0.0),
        camera_fov_rad=math.pi / 2.0, camera_range_m=3.0,
    )

    assert not plan.observation_poses
    assert plan.coverage_ratio == 0.0
    assert plan.target_cell_count == 0
