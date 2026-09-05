"""严格房间巡检计划的离线行为测试。"""

from pathlib import Path
from types import SimpleNamespace
import math
import sys

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
NAV_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav'
if str(NAV_SRC) not in sys.path:
    sys.path.insert(0, str(NAV_SRC))

from hazardwalker_nav.room_inspection_planner import (  # noqa: E402
    AnchoredInspectionGoalProjector,
    InspectionProgress,
    RoomInspectionExecution,
    bounded_inspection_turn_rate,
    build_room_visibility_inspection_plan,
    build_strict_room_inspection_plan,
    physical_pose_has_progressed,
    reproject_planar_pose_between_robot_frames,
    visibility_coverage_requirement_met,
)


def test_physical_goal_is_frozen_while_slam_pose_changes():
    projector = AnchoredInspectionGoalProjector()
    first = projector.resolve(
        'view-1', (5.0, 1.0), 0.4,
        (4.0, 1.0, 0.0), (10.0, 20.0, 0.0),
    )
    drifted = projector.resolve(
        'view-1', (5.0, 1.0), 0.4,
        (4.8, 1.2, 0.2), (10.7, 20.1, 0.1),
    )
    next_goal = projector.resolve(
        'view-2', (6.0, 1.0), 0.4,
        (4.8, 1.2, 0.2), (10.7, 20.1, 0.1),
    )

    assert first == drifted
    assert next_goal != first


def test_physical_progress_accepts_detour_translation_or_in_place_rotation():
    anchor = (1.0, 2.0, math.pi - 0.05)
    assert not physical_pose_has_progressed(
        anchor, (1.10, 2.05, -math.pi + 0.05))
    assert physical_pose_has_progressed(
        anchor, (1.21, 2.0, math.pi - 0.05))
    assert physical_pose_has_progressed(
        anchor, (1.0, 2.0, -math.pi + 0.25))
from hazardwalker_nav.room_obstacle_profiler import (  # noqa: E402
    OCCUPIED_THRESHOLD,
)


def _grid_message(grid, resolution=0.25):
    return SimpleNamespace(
        info=SimpleNamespace(
            width=grid.shape[1],
            height=grid.shape[0],
            resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
            ),
        ),
        data=grid.reshape(-1).tolist(),
    )


def _room_with_obstacle(narrow_passage=False):
    grid = np.full((48, 64), OCCUPIED_THRESHOLD, dtype=np.int16)
    # 走廊位于左侧；房间 free x=16..58, y=6..41，门洞在 x=15。
    grid[:, 0:15] = 0
    grid[6:42, 16:59] = 0
    grid[22:27, 15] = 0
    grid[21:28, 34:41] = OCCUPIED_THRESHOLD
    if narrow_passage:
        # 原始 free 连通域可从单格缝隙进入右半房，因此障碍仍属于同一房间；
        # 机器人安全膨胀后缝隙关闭，右侧观察点对当前机体不可达。
        grid[6:42, 29:32] = OCCUPIED_THRESHOLD
        grid[24:25, 29:32] = 0
    return grid


def _build(grid, required=3):
    return build_strict_room_inspection_plan(
        grid,
        _grid_message(grid),
        entry_world=(15.0 * 0.25, 24.5 * 0.25),
        entry_yaw_rad=0.0,
        start_world=(18.0 * 0.25, 24.5 * 0.25),
        door_width_m=1.25,
        seed_offset_m=0.75,
        minimum_room_free_cells=200,
        minimum_obstacle_area_m2=0.10,
        wall_margin_m=0.75,
        viewpoint_count=12,
        required_views_per_obstacle=required,
        viewpoint_standoff_m=0.75,
        viewpoint_clearance_m=0.20,
        path_inflation_radius_m=0.15,
    )


def test_reachable_obstacle_gets_three_distinct_physical_observation_goals():
    plan = _build(_room_with_obstacle(), required=3)

    assert plan.executable
    assert plan.obstacle_count == 1
    assert len(plan.goals) == 3
    assert len({goal.direction_bucket for goal in plan.goals}) == 3
    assert all(goal.path for goal in plan.goals)


def test_unreachable_directions_make_plan_incomplete_instead_of_counting_skip():
    plan = _build(_room_with_obstacle(narrow_passage=True), required=4)

    assert not plan.executable
    assert plan.obstacle_count >= 1
    assert plan.uncovered_obstacles
    assert any(
        obstacle.reachable_direction_count
        < obstacle.required_direction_count
        for obstacle in plan.uncovered_obstacles
    )


def test_failed_capture_never_advances_strict_completion():
    plan = _build(_room_with_obstacle(), required=3)
    progress = InspectionProgress(plan)

    first = plan.goals[0]
    assert not progress.record_capture(first.goal_id, succeeded=False)
    progress.record_failure(first.goal_id, 'camera_timeout')
    assert progress.completed_goal_count == 0
    assert not progress.complete

    for goal in plan.goals:
        assert progress.record_capture(goal.goal_id, succeeded=True)
    assert progress.complete
    assert progress.pending_goal_ids == tuple()


def test_unknown_goal_id_cannot_forge_completion():
    plan = _build(_room_with_obstacle(), required=3)
    progress = InspectionProgress(plan)

    assert not progress.record_capture('not_in_plan', succeeded=True)
    assert progress.completed_goal_count == 0
    assert not progress.complete


def test_execution_requires_move_orient_and_successful_capture_in_order():
    plan = _build(_room_with_obstacle(), required=3)
    execution = RoomInspectionExecution(plan)

    assert execution.phase == execution.MOVE
    assert not execution.mark_orientation_reached()
    assert execution.mark_position_reached()
    assert execution.phase == execution.ORIENT
    assert execution.mark_orientation_reached()
    assert execution.phase == execution.CAPTURE
    assert not execution.mark_capture(False)
    assert execution.phase == execution.CAPTURE
    assert execution.progress.completed_goal_count == 0
    assert execution.mark_capture(True)
    assert execution.phase == execution.MOVE
    assert execution.goal_index == 1


def test_motion_failure_never_skips_goal_and_retry_keeps_same_goal():
    plan = _build(_room_with_obstacle(), required=3)
    execution = RoomInspectionExecution(plan)
    first_goal_id = execution.current_goal.goal_id

    execution.mark_motion_failure('a_star_unreachable_after_map_update')
    assert execution.phase == execution.FAILED
    assert not execution.complete
    assert execution.current_goal.goal_id == first_goal_id
    assert execution.retry_current_goal()
    assert execution.phase == execution.MOVE
    assert execution.current_goal.goal_id == first_goal_id


def test_execution_completes_only_after_every_planned_capture():
    plan = _build(_room_with_obstacle(), required=3)
    execution = RoomInspectionExecution(plan)

    for index in range(len(plan.goals)):
        assert execution.mark_position_reached()
        assert execution.mark_orientation_reached()
        assert execution.mark_capture(True)
        if index < len(plan.goals) - 1:
            assert not execution.complete

    assert execution.complete
    assert execution.progress.completed_goal_count == len(plan.goals)


def test_pose_reprojection_uses_synchronized_robot_anchor_not_fixed_map_offset():
    projected = reproject_planar_pose_between_robot_frames(
        point=(3.0, 1.0),
        heading_rad=0.0,
        source_robot_pose=(1.0, 1.0, 0.0),
        target_robot_pose=(10.0, 20.0, math.pi / 2.0),
    )
    assert projected[0] == pytest.approx(10.0)
    assert projected[1] == pytest.approx(22.0)
    assert projected[2] == pytest.approx(math.pi / 2.0)


def test_inspection_turn_rate_is_bounded_and_stops_inside_tolerance():
    assert bounded_inspection_turn_rate(0.1, 0.2, 0.8, 0.15) == 0.0
    assert bounded_inspection_turn_rate(2.0, 0.2, 0.8, 0.15) == 0.8
    assert bounded_inspection_turn_rate(-2.0, 0.2, 0.8, 0.15) == -0.8
    assert bounded_inspection_turn_rate(0.25, 0.2, 0.8, 0.15) == 0.25


def test_visibility_requirement_uses_integer_cells_with_one_cell_tolerance():
    assert visibility_coverage_requirement_met(379, 400, 0.95)
    assert not visibility_coverage_requirement_met(378, 400, 0.95)
    assert not visibility_coverage_requirement_met(0, 0, 0.95)


def test_visibility_plan_covers_open_room_and_requires_physical_captures():
    grid = _room_with_obstacle()
    plan = build_room_visibility_inspection_plan(
        grid,
        _grid_message(grid),
        entry_world=(15.0 * 0.25, 24.5 * 0.25),
        entry_yaw_rad=0.0,
        start_world=(18.0 * 0.25, 24.5 * 0.25),
        door_width_m=1.25,
        seed_offset_m=0.75,
        minimum_room_free_cells=200,
        camera_fov_rad=math.radians(90.0),
        camera_range_m=8.0,
        target_spacing_m=0.5,
        candidate_spacing_m=1.0,
        maximum_viewpoints=12,
        desired_coverage_ratio=0.90,
        path_inflation_radius_m=0.15,
        goal_id_prefix='floor_2_near_left',
    )
    assert plan.executable
    assert plan.visibility_coverage_ratio >= 0.90
    assert plan.visibility_target_cell_count > 0
    assert plan.goals
    assert all(
        goal.goal_id.startswith('floor_2_near_left_room_visibility_')
        for goal in plan.goals)
    execution = RoomInspectionExecution(plan)
    assert not execution.complete
    for goal in plan.goals:
        assert goal.path
        assert execution.mark_position_reached()
        assert execution.mark_orientation_reached()
        assert execution.mark_capture(True)
    assert execution.complete
