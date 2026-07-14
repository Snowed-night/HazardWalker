"""侧向重观察几何规划的离线回归测试。"""

import math
import os
import sys


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_perception'))

from hazardwalker_perception.active_view_geometry import plan_lateral_reobservation


def test_left_plan_generates_short_arc_and_faces_target():
    plan = plan_lateral_reobservation(
        camera_position=(0.0, 0.0, 0.3),
        target_position=(2.0, 0.0, 0.3),
        action='move_left',
        min_bearing_change_deg=25.0,
        max_step_distance_m=0.30,
    )

    assert plan.feasible is True
    assert len(plan.waypoints) >= 2
    final = plan.waypoints[-1]
    assert final.y > 0.0
    assert abs(final.expected_bearing_change_deg - 25.0) < 1e-6
    assert abs(final.yaw_rad - math.atan2(-final.y, 2.0 - final.x)) < 1e-6


def test_right_plan_mirrors_left_plan():
    plan = plan_lateral_reobservation(
        camera_position=(0.0, 0.0, 0.3),
        target_position=(2.0, 0.0, 0.3),
        action='move_right',
    )

    assert plan.feasible is True
    assert plan.waypoints[-1].y < 0.0
    assert plan.waypoints[-1].expected_bearing_change_deg == 25.0


def test_non_lateral_action_cannot_claim_a_side_view_plan():
    plan = plan_lateral_reobservation(
        camera_position=(0.0, 0.0, 0.3),
        target_position=(2.0, 0.0, 0.3),
        action='move_forward',
    )

    assert plan.feasible is False
    assert not plan.waypoints


def test_near_target_requires_safety_reposition_before_orbit():
    plan = plan_lateral_reobservation(
        camera_position=(0.0, 0.0, 0.3),
        target_position=(0.05, 0.0, 0.3),
        action='move_left',
    )

    assert plan.feasible is False
    assert '过近' in plan.reason
