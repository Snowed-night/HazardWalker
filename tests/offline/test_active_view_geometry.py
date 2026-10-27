"""侧向重观察几何规划的离线回归测试。"""

import math
import os
import sys
from types import SimpleNamespace


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_perception'))

from hazardwalker_perception.active_view_geometry import (
    camera_pose_signature,
    plan_lateral_reobservation,
    quantized_camera_view_id,
)


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


def _transform(rotation, x=0.0, y=0.0, z=0.3):
    return SimpleNamespace(
        translation=SimpleNamespace(x=x, y=y, z=z),
        rotation=rotation,
    )


def test_gazebo_x_forward_camera_rotation_changes_stability_yaw_and_view_id():
    identity = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    yaw_90 = (
        (0.0, -1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
    )

    first = camera_pose_signature(
        _transform(identity), 'gazebo_link_x_forward',
    )
    second = camera_pose_signature(
        _transform(yaw_90), 'gazebo_link_x_forward',
    )

    assert abs(first[3]) < 1e-9
    assert abs(math.degrees(second[3]) - 90.0) < 1e-9
    assert quantized_camera_view_id(
        _transform(identity), 'gazebo_link_x_forward',
    ).endswith('yaw:0')
    assert quantized_camera_view_id(
        _transform(yaw_90), 'gazebo_link_x_forward',
    ).endswith('yaw:90')


def test_optical_z_forward_camera_keeps_using_rotation_third_column():
    optical_forward_world_x = (
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
    )

    signature = camera_pose_signature(
        _transform(optical_forward_world_x), 'optical_z_forward',
    )

    assert abs(signature[3]) < 1e-9
    assert quantized_camera_view_id(
        _transform(optical_forward_world_x), 'optical_z_forward',
    ).endswith('yaw:0')
