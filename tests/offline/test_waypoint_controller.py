import math
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_nav'))

from hazardwalker_nav.waypoint_controller import compute_waypoint_command, normalize_angle


def test_normalize_angle_wraps_to_pi_range():
    assert -math.pi <= normalize_angle(4.0) <= math.pi
    assert -math.pi <= normalize_angle(-4.0) <= math.pi


def test_waypoint_command_moves_forward_when_facing_goal():
    result = compute_waypoint_command(
        x=0.0,
        y=0.0,
        yaw=0.0,
        waypoints=[(1.0, 0.0), (0.0, 0.0)],
        goal_index=0,
    )

    assert result.state == 'NAVIGATING'
    assert result.linear_x > 0.0
    assert abs(result.angular_z) < 1e-6
    assert not result.completed


def test_waypoint_command_rotates_before_forward_motion():
    result = compute_waypoint_command(
        x=0.0,
        y=0.0,
        yaw=math.pi,
        waypoints=[(1.0, 0.0), (0.0, 0.0)],
        goal_index=0,
    )

    assert result.linear_x == 0.0
    assert abs(result.angular_z) > 0.0


def test_waypoint_command_finishes_after_last_goal():
    result = compute_waypoint_command(
        x=0.0,
        y=0.0,
        yaw=0.0,
        waypoints=[(0.0, 0.0)],
        goal_index=0,
    )

    assert result.state == 'FINISHED'
    assert result.completed
