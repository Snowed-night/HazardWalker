"""官方多层楼宇返航纯函数测试。"""

import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
NAV_SRC = ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav'
sys.path.insert(0, str(NAV_SRC))

from hazardwalker_nav.official_return import (  # noqa: E402
    planar_velocity_to_goal,
    staged_corridor_goal,
)


def test_long_corridor_goal_is_bounded_toward_elevator():
    goal = staged_corridor_goal(
        0.1, 35.0, 0.8, 2.6, math.pi,
        corridor_center_x=0.0, lookahead_m=3.0,
    )
    assert goal[0] == 0.0
    assert goal[1] == 32.0
    assert math.isclose(goal[2], -math.pi / 2.0, abs_tol=0.04)


def test_room_exit_first_returns_to_corridor_centerline():
    goal = staged_corridor_goal(
        2.0, 15.0, 0.8, 2.6, math.pi,
        corridor_center_x=0.0, lookahead_m=3.0,
    )
    assert goal[:2] == (0.0, 15.0)
    assert math.isclose(abs(goal[2]), math.pi)


def test_near_final_goal_uses_exact_pose():
    goal = staged_corridor_goal(
        0.0, 3.2, 0.8, 2.6, math.pi,
        corridor_center_x=0.0, lookahead_m=3.0,
    )
    assert goal == (0.8, 2.6, math.pi)


def test_direct_fallback_turns_before_advancing():
    command = planar_velocity_to_goal(
        0.0, 35.0, math.pi / 2.0, 0.0, 32.0,
        linear_speed=0.9, minimum_linear_speed=0.45,
        angular_speed=1.4, minimum_turn_speed=0.3,
        heading_tolerance_rad=0.25,
    )
    assert command.linear_x == 0.0
    assert command.angular_z < 0.0


def test_direct_fallback_advances_after_alignment():
    command = planar_velocity_to_goal(
        0.0, 35.0, -math.pi / 2.0, 0.0, 32.0,
        linear_speed=0.9, minimum_linear_speed=0.45,
        angular_speed=1.4, minimum_turn_speed=0.3,
        heading_tolerance_rad=0.25,
    )
    assert command.linear_x == 0.9
    assert abs(command.angular_z) < 1e-9
