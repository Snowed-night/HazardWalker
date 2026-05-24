"""固定航点控制离线测试。

所属组：导航组 / 测试组。
文件作用：
- 验证 `waypoint_controller.py` 的纯函数控制逻辑。
- 不依赖 ROS、Nav2、Gazebo 或真实机器人。

当前验证内容：
- 角度会被规范到 `[-pi, pi]`。
- 机器人朝向目标时会前进。
- 机器人背对目标时先旋转，不直接前进。
- 最后一个航点到达后进入 `FINISHED`。

后续扩展：
- 增加 RETURNING 状态测试。
- 增加多个航点切换测试。
- 增加不同速度上限和到点容差测试。
"""
import math
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_nav'))

from hazardwalker_nav.waypoint_controller import compute_waypoint_command, normalize_angle


def test_normalize_angle_wraps_to_pi_range():
    """验证角度规范化不会超出 [-pi, pi]。"""
    assert -math.pi <= normalize_angle(4.0) <= math.pi
    assert -math.pi <= normalize_angle(-4.0) <= math.pi


def test_waypoint_command_moves_forward_when_facing_goal():
    """验证朝向目标时输出正向速度。"""
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
    """验证朝向误差过大时先转向，线速度保持为 0。"""
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
    """验证最后一个航点到达后任务完成。"""
    result = compute_waypoint_command(
        x=0.0,
        y=0.0,
        yaw=0.0,
        waypoints=[(0.0, 0.0)],
        goal_index=0,
    )

    assert result.state == 'FINISHED'
    assert result.completed
