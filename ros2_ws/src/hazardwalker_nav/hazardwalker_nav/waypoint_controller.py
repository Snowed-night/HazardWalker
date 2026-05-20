"""固定航点巡检的离线控制函数。

本文件不依赖 ROS。导航组可以先用普通 Python 测试航点切换、返航状态和
简单控制律，后续再把控制逻辑替换为 Nav2 goal client。
"""

import math
from dataclasses import dataclass


@dataclass
class WaypointCommand:
    """固定航点控制函数的输出。"""

    linear_x: float
    angular_z: float
    state: str
    goal_index: int
    completed: bool


def normalize_angle(angle):
    """把角度规范到 [-pi, pi]。"""

    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def compute_waypoint_command(x, y, yaw, waypoints, goal_index, completed=False,
                             goal_tolerance_m=0.5, linear_speed=0.35,
                             angular_speed=0.8, heading_tolerance_rad=0.25):
    """根据当前位置和航点列表计算速度命令。

    Args:
        x, y, yaw: 机器人当前二维位姿。
        waypoints: [(x1, y1), (x2, y2), ...] 航点列表。
        goal_index: 当前目标航点下标。
        completed: 任务是否已经完成。
        goal_tolerance_m: 到达航点的距离阈值。
        linear_speed: 最大前进速度。
        angular_speed: 最大角速度。
        heading_tolerance_rad: 朝向误差小于该值时才向前走。

    Returns:
        WaypointCommand。
    """

    if completed or not waypoints:
        return WaypointCommand(0.0, 0.0, 'FINISHED', goal_index, True)

    goal_index = min(max(goal_index, 0), len(waypoints) - 1)
    goal_x, goal_y = waypoints[goal_index]
    dx = goal_x - x
    dy = goal_y - y
    distance = math.hypot(dx, dy)

    if distance <= goal_tolerance_m:
        goal_index += 1
        if goal_index >= len(waypoints):
            return WaypointCommand(0.0, 0.0, 'FINISHED', goal_index, True)
        goal_x, goal_y = waypoints[goal_index]
        dx = goal_x - x
        dy = goal_y - y
        distance = math.hypot(dx, dy)

    state = 'NAVIGATING' if goal_index < len(waypoints) - 1 else 'RETURNING'
    target_yaw = math.atan2(dy, dx)
    heading_error = normalize_angle(target_yaw - yaw)
    angular_z = max(-angular_speed, min(angular_speed, heading_error))

    if abs(heading_error) > heading_tolerance_rad:
        linear_x = 0.0
    else:
        linear_x = min(linear_speed, distance)

    return WaypointCommand(linear_x, angular_z, state, goal_index, False)
