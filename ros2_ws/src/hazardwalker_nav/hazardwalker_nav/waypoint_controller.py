"""固定航点巡检的离线控制函数。

所属组：导航组。
文件作用：
- 提供不依赖 ROS 的航点控制律。
- 让导航组先用纯 Python 测试航点切换、返航状态、角度控制和完成状态。

当前函数职责：
- `normalize_angle`：把角度压到 `[-pi, pi]`，避免航向误差跳变。
- `compute_waypoint_command`：根据当前位姿、航点列表和容差，输出线速度、角速度、状态和目标下标。

后续扩展方式：
- 目前这个文件是「控制决策层」，不是 Nav2 包装层。
- 后续可新增一个 `nav2_goal_adapter.py` 或在 ROS 节点中加入 action client，把这里的航点结果转换成 Nav2 goal。
- 若要做 Frontier 或更复杂导航，可新增 `frontier_controller.py`，复用这里的状态枚举和航点完成判定。

验证方式：
- 用 `tests/offline/test_waypoint_controller.py` 验证朝向对齐前先转向、对准后前进、到达最后航点后进入 `FINISHED`。
- 先确认二维控制逻辑正确，再接 ROS `Odometry` 和 `/hw/cmd_vel`。
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

    说明：
    - 该函数只处理二维平面上的「朝向 -> 前进 -> 结束」逻辑。
    - 当前返回的 `state` 只用于最小 demo；未来若接 Nav2，可以保留这个函数作为航点策略层。
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
