"""官方多层楼宇返航的可测试几何与速度计算。

本模块只处理当前位姿、走廊中心线和任务终点，不读取 ROS、Gazebo 真值文件或
固定房间坐标。上层可使用平台公开的控制里程计生成短距离目标，再交给赛事随附
的 move_base/DWA；DWA 暂时无速度时也复用同一目标，避免追踪虚拟地图点。
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PlanarVelocity:
    """面向目标点的机体系速度与剩余几何量。"""

    linear_x: float
    angular_z: float
    distance_m: float
    heading_error_rad: float


def staged_corridor_goal(
        current_x, current_y, final_x, final_y, final_yaw,
        corridor_center_x=0.0, lookahead_m=3.0,
        lateral_stage_threshold_m=0.55,
        longitudinal_final_threshold_m=1.0):
    """将远距离返航拆成“回中线→短前视→最终点”三个阶段。

    rolling costmap 只能规划有限范围。直接下发几十米外电梯会让 DWA 接受目标却
    无法生成速度，因此长走廊始终只给出不超过 ``lookahead_m`` 的局部目标。
    """

    values = (
        current_x, current_y, final_x, final_y, final_yaw,
        corridor_center_x, lookahead_m,
        lateral_stage_threshold_m, longitudinal_final_threshold_m,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('return goal inputs must be finite')
    lookahead = max(0.25, float(lookahead_m))
    lateral_threshold = max(0.05, float(lateral_stage_threshold_m))
    final_threshold = max(0.05, float(longitudinal_final_threshold_m))
    current_x = float(current_x)
    current_y = float(current_y)
    final_x = float(final_x)
    final_y = float(final_y)
    center_x = float(corridor_center_x)

    if (abs(current_y - final_y) > final_threshold
            and abs(current_x - center_x) > lateral_threshold):
        return (
            center_x,
            current_y,
            math.atan2(0.0, center_x - current_x),
        )

    longitudinal_delta = final_y - current_y
    if abs(longitudinal_delta) > final_threshold:
        step = math.copysign(min(abs(longitudinal_delta), lookahead),
                             longitudinal_delta)
        target_y = current_y + step
        return (
            center_x,
            target_y,
            math.atan2(step, center_x - current_x),
        )
    return final_x, final_y, float(final_yaw)


def planar_velocity_to_goal(
        current_x, current_y, current_yaw, target_x, target_y,
        linear_speed, minimum_linear_speed, angular_speed,
        minimum_turn_speed, heading_tolerance_rad):
    """计算面向目标的速度；避障仍由上层 DWA/激光门禁负责。"""

    values = (
        current_x, current_y, current_yaw, target_x, target_y,
        linear_speed, minimum_linear_speed, angular_speed,
        minimum_turn_speed, heading_tolerance_rad,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('return velocity inputs must be finite')
    dx = float(target_x) - float(current_x)
    dy = float(target_y) - float(current_y)
    distance = math.hypot(dx, dy)
    target_yaw = math.atan2(dy, dx) if distance > 1e-9 else float(current_yaw)
    heading_error = math.atan2(
        math.sin(target_yaw - float(current_yaw)),
        math.cos(target_yaw - float(current_yaw)),
    )
    max_angular = abs(float(angular_speed))
    angular = max(-max_angular, min(max_angular, heading_error))
    minimum_turn = min(max_angular, abs(float(minimum_turn_speed)))
    if 0.0 < abs(angular) < minimum_turn:
        angular = math.copysign(minimum_turn, angular)

    linear = 0.0
    if abs(heading_error) <= max(0.0, float(heading_tolerance_rad)):
        maximum_linear = abs(float(linear_speed))
        linear = min(maximum_linear, distance)
        minimum_linear = min(maximum_linear, abs(float(minimum_linear_speed)))
        if 0.0 < linear < minimum_linear:
            linear = minimum_linear
    return PlanarVelocity(linear, angular, distance, heading_error)
