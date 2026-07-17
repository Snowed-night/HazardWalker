"""为红球候选规划可审计的侧向重观察相机目标。

本模块不依赖 ROS，也不发布速度。它把“向左/右横移复查”的语义请求变成围绕候选目标的
一组相机平面目标点，并让每一步长度受限。导航层必须在避障、SLAM 和机器人动力学约束下
执行这些目标；只有实际 TF 证明已到达，跟踪器才会把新画面作为独立视角。
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class ReobservationWaypoint:
    """一个面向候选的相机平面目标点。"""

    x: float
    y: float
    yaw_rad: float
    expected_bearing_change_deg: float

    def to_dict(self):
        return {
            'position': [round(float(self.x), 4), round(float(self.y), 4)],
            'yaw_rad': round(float(self.yaw_rad), 6),
            'expected_bearing_change_deg': round(float(self.expected_bearing_change_deg), 3),
        }


@dataclass(frozen=True)
class ReobservationPlan:
    """候选复查的规划结果；空 waypoint 表示执行层不得把它当作移动命令。"""

    action: str
    target_position: Tuple[float, float, float]
    current_radius_m: float
    requested_bearing_change_deg: float
    waypoints: Tuple[ReobservationWaypoint, ...]
    feasible: bool
    reason: str

    def to_dict(self):
        return {
            'action': self.action,
            'target_position': [round(float(value), 4) for value in self.target_position],
            'current_radius_m': round(float(self.current_radius_m), 4),
            'requested_bearing_change_deg': round(float(self.requested_bearing_change_deg), 3),
            'feasible': bool(self.feasible),
            'reason': self.reason,
            'waypoints': [item.to_dict() for item in self.waypoints],
        }


def plan_lateral_reobservation(
        camera_position: Sequence[float],
        target_position: Sequence[float],
        action: str,
        min_bearing_change_deg: float = 25.0,
        max_step_distance_m: float = 0.45,
        min_target_distance_m: float = 0.25,
) -> ReobservationPlan:
    """围绕目标生成左/右侧视路径，避免用前后靠近冒充独立视角。

    目标始终位于圆心。通过沿圆弧旋转相机位置并朝向圆心，最终视线方位必然改变指定角度；
    将弦长拆成不超过 ``max_step_distance_m`` 的若干步，供局部规划器逐段验证可达性。
    """

    camera = _position3(camera_position, 'camera_position')
    target = _position3(target_position, 'target_position')
    normalized_action = str(action).strip().lower()
    if normalized_action not in ('move_left', 'move_right'):
        return _infeasible_plan(
            normalized_action, target, 0.0, min_bearing_change_deg,
            '当前建议不是横移，不能生成伪侧视路径。',
        )
    requested = float(min_bearing_change_deg)
    if not math.isfinite(requested) or requested <= 0.0 or requested >= 180.0:
        raise ValueError('min_bearing_change_deg must be within (0, 180).')
    step_limit = float(max_step_distance_m)
    if not math.isfinite(step_limit) or step_limit <= 0.0:
        raise ValueError('max_step_distance_m must be positive.')

    dx, dy = camera[0] - target[0], camera[1] - target[1]
    radius = math.hypot(dx, dy)
    if radius < float(min_target_distance_m):
        return _infeasible_plan(
            normalized_action, target, radius, requested,
            '候选距离过近，先后退或重新定位，不能安全规划环绕侧视。',
        )

    total_rad = math.radians(requested)
    total_chord = 2.0 * radius * math.sin(total_rad / 2.0)
    steps = max(1, int(math.ceil(total_chord / step_limit)))
    initial_angle = math.atan2(dy, dx)
    # 面向目标时向左横移对应相机相对目标的极角减小；右移则增大。
    signed_total = -total_rad if normalized_action == 'move_left' else total_rad
    waypoints = []
    for index in range(1, steps + 1):
        fraction = index / float(steps)
        angle = initial_angle + signed_total * fraction
        x = target[0] + radius * math.cos(angle)
        y = target[1] + radius * math.sin(angle)
        yaw = math.atan2(target[1] - y, target[0] - x)
        waypoints.append(ReobservationWaypoint(
            x=x,
            y=y,
            yaw_rad=yaw,
            expected_bearing_change_deg=requested * fraction,
        ))
    return ReobservationPlan(
        action=normalized_action,
        target_position=target,
        current_radius_m=radius,
        requested_bearing_change_deg=requested,
        waypoints=tuple(waypoints),
        feasible=True,
        reason='围绕候选生成侧向弧线目标；每段须由导航避障和真实 TF 到达证明。',
    )


def _position3(values: Sequence[float], name: str) -> Tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError('%s must contain exactly three values.' % name)
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError('%s must be finite.' % name)
    return result


def _infeasible_plan(action, target, radius, requested, reason):
    return ReobservationPlan(
        action=action,
        target_position=target,
        current_radius_m=radius,
        requested_bearing_change_deg=float(requested),
        waypoints=tuple(),
        feasible=False,
        reason=reason,
    )
