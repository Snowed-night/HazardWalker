"""感知主动复查建议与导航执行器之间的纯数据契约。

所属组：感知定位组 / 导航探索组。
本模块不依赖 ROS，只负责校验感知载荷中的 ``view_recommendation``，避免字段或动作
名称不一致时导航静默忽略候选。实际速度、避障和状态切换仍由导航节点负责。
"""

import math


SUPPORTED_REOBSERVATION_ACTIONS = {
    'turn_left',
    'turn_right',
    'move_left',
    'move_right',
    'move_forward',
    'hold_observation',
}


def parse_reobservation_request(payload):
    """返回规范化复查请求；无请求或不安全动作返回 ``None``。"""

    if not isinstance(payload, dict):
        return None
    recommendation = payload.get('view_recommendation')
    if not isinstance(recommendation, dict):
        return None
    action = str(recommendation.get('action', '')).strip()
    if action == 'continue_exploring':
        return None
    # 兼容旧策略的无方向横移动作，但新感知必须尽量发布明确方向。
    if action == 'move_laterally':
        action = 'move_left'
    if action not in SUPPORTED_REOBSERVATION_ACTIONS:
        return None
    try:
        priority = int(recommendation.get('priority', 0))
    except (TypeError, ValueError):
        priority = 0
    return {
        'action': action,
        'reason': str(recommendation.get('reason', '')).strip(),
        'priority': max(0, min(priority, 100)),
        'target_id': str(recommendation.get('target_id', '')).strip(),
    }


def reobservation_request_is_eligible(
        request, state, attempts_by_target, max_attempts_per_target):
    """检查当前状态与目标预算，防止同一逐帧请求无限重置复查计时器。"""

    if request is None or str(state) != 'EXPLORING':
        return False
    target_id = str(request.get('target_id', '')).strip()
    if not target_id:
        return False
    attempts = int(attempts_by_target.get(target_id, 0))
    return attempts < max(1, int(max_attempts_per_target))


def action_has_scan_clearance(
        action, ranges, angle_min, angle_increment, minimum_clearance_m):
    """按激光扇区检查短时复查动作是否有安全余量。

    横移与转身是主动视角最容易撞墙的环节。这里采用保守门禁：无新鲜扫描、扇区
    没有有效样本或任一样本距离不足时都拒绝运动；``inf`` 表示该方向无量程内障碍。
    """

    action = str(action)
    if action == 'hold_observation':
        return True
    if not ranges or not math.isfinite(float(angle_increment)) or angle_increment == 0.0:
        return False

    sectors = {
        'move_forward': (-35.0, 35.0),
        'move_left': (55.0, 125.0),
        'move_right': (-125.0, -55.0),
        # 原地转向仍会扫过 A1 的机体外接圆，因此检查全周。
        'turn_left': (-180.0, 180.0),
        'turn_right': (-180.0, 180.0),
    }
    if action not in sectors:
        return False

    low_rad = math.radians(sectors[action][0])
    high_rad = math.radians(sectors[action][1])
    selected = []
    for index, value in enumerate(ranges):
        angle = float(angle_min) + index * float(angle_increment)
        if angle < low_rad or angle > high_rad:
            continue
        if value is None:
            continue
        distance = float(value)
        if math.isnan(distance) or distance <= 0.0:
            continue
        selected.append(distance)
    if not selected:
        return False
    clearance = max(0.0, float(minimum_clearance_m))
    # Gazebo ray 偶发单点近场毛刺；单个坏点不应让 10 Hz 控制持续零速。
    # 至少三个相邻量级的危险回波才阻止运动，真实墙面/物体通常覆盖远多于三束。
    blocked_samples = sum(distance < clearance for distance in selected)
    required_blocked_samples = 3 if len(selected) >= 3 else 1
    return blocked_samples < required_blocked_samples
