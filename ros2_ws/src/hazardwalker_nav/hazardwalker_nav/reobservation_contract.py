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
    request = {
        'action': action,
        'reason': str(recommendation.get('reason', '')).strip(),
        'priority': max(0, min(priority, 100)),
        # 感知在首帧会使用 ``untracked:<id>``，轨迹建立后改用 ``<id>``。
        # 两者必须共用一次复查预算，否则同一目标可绕过最大尝试次数。
        'target_id': _canonical_target_id(recommendation.get('target_id')),
    }
    raw_target_id = str(recommendation.get('target_id') or '').strip()
    if raw_target_id.startswith('untracked:'):
        request['target_was_untracked'] = True
    try:
        required_bearing_change_deg = float(
            payload.get('required_min_view_bearing_span_deg')
        )
    except (TypeError, ValueError):
        required_bearing_change_deg = None
    if (required_bearing_change_deg is not None
            and math.isfinite(required_bearing_change_deg)
            and 0.0 < required_bearing_change_deg < 180.0):
        request['required_bearing_change_deg'] = required_bearing_change_deg

    target_id = request['target_id']
    detections = payload.get('detections_2d')
    if isinstance(detections, list):
        detection = find_target_detection(
            payload,
            target_id,
            allow_untracked_upgrade=bool(
                request.get('target_was_untracked', False)
            ),
        )
        if detection is not None:
            try:
                bearing_deg = float(detection.get('view_bearing_deg'))
            except (TypeError, ValueError):
                bearing_deg = None
            if bearing_deg is not None and math.isfinite(bearing_deg):
                request['view_bearing_deg'] = bearing_deg
            position = detection.get('localized_position')
            if (isinstance(position, (list, tuple)) and len(position) == 3):
                try:
                    normalized_position = [
                        float(position[0]), float(position[1]), float(position[2]),
                    ]
                except (TypeError, ValueError):
                    normalized_position = None
                if (normalized_position is not None
                        and all(math.isfinite(value) for value in normalized_position)):
                    request['target_position'] = normalized_position
    return request


def reobservation_request_is_eligible(
        request, state, attempts_by_target, max_attempts_per_target):
    """检查当前状态与目标预算，防止同一逐帧请求无限重置复查计时器。"""

    if request is None or str(state) != 'EXPLORING':
        return False
    target_id = _canonical_target_id(request.get('target_id'))
    if not target_id:
        return False
    attempts = max(
        (
            int(value)
            for key, value in attempts_by_target.items()
            if _canonical_target_id(key) == target_id
        ),
        default=0,
    )
    return attempts < max(1, int(max_attempts_per_target))


def bearing_change_deg(first_bearing_deg, second_bearing_deg):
    """返回两个世界视线方位的最小夹角，处理 ±180° 环绕。"""

    try:
        first = float(first_bearing_deg)
        second = float(second_bearing_deg)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(first) or not math.isfinite(second):
        return None
    delta_rad = math.atan2(
        math.sin(math.radians(second - first)),
        math.cos(math.radians(second - first)),
    )
    return abs(math.degrees(delta_rad))


def find_target_detection(
        payload, target_id, allow_untracked_upgrade=False):
    """从当前感知载荷中找到同一目标的检测框。

    已建立轨迹后默认只接受精确 ``track_id``，不能再把新的
    ``untracked:<id>`` 仅凭数字后缀当作原目标。只有复查由未跟踪首帧触发时，
    调用方才可临时允许一次 ``untracked:1 -> 1`` 的轨迹升级。
    """

    if not isinstance(payload, dict):
        return None
    detections = payload.get('detections_2d')
    if not isinstance(detections, list):
        return None
    expected = str(target_id or '').strip()
    if not expected:
        return None
    expected_canonical = _canonical_target_id(expected)
    return next(
        (
            item for item in detections
            if isinstance(item, dict)
            and (
                expected in _detection_identities(item)
                or (
                    bool(allow_untracked_upgrade)
                    and expected_canonical in {
                        _canonical_target_id(value)
                        for value in _detection_identities(item)
                    }
                )
            )
        ),
        None,
    )


def target_centered_in_image(
        detection, image_width, center_tolerance_ratio=0.18):
    """目标框进入图像中央带时返回真，供转向视觉伺服提前停车。"""

    if not isinstance(detection, dict):
        return False
    bbox = detection.get('bbox')
    if not isinstance(bbox, dict):
        return False
    try:
        width = float(image_width)
        x_min = float(bbox.get('x_min'))
        x_max = float(bbox.get('x_max'))
        tolerance = float(center_tolerance_ratio)
    except (TypeError, ValueError):
        return False
    if (not all(math.isfinite(value) for value in (
            width, x_min, x_max, tolerance))
            or width <= 0.0 or x_max < x_min):
        return False
    center_x = 0.5 * (x_min + x_max)
    tolerance_px = width * max(0.02, min(0.45, tolerance))
    return abs(center_x - 0.5 * width) <= tolerance_px


def reobservation_actions_conflict(current_action, recommended_action):
    """实时建议与当前机动方向相反时返回真。"""

    opposites = {
        'move_left': 'move_right',
        'move_right': 'move_left',
        'turn_left': 'turn_right',
        'turn_right': 'turn_left',
    }
    current = str(current_action or '').strip()
    recommended = str(recommended_action or '').strip()
    return opposites.get(current) == recommended


def find_target_status(payload, target_id):
    """返回同一目标的 confirmed/rejected 等轨迹状态。"""

    if not isinstance(payload, dict):
        return ''
    hazards = payload.get('hazards')
    if not isinstance(hazards, list):
        return ''
    for item in hazards:
        if not isinstance(item, dict):
            continue
        identities = [item.get('id'), item.get('track_id')]
        aliases = item.get('candidate_ids', [])
        if isinstance(aliases, (list, tuple)):
            identities.extend(aliases)
        if any(_same_target_id(value, target_id) for value in identities):
            return str(item.get('status', '')).strip()
    return ''


def _canonical_target_id(value):
    """统一首帧未跟踪 ID 与后续轨迹 ID。"""

    text = str(value or '').strip()
    return text.split(':', 1)[1] if text.startswith('untracked:') else text


def _same_target_id(left, right):
    normalized_left = _canonical_target_id(left)
    normalized_right = _canonical_target_id(right)
    return bool(normalized_left and normalized_left == normalized_right)


def _detection_identity(detection):
    """优先返回显式轨迹 ID；没有轨迹字段时才退回帧内检测 ID。"""

    if not isinstance(detection, dict):
        return ''
    track_id = str(detection.get('track_id') or '').strip()
    if track_id:
        return track_id
    return str(detection.get('id') or '').strip()


def _detection_identities(detection):
    """返回轨迹 ID、帧内 ID 和感知短时候选别名。"""

    if not isinstance(detection, dict):
        return set()
    track_id = str(detection.get('track_id') or '').strip()
    # 一旦有显式 track_id，帧内 ``id`` 只是显示序号，不能绕过精确轨迹门禁。
    values = [track_id or detection.get('id'), detection.get('candidate_id')]
    aliases = detection.get('candidate_aliases', [])
    if isinstance(aliases, (list, tuple)):
        values.extend(aliases)
    return {
        str(value).strip() for value in values
        if str(value or '').strip()
    }


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
