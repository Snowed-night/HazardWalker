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


def strict_room_reobservation_allowed(
        strict_room_inspection_enabled,
        deterministic_route_phase,
        inspection_phase,
        camera_stable=True):
    """判断候选复查是否可在严格房间巡检期间接管运动。

    普通探索继续沿用原有复查行为；严格房间模式只允许在算法选中的观察位
    已经到达、朝向完成并进入 ``CAPTURE`` 后复查。这样候选可触发靠近或侧视，
    但不会在走廊、穿门或前往下一个观察位时抢占导航。
    """

    if not bool(strict_room_inspection_enabled):
        return True
    return (
        str(deterministic_route_phase) == 'room_inspection'
        and str(inspection_phase).upper() == 'CAPTURE'
        and bool(camera_stable)
    )


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
        request, state, attempts_by_target, max_attempts_per_target,
        allow_returning=False):
    """检查当前状态与目标预算，防止同一逐帧请求无限重置复查计时器。

    默认只允许探索阶段复查。正式业务可显式允许返航途中对刚进入视场的目标
    做少量有上限的复查；调用方必须另设更小预算并在完成后恢复返航。
    """

    allowed_states = {'EXPLORING'}
    if bool(allow_returning):
        allowed_states.add('RETURNING')
    if request is None or str(state) not in allowed_states:
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

    error_ratio = target_horizontal_error_ratio(detection, image_width)
    try:
        tolerance = float(center_tolerance_ratio)
    except (TypeError, ValueError):
        return False
    if error_ratio is None or not math.isfinite(tolerance):
        return False
    # error_ratio 以半幅图像为 1.0；原参数以整幅宽度为比例，因此乘 2。
    return abs(error_ratio) <= 2.0 * max(0.02, min(0.45, tolerance))


def target_horizontal_error_ratio(detection, image_width):
    """返回目标中心相对图像中心的归一化水平误差，左负右正。"""

    if not isinstance(detection, dict):
        return None
    bbox = detection.get('bbox')
    if not isinstance(bbox, dict):
        return None
    try:
        width = float(image_width)
        x_min = float(bbox.get('x_min'))
        x_max = float(bbox.get('x_max'))
    except (TypeError, ValueError):
        return None
    if (not all(math.isfinite(value) for value in (width, x_min, x_max))
            or width <= 0.0 or x_max < x_min):
        return None
    half_width = 0.5 * width
    return max(-1.5, min(1.5, (0.5 * (x_min + x_max) - half_width) / half_width))


def lateral_centering_angular_velocity(
        error_ratio, gain, maximum_turn_speed, deadband_ratio=0.05):
    """把目标水平误差转成横移期间的限幅转向速度。"""

    try:
        error = float(error_ratio)
        proportional_gain = float(gain)
        maximum = float(maximum_turn_speed)
        deadband = float(deadband_ratio)
    except (TypeError, ValueError):
        return 0.0
    values = (error, proportional_gain, maximum, deadband)
    if (not all(math.isfinite(value) for value in values)
            or proportional_gain < 0.0 or maximum < 0.0 or deadband < 0.0):
        return 0.0
    if abs(error) <= deadband:
        return 0.0
    # 图像右侧目标需要负角速度右转；左侧目标需要正角速度左转。
    return max(-maximum, min(maximum, -proportional_gain * error))


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


def live_reobservation_action_update_allowed(
        current_action, recommended_action):
    """限制实时建议切换，避免横移与转向在相邻帧之间来回振荡。

    转向把贴边目标带回中央后，可以切换到同侧横移以制造视差。进入横移后，
    执行器已经用连续角速度闭环保持目标居中，因此忽略新的纯转向建议；否则
    候选每次略微离开中央就会 ``move_left -> turn_left -> move_left``，耗尽整段
    机动窗口却没有形成独立侧视位置。相反方向仍由冲突检查负责立即停车。
    """

    current = str(current_action or '').strip()
    recommended = str(recommended_action or '').strip()
    if not current or not recommended or current == recommended:
        return False
    if current in ('move_left', 'move_right') and recommended in (
            'turn_left', 'turn_right'):
        return False
    if current in ('turn_left', 'turn_right') and recommended in (
            'move_left', 'move_right'):
        return current.rsplit('_', 1)[-1] == recommended.rsplit('_', 1)[-1]
    return True


def bounded_planar_pose_increment(
        previous_pose, current_pose, maximum_increment_m):
    """返回可用于复查限幅的逐帧平面位移，异常跳变返回 ``None``。

    SLAM 闭环或地图重定位可能让 ``map -> base`` 在单帧内整体跳变。该变化不是
    机器人真实横移，不能直接计入主动复查的 0.8 m 运动预算。调用方应在丢弃
    跳变后把当前位姿作为下一帧的新锚点，继续累计后续正常增量。
    """

    try:
        previous_x, previous_y = map(float, previous_pose)
        current_x, current_y = map(float, current_pose)
        maximum_increment = float(maximum_increment_m)
    except (TypeError, ValueError):
        return None
    values = (
        previous_x, previous_y, current_x, current_y, maximum_increment,
    )
    if (not all(math.isfinite(value) for value in values)
            or maximum_increment <= 0.0):
        return None
    increment = math.hypot(
        current_x - previous_x,
        current_y - previous_y,
    )
    if increment > maximum_increment:
        return None
    return increment


def select_live_reobservation_update(
        payload, active_target_id, current_action):
    """筛出同一目标的实时动作更新，防止复查期间被其他候选劫持。

    返回值沿用 :func:`parse_reobservation_request` 的结构。动作没有变化、目标
    不一致、字段非法或感知已经要求继续探索时返回 ``None``。方向相反的建议
    仍会返回，由执行器按安全策略停车，而不是直接反向运动。
    """

    request = parse_reobservation_request(payload)
    if request is None:
        return None
    if (not _same_target_id(request.get('target_id'), active_target_id)
            and not _payload_links_target_aliases(
                payload, request.get('target_id'), active_target_id)):
        return None
    if (str(request.get('action') or '').strip()
            == str(current_action or '').strip()):
        return None
    return request


def _payload_links_target_aliases(payload, left_target_id, right_target_id):
    """只在同一检测框显式列出轨迹与候选别名时连接两个目标 ID。"""

    detections = payload.get('detections_2d') if isinstance(payload, dict) else None
    if not isinstance(detections, list):
        return False
    expected = {
        _canonical_target_id(left_target_id),
        _canonical_target_id(right_target_id),
    }
    if '' in expected:
        return False
    for detection in detections:
        identities = {
            _canonical_target_id(value)
            for value in _detection_identities(detection)
        }
        if expected.issubset(identities):
            return True
    return False


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
