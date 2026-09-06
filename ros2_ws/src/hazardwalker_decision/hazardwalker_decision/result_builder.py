"""任务结果构建与官方 SimEnv 危险源结果导出函数。

所属组：决策组 / 测试组。
文件作用：
- 把任务状态、危险源列表和运行统计整理成最终结果字典。
- 给结果写文件和结果检查脚本提供统一的 JSON 结构。

当前函数职责：
- `build_mission_result`：生成可直接写入 JSON 的结果对象，并统一补齐 `confirmed` 计数。
- `build_official_detected_danger_result`：仅导出已确认、坐标系正确且空间去重后的红球位置，
  对齐官方 `results/detected_danger.json` 格式。

后续扩展方式：
- 如果将来结果结构要增加 `mission_time`, `return_pose`, `false_positive_estimate` 等字段，应优先在这里集中改。
- 只要这个函数输出结构稳定，`mission_state_machine_node.py` 和 `scripts/evaluate_result.py` 都能同步复用。

验证方式：
- 用 `tests/offline/test_result_builder.py` 构造确认/未确认危险源，检查 `num_confirmed_hazards` 是否正确。
"""

import math


def build_mission_result(mission_id, status, hazards, duration_sec, return_success=True):
    """构建任务结果字典。

    Args:
        mission_id: 本次任务 ID。
        status: 任务结束状态，例如 FINISHED 或 FAILED。
        hazards: 危险源列表，每个元素是 dict。
        duration_sec: 任务持续时间。
        return_success: 是否成功返航。

    Returns:
        可直接写入 JSON 的 dict。

    说明：
    - `hazards` 里的单项数据应尽量和 `hazardwalker_msgs/Hazard` 保持一致。
    - 当前默认把未显式标记状态的危险源视为 `confirmed`，便于最小 demo 输出结果。
    """

    normalized_hazards = []
    for hazard in hazards:
        item = dict(hazard)
        item.setdefault('status', 'confirmed')
        normalized_hazards.append(item)

    confirmed_count = sum(1 for hazard in normalized_hazards if hazard.get('status') == 'confirmed')
    return {
        'mission_id': mission_id,
        'status': status,
        'hazards': normalized_hazards,
        'metrics': {
            'duration_sec': float(duration_sec),
            'return_success': bool(return_success),
            'num_confirmed_hazards': confirmed_count,
        },
    }


def build_official_detected_danger_result(
    hazards,
    exploration_time_sec,
    expected_frame='world',
    source_frame=None,
    world_from_source=None,
    world_from_source_by_floor=None,
    snap_sphere_height_to_floor=False,
    floor_height_m=2.6,
    sphere_center_height_m=0.15,
    dedup_distance_m=0.30,
    require_legal_localization=False,
    require_sphere_evidence=False,
    require_multiview_sphere_evidence=False,
    allowed_localization_provenance=(
        'lidar_imu_slam',
        'visual_inertial_slam',
        'lidar_imu_slam+public_floor_action',
    ),
    allowed_detection_sources=(
        'hsv_depth_tf',
        'official_ros1_rgbd',
    ),
):
    """构建官方 SimEnv 的最终危险源输出。

    官方评分文件只接受 `exploration_time` 和三维位置数组，且每个提交位置都会
    参与一对一匹配和虚警率统计。因此本函数刻意不导出 tentative、reobserve、
    rejected_non_spherical 等轨迹，也不导出坐标系不明确或不是 world 的位置。

    Args:
        hazards: 感知跟踪器输出的危险源字典列表。
        exploration_time_sec: 从自主探索开始至返航结束的耗时。
        expected_frame: 官方结果要求的坐标系，当前官方仓库为 `world`。
        dedup_distance_m: 对同一球体重复 confirmed 轨迹的保守空间去重距离；官方
            生成器源之间最小间距约 0.65m，0.30m 不会合并两个合法独立源。
        require_legal_localization: 为真时仅导出明确标为合法 SLAM 定位的轨迹，
            防止调试用 Gazebo 真值里程计/TF 混入比赛提交。
            `lidar_imu_slam+public_floor_action` 只表示 SLAM 位姿的楼层高度来自
            主办方公开电梯动作/楼层状态，不包含 Gazebo ground truth。
        require_multiview_sphere_evidence: 为真时仅导出已完成 RGB-D 多视角球面
            一致性确认的轨迹，防止红色圆柱端面或圆锥端面误入最终评分文件。

    Returns:
        严格符合官方 `results/detected_danger.json` 基础格式的 dict。
    """

    duration = float(exploration_time_sec)
    if not math.isfinite(duration) or duration < 0.0:
        raise ValueError('exploration_time_sec must be a finite non-negative number.')
    threshold = float(dedup_distance_m)
    if not math.isfinite(threshold) or threshold < 0.0:
        raise ValueError('dedup_distance_m must be a finite non-negative number.')

    confirmed = []
    for hazard in hazards:
        if str(hazard.get('status', '')) != 'confirmed':
            continue
        if (require_legal_localization
                and str(hazard.get('localization_provenance', 'unverified'))
                not in set(allowed_localization_provenance)):
            continue
        if (require_sphere_evidence
                and not _has_valid_sphere_evidence(
                    hazard,
                    allowed_detection_sources=allowed_detection_sources,
                    require_multiview=require_multiview_sphere_evidence,
                )):
            continue
        if (not require_sphere_evidence
                and require_multiview_sphere_evidence
                and not _has_valid_multiview_sphere_evidence(
                    hazard,
                    allowed_detection_sources=allowed_detection_sources,
                )):
            continue
        # 坐标系必须由上游明确声明；字段缺失不能默认为 world。
        frame_id = str(hazard.get('position_frame_id', ''))
        required_source_frame = (
            str(source_frame) if source_frame is not None
            else str(expected_frame)
        )
        if frame_id != required_source_frame:
            continue
        position = _validated_position(hazard.get('position'))
        if position is None:
            continue
        transform = _validated_planar_transform(world_from_source)
        floor_transform = _floor_transform_for_position(
            position,
            world_from_source_by_floor,
            floor_height_m=float(floor_height_m),
        )
        if floor_transform is not None:
            transform = floor_transform
        if transform is not None:
            position = _transform_planar_position(position, transform)
        if snap_sphere_height_to_floor:
            position = _snap_sphere_height(
                position,
                floor_height_m=float(floor_height_m),
                sphere_center_height_m=float(sphere_center_height_m),
            )
        confirmed.append((
            -float(hazard.get('confidence', 0.0)),
            str(hazard.get('id', '')),
            position,
        ))

    # 先保留置信度高的轨迹，随后对同一球的近邻重复轨迹只输出一次。
    confirmed.sort()
    exported = []
    for _negative_confidence, _track_id, position in confirmed:
        if any(_distance_m(position, item['position']) <= threshold for item in exported):
            continue
        exported.append({'position': [round(value, 4) for value in position]})

    return {
        'exploration_time': round(duration, 3),
        'detected_danger_sources': exported,
    }


def _has_valid_sphere_evidence(
        hazard, allowed_detection_sources, require_multiview=False):
    """按轨迹随附的正式门槛复核球面正证据，不再私自写死三帧。"""

    if require_multiview:
        return _has_valid_multiview_sphere_evidence(
            hazard, allowed_detection_sources)
    if str(hazard.get('evidence_status', '')) not in (
            'single_view_sphere_confirmed',
            'multi_view_sphere_consistent'):
        return False
    if str(hazard.get('source', '')) not in set(allowed_detection_sources):
        return False
    eligible_view_ids = _unique_nonempty_strings(
        hazard.get('eligible_view_ids'))
    spherical_view_ids = _unique_nonempty_strings(
        hazard.get('spherical_view_ids'))
    if eligible_view_ids is None or spherical_view_ids is None:
        return False
    if not set(spherical_view_ids).issubset(set(eligible_view_ids)):
        return False
    distinct_view_count = _validated_integer(
        hazard.get('distinct_view_count'))
    eligible_observation_count = _validated_integer(
        hazard.get('eligible_observation_count'))
    required_observations = _validated_integer(
        hazard.get('required_min_eligible_observations'))
    required_distinct_views = _validated_integer(
        hazard.get('required_min_distinct_views'))
    required_spherical_views = _validated_integer(
        hazard.get('required_min_spherical_views'))
    if any(value is None for value in (
            distinct_view_count, eligible_observation_count,
            required_observations, required_distinct_views,
            required_spherical_views)):
        return False
    return (
        distinct_view_count == len(eligible_view_ids)
        and distinct_view_count >= max(1, required_distinct_views)
        and eligible_observation_count >= max(1, required_observations)
        and len(spherical_view_ids) >= max(1, required_spherical_views)
    )


def formal_navigation_sequence_completed(states):
    """仅接受 Frontier 实际经历探索、返航后结束的有序状态序列。"""

    normalized = [str(value).strip().upper() for value in states]
    try:
        exploring_index = normalized.index('EXPLORING')
        returning_index = normalized.index('RETURNING', exploring_index + 1)
        finished_index = normalized.index('FINISHED', returning_index + 1)
    except ValueError:
        return False
    return finished_index == len(normalized) - 1


def _has_valid_multiview_sphere_evidence(
        hazard, allowed_detection_sources):
    """复核确认轨迹的完整多视角 RGB-D 证据，拒绝只伪造状态标签的记录。"""

    if str(hazard.get('evidence_status', '')) != 'multi_view_sphere_consistent':
        return False
    if str(hazard.get('source', '')) not in set(allowed_detection_sources):
        return False

    eligible_view_ids = _unique_nonempty_strings(
        hazard.get('eligible_view_ids'),
    )
    spherical_view_ids = _unique_nonempty_strings(
        hazard.get('spherical_view_ids'),
    )
    if eligible_view_ids is None or spherical_view_ids is None:
        return False
    if not set(spherical_view_ids).issubset(set(eligible_view_ids)):
        return False

    distinct_view_count = _validated_integer(
        hazard.get('distinct_view_count'),
    )
    eligible_observation_count = _validated_integer(
        hazard.get('eligible_observation_count'),
    )
    required_observations = _validated_integer(
        hazard.get('required_min_eligible_observations'),
    )
    required_distinct_views = _validated_integer(
        hazard.get('required_min_distinct_views'),
    )
    required_spherical_views = _validated_integer(
        hazard.get('required_min_spherical_views'),
    )
    if any(value is None for value in (
            distinct_view_count,
            eligible_observation_count,
            required_observations,
            required_distinct_views,
            required_spherical_views,
    )):
        return False

    # 官方比赛策略的安全下限不可由消息发送者自行调低；轨迹携带的门槛只能提高。
    required_observations = max(required_observations, 3)
    required_distinct_views = max(required_distinct_views, 3)
    required_spherical_views = max(required_spherical_views, 2)
    if distinct_view_count != len(eligible_view_ids):
        return False
    if distinct_view_count < required_distinct_views:
        return False
    if eligible_observation_count < required_observations:
        return False
    if len(spherical_view_ids) < required_spherical_views:
        return False

    try:
        bearing_span_deg = float(hazard.get('view_bearing_span_deg'))
        required_bearing_span_deg = float(
            hazard.get('required_min_view_bearing_span_deg'),
        )
    except (TypeError, ValueError):
        return False
    required_bearing_span_deg = max(required_bearing_span_deg, 25.0)
    return (
        math.isfinite(bearing_span_deg)
        and math.isfinite(required_bearing_span_deg)
        and bearing_span_deg >= required_bearing_span_deg
    )


def _unique_nonempty_strings(value):
    """返回去重后的非空字符串列表；非列表或重复 ID 都视为证据损坏。"""

    if not isinstance(value, (list, tuple)):
        return None
    normalized = [str(item).strip() for item in value]
    if any(not item for item in normalized):
        return None
    if len(set(normalized)) != len(normalized):
        return None
    return normalized


def _validated_integer(value):
    """只接受非负整数，避免 True、浮点截断或字符串宽松转换绕过门槛。"""

    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def _validated_position(value):
    """把可提交的三维位置规范化；无效位置绝不能进入官方结果。"""

    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        position = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return None
    return position if all(math.isfinite(item) for item in position) else None


def _validated_planar_transform(value):
    """校验 ``(world_x, world_y, world_yaw)``；未配置时返回 None。"""

    if value is None:
        return None
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError('world_from_source must contain x, y and yaw.')
    try:
        transform = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise ValueError('world_from_source must be numeric.') from exc
    if not all(math.isfinite(item) for item in transform):
        raise ValueError('world_from_source must be finite.')
    return transform


def _floor_transform_for_position(
        position, transforms_by_floor, floor_height_m):
    """按原始 SLAM 高度选择当前楼层锚点；缺失时由调用方使用全局回退。"""

    if transforms_by_floor is None:
        return None
    if not isinstance(transforms_by_floor, dict):
        raise ValueError('world_from_source_by_floor must be a dict.')
    height = float(floor_height_m)
    if not math.isfinite(height) or height <= 0.0:
        raise ValueError('floor_height_m must be positive and finite.')
    floor_index = max(0, int(round(float(position[2]) / height)))
    value = transforms_by_floor.get(
        floor_index, transforms_by_floor.get(str(floor_index)))
    return _validated_planar_transform(value)


def _transform_planar_position(position, transform):
    """把 source/map 平面坐标转换到 world；z 暂保持原值。"""

    origin_x, origin_y, origin_yaw = transform
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    source_x, source_y, source_z = position
    return (
        origin_x + cosine * source_x - sine * source_y,
        origin_y + sine * source_x + cosine * source_y,
        source_z,
    )


def _snap_sphere_height(position, floor_height_m, sphere_center_height_m):
    """用公开楼层高度和目标半径恢复球心 z，消除二维 SLAM 高度漂移。"""

    if (not math.isfinite(floor_height_m) or floor_height_m <= 0.0
            or not math.isfinite(sphere_center_height_m)
            or sphere_center_height_m < 0.0):
        raise ValueError('floor and sphere heights must be finite and valid.')
    source_x, source_y, source_z = position
    floor_index = max(0, int(round(source_z / floor_height_m)))
    return (
        source_x,
        source_y,
        floor_index * floor_height_m + sphere_center_height_m,
    )


def _distance_m(first, second):
    return math.sqrt(sum((float(first[index]) - float(second[index])) ** 2 for index in range(3)))
