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
    dedup_distance_m=0.30,
    require_legal_localization=False,
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
        if (require_multiview_sphere_evidence
                and not _has_valid_multiview_sphere_evidence(
                    hazard,
                    allowed_detection_sources=allowed_detection_sources,
                )):
            continue
        # 坐标系必须由上游明确声明；字段缺失不能默认为 world。
        frame_id = str(hazard.get('position_frame_id', ''))
        if frame_id != str(expected_frame):
            # `start` 坐标不能直接冒充 `world` 坐标提交；调用层必须完成 TF/起点变换。
            continue
        position = _validated_position(hazard.get('position'))
        if position is None:
            continue
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

    confirmation_path = str(
        hazard.get('confirmation_path', 'regular_multiview')
    ).strip()
    if confirmation_path == 'regular_multiview':
        expected_status = 'multi_view_sphere_consistent'
        minimum_distinct_views = 3
        minimum_bearing_span_deg = 25.0
    elif confirmation_path == 'strong_rgbd_geometry':
        expected_status = 'strong_rgbd_sphere_geometry_consistent'
        minimum_distinct_views = 2
        minimum_bearing_span_deg = 5.0
        if not _has_valid_strong_rgbd_geometry_summary(hazard):
            return False
    else:
        return False
    if str(hazard.get('evidence_status', '')) != expected_status:
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
    required_distinct_views = max(
        required_distinct_views, minimum_distinct_views,
    )
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
    required_bearing_span_deg = max(
        required_bearing_span_deg, minimum_bearing_span_deg,
    )
    return (
        math.isfinite(bearing_span_deg)
        and math.isfinite(required_bearing_span_deg)
        and bearing_span_deg >= required_bearing_span_deg
    )


def _has_valid_strong_rgbd_geometry_summary(hazard):
    """复核强 RGB-D 路径随轨迹发布的尺寸、轮廓和曲率摘要。"""

    flat_view_ids = _unique_nonempty_strings(hazard.get('flat_view_ids', []))
    if flat_view_ids is None or flat_view_ids:
        return False
    try:
        diameter = float(hazard.get('median_apparent_diameter_m'))
        aspect = float(hazard.get('min_multiview_aspect_ratio'))
        curvature_cv = float(hazard.get('depth_curvature_cv'))
        normalized_curvature = float(
            hazard.get('median_normalized_depth_curvature'),
        )
    except (TypeError, ValueError):
        return False
    values = (diameter, aspect, curvature_cv, normalized_curvature)
    if not all(math.isfinite(value) for value in values):
        return False
    return (
        abs(diameter - 0.30) / 0.30 <= 0.35
        and aspect >= 0.82
        and curvature_cv <= 0.65
        and 0.10 <= normalized_curvature <= 0.30
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


def _distance_m(first, second):
    return math.sqrt(sum((float(first[index]) - float(second[index])) ** 2 for index in range(3)))
