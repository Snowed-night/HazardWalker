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
    allowed_localization_provenance=('lidar_imu_slam', 'visual_inertial_slam'),
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
                and str(hazard.get('evidence_status', '')) != 'multi_view_sphere_consistent'):
            continue
        frame_id = str(hazard.get('position_frame_id', expected_frame))
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
