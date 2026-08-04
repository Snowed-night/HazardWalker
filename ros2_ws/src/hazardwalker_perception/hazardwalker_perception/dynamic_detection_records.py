"""动态感知实验记录的纯函数。

所属组：感知定位组 / 测试组。
文件作用：
把连续帧记录汇总为 ``summary.json`` 和测试表需要的动态指标。
当前实现边界：
不计算没有真值标注的识别率、虚警率和定位误差，避免把未知值写成实验结论。
验证方式：
运行 ``python scripts/run_offline_tests.py``，查看 ``test_dynamic_detection_records.py``。
"""

from collections import Counter
import math


_ALLOWED_LOCALIZATION_PROVENANCE = {
    'lidar_imu_slam',
    'lidar_imu_slam+public_floor_action',
    'visual_inertial_slam',
}
_FORBIDDEN_POSE_TOPIC_PARTS = (
    '/odometry_gazebo',
    '/ground_truth',
    '/hw/odom',
)


def build_perception_evidence_contract(
        run_mode, scenario_seed, code_version, legal_pose_topic, localization_provenance):
    """生成正式随机场景证据的 fail-closed 准入结论。

    该函数只审计记录元数据和禁用输入名称，不能替代裁判的运行期审计；它的目的
    是防止受控夹具、未记录 SEED 或 Gazebo 真值里程计被误归档为正式成绩。
    """

    mode = str(run_mode).strip()
    seed = str(scenario_seed).strip()
    version = str(code_version).strip()
    pose_topic = str(legal_pose_topic).strip()
    provenance = str(localization_provenance).strip()
    violations = []
    if mode != 'official_random_scene':
        violations.append('run_mode_not_official_random_scene')
    if not seed:
        violations.append('missing_fixed_scenario_seed')
    if not version:
        violations.append('missing_code_version')
    if not pose_topic:
        violations.append('missing_legal_pose_topic')
    elif any(part in pose_topic.lower() for part in _FORBIDDEN_POSE_TOPIC_PARTS):
        violations.append('forbidden_pose_topic')
    if provenance not in _ALLOWED_LOCALIZATION_PROVENANCE:
        violations.append('unverified_localization_provenance')
    return {
        'run_mode': mode,
        'scenario_seed': seed,
        'code_version': version,
        'legal_pose_topic': pose_topic,
        'localization_provenance': provenance,
        'formal_evidence_eligible': not violations,
        'contract_violations': violations,
        'truth_inputs_used': False,
    }


def build_dynamic_summary(records):
    """从逐帧记录生成不依赖真值的动态识别摘要。"""

    records = list(records)
    detections = [item for record in records for item in record.get('detections_2d', [])]
    recommendations = [record.get('view_recommendation', {}) for record in records]
    confirmed_ids = {
        str(hazard.get('id'))
        for record in records
        for hazard in record.get('hazards', [])
        if hazard.get('status') == 'confirmed'
    }
    confidences = [float(item.get('confidence', 0.0)) for item in detections]
    action_counts = Counter(item.get('action', 'unknown') for item in recommendations)
    processing_times_ms = [
        float(record['processing_time_ms'])
        for record in records
        if _valid_nonnegative(record.get('processing_time_ms'))
    ]
    record_stamps = [_optional_record_stamp_sec(record) for record in records]
    complete_timing = bool(records) and all(
        stamp is not None for stamp in record_stamps)
    observed_duration_sec = (
        max(record_stamps) - min(record_stamps)
        if complete_timing and len(record_stamps) >= 2 else 0.0
    )

    episode_metrics = build_reobservation_episode_metrics(records)
    return {
        'frame_count': len(records),
        'frames_with_candidates': sum(1 for record in records if record.get('detections_2d')),
        'total_candidate_count': len(detections),
        'localized_candidate_count': sum(1 for item in detections if item.get('localization_status') == 'localized'),
        'confirmed_hazard_count': len(confirmed_ids),
        'average_confidence': round(sum(confidences) / len(confidences), 4) if confidences else 0.0,
        'view_action_counts': dict(sorted(action_counts.items())),
        'reobservation_episode_count': episode_metrics['episode_count'],
        'partial_start_episode_count': episode_metrics['partial_start_count'],
        'resolved_confirmed_episode_count': episode_metrics[
            'resolved_confirmed_count'
        ],
        'resolved_rejected_episode_count': episode_metrics[
            'resolved_rejected_count'
        ],
        'incomplete_reobservation_episode_count': episode_metrics[
            'incomplete_count'
        ],
        'candidate_to_confirmation_latency_sec': episode_metrics[
            'candidate_to_confirmation_latency_sec'
        ],
        'processing_time_ms': _timing_summary(processing_times_ms),
        'observed_output_rate_hz': round(
            (len(records) - 1) / observed_duration_sec, 4,
        ) if observed_duration_sec > 0.0 else None,
        'reobservation_episodes': episode_metrics['episodes'],
        'ground_truth_available': False,
        'limitations': [
            '未提供逐帧真值时，不计算识别率、虚警率、定位误差。',
            '三维定位结果需在相机内参、对齐深度和 TF 坐标链均可用时再作结论。',
        ],
    }


def select_synchronized_evidence(
        detection_stamp_sec, image_items, depth_items,
        max_detection_rgb_delta_sec, max_rgb_depth_delta_sec):
    """按时间戳选择检测对应 RGB 和深度，禁止用到达顺序冒充同步。"""

    try:
        detection_stamp = float(detection_stamp_sec)
        max_detection_delta = float(max_detection_rgb_delta_sec)
        max_depth_delta = float(max_rgb_depth_delta_sec)
    except (TypeError, ValueError):
        return None
    if (not math.isfinite(detection_stamp)
            or max_detection_delta < 0.0 or max_depth_delta < 0.0):
        return None

    def valid_items(items):
        result = []
        for item in items:
            if not isinstance(item, dict):
                continue
            try:
                stamp = float(item.get('stamp_sec'))
            except (TypeError, ValueError):
                continue
            if math.isfinite(stamp):
                result.append((stamp, item))
        return result

    images = valid_items(image_items)
    if not images:
        return None
    image_stamp, image = min(
        images,
        key=lambda pair: (abs(pair[0] - detection_stamp), -pair[0]),
    )
    detection_rgb_delta = abs(image_stamp - detection_stamp)
    if detection_rgb_delta > max_detection_delta:
        return None

    depth = None
    depth_stamp = None
    rgb_depth_delta = None
    depths = valid_items(depth_items)
    if depths:
        candidate_stamp, candidate = min(
            depths,
            key=lambda pair: (abs(pair[0] - image_stamp), -pair[0]),
        )
        candidate_delta = abs(candidate_stamp - image_stamp)
        if candidate_delta <= max_depth_delta:
            depth = candidate
            depth_stamp = candidate_stamp
            rgb_depth_delta = candidate_delta
    return {
        'image': image,
        'image_stamp_sec': image_stamp,
        'depth': depth,
        'depth_stamp_sec': depth_stamp,
        'detection_rgb_delta_sec': detection_rgb_delta,
        'rgb_depth_delta_sec': rgb_depth_delta,
    }


def build_reobservation_episode_metrics(records):
    """重建“候选→复查动作→确认/拒绝”闭环，不依赖场景真值。

    连续帧中同一 ``target_id`` 的靠近、转向、横移与停稳观察属于同一 episode。
    只有感知载荷里的实际 ``confirmed``/``rejected`` 轨迹才能关闭 episode；记录
    结束仍未闭环的请求明确计为 incomplete，避免把“发过动作建议”包装成成功。
    """

    episodes = []
    active = None
    for frame_index, record in enumerate(records):
        measured_stamp_sec = _optional_record_stamp_sec(record)
        stamp_sec = (
            measured_stamp_sec
            if measured_stamp_sec is not None else float(frame_index)
        )
        recommendation = record.get('view_recommendation', {})
        action = str(recommendation.get('action', '')).strip()
        target_id = _canonical_target_id(recommendation.get('target_id'))
        has_request = (
            action
            and action not in ('continue_exploring', 'unknown')
            and bool(target_id)
        )

        if has_request and (
            active is None or active['target_id'] != target_id
        ):
            if active is not None:
                active['end_stamp_sec'] = stamp_sec
                episodes.append(active)
            active = {
                'target_id': target_id,
                'start_stamp_sec': stamp_sec,
                'end_stamp_sec': stamp_sec,
                'initial_partial': _target_is_partial(record, target_id),
                'actions': [],
                'outcome': 'incomplete',
                'resolution_stamp_sec': None,
                'candidate_to_resolution_latency_sec': None,
                'timing_valid': measured_stamp_sec is not None,
            }
        if active is not None:
            active['end_stamp_sec'] = stamp_sec
            if has_request and target_id == active['target_id']:
                if not active['actions'] or active['actions'][-1] != action:
                    active['actions'].append(action)
            outcome = _target_resolution(record, active['target_id'])
            if outcome:
                active['outcome'] = outcome
                active['resolution_stamp_sec'] = stamp_sec
                active['timing_valid'] = bool(
                    active['timing_valid']
                    and measured_stamp_sec is not None
                )
                if active['timing_valid']:
                    active['candidate_to_resolution_latency_sec'] = round(
                        max(0.0, stamp_sec - active['start_stamp_sec']), 4,
                    )
                episodes.append(active)
                active = None

    if active is not None:
        episodes.append(active)

    confirmed_latencies = [
        item['candidate_to_resolution_latency_sec']
        for item in episodes
        if (
            item['outcome'] == 'confirmed'
            and item['candidate_to_resolution_latency_sec'] is not None
        )
    ]
    return {
        'episode_count': len(episodes),
        'partial_start_count': sum(
            1 for item in episodes if item['initial_partial']
        ),
        'resolved_confirmed_count': sum(
            1 for item in episodes if item['outcome'] == 'confirmed'
        ),
        'resolved_rejected_count': sum(
            1 for item in episodes if item['outcome'] == 'rejected'
        ),
        'incomplete_count': sum(
            1 for item in episodes if item['outcome'] == 'incomplete'
        ),
        'candidate_to_confirmation_latency_sec': {
            'count': len(confirmed_latencies),
            'mean': (
                round(sum(confirmed_latencies) / len(confirmed_latencies), 4)
                if confirmed_latencies else None
            ),
            'max': round(max(confirmed_latencies), 4)
            if confirmed_latencies else None,
        },
        'episodes': episodes,
    }


def build_dynamic_testing_record(summary, scenario, notes=''):
    """生成一行测试组可合并的感知动态实验记录。"""

    return {
        'scenario': scenario,
        'frame_count': int(summary.get('frame_count', 0)),
        'truth_count': '',
        'detected_count': int(summary.get('confirmed_hazard_count', 0)),
        'missed_count': '',
        'false_positive_count': '',
        'average_confidence': float(summary.get('average_confidence', 0.0)),
        'dynamic': True,
        'frames_with_candidates': int(summary.get('frames_with_candidates', 0)),
        'total_candidate_count': int(summary.get('total_candidate_count', 0)),
        'localized_candidate_count': int(summary.get('localized_candidate_count', 0)),
        'processing_time_mean_ms': summary.get(
            'processing_time_ms', {}).get('mean'),
        'processing_time_p95_ms': summary.get(
            'processing_time_ms', {}).get('p95'),
        'observed_output_rate_hz': summary.get('observed_output_rate_hz'),
        'confirmation_latency_mean_sec': summary.get(
            'candidate_to_confirmation_latency_sec', {}).get('mean'),
        'confirmation_latency_max_sec': summary.get(
            'candidate_to_confirmation_latency_sec', {}).get('max'),
        'view_action_counts': summary.get('view_action_counts', {}),
        'reobservation_episode_count': int(
            summary.get('reobservation_episode_count', 0)
        ),
        'resolved_confirmed_episode_count': int(
            summary.get('resolved_confirmed_episode_count', 0)
        ),
        'incomplete_reobservation_episode_count': int(
            summary.get('incomplete_reobservation_episode_count', 0)
        ),
        'notes': notes or '未提供真值，不计算识别率、漏检率、虚警率和定位误差。',
    }


def _optional_record_stamp_sec(record):
    """读取真实记录时间；缺失时返回 ``None``，耗时指标不得用帧号冒充秒。"""

    try:
        value = float(record.get('timestamp_sec'))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value >= 0.0 else None


def _valid_nonnegative(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(number) and number >= 0.0


def _timing_summary(values):
    """汇总检测处理耗时；缺失时保留 ``None``，不能伪造为 0 ms。"""

    ordered = sorted(float(value) for value in values)
    if not ordered:
        return {'count': 0, 'mean': None, 'p95': None, 'max': None}
    p95_index = max(0, math.ceil(len(ordered) * 0.95) - 1)
    return {
        'count': len(ordered),
        'mean': round(sum(ordered) / len(ordered), 4),
        'p95': round(ordered[p95_index], 4),
        'max': round(ordered[-1], 4),
    }


def _canonical_target_id(value):
    text = str(value or '').strip()
    return text.split(':', 1)[1] if text.startswith('untracked:') else text


def _target_is_partial(record, target_id):
    """判断 episode 首帧目标是否来自局部可见/强制复查候选。"""

    expected = _canonical_target_id(target_id)
    for detection in record.get('detections_2d', []):
        if not isinstance(detection, dict):
            continue
        identities = (
            detection.get('id'),
            detection.get('track_id'),
            detection.get('candidate_id'),
        )
        if expected not in {
            _canonical_target_id(value) for value in identities
        }:
            continue
        return bool(
            detection.get('is_partial')
            or detection.get('requires_reobservation')
        )
    return False


def _target_resolution(record, target_id):
    """返回当前帧同目标的 confirmed/rejected，其他状态返回空串。"""

    expected = _canonical_target_id(target_id)
    for hazard in record.get('hazards', []):
        if not isinstance(hazard, dict):
            continue
        identities = [hazard.get('id'), hazard.get('track_id')]
        aliases = hazard.get('candidate_ids', [])
        if isinstance(aliases, (list, tuple)):
            identities.extend(aliases)
        if expected not in {
            _canonical_target_id(value) for value in identities
        }:
            continue
        status = str(hazard.get('status', '')).strip()
        if status == 'confirmed':
            return 'confirmed'
        if status in ('rejected', 'rejected_non_spherical'):
            return 'rejected'
    return ''
