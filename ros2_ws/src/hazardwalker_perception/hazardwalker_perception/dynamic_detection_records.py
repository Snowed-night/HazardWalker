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

    return {
        'frame_count': len(records),
        'frames_with_candidates': sum(1 for record in records if record.get('detections_2d')),
        'total_candidate_count': len(detections),
        'localized_candidate_count': sum(1 for item in detections if item.get('localization_status') == 'localized'),
        'confirmed_hazard_count': len(confirmed_ids),
        'average_confidence': round(sum(confidences) / len(confidences), 4) if confidences else 0.0,
        'view_action_counts': dict(sorted(action_counts.items())),
        'ground_truth_available': False,
        'limitations': [
            '未提供逐帧真值时，不计算识别率、虚警率、定位误差。',
            '三维定位结果需在相机内参、对齐深度和 TF 坐标链均可用时再作结论。',
        ],
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
        'view_action_counts': summary.get('view_action_counts', {}),
        'notes': notes or '未提供真值，不计算识别率、漏检率、虚警率和定位误差。',
    }
