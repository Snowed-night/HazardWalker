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
