"""动态实验记录汇总离线测试。

所属组：感知定位组 / 测试组。
文件作用：确保没有真值时只统计可验证动态指标，不伪造识别率或定位误差。
"""

import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_perception'))

from hazardwalker_perception.dynamic_detection_records import (
    build_dynamic_summary,
    build_dynamic_testing_record,
)


def test_summary_counts_candidates_localization_and_confirmed_tracks():
    records = [
        {
            'detections_2d': [
                {'confidence': 0.8, 'localization_status': 'localized'},
                {'confidence': 0.6, 'localization_status': 'unlocalized'},
            ],
            'hazards': [{'id': 3, 'status': 'tentative'}],
            'view_recommendation': {'action': 'turn_left'},
        },
        {
            'detections_2d': [{'confidence': 1.0, 'localization_status': 'localized'}],
            'hazards': [{'id': 3, 'status': 'confirmed'}],
            'view_recommendation': {'action': 'hold_observation'},
        },
    ]
    summary = build_dynamic_summary(records)

    assert summary['frame_count'] == 2
    assert summary['frames_with_candidates'] == 2
    assert summary['total_candidate_count'] == 3
    assert summary['localized_candidate_count'] == 2
    assert summary['confirmed_hazard_count'] == 1
    assert summary['average_confidence'] == 0.8
    assert summary['view_action_counts'] == {'hold_observation': 1, 'turn_left': 1}
    assert summary['ground_truth_available'] is False


def test_testing_record_leaves_truth_dependent_metrics_empty():
    summary = build_dynamic_summary([])
    row = build_dynamic_testing_record(summary, scenario='dynamic_smoke')

    assert row['scenario'] == 'dynamic_smoke'
    assert row['dynamic'] is True
    assert row['truth_count'] == ''
    assert row['missed_count'] == ''
    assert row['false_positive_count'] == ''
