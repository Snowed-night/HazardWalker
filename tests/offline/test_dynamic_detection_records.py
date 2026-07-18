"""动态实验记录汇总离线测试。

所属组：感知定位组 / 测试组。
文件作用：确保没有真值时只统计可验证动态指标，不伪造识别率或定位误差。
"""

import os
import sys
from pathlib import Path

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_perception'))

from hazardwalker_perception.dynamic_detection_records import (
    build_perception_evidence_contract,
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


def test_formal_evidence_contract_requires_fixed_seed_code_version_and_legal_slam_pose():
    eligible = build_perception_evidence_contract(
        run_mode='official_random_scene',
        scenario_seed='20260715_42',
        code_version='59817d4',
        legal_pose_topic='/hw/slam/odometry',
        localization_provenance='lidar_imu_slam',
    )
    assert eligible['formal_evidence_eligible'] is True
    assert eligible['contract_violations'] == []
    assert eligible['truth_inputs_used'] is False

    multi_floor = build_perception_evidence_contract(
        run_mode='official_random_scene',
        scenario_seed='20260715_42',
        code_version='59817d4',
        legal_pose_topic='/hazardwalker/slam/odometry',
        localization_provenance='lidar_imu_slam+public_floor_action',
    )
    assert multi_floor['formal_evidence_eligible'] is True

    rejected = build_perception_evidence_contract(
        run_mode='internal_regression',
        scenario_seed='',
        code_version='',
        legal_pose_topic='/hw/odom',
        localization_provenance='unverified',
    )
    assert rejected['formal_evidence_eligible'] is False
    assert set(rejected['contract_violations']) >= {
        'run_mode_not_official_random_scene',
        'missing_fixed_scenario_seed',
        'missing_code_version',
        'forbidden_pose_topic',
        'unverified_localization_provenance',
    }


def test_dynamic_recorder_module_has_direct_execution_entrypoint():
    """避免 ``python -m`` 静默退出，确保官方实验能实际落盘记录。"""

    source_path = Path(REPO_ROOT) / 'ros2_ws' / 'src' / 'hazardwalker_perception' / (
        'hazardwalker_perception/dynamic_detection_recorder_node.py'
    )
    source = source_path.read_text(encoding='utf-8')

    assert "if __name__ == '__main__':" in source
    assert "OccupancyGrid, Odometry" in source
    assert "self.declare_parameter('map_topic', '/map')" in source
    assert "self.output_dir / 'cartographer_map.pgm'" in source
    assert "'map_snapshot_file': self._save_map_snapshot()" in source
    assert "declare_parameter('run_mode', 'internal_regression')" in source
    assert "declare_parameter('depth_topic', '/hw/camera/depth_image')" in source
    assert "declare_parameter('detection_topic', '/hw/perception/hazard_detections')" in source
    assert "declare_parameter('mission_state_topic', '/hw/mission/state')" in source
    assert "'mission_completion_required': True" in source
    assert 'self.mission_completed = True' in source
    assert "'mission_completed': self.mission_completed" in source
    assert "'forbidden_pose_topic' in self.evidence_contract.get('contract_violations', [])" in source
    assert 'trajectory.jsonl' in source
