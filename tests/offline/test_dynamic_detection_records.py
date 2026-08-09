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
    build_reobservation_episode_metrics,
    build_dynamic_summary,
    build_dynamic_testing_record,
    evidence_detection_display_status,
    select_synchronized_evidence,
)


def test_evidence_display_uses_linked_track_status_without_promoting_candidates():
    assert evidence_detection_display_status({
        'track_status': 'confirmed',
        'requires_reobservation': False,
    }) == 'confirmed'
    assert evidence_detection_display_status({
        'track_status': 'tentative',
        'requires_reobservation': True,
    }) == 'reobserve'
    assert evidence_detection_display_status({
        'track_status': 'tentative',
        'requires_reobservation': False,
    }) == '2d_candidate'
    assert evidence_detection_display_status({
        'track_status': 'rejected_non_sphere',
        'requires_reobservation': True,
    }) == 'rejected_non_sphere'


def test_evidence_pairing_uses_nearest_timestamps_not_latest_arrival():
    images = [
        {'stamp_sec': 10.00, 'data': 'matching-rgb'},
        {'stamp_sec': 10.20, 'data': 'newer-but-wrong-rgb'},
    ]
    depths = [
        {'stamp_sec': 10.01, 'data': 'matching-depth'},
        {'stamp_sec': 10.22, 'data': 'newer-depth'},
    ]
    result = select_synchronized_evidence(
        10.02, images, depths, 0.12, 0.05)
    assert result['image']['data'] == 'matching-rgb'
    assert result['depth']['data'] == 'matching-depth'
    assert abs(result['detection_rgb_delta_sec'] - 0.02) < 1e-9
    assert abs(result['rgb_depth_delta_sec'] - 0.01) < 1e-9


def test_evidence_pairing_fails_closed_for_stale_rgb_and_drops_stale_depth():
    assert select_synchronized_evidence(
        2.0, [{'stamp_sec': 1.0}], [], 0.1, 0.1) is None
    result = select_synchronized_evidence(
        2.0,
        [{'stamp_sec': 2.0, 'data': 'rgb'}],
        [{'stamp_sec': 1.7, 'data': 'stale-depth'}],
        0.1,
        0.1,
    )
    assert result['image']['data'] == 'rgb'
    assert result['depth'] is None
    assert result['rgb_depth_delta_sec'] is None


def test_summary_counts_candidates_localization_and_confirmed_tracks():
    records = [
        {
            'timestamp_sec': 1.0,
            'processing_time_ms': 12.0,
            'camera_stable': False,
            'stable_view_command_stopped': True,
            'stable_view_frame_count': 1,
            'detections_2d': [
                {'confidence': 0.8, 'localization_status': 'localized'},
                {'confidence': 0.6, 'localization_status': 'unlocalized'},
            ],
            'hazards': [{'id': 3, 'status': 'tentative'}],
            'view_recommendation': {'action': 'turn_left'},
        },
        {
            'timestamp_sec': 1.1,
            'processing_time_ms': 20.0,
            'camera_stable': True,
            'stable_view_command_stopped': True,
            'stable_view_frame_count': 4,
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
    assert summary['stable_view_frame_count'] == 1
    assert summary['stopped_command_frame_count'] == 2
    assert summary['max_consecutive_stable_view_frames'] == 4
    assert summary['average_confidence'] == 0.8
    assert summary['view_action_counts'] == {'hold_observation': 1, 'turn_left': 1}
    assert summary['ground_truth_available'] is False
    assert summary['processing_time_ms'] == {
        'count': 2, 'mean': 16.0, 'p95': 20.0, 'max': 20.0,
    }
    assert summary['observed_output_rate_hz'] == 10.0
    row = build_dynamic_testing_record(summary, scenario='dynamic_smoke')
    assert row['stable_view_frame_count'] == 1
    assert row['stopped_command_frame_count'] == 2
    assert row['max_consecutive_stable_view_frames'] == 4


def test_testing_record_leaves_truth_dependent_metrics_empty():
    summary = build_dynamic_summary([])
    row = build_dynamic_testing_record(summary, scenario='dynamic_smoke')

    assert row['scenario'] == 'dynamic_smoke'
    assert row['dynamic'] is True
    assert row['truth_count'] == ''
    assert row['missed_count'] == ''
    assert row['false_positive_count'] == ''
    assert summary['observed_output_rate_hz'] is None


def test_reobservation_episode_records_partial_approach_lateral_and_confirmation():
    records = [
        {
            'timestamp_sec': 10.0,
            'detections_2d': [{
                'id': 'untracked:candidate-1',
                'candidate_id': 'candidate-1',
                'is_partial': True,
                'requires_reobservation': True,
            }],
            'hazards': [],
            'view_recommendation': {
                'action': 'move_forward',
                'target_id': 'untracked:candidate-1',
            },
        },
        {
            'timestamp_sec': 12.0,
            'detections_2d': [{
                'id': '7',
                'track_id': '7',
                'candidate_id': 'candidate-1',
                'requires_reobservation': True,
            }],
            'hazards': [{
                'id': '7',
                'status': 'tentative',
                'candidate_ids': ['candidate-1'],
            }],
            'view_recommendation': {
                'action': 'move_left',
                'target_id': 'candidate-1',
            },
        },
        {
            'timestamp_sec': 14.5,
            'detections_2d': [{
                'id': '7',
                'track_id': '7',
                'candidate_id': 'candidate-1',
            }],
            'hazards': [{
                'id': '7',
                'status': 'confirmed',
                'candidate_ids': ['candidate-1'],
            }],
            'view_recommendation': {
                'action': 'hold_observation',
                'target_id': 'candidate-1',
            },
        },
    ]

    metrics = build_reobservation_episode_metrics(records)

    assert metrics['episode_count'] == 1
    assert metrics['partial_start_count'] == 1
    assert metrics['resolved_confirmed_count'] == 1
    assert metrics['incomplete_count'] == 0
    assert metrics['episodes'][0]['actions'] == [
        'move_forward', 'move_left', 'hold_observation',
    ]
    assert metrics['episodes'][0]['candidate_to_resolution_latency_sec'] == 4.5
    assert metrics['candidate_to_confirmation_latency_sec'] == {
        'count': 1,
        'mean': 4.5,
        'max': 4.5,
    }
    row = build_dynamic_testing_record(
        build_dynamic_summary(records), scenario='partial_confirmed')
    assert row['confirmation_latency_mean_sec'] == 4.5
    assert row['confirmation_latency_max_sec'] == 4.5


def test_reobservation_episode_stays_incomplete_without_track_resolution():
    records = [{
        'timestamp_sec': 3.0,
        'detections_2d': [{
            'id': 'untracked:1',
            'requires_reobservation': True,
        }],
        'hazards': [],
        'view_recommendation': {
            'action': 'turn_right',
            'target_id': 'untracked:1',
        },
    }]

    summary = build_dynamic_summary(records)
    row = build_dynamic_testing_record(summary, scenario='partial_unresolved')

    assert summary['reobservation_episode_count'] == 1
    assert summary['resolved_confirmed_episode_count'] == 0
    assert summary['incomplete_reobservation_episode_count'] == 1
    assert row['incomplete_reobservation_episode_count'] == 1


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
    assert "'schema': 'hazardwalker_perception_official_evidence_v2'" in source
    assert "'evidence_raw_image': raw_image_path" in source
    assert "self.raw_image_dir" in source
    assert "self.annotated_image_dir" in source
    assert "declare_parameter('run_mode', 'internal_regression')" in source
    assert "declare_parameter('depth_topic', '/hw/camera/depth_image')" in source
    assert "declare_parameter('evidence_buffer_size', 20)" in source
    assert "declare_parameter('max_detection_rgb_delta_sec', 0.12)" in source
    assert 'select_synchronized_evidence(' in source
    assert "'evidence_detection_rgb_delta_sec'" in source
    assert "'evidence_rgb_depth_delta_sec'" in source
    assert "and depth_item is None" in source
    assert '不能留下只有 RGB 的半套证据' in source
    assert "declare_parameter('detection_topic', '/hw/perception/hazard_detections')" in source
    assert "declare_parameter('mission_state_topic', '/hw/mission/state')" in source
    assert "'mission_completion_required': True" in source
    assert 'self.mission_completed = True' in source
    assert "'mission_completed': self.mission_completed" in source
    assert "'forbidden_pose_topic' in self.evidence_contract.get('contract_violations', [])" in source
    assert 'result_path.resolve() != result_copy_path.resolve()' in source
    assert "self.output_dir / 'testing_record_perception.json'" in source
    assert "self.output_dir / 'testing_record_perception.csv'" in source
    assert "self.output_dir / 'README.md'" in source
    assert "'formal_task_evidence_eligible'" in source
    assert 'trajectory.jsonl' in source


def test_detector_is_the_single_owner_of_the_view_recommendation_topic():
    perception_package = Path(REPO_ROOT) / 'ros2_ws' / 'src' / (
        'hazardwalker_perception/hazardwalker_perception'
    )
    detector = (perception_package / 'hsv_detector_node.py').read_text(
        encoding='utf-8')
    recorder = (
        perception_package / 'dynamic_detection_recorder_node.py'
    ).read_text(encoding='utf-8')
    assert "'/hw/perception/view_recommendation'" in detector
    assert 'self.recommendation_pub.publish(recommendation_out)' in detector
    assert 'create_publisher(String, \'/hw/perception/view_recommendation\'' not in recorder
