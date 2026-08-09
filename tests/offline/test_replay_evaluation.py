"""固定 SEED 回放人工标注指标测试。"""

import hashlib
import importlib.util
import json
import sys
import tempfile
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
PERCEPTION_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
if str(PERCEPTION_SRC) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SRC))

from hazardwalker_perception.replay_evaluation import (  # noqa: E402
    evaluate_labeled_replay,
)


def _load_evaluator_script():
    """加载命令行脚本，验证落盘元数据而不启动 ROS。"""

    path = REPO_ROOT / 'scripts' / 'evaluate_perception_replay.py'
    spec = importlib.util.spec_from_file_location(
        'evaluate_perception_replay_script', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_multi_object_matching_counts_candidates_and_confirmed_separately():
    records = [{
        'detections_2d': [
            {'bbox': {'x_min': 0, 'y_min': 0, 'x_max': 10, 'y_max': 10},
             'track_status': 'confirmed'},
            {'bbox': {'x_min': 20, 'y_min': 20, 'x_max': 30, 'y_max': 30},
             'track_status': 'needs_reobservation'},
        ],
        'hazards': [],
    }]
    annotations = {
        'annotation_provenance': 'manual_image_annotation',
        'frames': [{'record_index': 0, 'objects': [
            {'id': 'ball-1', 'bbox': [0, 0, 10, 10]},
            {'id': 'ball-2', 'bbox': [20, 20, 30, 30]},
        ]}],
    }
    result = evaluate_labeled_replay(records, annotations)
    assert result['candidate_metrics']['recall'] == 1.0
    assert result['confirmed_output_metrics']['true_positive'] == 1
    assert result['confirmed_output_metrics']['false_negative'] == 1


def test_empty_negative_frame_contributes_true_negative_without_fake_precision():
    result = evaluate_labeled_replay(
        [{'detections_2d': [], 'hazards': []}],
        {'annotation_provenance': 'manual_image_annotation',
         'frames': [{'record_index': 0, 'objects': []}]},
    )
    assert result['candidate_metrics']['true_negative_frames'] == 1
    assert result['candidate_metrics']['precision'] == 0.0


def test_confirmed_start_frame_positions_produce_metric_error():
    result = evaluate_labeled_replay(
        [{'detections_2d': [], 'hazards': [{
            'status': 'confirmed', 'position': [1.1, 2.0, 0.0],
            'position_frame_id': 'start',
        }]}],
        {'annotation_provenance': 'public_postrun_reference',
         'frames': [{'record_index': 0, 'objects': [{
             'bbox': [0, 0, 10, 10],
             'world_position': [1.0, 2.0, 0.0],
             'position_frame_id': 'start',
         }]}]},
    )
    assert result['localization_error_m']['count'] == 1
    assert result['localization_error_m']['reference_count'] == 1
    assert result['localization_error_m']['prediction_count'] == 1
    assert result['localization_error_m']['coverage'] == 1.0
    assert result['localization_error_m']['mean'] == 0.1


def test_missing_localization_is_reported_as_zero_coverage_not_zero_error():
    result = evaluate_labeled_replay(
        [{'detections_2d': [], 'hazards': []}],
        {'annotation_provenance': 'public_postrun_reference',
         'frames': [{'record_index': 0, 'objects': [{
             'bbox': [0, 0, 10, 10],
             'world_position': [1.0, 2.0, 0.0],
             'position_frame_id': 'start',
         }]}]},
    )
    location = result['localization_error_m']
    assert location['count'] == 0
    assert location['reference_count'] == 1
    assert location['coverage'] == 0.0
    assert location['mean'] is None


def test_internal_or_gazebo_truth_provenance_is_rejected():
    try:
        evaluate_labeled_replay(
            [{}],
            {'annotation_provenance': 'gazebo_truth',
             'frames': [{'record_index': 0, 'objects': []}]},
        )
    except ValueError:
        pass
    else:
        raise AssertionError('运行期 Gazebo 真值不得进入正式指标链路')


def test_replay_reports_processing_and_candidate_confirmation_latency():
    records = [
        {
            'timestamp_sec': 5.0,
            'processing_time_ms': 10.0,
            'detections_2d': [{
                'bbox': [0, 0, 10, 10],
                'candidate_id': 'candidate-1',
                'requires_reobservation': True,
            }],
            'hazards': [],
            'view_recommendation': {
                'action': 'move_left', 'target_id': 'candidate-1',
            },
        },
        {
            'timestamp_sec': 7.5,
            'processing_time_ms': 20.0,
            'detections_2d': [{
                'bbox': [0, 0, 10, 10],
                'candidate_id': 'candidate-1',
                'track_status': 'confirmed',
            }],
            'hazards': [{
                'id': '8', 'candidate_ids': ['candidate-1'],
                'status': 'confirmed',
            }],
            'view_recommendation': {
                'action': 'hold_observation', 'target_id': 'candidate-1',
            },
        },
    ]
    result = evaluate_labeled_replay(
        records,
        {'annotation_provenance': 'manual_image_annotation',
         'frames': [{'record_index': 1, 'objects': [{
             'bbox': [0, 0, 10, 10],
         }]}]},
    )
    assert result['candidate_to_confirmation_latency_sec']['mean'] == 2.5
    assert result['processing_time_ms']['mean'] == 15.0
    assert result['processing_time_ms']['p95'] == 20.0


def test_algorithm_manifest_records_label_git_and_parameter_hash():
    evaluator = _load_evaluator_script()
    parameter_file = REPO_ROOT / 'config' / 'perception.yaml'
    result = evaluator.build_algorithm_metadata(
        'hsv_depth_tf_baseline', [parameter_file])

    assert result['label'] == 'hsv_depth_tf_baseline'
    assert result['git']['commit']
    assert result['parameter_files'] == [{
        'path': 'config/perception.yaml',
        'sha256': hashlib.sha256(parameter_file.read_bytes()).hexdigest(),
        'size_bytes': parameter_file.stat().st_size,
    }]


def test_evaluation_inputs_verify_actual_replay_label_parameter_and_source():
    evaluator = _load_evaluator_script()
    parameter_file = REPO_ROOT / 'config' / 'perception.yaml'
    algorithm = evaluator.build_algorithm_metadata(
        'controlled_baseline', [parameter_file])
    # 单元测试工作树本身包含待测改动；合同测试显式构造正式干净状态。
    algorithm['git']['dirty'] = False
    algorithm['git']['dirty_entries'] = []
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        frames = root / 'frames.jsonl'
        annotations = root / 'annotations.json'
        frames.write_text('{}\n', encoding='utf-8')
        annotations.write_text('{"frames": []}\n', encoding='utf-8')
        (root / 'replay_experiment_manifest.json').write_text(json.dumps({
            'source_session': '/data/seed-101',
            'source_bag_fingerprint_sha256': 'd' * 64,
            'scenario_seed': '101',
            'algorithm_label': 'controlled_baseline',
            'parameter_sha256': hashlib.sha256(
                parameter_file.read_bytes()).hexdigest(),
            'git': algorithm['git'],
        }), encoding='utf-8')

        inputs = evaluator.build_evaluation_input_metadata(
            frames, annotations, root)

        assert inputs['source_session'] == '/data/seed-101'
        assert inputs['source_bag_fingerprint_sha256'] == 'd' * 64
        assert inputs['scenario_seed'] == '101'
        assert inputs['annotation_file']['sha256'] == hashlib.sha256(
            annotations.read_bytes()).hexdigest()
        assert evaluator.verify_replay_control_contract(algorithm, inputs)


def test_evaluation_rejects_parameter_hash_different_from_actual_replay():
    evaluator = _load_evaluator_script()
    algorithm = evaluator.build_algorithm_metadata(
        'controlled_baseline', [REPO_ROOT / 'config' / 'perception.yaml'])
    inputs = {
        'replay_experiment_manifest_verified': True,
        'replay_algorithm_label': 'controlled_baseline',
        'replay_parameter_sha256': 'different-parameter-hash',
        'source_session': '/data/seed-101',
    }
    with pytest.raises(ValueError, match='参数哈希'):
        evaluator.verify_replay_control_contract(algorithm, inputs)


def test_normalized_outputs_include_candidate_confirmed_and_readme_metrics():
    evaluator = _load_evaluator_script()
    evaluation = evaluate_labeled_replay(
        [{'timestamp_sec': 1.0, 'processing_time_ms': 8.0,
          'detections_2d': [{
              'bbox': [0, 0, 10, 10], 'track_status': 'confirmed',
          }], 'hazards': []}],
        {'annotation_provenance': 'manual_image_annotation',
         'frames': [{'record_index': 0, 'objects': [{
             'bbox': [0, 0, 10, 10],
         }]}]},
    )
    evaluation['algorithm_run'] = evaluator.build_algorithm_metadata(
        'test_algorithm', [REPO_ROOT / 'config' / 'perception.yaml'])
    with tempfile.TemporaryDirectory() as temporary:
        output_dir = Path(temporary)
        evaluator.write_outputs(output_dir, evaluation, 'seed_case')
        row = json.loads((
            output_dir / 'testing_record_perception_labeled.json'
        ).read_text(encoding='utf-8'))[0]
        readme = (output_dir / 'README.md').read_text(encoding='utf-8')

        assert row['candidate_true_positive'] == 1
        assert row['confirmed_true_positive'] == 1
        assert row['candidate_f1_score'] == 1.0
        assert row['confirmed_f1_score'] == 1.0
        assert row['replay_contract_verified'] is False
        assert '人工标注回放评测' in readme
        assert '最终确认 Precision / Recall / F1：1.0 / 1.0 / 1.0' in readme
