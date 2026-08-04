"""同源感知回放横向比较离线测试。"""
import importlib.util
import hashlib
import json
from pathlib import Path
import sys
import tempfile

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'compare_perception_replay_runs.py'
SPEC = importlib.util.spec_from_file_location('compare_replays', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _run(root, name, source='/data/seed1', annotation='sha-a', f1=0.8):
    run = root / name
    run.mkdir()
    bag_fingerprint = hashlib.sha256(source.encode('utf-8')).hexdigest()
    (run / 'replay_experiment_manifest.json').write_text(json.dumps({
        'status': 'complete',
        'source_session': source,
        'source_bag_fingerprint_sha256': bag_fingerprint,
        'scenario_seed': '101',
        'git': {'commit': 'abc', 'dirty': False},
    }), encoding='utf-8')
    metrics = {
        'true_positive': 8,
        'precision': f1, 'recall': f1, 'f1_score': f1,
        'false_positive': int(f1 < 0.9), 'false_negative': 2,
    }
    (run / 'evaluation.json').write_text(json.dumps({
        'evaluation_inputs': {
            'replay_control_contract_verified': True,
            'source_session': source,
            'source_bag_fingerprint_sha256': bag_fingerprint,
            'scenario_seed': '101',
            'annotation_file': {'sha256': annotation},
        },
        'algorithm_run': {
            'label': name,
            'git': {'commit': 'abc', 'dirty': False},
            'parameter_files': [{'sha256': f'params-{name}'}],
        },
        'candidate_metrics': metrics,
        'confirmed_output_metrics': metrics,
        'candidate_to_confirmation_latency_sec': {
            'count': 2, 'mean': 1.0, 'max': 1.2},
        'localization_error_m': {
            'reference_count': 2, 'prediction_count': 2,
            'coverage': 1.0, 'mean': 0.2, 'median': 0.2, 'max': 0.3},
        'processing_time_ms': {
            'count': 5, 'mean': 10.0, 'p95': 12.0, 'max': 13.0},
        'observed_output_rate_hz': 20.0,
    }), encoding='utf-8')
    return run


def test_same_source_and_annotation_runs_are_ranked_without_claiming_official_score():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        low = _run(root, 'low', f1=0.7)
        high = _run(root, 'high', f1=0.9)
        comparison = MODULE.compare_runs([low, high])
        assert comparison['run_count'] == 2
        assert comparison['runs'][0]['algorithm_label'] == 'high'
        assert comparison['runs'][0]['comparison_rank'] == 1
        assert comparison['runs'][0]['confirmed_true_positive'] == 8
        assert comparison['runs'][0]['confirmation_latency_max_sec'] == 1.2
        assert comparison['runs'][0]['localization_error_max_m'] == 0.3
        assert comparison['runs'][0]['processing_time_max_ms'] == 13.0
        assert comparison['source_bag_fingerprint_sha256']
        assert '不替代官方总分' in comparison['ranking_note']


def test_cross_bag_or_cross_annotation_comparison_is_rejected():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        first = _run(root, 'first')
        second = _run(root, 'second', source='/data/seed2')
        with pytest.raises(ValueError, match='同一 rosbag'):
            MODULE.compare_runs([first, second])


def test_same_path_with_changed_bag_content_fingerprint_is_rejected():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        first = _run(root, 'first')
        second = _run(root, 'second')
        replay_path = second / 'replay_experiment_manifest.json'
        evaluation_path = second / 'evaluation.json'
        replay = json.loads(replay_path.read_text(encoding='utf-8'))
        evaluation = json.loads(evaluation_path.read_text(encoding='utf-8'))
        replay['source_bag_fingerprint_sha256'] = 'e' * 64
        evaluation['evaluation_inputs'][
            'source_bag_fingerprint_sha256'] = 'e' * 64
        replay_path.write_text(json.dumps(replay), encoding='utf-8')
        evaluation_path.write_text(json.dumps(evaluation), encoding='utf-8')
        with pytest.raises(ValueError, match='同一 rosbag'):
            MODULE.compare_runs([first, second])


def test_duplicate_algorithm_and_parameter_variant_is_rejected():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        first = _run(root, 'same')
        second = _run(root, 'other')
        evaluation = json.loads((second / 'evaluation.json').read_text())
        evaluation['algorithm_run'] = json.loads(
            (first / 'evaluation.json').read_text())['algorithm_run']
        (second / 'evaluation.json').write_text(
            json.dumps(evaluation), encoding='utf-8')
        with pytest.raises(ValueError, match='重复算法'):
            MODULE.compare_runs([first, second])


def test_evaluation_and_replay_manifest_source_mismatch_is_rejected():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        run = _run(root, 'mismatch')
        evaluation_path = run / 'evaluation.json'
        evaluation = json.loads(evaluation_path.read_text(encoding='utf-8'))
        evaluation['evaluation_inputs']['source_session'] = '/data/other'
        evaluation_path.write_text(json.dumps(evaluation), encoding='utf-8')
        with pytest.raises(ValueError, match='源会话不一致'):
            MODULE.compare_runs([run])


def test_dirty_or_unversioned_algorithm_is_rejected():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        dirty = _run(root, 'dirty')
        evaluation_path = dirty / 'evaluation.json'
        evaluation = json.loads(evaluation_path.read_text(encoding='utf-8'))
        evaluation['algorithm_run']['git']['dirty'] = True
        evaluation_path.write_text(json.dumps(evaluation), encoding='utf-8')
        with pytest.raises(ValueError, match='未提交代码'):
            MODULE.compare_runs([dirty])

    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        unversioned = _run(root, 'unversioned')
        evaluation_path = unversioned / 'evaluation.json'
        evaluation = json.loads(evaluation_path.read_text(encoding='utf-8'))
        evaluation['algorithm_run']['git']['commit'] = ''
        evaluation_path.write_text(json.dumps(evaluation), encoding='utf-8')
        with pytest.raises(ValueError, match='缺少 Git 提交'):
            MODULE.compare_runs([unversioned])
