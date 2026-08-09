"""跨多 SEED 感知回放完整矩阵比较离线测试。"""

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))
SCRIPT = SCRIPTS_DIR / 'compare_perception_replay_campaign.py'
SPEC = importlib.util.spec_from_file_location('compare_campaign', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _run(root, seed, variant, *, true_positive, false_positive,
         false_negative, git_commit='abc'):
    run = root / f'seed_{seed}_{variant}'
    run.mkdir()
    source = f'/data/seed_{seed}'
    bag_fingerprint = hashlib.sha256(source.encode('utf-8')).hexdigest()
    replay = {
        'status': 'complete',
        'source_session': source,
        'source_bag_fingerprint_sha256': bag_fingerprint,
        'scenario_seed': str(seed),
        'git': {'commit': git_commit, 'dirty': False},
    }
    (run / 'replay_experiment_manifest.json').write_text(
        json.dumps(replay), encoding='utf-8')
    precision = true_positive / (true_positive + false_positive)
    recall = true_positive / (true_positive + false_negative)
    f1_score = 2 * precision * recall / (precision + recall)
    metrics = {
        'true_positive': true_positive,
        'false_positive': false_positive,
        'false_negative': false_negative,
        'precision': precision,
        'recall': recall,
        'f1_score': f1_score,
    }
    evaluation = {
        'evaluation_inputs': {
            'replay_control_contract_verified': True,
            'source_session': source,
            'source_bag_fingerprint_sha256': bag_fingerprint,
            'scenario_seed': str(seed),
            'annotation_file': {'sha256': f'annotation-{seed}'},
        },
        'algorithm_run': {
            'label': variant,
            'git': {'commit': git_commit, 'dirty': False},
            'parameter_files': [{'sha256': f'params-{variant}'}],
        },
        'candidate_metrics': metrics,
        'confirmed_output_metrics': metrics,
        'candidate_to_confirmation_latency_sec': {
            'count': true_positive,
            'mean': 0.5 if variant == 'strong' else 1.0,
            'max': 0.8 if variant == 'strong' else 1.5,
        },
        'localization_error_m': {
            'reference_count': true_positive + false_negative,
            'prediction_count': true_positive,
            'count': true_positive,
            'coverage': recall,
            'mean': 0.1 if variant == 'strong' else 0.3,
            'median': 0.1 if variant == 'strong' else 0.3,
            'max': 0.2 if variant == 'strong' else 0.5,
        },
        'processing_time_ms': {
            'count': 10,
            'mean': 8.0 if variant == 'strong' else 12.0,
            'p95': 10.0 if variant == 'strong' else 15.0,
            'max': 11.0 if variant == 'strong' else 17.0,
        },
        'observed_output_rate_hz': 12.0 if variant == 'strong' else 9.0,
    }
    (run / 'evaluation.json').write_text(
        json.dumps(evaluation), encoding='utf-8')
    return run


def _complete_matrix(root):
    runs = []
    for seed in ('101', '102', '103'):
        runs.append(_run(
            root, seed, 'strong', true_positive=9,
            false_positive=1, false_negative=1))
        runs.append(_run(
            root, seed, 'weak', true_positive=6,
            false_positive=2, false_negative=4))
    return runs


def test_complete_three_seed_matrix_outputs_aggregate_and_worst_case_metrics():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        campaign = MODULE.compare_campaign(_complete_matrix(root))
        assert campaign['seed_count'] == 3
        assert campaign['variant_count'] == 2
        assert campaign['complete_seed_variant_matrix'] is True
        assert campaign['truth_inputs_used_at_runtime'] is False
        best = campaign['variants'][0]
        assert best['algorithm_label'] == 'strong'
        assert best['campaign_rank'] == 1
        assert best['candidate_true_positive'] == 27
        assert best['confirmed_f1_score'] == pytest.approx(0.9)
        assert best['worst_seed_confirmed_f1'] == pytest.approx(0.9)
        assert best['localization_error_count'] == 27
        assert best['localization_error_mean_m'] == pytest.approx(0.1)
        assert len(best['seed_metrics']) == 3

        output = root / 'output'
        MODULE.write_campaign(output, campaign)
        assert (output / 'replay_campaign_comparison.json').is_file()
        assert (output / 'replay_campaign_comparison.csv').is_file()


def test_campaign_rejects_fewer_than_three_seeds():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        runs = _complete_matrix(root)[:4]
        with pytest.raises(ValueError, match='至少需要 3 个不同 SEED'):
            MODULE.compare_campaign(runs)


def test_campaign_rejects_incomplete_or_changed_variant_matrix():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        runs = _complete_matrix(root)
        runs.pop()
        runs.append(_run(
            root, '103', 'other', true_positive=7,
            false_positive=1, false_negative=3))
        with pytest.raises(ValueError, match='算法矩阵不完整'):
            MODULE.compare_campaign(runs)


def test_campaign_rejects_git_version_drift_between_seeds():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        runs = _complete_matrix(root)
        target = next(path for path in runs if path.name == 'seed_103_strong')
        replay_path = target / 'replay_experiment_manifest.json'
        evaluation_path = target / 'evaluation.json'
        replay = json.loads(replay_path.read_text(encoding='utf-8'))
        evaluation = json.loads(evaluation_path.read_text(encoding='utf-8'))
        replay['git']['commit'] = 'different'
        evaluation['algorithm_run']['git']['commit'] = 'different'
        replay_path.write_text(json.dumps(replay), encoding='utf-8')
        evaluation_path.write_text(json.dumps(evaluation), encoding='utf-8')
        with pytest.raises(ValueError, match='算法矩阵不完整'):
            MODULE.compare_campaign(runs)
