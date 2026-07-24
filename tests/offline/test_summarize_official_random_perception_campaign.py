"""官方随机场景多 SEED 活动汇总器的离线回归测试。"""

import hashlib
import importlib.util
import json
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'summarize_official_random_perception_campaign.py'
SPEC = importlib.util.spec_from_file_location('campaign_summary', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _passing_structure_validator(evidence_dir, result_path, require_active):
    del evidence_dir, result_path
    return {
        'structural_evidence_complete': True,
        'active_reobservation_required': require_active,
        'errors': [],
    }


def _write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )


def _make_campaign(root, seeds=('101', '102', '103'), failed_seed=None):
    code_version = 'a' * 40
    config_bytes = b'perception:\n  min_area_px: 200\n'
    config_sha256 = hashlib.sha256(config_bytes).hexdigest()
    runs = []
    for seed in seeds:
        evidence = root / ('seed_' + seed) / 'formal_01'
        evidence.mkdir(parents=True)
        config_path = evidence / 'perception_config.yaml'
        config_path.write_bytes(config_bytes)
        _write_json(evidence / 'run_manifest.json', {
            'evidence_contract': {
                'scenario_seed': seed,
                'code_version': code_version,
                'formal_evidence_eligible': True,
                'truth_inputs_used': False,
            },
        })
        _write_json(evidence / 'summary.json', {'mission_completed': True})
        _write_json(evidence / 'detected_danger.json', {
            'exploration_time': 480.0,
            'detected_danger_sources': [{'position': [1.0, 2.0, 0.3]}],
        })
        structure_ok = seed != failed_seed
        _write_json(evidence / 'independent_post_evaluation.json', {
            'structural_evidence_complete': structure_ok,
            'active_reobservation_required': True,
            'errors': [] if structure_ok else ['no_complete_active_reobservation_episode'],
        })
        _write_json(evidence / 'evaluation_result.json', {
            'metrics': {
                'truth_count': 3,
                'detected_count': 3,
                'correct': 3,
                'missed': 0,
                'false_alarms': 0,
                'exploration_time': 480.0,
                'threshold_used': 1.0,
            },
            'scores': {'technical_objective_total': 37.0},
        })
        (evidence / 'testing_record_perception.csv').write_text(
            'record_id,result\n%s,pass\n' % seed,
            encoding='utf-8',
        )
        _write_json(
            evidence / 'testing_record_perception.json',
            {'record_id': seed, 'result': 'pass'},
        )
        runs.append({
            'seed': seed,
            'evidence_dir': str(evidence.relative_to(root)),
            'test_record_dir': str(evidence.relative_to(root)),
            'config_snapshot': str(config_path.relative_to(root)),
        })
    manifest = {
        'schema': MODULE.SCHEMA,
        'campaign_id': 'official_random_20260730',
        'pre_registered_seeds': list(seeds),
        'code_version': code_version,
        'config_sha256': config_sha256,
        'runs': runs,
    }
    manifest_path = root / 'campaign_manifest.json'
    _write_json(manifest_path, manifest)
    return manifest_path


def test_campaign_accepts_three_pre_registered_passing_seeds():
    with tempfile.TemporaryDirectory() as temporary:
        report = MODULE.summarize_campaign(
            _make_campaign(Path(temporary)),
            structure_validator=_passing_structure_validator,
        )
    assert report['campaign_pass'] is True
    assert report['run_count'] == 3
    assert report['aggregate']['minimum_recall'] == 1.0
    assert report['aggregate']['maximum_false_alarm_rate'] == 0.0


def test_campaign_rejects_cherry_picked_seed_set():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        manifest_path = _make_campaign(root)
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        manifest['runs'].pop()
        _write_json(manifest_path, manifest)
        report = MODULE.summarize_campaign(
            manifest_path,
            structure_validator=_passing_structure_validator,
        )
    assert report['campaign_pass'] is False
    assert 'runs_do_not_exactly_match_pre_registered_seeds' in report['errors']


def test_campaign_rejects_uncommitted_code_label():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        manifest_path = _make_campaign(root)
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        manifest['code_version'] = 'local-uncommitted'
        _write_json(manifest_path, manifest)
        report = MODULE.summarize_campaign(
            manifest_path,
            structure_validator=_passing_structure_validator,
        )
    assert report['campaign_pass'] is False
    assert 'code_version_is_not_a_clean_git_commit' in report['errors']


def test_campaign_fails_when_one_seed_lacks_active_reobservation_evidence():
    with tempfile.TemporaryDirectory() as temporary:
        report = MODULE.summarize_campaign(
            _make_campaign(Path(temporary), failed_seed='102'),
            structure_validator=_passing_structure_validator,
        )
    assert report['campaign_pass'] is False
    assert any(
        error.startswith('seed_102:independent_')
        for error in report['errors']
    )


def test_campaign_rebuilds_structure_validation_instead_of_trusting_archive():
    def failing_validator(evidence_dir, result_path, require_active):
        del evidence_dir, result_path, require_active
        return {
            'structural_evidence_complete': False,
            'active_reobservation_required': True,
            'errors': ['stale_track_evidence'],
        }

    with tempfile.TemporaryDirectory() as temporary:
        report = MODULE.summarize_campaign(
            _make_campaign(Path(temporary)),
            structure_validator=failing_validator,
        )
    assert report['campaign_pass'] is False
    assert any(
        error.endswith(':rebuilt_structure_validation_failed')
        for error in report['errors']
    )
