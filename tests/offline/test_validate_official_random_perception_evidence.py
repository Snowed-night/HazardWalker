"""官方随机场景感知证据赛后结构校验的离线测试。"""

import json
import sys
import tempfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'scripts'))

from validate_official_random_perception_evidence import validate


def _write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False), encoding='utf-8')


def _make_valid_evidence(root):
    contract = {
        'formal_evidence_eligible': True,
        'truth_inputs_used': False,
        'scenario_seed': 'seed_42',
        'code_version': 'abc1234',
    }
    _write_json(root / 'run_manifest.json', {
        'schema': 'hazardwalker_perception_official_evidence_v1',
        'evidence_contract': contract,
    })
    _write_json(root / 'summary.json', {'evidence_contract': contract})
    _write_json(root / 'failure_reasons.json', {'observed_failure_reasons': []})
    (root / 'selected_images').mkdir()
    (root / 'selected_depth').mkdir()
    (root / 'selected_images' / 'frame.png').write_bytes(b'png')
    (root / 'selected_depth' / 'frame.npy').write_bytes(b'npy')
    frame = {
        'detections_2d': [{'id': 1}],
        'hazards': [{'id': 1, 'status': 'confirmed'}],
        'evidence_image': 'selected_images/frame.png',
        'evidence_depth': 'selected_depth/frame.npy',
    }
    (root / 'frames.jsonl').write_text(json.dumps(frame) + '\n', encoding='utf-8')
    (root / 'trajectory.jsonl').write_text(json.dumps({'frame_id': 'world'}) + '\n', encoding='utf-8')
    result_path = root / 'detected_danger.json'
    _write_json(result_path, {
        'exploration_time': 599.9,
        'detected_danger_sources': [{'position': [1.0, 2.0, 0.15]}],
    })
    return result_path


def test_validator_accepts_complete_structural_evidence():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        result_path = _make_valid_evidence(root)
        report = validate(root, result_path)

        assert report['structural_evidence_complete'] is True
        assert report['errors'] == []
        assert report['rgb_evidence_count'] == 1
        assert report['depth_evidence_count'] == 1


def test_validator_rejects_missing_confirmation_and_truth_safe_contract():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        result_path = _make_valid_evidence(root)
        manifest = json.loads((root / 'run_manifest.json').read_text(encoding='utf-8'))
        manifest['evidence_contract']['formal_evidence_eligible'] = False
        _write_json(root / 'run_manifest.json', manifest)
        _write_json(root / 'summary.json', {'evidence_contract': manifest['evidence_contract']})
        frame = json.loads((root / 'frames.jsonl').read_text(encoding='utf-8'))
        frame['hazards'] = []
        (root / 'frames.jsonl').write_text(json.dumps(frame) + '\n', encoding='utf-8')

        report = validate(root, result_path)

        assert report['structural_evidence_complete'] is False
        assert 'formal_evidence_contract_rejected' in report['errors']
        assert 'no_confirmed_red_ball' in report['errors']


def test_validator_requires_result_copy_inside_evidence_directory():
    """正式结果必须随场景证据归档，不能只在 SimEnv 临时目录留一个引用。"""
    with tempfile.TemporaryDirectory() as evidence_temporary, tempfile.TemporaryDirectory() as other_temporary:
        root = Path(evidence_temporary)
        result_path = _make_valid_evidence(root)
        external_result = Path(other_temporary) / 'detected_danger.json'
        external_result.write_bytes(result_path.read_bytes())
        report = validate(root, external_result)

        assert report['structural_evidence_complete'] is False
        assert 'result_not_archived_in_evidence_dir' in report['errors']
