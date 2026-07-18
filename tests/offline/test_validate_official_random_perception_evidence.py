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
        'mission_completion_required': True,
    })
    _write_json(root / 'summary.json', {
        'evidence_contract': contract,
        'mission_completed': True,
    })
    _write_json(root / 'failure_reasons.json', {'observed_failure_reasons': []})
    (root / 'selected_images').mkdir()
    (root / 'selected_depth').mkdir()
    (root / 'selected_images' / 'frame.png').write_bytes(b'png')
    (root / 'selected_depth' / 'frame.npy').write_bytes(b'npy')
    views = [
        ('front', -15.0, 'spherical'),
        ('center', 0.0, 'unknown'),
        ('right', 15.0, 'spherical'),
    ]
    evidence_fields = {
        'track_id': 'track-1',
        'position': [1.0, 2.0, 0.15],
        'position_frame_id': 'world',
        'source': 'hsv_depth_tf',
        'evidence_status': 'multi_view_sphere_consistent',
        'distinct_view_count': 3,
        'eligible_observation_count': 3,
        'eligible_view_ids': ['front', 'center', 'right'],
        'spherical_view_ids': ['front', 'right'],
        'view_bearing_span_deg': 30.0,
        'required_min_eligible_observations': 3,
        'required_min_distinct_views': 3,
        'required_min_spherical_views': 2,
        'required_min_view_bearing_span_deg': 25.0,
    }
    frames = []
    for frame_index, (view_id, bearing, depth_status) in enumerate(views):
        detection = {
            'id': frame_index + 1,
            'track_id': 'track-1',
            'view_id': view_id,
            'view_bearing_deg': bearing,
            'confirmation_eligible': True,
            'depth_synchronized': True,
            'tf_synchronized': True,
            'localization_status': 'localized',
            'localized_position': [1.0, 2.0, 0.15],
            'depth_shape': {'status': depth_status},
        }
        hazard = dict(evidence_fields)
        hazard['id'] = 'track-1'
        hazard['status'] = 'confirmed' if frame_index == len(views) - 1 else 'tentative'
        frame = {
            'detections_2d': [detection],
            'hazards': [hazard],
            'evidence_image': (
                'selected_images/frame.png' if frame_index == len(views) - 1 else ''
            ),
            'evidence_depth': (
                'selected_depth/frame.npy' if frame_index == len(views) - 1 else ''
            ),
        }
        frames.append(frame)
    (root / 'frames.jsonl').write_text(
        ''.join(json.dumps(frame) + '\n' for frame in frames),
        encoding='utf-8',
    )
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
        frames = [
            json.loads(line)
            for line in (root / 'frames.jsonl').read_text(encoding='utf-8').splitlines()
        ]
        for frame in frames:
            frame['hazards'] = []
        (root / 'frames.jsonl').write_text(
            ''.join(json.dumps(frame) + '\n' for frame in frames),
            encoding='utf-8',
        )

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


def test_validator_rejects_candidate_evidence_without_finished_mission():
    """入口短跑即使有确认目标，也不能冒充完成探索与返航的正式证据。"""
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        result_path = _make_valid_evidence(root)
        summary = json.loads((root / 'summary.json').read_text(encoding='utf-8'))
        summary['mission_completed'] = False
        _write_json(root / 'summary.json', summary)

        report = validate(root, result_path)

        assert report['structural_evidence_complete'] is False
        assert 'mission_not_completed' in report['errors']


def test_validator_rejects_confirmed_label_without_strict_result_evidence():
    """confirmed/evidence_status 标签不能替代 track 的视角、深度和夹角门禁。"""
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        result_path = _make_valid_evidence(root)
        frames = [
            json.loads(line)
            for line in (root / 'frames.jsonl').read_text(encoding='utf-8').splitlines()
        ]
        final_hazard = frames[-1]['hazards'][0]
        final_hazard['eligible_view_ids'] = ['front', 'front', 'front']
        final_hazard['spherical_view_ids'] = ['front']
        final_hazard['distinct_view_count'] = 3
        final_hazard['eligible_observation_count'] = 2
        final_hazard['view_bearing_span_deg'] = 5.0
        (root / 'frames.jsonl').write_text(
            ''.join(json.dumps(frame) + '\n' for frame in frames),
            encoding='utf-8',
        )

        report = validate(root, result_path)

        assert report['structural_evidence_complete'] is False
        assert 'invalid_eligible_view_ids_danger_0_linked_track' in report['errors']
        assert 'insufficient_eligible_observations_danger_0_linked_track' in report['errors']
        assert 'insufficient_spherical_views_danger_0_linked_track' in report['errors']
        assert 'insufficient_bearing_span_danger_0_linked_track' in report['errors']


def test_validator_requires_result_track_to_link_to_frame_hazard_evidence():
    """官方纯坐标结果在逐帧证据中找不到对应 track 时不得通过。"""
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        result_path = _make_valid_evidence(root)
        result = json.loads(result_path.read_text(encoding='utf-8'))
        result['detected_danger_sources'][0]['position'] = [9.0, 9.0, 9.0]
        _write_json(result_path, result)

        report = validate(root, result_path)

        assert report['structural_evidence_complete'] is False
        assert 'danger_not_linked_to_frame_evidence_0' in report['errors']


def test_validator_rejects_result_fields_outside_official_basic_schema():
    """detected_danger_sources 条目必须保持官方仅含 position 的基础格式。"""
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        result_path = _make_valid_evidence(root)
        result = json.loads(result_path.read_text(encoding='utf-8'))
        source = result['detected_danger_sources'][0]
        source['position_frame_id'] = 'world'
        source['source'] = 'red_blob_only'
        source['track_id'] = 'track-1'
        _write_json(result_path, result)

        report = validate(root, result_path)

        assert report['structural_evidence_complete'] is False
        assert 'unexpected_danger_fields_0' in report['errors']


def test_validator_requires_linked_track_world_frame_and_approved_source():
    """只有 world 坐标且来自批准检测链的最终 confirmed track 可关联结果。"""
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        result_path = _make_valid_evidence(root)
        frames = [
            json.loads(line)
            for line in (root / 'frames.jsonl').read_text(encoding='utf-8').splitlines()
        ]
        frames[-1]['hazards'][0]['position_frame_id'] = 'odom'
        frames[-1]['hazards'][0]['source'] = 'red_blob_only'
        (root / 'frames.jsonl').write_text(
            ''.join(json.dumps(frame) + '\n' for frame in frames),
            encoding='utf-8',
        )

        report = validate(root, result_path)

        assert report['structural_evidence_complete'] is False
        assert 'danger_not_linked_to_frame_evidence_0' in report['errors']


def test_validator_does_not_reuse_stale_confirmation_after_track_regresses():
    """同一轨迹后续已回退时，早期 confirmed 快照不能继续支撑正式结果。"""
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        result_path = _make_valid_evidence(root)
        frames = [
            json.loads(line)
            for line in (root / 'frames.jsonl').read_text(encoding='utf-8').splitlines()
        ]
        regressed = json.loads(json.dumps(frames[-1]))
        regressed['hazards'][0]['status'] = 'needs_reobservation'
        frames.append(regressed)
        (root / 'frames.jsonl').write_text(
            ''.join(json.dumps(frame) + '\n' for frame in frames),
            encoding='utf-8',
        )

        report = validate(root, result_path)

        assert report['structural_evidence_complete'] is False
        assert 'danger_not_linked_to_frame_evidence_0' in report['errors']


def test_validator_rejects_reusing_one_track_for_two_result_positions():
    """同一 confirmed track 不得被两个结果位置重复引用。"""
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        result_path = _make_valid_evidence(root)
        result = json.loads(result_path.read_text(encoding='utf-8'))
        result['detected_danger_sources'].append({'position': [1.01, 2.0, 0.15]})
        _write_json(result_path, result)

        report = validate(root, result_path)

        assert report['structural_evidence_complete'] is False
        assert 'danger_track_reused_1' in report['errors']


def test_validator_rebuilds_multiview_gate_from_raw_frame_observations():
    """result 和累计 hazard 都声称通过时，逐帧观测仍必须真实覆盖三个视角。"""
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        result_path = _make_valid_evidence(root)
        frames = [
            json.loads(line)
            for line in (root / 'frames.jsonl').read_text(encoding='utf-8').splitlines()
        ]
        for frame in frames:
            frame['detections_2d'][0]['view_id'] = 'same-view'
            frame['detections_2d'][0]['view_bearing_deg'] = 0.0
            frame['detections_2d'][0]['depth_shape']['status'] = 'unknown'
        (root / 'frames.jsonl').write_text(
            ''.join(json.dumps(frame) + '\n' for frame in frames),
            encoding='utf-8',
        )

        report = validate(root, result_path)

        assert report['structural_evidence_complete'] is False
        assert 'insufficient_frame_views_0' in report['errors']
        assert 'insufficient_frame_spherical_views_0' in report['errors']
        assert 'insufficient_frame_bearing_span_0' in report['errors']
        assert 'track_views_not_found_in_frames_0' in report['errors']
