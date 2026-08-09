"""多 SEED 感知回放矩阵编排器离线测试。"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'run_perception_replay_campaign.py'
SPEC = importlib.util.spec_from_file_location('run_replay_campaign', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _catalog(root: Path) -> dict:
    sessions = []
    for seed in ('101', '102', '103'):
        session = root / f'session_{seed}'
        session.mkdir()
        sessions.append({
            'scenario_seed': seed,
            'session_dir': str(session),
            'validation_status': 'passed',
        })
    return {
        'schema': MODULE.CATALOG_SCHEMA,
        'one_baseline_patrol_per_seed': True,
        'sessions': sessions,
    }


def _plan(root: Path) -> dict:
    baseline = root / 'baseline.yaml'
    candidate = root / 'candidate.yaml'
    baseline.write_text('threshold: 1\n', encoding='utf-8')
    candidate.write_text('threshold: 2\n', encoding='utf-8')
    annotations = {}
    for seed in ('101', '102', '103'):
        path = root / f'annotations_{seed}.json'
        path.write_text('{"frames": []}\n', encoding='utf-8')
        annotations[seed] = str(path)
    return {
        'schema': MODULE.PLAN_SCHEMA,
        'campaign_id': 'official_random_v1',
        'variants': [
            {'algorithm_label': 'baseline', 'parameter_file': str(baseline)},
            {'algorithm_label': 'candidate', 'parameter_file': str(candidate)},
        ],
        'annotations_by_seed': annotations,
        'rate': 1.0,
    }


def test_matrix_covers_every_seed_and_variant_once():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        matrix = MODULE.build_execution_matrix(
            _catalog(root), _plan(root), output_root=root / 'outputs',
            domain_id_start=142)

    assert len(matrix) == 6
    assert {(row['scenario_seed'], row['algorithm_label']) for row in matrix} == {
        (seed, variant)
        for seed in ('101', '102', '103')
        for variant in ('baseline', 'candidate')
    }
    assert [row['domain_id'] for row in matrix] == list(range(142, 148))


def test_matrix_requires_exact_annotation_seed_set():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        plan = _plan(root)
        del plan['annotations_by_seed']['103']
        with pytest.raises(ValueError, match='标注集合'):
            MODULE.build_execution_matrix(
                _catalog(root), plan, output_root=root / 'outputs')


def test_matrix_rejects_duplicate_parameter_snapshot_under_new_label():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        plan = _plan(root)
        plan['variants'][1]['parameter_file'] = plan['variants'][0][
            'parameter_file']
        with pytest.raises(ValueError, match='完全相同的参数快照'):
            MODULE.build_execution_matrix(
                _catalog(root), plan, output_root=root / 'outputs')


def test_relative_annotations_resolve_next_to_campaign_plan():
    """数据盘方案中的相对标注路径不得错误地按仓库根目录解析。"""

    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        plan_dir = root / 'campaign'
        annotation_dir = plan_dir / 'annotations'
        annotation_dir.mkdir(parents=True)
        plan = _plan(root)
        for seed in ('101', '102', '103'):
            annotation = annotation_dir / f'seed_{seed}.json'
            annotation.write_text('{"frames": []}\n', encoding='utf-8')
            plan['annotations_by_seed'][seed] = (
                f'annotations/seed_{seed}.json')

        matrix = MODULE.build_execution_matrix(
            _catalog(root), plan, output_root=root / 'outputs',
            plan_dir=plan_dir)

    assert {Path(row['annotation_file']) for row in matrix} == {
        (annotation_dir / f'seed_{seed}.json').resolve()
        for seed in ('101', '102', '103')
    }


def test_execution_stops_at_first_failed_matrix_item():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        matrix = MODULE.build_execution_matrix(
            _catalog(root), _plan(root), output_root=root / 'outputs')
        calls = []

        class Result:
            returncode = 7

        def fail_once(command, cwd):
            calls.append((command, cwd))
            return Result()

        with pytest.raises(RuntimeError, match='回放失败'):
            MODULE.execute_campaign(
                matrix, output_root=root / 'outputs',
                campaign_id='official_random_v1', runner=fail_once)
        manifest = json.loads((
            root / 'outputs' / 'campaign_execution_manifest.json'
        ).read_text(encoding='utf-8'))

    assert len(calls) == 1
    assert manifest['status'] == 'failed'
    assert manifest['completed_count'] == 0
    assert manifest['failed_item']['returncode'] == 7


def test_resume_preserves_previous_attempt_summary():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        output = root / 'outputs'
        output.mkdir()
        (output / 'campaign_execution_manifest.json').write_text(json.dumps({
            'started_at_utc': '2026-08-09T00:00:00+00:00',
            'finished_at_utc': '2026-08-09T00:01:00+00:00',
            'status': 'failed',
            'completed_count': 0,
            'failed_item': {'reason': 'old failure'},
        }), encoding='utf-8')
        matrix = MODULE.build_execution_matrix(
            _catalog(root), _plan(root), output_root=output)

        class Result:
            returncode = 9

        with pytest.raises(RuntimeError, match='回放失败'):
            MODULE.execute_campaign(
                matrix, output_root=output,
                campaign_id='official_random_v1', resume=True,
                runner=lambda command, cwd: Result())
        manifest = json.loads((
            output / 'campaign_execution_manifest.json'
        ).read_text(encoding='utf-8'))

    assert manifest['attempt_history'] == [{
        'started_at_utc': '2026-08-09T00:00:00+00:00',
        'finished_at_utc': '2026-08-09T00:01:00+00:00',
        'status': 'failed',
        'completed_count': 0,
        'failed_item': {'reason': 'old failure'},
    }]
