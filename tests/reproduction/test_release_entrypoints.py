"""验证外部资源的整批导入保护，以及结果核验对假完成、缺失评估的拒绝。"""
from pathlib import Path
import hashlib
import importlib.util
import json
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

assets = load('release_assets', ROOT / 'scripts/reproduction/prepare_assets.py')
results = load('release_results', ROOT / 'scripts/reproduction_check_result.py')

class AssetImportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'repo'
        self.source = Path(self.temp.name) / 'official'
        (self.root / 'reproduction').mkdir(parents=True)
        self.source.mkdir()

    def manifest(self, mapping):
        (self.root / 'reproduction/runtime-assets.json').write_text(json.dumps(mapping), encoding='utf-8')

    def make_source(self, rel, data):
        path = self.source / rel
        path.parent.mkdir(exist_ok=True, parents=True)
        path.write_bytes(data)
        return hashlib.sha256(data).hexdigest()

    def test_import_is_idempotent(self):
        sha = self.make_source('src/policy.pt', b'official-test-resource')
        self.manifest({'src/policy.pt': sha})
        self.assertEqual(assets.prepare(self.root, self.source), (1, 1))
        self.assertEqual(assets.prepare(self.root, None), (0, 1))

    def test_bad_batch_does_not_partially_write(self):
        sha = self.make_source('src/a.pt', b'a')
        self.manifest({'src/a.pt': sha, 'src/missing.pt': sha})
        with self.assertRaises(ValueError):
            assets.prepare(self.root, self.source)
        self.assertFalse((self.root / 'ros2_ws/src/hazardwalker_platform/src/a.pt').exists())

    def test_existing_other_resource_is_not_overwritten(self):
        sha = self.make_source('src/a.pt', b'new')
        self.manifest({'src/a.pt': sha})
        target = self.root / 'ros2_ws/src/hazardwalker_platform/src/a.pt'
        target.parent.mkdir(parents=True)
        target.write_bytes(b'user-existing-resource')
        with self.assertRaises(ValueError):
            assets.prepare(self.root, self.source)
        self.assertEqual(target.read_bytes(), b'user-existing-resource')

    def test_manifest_cannot_escape_platform(self):
        self.manifest({'../outside.pt': hashlib.sha256(b'a').hexdigest()})
        with self.assertRaises(ValueError):
            assets.prepare(self.root, self.source)

class ResultAcceptanceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name) / 'result'
        # 人工构造最小合法结果，不依赖历史实验归档或服务器路径。
        (self.output / 'navigation').mkdir(parents=True)
        manifest = {
            'status': 'complete', 'final_nav_state': 'FINISHED',
            'scenario_seed': 'unit-test',
            'navigation_acceptance': {'passed': True},
            'slam_physical_alignment': {'accepted': True, 'floor_count': 3},
            'preflight': {'world_from_map': {'x': 0.0, 'y': 0.0}},
        }
        (self.output / 'run_manifest.json').write_text(json.dumps(manifest), encoding='utf-8')
        (self.output / 'evaluation_result.json').write_text(json.dumps({
            'metrics': {'truth_count': 1, 'correct': 1, 'missed': 0, 'false_alarms': 0},
        }), encoding='utf-8')
        rooms = [
            {'floor': f, 'sector': s, 'phase': 'completed'}
            for f in range(3)
            for s in ('near_left', 'near_right', 'far_left', 'far_right')
        ]
        (self.output / 'navigation/room_coverage.jsonl').write_text(
            '\n'.join(json.dumps(row) for row in rooms) + '\n', encoding='utf-8')
        (self.output / 'navigation/trajectory.jsonl').write_text(json.dumps({
            'floor_index': 0, 'official_x': 0.2, 'official_y': 0.0,
        }) + '\n', encoding='utf-8')

    def test_complete_result_has_all_required_evidence(self):
        report = results.evaluate(self.output)
        self.assertTrue(report['passed'], report)

    def test_finished_on_another_floor_is_not_success(self):
        path = self.output / 'navigation/trajectory.jsonl'
        record = json.loads(path.read_text().splitlines()[-1])
        record['floor_index'] = 2
        path.write_text(json.dumps(record) + '\n', encoding='utf-8')
        report = results.evaluate(self.output)
        self.assertFalse(report['passed'])
        self.assertFalse(report['checks']['returned_ground_floor'])

    def test_missing_perception_evaluation_is_not_zero_misses(self):
        (self.output / 'evaluation_result.json').unlink()
        report = results.evaluate(self.output)
        self.assertFalse(report['passed'])
        self.assertFalse(report['checks']['all_hazards_correct'])
        self.assertEqual(report['official_metrics'], {})
