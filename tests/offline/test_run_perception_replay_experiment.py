"""受控感知回放实验入口离线测试。"""
import importlib.util
import hashlib
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
from unittest.mock import patch

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_perception_replay_experiment.py'
spec = importlib.util.spec_from_file_location('replay_experiment', SCRIPT_PATH)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)

from official_perception_bag import (  # noqa: E402
    REQUIRED_RECORD_TOPICS,
    fingerprint_bag_directory,
)


def _preflight_bytes():
    return json.dumps({
        'passed': True,
        'failures': [],
        'traffic_checked': True,
        'control_source': 'keyboard',
        'expected_localization_provenance': 'lidar_imu_slam',
        'generated_at_utc': '2026-08-03T00:00:00+00:00',
        'git': {'commit': 'abc', 'dirty': False},
    }, sort_keys=True).encode('utf-8')


def test_launch_command_applies_the_same_parameter_file_that_is_audited():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        parameter_file = root / 'perception.yaml'
        output_dir = root / 'results'
        command = module.build_launch_command(
            output_dir=output_dir,
            parameter_file=parameter_file,
            seed='20260803',
            code_version='abc1234',
            recompute_localization=False,
        )
        assert f'perception_parameter_file:={parameter_file}' in command
        assert f'evidence_output_dir:={output_dir}' in command
        assert f'test_record_dir:={output_dir}' in command
        assert 'control_mode:=stopped' in command
        assert 'start_navigation:=false' in command
        assert 'start_slam:=false' in command
        assert 'start_legal_localization:=false' in command


def test_recompute_localization_enables_only_current_localization_stack():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        command = module.build_launch_command(
            output_dir=root / 'results',
            parameter_file=root / 'perception.yaml',
            seed='7',
            code_version='abc',
            recompute_localization=True,
        )
        assert 'start_slam:=true' in command
        assert 'start_legal_localization:=true' in command


def test_segment_preroll_replays_only_latched_legal_contract_topics():
    command = module.build_segment_preroll_command(Path('/tmp/source-bag'))
    assert '/tf_static' in command
    assert '/hazardwalker/slam/localization_provenance' in command
    assert '/hw/camera/image_raw' not in command
    assert '--playback-duration' in command
    assert '--disable-keyboard-controls' in command


def test_stop_process_group_lets_launch_forward_sigint_only_once():
    class FakeProcess:
        pid = 4321
        returncode = None

        def __init__(self):
            self.signals = []

        def poll(self):
            return None

        def send_signal(self, requested_signal):
            self.signals.append(requested_signal)

        def wait(self, timeout):
            assert timeout == 30.0
            return 130

    process = FakeProcess()
    with patch.object(module.os, 'killpg', create=True) as killpg:
        assert module.stop_process_group(process) == 130
    assert process.signals == [module.signal.SIGINT]
    killpg.assert_not_called()


def test_stop_process_group_escalates_to_process_group_after_timeout():
    class FakeProcess:
        pid = 4321
        returncode = None

        def __init__(self):
            self.wait_calls = 0

        def poll(self):
            return None

        def send_signal(self, requested_signal):
            assert requested_signal == module.signal.SIGINT

        def wait(self, timeout):
            self.wait_calls += 1
            if self.wait_calls == 1:
                raise subprocess.TimeoutExpired('launch', timeout)
            assert timeout == 10.0
            return -module.signal.SIGTERM

    process = FakeProcess()
    with patch.object(module.os, 'killpg', create=True) as killpg:
        assert module.stop_process_group(process) == -module.signal.SIGTERM
    killpg.assert_called_once_with(process.pid, module.signal.SIGTERM)


def test_invalid_source_session_is_rejected():
    with tempfile.TemporaryDirectory() as directory:
        session = Path(directory) / 'session'
        (session / 'bag').mkdir(parents=True)
        (session / 'live_chain_preflight.json').write_bytes(b'{}')
        (session / 'run_manifest.json').write_text(json.dumps({
            'status': 'invalid',
            'bag_validation': {'status': 'failed'},
            'bag_relative_path': 'bag',
        }), encoding='utf-8')
        with pytest.raises(ValueError, match='合同无效'):
            module.load_valid_session(session)


def test_focus_diagnostic_source_rejects_non_coverage_contract_errors():
    with tempfile.TemporaryDirectory() as directory:
        session = Path(directory) / 'session'
        (session / 'bag').mkdir(parents=True)
        (session / 'run_manifest.json').write_text(json.dumps({
            'status': 'invalid',
            'truth_inputs_used': True,
            'bag_validation': {'status': 'failed'},
            'bag_relative_path': 'bag',
        }), encoding='utf-8')
        with pytest.raises(ValueError, match='非覆盖类合同错误'):
            module.load_focus_diagnostic_session(session)


def test_focus_diagnostic_source_preserves_coverage_errors():
    with tempfile.TemporaryDirectory() as directory:
        session = Path(directory) / 'session'
        (session / 'bag').mkdir(parents=True)
        manifest = {
            'status': 'invalid',
            'bag_relative_path': 'bag',
            'scenario_seed': '20260805',
        }
        (session / 'run_manifest.json').write_text(
            json.dumps(manifest), encoding='utf-8')
        expected_errors = [
            "status='invalid'",
            'bag_validation.status 不是 passed',
            '巡检运动覆盖门禁未通过',
            '平面路程 2.8 小于最低 8',
            '平面覆盖跨度 1.6 小于最低 3',
        ]
        with patch.object(
                module, 'validate_completed_session_manifest',
                lambda _manifest: expected_errors), patch.object(
                    module, 'validate_session_bag_payload',
                    lambda _bag, _manifest: []):
            observed, errors = module.load_focus_diagnostic_session(session)
        assert observed == manifest
        assert errors == expected_errors


def test_complete_validated_source_session_is_accepted():
    with tempfile.TemporaryDirectory() as directory:
        session = Path(directory) / 'session'
        (session / 'bag').mkdir(parents=True)
        preflight_bytes = _preflight_bytes()
        (session / 'live_chain_preflight.json').write_bytes(preflight_bytes)
        topics = list(REQUIRED_RECORD_TOPICS) + [
            '/hw/control/keyboard_cmd_vel']
        database = sqlite3.connect(str(session / 'bag' / 'part.db3'))
        database.executescript(
            'CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT);'
            'CREATE TABLE messages ('
            'id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER);'
        )
        for topic_id, topic in enumerate(topics, start=1):
            database.execute(
                'INSERT INTO topics(id, name) VALUES (?, ?)',
                (topic_id, topic))
            for timestamp in (1_000_000_000, 121_000_000_000):
                database.execute(
                    'INSERT INTO messages(topic_id, timestamp) VALUES (?, ?)',
                    (topic_id, timestamp))
        database.commit()
        database.close()
        counts = {topic: 2 for topic in topics}
        fingerprint = fingerprint_bag_directory(session / 'bag')
        expected = {
            'status': 'complete',
            'bag_validation': {
                'status': 'passed',
                'message_counts': counts,
                'duration_sec': 120.0,
                'first_timestamp_ns': 1_000_000_000,
                'last_timestamp_ns': 121_000_000_000,
                'content_fingerprint_sha256': fingerprint['sha256'],
                'files': fingerprint['files'],
            },
            'bag_relative_path': 'bag',
            'scenario_seed': '20260803',
            'git': {'commit': 'abc', 'dirty': False},
            'truth_inputs_used': False,
            'localization_provenance': 'lidar_imu_slam',
            'runtime_localization_provenance': 'lidar_imu_slam',
            'live_chain_preflight': {
                'passed': True,
                'sha256': hashlib.sha256(preflight_bytes).hexdigest(),
                'relative_path': 'live_chain_preflight.json',
                'control_source': 'keyboard',
                'expected_localization_provenance': 'lidar_imu_slam',
                'generated_at_utc': '2026-08-03T00:00:00+00:00',
                'git': {'commit': 'abc', 'dirty': False},
            },
            'patrol_coverage': {
                'status': 'passed',
                'minimum_samples': 20,
                'minimum_path_length_m': 8.0,
                'minimum_planar_span_m': 3.0,
                'metrics': {
                    'sample_count': 100,
                    'planar_path_length_m': 12.0,
                    'planar_span_m': 5.0,
                    'vertical_span_m': 2.6,
                },
                'errors': [],
            },
        }
        (session / 'run_manifest.json').write_text(
            json.dumps(expected), encoding='utf-8')
        assert module.load_valid_session(session) == expected


def test_stale_passed_manifest_without_full_topic_contract_is_rejected():
    with tempfile.TemporaryDirectory() as directory:
        session = Path(directory) / 'session'
        (session / 'bag').mkdir(parents=True)
        (session / 'run_manifest.json').write_text(json.dumps({
            'status': 'complete',
            'scenario_seed': '20260803',
            'truth_inputs_used': False,
            'localization_provenance': 'lidar_imu_slam',
            'runtime_localization_provenance': 'lidar_imu_slam',
            'live_chain_preflight': {
                'passed': True,
                'sha256': hashlib.sha256(b'{}').hexdigest(),
                'relative_path': 'live_chain_preflight.json',
                'control_source': 'keyboard',
                'expected_localization_provenance': 'lidar_imu_slam',
            },
            'bag_validation': {
                'status': 'passed',
                'message_counts': {'/hw/camera/image_raw': 100},
                'duration_sec': 120.0,
            },
        }), encoding='utf-8')
        with pytest.raises(ValueError, match='关键话题'):
            module.load_valid_session(session)


def test_historical_localization_requires_explicit_legal_provenance_contract():
    source = {
        'localization_provenance': 'lidar_imu_slam',
        'historical_localization_reuse_eligible': True,
    }
    assert module.resolve_localization_provenance(
        source,
        recompute_localization=False,
        requested_provenance='visual_inertial_slam',
    ) == 'lidar_imu_slam'
    source['historical_localization_reuse_eligible'] = False
    with pytest.raises(ValueError, match='未证明历史'):
        module.resolve_localization_provenance(
            source,
            recompute_localization=False,
            requested_provenance='lidar_imu_slam',
        )


def _write_valid_normalized_outputs(root):
    for name in ('selected_images/raw', 'selected_images/annotated',
                 'selected_depth'):
        (root / name).mkdir(parents=True, exist_ok=True)
    (root / 'frames.jsonl').write_text(json.dumps({
        'localization_ready': True,
        'evidence_image': 'selected_images/annotated/frame_1_annotated.png',
        'evidence_raw_image': 'selected_images/raw/frame_1_raw.png',
        'evidence_depth': 'selected_depth/frame_1.npy',
        'evidence_detection_rgb_delta_sec': 0.02,
        'evidence_rgb_depth_delta_sec': 0.01,
    }) + '\n', encoding='utf-8')
    (root / 'summary.json').write_text(json.dumps({
        'frame_count': 1,
        'trajectory_sample_count': 2,
        'trajectory_file': 'trajectory.jsonl',
        'map_snapshot_file': 'cartographer_map.yaml',
    }), encoding='utf-8')
    (root / 'run_manifest.json').write_text(json.dumps({
        'capture_status': 'closed',
        'evidence_contract': {
            'truth_inputs_used': False,
            'contract_violations': [],
        },
        'evidence_synchronization': {
            'max_detection_rgb_delta_sec': 0.12,
            'max_rgb_depth_delta_sec': 0.15,
        },
    }), encoding='utf-8')
    for name, content in (
        ('testing_record_perception.csv', 'scenario\ncase\n'),
        ('testing_record_perception.json', '{}\n'),
        ('README.md', '# replay\n'),
        ('failure_reasons.json', '{}\n'),
        ('perception_config.yaml', 'hsv:\n  saturation_min: 70\n'),
        ('cartographer_map.yaml', 'image: cartographer_map.pgm\n'),
    ):
        (root / name).write_text(content, encoding='utf-8')
    (root / 'cartographer_map.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
    (root / 'selected_images/raw/frame_1_raw.png').write_bytes(b'raw')
    (root / 'selected_images/annotated/frame_1_annotated.png').write_bytes(
        b'annotated')
    (root / 'selected_depth/frame_1.npy').write_bytes(b'depth')
    (root / 'trajectory.jsonl').write_text(
        '{"x": 0.0, "y": 0.0}\n{"x": 0.1, "y": 0.0}\n',
        encoding='utf-8')
    (root / 'replay_experiment_manifest.json').write_text(json.dumps({
        'parameter_sha256': module._sha256(root / 'perception_config.yaml'),
    }), encoding='utf-8')


def test_normalized_replay_outputs_require_complete_self_contained_evidence():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        _write_valid_normalized_outputs(root)
        assert module.validate_normalized_outputs(root) == []


def test_normalized_replay_outputs_reject_missing_pair_and_localization():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        _write_valid_normalized_outputs(root)
        (root / 'selected_images/annotated/frame_1_annotated.png').unlink()
        summary_path = root / 'summary.json'
        summary = json.loads(summary_path.read_text(encoding='utf-8'))
        summary['trajectory_sample_count'] = 0
        summary_path.write_text(json.dumps(summary), encoding='utf-8')
        failures = module.validate_normalized_outputs(root)
        assert any('轨迹' in item for item in failures)
        assert any('一一配对' in item for item in failures)


def test_normalized_replay_outputs_reject_unsynchronized_evidence_claim():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        _write_valid_normalized_outputs(root)
        frames_path = root / 'frames.jsonl'
        record = json.loads(frames_path.read_text(encoding='utf-8'))
        record['evidence_detection_rgb_delta_sec'] = 0.5
        frames_path.write_text(json.dumps(record) + '\n', encoding='utf-8')
        failures = module.validate_normalized_outputs(root)
        assert any('检测与 RGB 时间不同步' in item for item in failures)


def test_formal_output_must_be_a_child_of_perception_reports():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory).resolve()
        original_root = module.FORMAL_EVIDENCE_ROOT
        module.FORMAL_EVIDENCE_ROOT = root
        try:
            module.validate_formal_output_dir(
                root / 'official_random' / 'run01')
            with pytest.raises(ValueError, match='reports/perception'):
                module.validate_formal_output_dir(
                    root.parent / 'temporary-result')
            with pytest.raises(ValueError, match='根目录'):
                module.validate_formal_output_dir(root)
        finally:
            module.FORMAL_EVIDENCE_ROOT = original_root


def test_dirty_code_can_only_run_as_explicit_external_diagnostic():
    git_state = {
        'commit': 'abc',
        'dirty': True,
        'dirty_entries': ['config/perception.yaml'],
    }
    with pytest.raises(ValueError, match='拒绝未提交'):
        module.validate_git_state_for_run(
            git_state,
            allow_external_output=False,
            allow_dirty_worktree=True,
        )
    with pytest.raises(ValueError, match='拒绝未提交'):
        module.validate_git_state_for_run(
            git_state,
            allow_external_output=True,
            allow_dirty_worktree=False,
        )
    module.validate_git_state_for_run(
        git_state,
        allow_external_output=True,
        allow_dirty_worktree=True,
    )


def test_normalized_outputs_reject_modified_parameter_snapshot():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        _write_valid_normalized_outputs(root)
        (root / 'perception_config.yaml').write_text(
            'hsv:\n  saturation_min: 1\n', encoding='utf-8')
        failures = module.validate_normalized_outputs(root)
        assert any('参数快照' in item for item in failures)


def test_normalized_outputs_reject_unlinked_or_missing_evidence_files():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        _write_valid_normalized_outputs(root)
        frames_path = root / 'frames.jsonl'
        record = json.loads(frames_path.read_text(encoding='utf-8'))
        record['evidence_raw_image'] = '../unrelated.png'
        frames_path.write_text(json.dumps(record) + '\n', encoding='utf-8')
        failures = module.validate_normalized_outputs(root)
        assert any('越出实验目录' in item for item in failures)


def test_labeled_outputs_require_metrics_and_same_annotation_snapshot():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        annotation_hash = 'a' * 64
        (root / 'replay_experiment_manifest.json').write_text(json.dumps({
            'annotation_sha256': annotation_hash,
        }), encoding='utf-8')
        (root / 'evaluation.json').write_text(json.dumps({
            'labeled_frame_count': 4,
            'evaluation_inputs': {
                'replay_control_contract_verified': True,
                'annotation_file': {'sha256': annotation_hash},
            },
            'candidate_metrics': {},
            'confirmed_output_metrics': {},
            'candidate_to_confirmation_latency_sec': {},
            'localization_error_m': {},
            'processing_time_ms': {},
        }), encoding='utf-8')
        for name in (
                'algorithm_run_manifest.json',
                'testing_record_perception_labeled.json'):
            (root / name).write_text('{}\n', encoding='utf-8')
        (root / 'testing_record_perception_labeled.csv').write_text(
            'scenario\ncase\n', encoding='utf-8')
        assert module.validate_labeled_outputs(root) == []
        evaluation = json.loads(
            (root / 'evaluation.json').read_text(encoding='utf-8'))
        evaluation['evaluation_inputs']['annotation_file']['sha256'] = 'b' * 64
        (root / 'evaluation.json').write_text(
            json.dumps(evaluation), encoding='utf-8')
        failures = module.validate_labeled_outputs(root)
        assert any('固化快照' in item for item in failures)
