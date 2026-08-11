"""多 SEED 人工巡检 rosbag 回归集索引离线测试。"""
import importlib.util
import hashlib
import json
from pathlib import Path
import sqlite3
import sys
import tempfile

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'build_perception_bag_regression_catalog.py'
SPEC = importlib.util.spec_from_file_location('bag_catalog', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

from official_perception_bag import (  # noqa: E402
    REQUIRED_RECORD_TOPICS,
    fingerprint_bag_directory,
)


def _session(root, seed, run_id):
    session = root / run_id
    bag = session / 'bag'
    bag.mkdir(parents=True)
    preflight_bytes = json.dumps({
        'passed': True,
        'failures': [],
        'traffic_checked': True,
        'control_source': 'keyboard',
        'expected_localization_provenance': 'lidar_imu_slam',
        'expected_scenario_seed': str(seed),
        'generated_at_utc': '2026-08-03T00:00:00+00:00',
        'git': {'commit': 'abc', 'dirty': False},
    }, sort_keys=True).encode('utf-8')
    (session / 'live_chain_preflight.json').write_bytes(preflight_bytes)
    topics = list(REQUIRED_RECORD_TOPICS) + [
        '/hw/control/keyboard_cmd_vel']
    database = sqlite3.connect(str(bag / 'part.db3'))
    database.executescript(
        'CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT);'
        'CREATE TABLE messages ('
        'id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER);'
    )
    for topic_id, topic in enumerate(topics, start=1):
        database.execute(
            'INSERT INTO topics(id, name) VALUES (?, ?)', (topic_id, topic))
        for timestamp in (1_000_000_000, 121_000_000_000):
            database.execute(
                'INSERT INTO messages(topic_id, timestamp) VALUES (?, ?)',
                (topic_id, timestamp))
    database.commit()
    database.close()
    required_counts = {topic: 2 for topic in topics}
    fingerprint = fingerprint_bag_directory(bag)
    (session / 'run_manifest.json').write_text(json.dumps({
        'status': 'complete',
        'run_id': run_id,
        'scenario_seed': str(seed),
        'bag_relative_path': 'bag',
        'truth_inputs_used': False,
        'localization_provenance': 'lidar_imu_slam',
        'runtime_localization_provenance': 'lidar_imu_slam',
        'live_chain_preflight': {
            'passed': True,
            'sha256': hashlib.sha256(preflight_bytes).hexdigest(),
            'relative_path': 'live_chain_preflight.json',
            'control_source': 'keyboard',
            'expected_localization_provenance': 'lidar_imu_slam',
            'expected_scenario_seed': str(seed),
            'generated_at_utc': '2026-08-03T00:00:00+00:00',
            'git': {'commit': 'abc', 'dirty': False},
        },
        'adapter_status': {
            phase: {
                'managed_lifecycle': True,
                'lifecycle_container': 'simenv_ros1_hazard_platform',
                'scenario_seed': str(seed),
                'enable_cmd_vel_relay': True,
                'enable_gui_overlay_relay': True,
                'gui_assist_request_topic': '/hazardwalker/gui/assist_request',
                'gui_control_status_topic': '/hazardwalker/gui/control_status',
                'image_throttle_rate_ms': 200,
            }
            for phase in ('start', 'end')
        },
        'historical_localization_reuse_eligible': True,
        'git': {'commit': 'abc', 'dirty': False},
        'bag_validation': {
            'status': 'passed',
            'message_counts': required_counts,
            'duration_sec': 120.0,
            'first_timestamp_ns': 1_000_000_000,
            'last_timestamp_ns': 121_000_000_000,
            'content_fingerprint_sha256': fingerprint['sha256'],
            'files': fingerprint['files'],
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
    }), encoding='utf-8')
    return session


def test_catalog_keeps_one_valid_baseline_per_seed_and_sorts_it():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        third = _session(root, 103, 'run_c')
        second = _session(root, 102, 'run_b')
        first = _session(root, 101, 'run_a')
        catalog = MODULE.build_catalog([third, second, first])
        assert catalog['seed_count'] == 3
        assert catalog['one_baseline_patrol_per_seed'] is True
        assert [row['scenario_seed'] for row in catalog['sessions']] == [
            '101', '102', '103']
        assert catalog['sessions'][0]['bag_size_bytes'] > 0
        assert len(catalog['sessions'][0]['bag_fingerprint_sha256']) == 64
        assert catalog['sessions'][0][
            'runtime_localization_provenance'] == 'lidar_imu_slam'
        assert catalog['sessions'][0][
            'slam_provenance_message_count'] == 2


def test_duplicate_seed_is_rejected_instead_of_cherry_picking_a_patrol():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        first = _session(root, 101, 'run_a')
        second = _session(root, 101, 'run_b')
        with pytest.raises(ValueError, match='出现多份基准巡检'):
            MODULE.build_catalog([first, second])


def test_catalog_rejects_dirty_source_patrol():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        session = _session(root, 101, 'dirty_run')
        manifest_path = session / 'run_manifest.json'
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        manifest['git']['dirty'] = True
        manifest_path.write_text(json.dumps(manifest), encoding='utf-8')
        with pytest.raises(ValueError, match='未提交代码'):
            MODULE.build_catalog([session])


def test_catalog_rejects_too_few_distinct_seeds_for_formal_regression():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        first = _session(root, 101, 'run_a')
        second = _session(root, 102, 'run_b')
        with pytest.raises(ValueError, match='至少需要 3 个不同 SEED'):
            MODULE.build_catalog([first, second])
