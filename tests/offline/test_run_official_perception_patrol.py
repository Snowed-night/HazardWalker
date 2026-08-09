"""正式人工巡检统一入口离线测试。"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'run_official_perception_patrol.py'
SPEC = importlib.util.spec_from_file_location('run_official_patrol', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_preflight_and_record_share_the_same_formal_contract():
    report = MODULE.OFFICIAL_RANDOM_ROOT / '20260809_seed_20260809_run01'
    session = Path(tempfile.gettempdir()) / 'seed_20260809_run01'
    preflight, record = MODULE.build_commands(
        seed='20260809', run_id='seed_20260809_run01', operator='姜晨',
        localization_provenance='lidar_imu_slam',
        control_source='keyboard', report_dir=report, session_dir=session,
        sample_sec=3.0,
        first_person_health_url='http://127.0.0.1:6082/healthz',
        topic_wait_timeout_sec=20.0, preflight_max_age_sec=300.0)

    assert preflight[preflight.index('--scenario-seed') + 1] == '20260809'
    assert record[record.index('--seed') + 1] == '20260809'
    assert preflight[preflight.index('--localization-provenance') + 1] == (
        record[record.index('--localization-provenance') + 1])
    assert record[record.index('--preflight-report') + 1] == str(
        report / 'live_chain_preflight.json')


def test_rosbag_cannot_be_written_inside_repository():
    report = MODULE.OFFICIAL_RANDOM_ROOT / 'new_run'
    with pytest.raises(ValueError, match='仓库外'):
        MODULE.validate_output_paths(report, ROOT / 'bags' / 'new_run')


def test_report_must_use_an_independent_official_random_directory():
    with tempfile.TemporaryDirectory() as temporary:
        session = Path(temporary) / 'session'
        with pytest.raises(ValueError, match='official_random'):
            MODULE.validate_output_paths(
                ROOT / 'reports' / 'perception' / 'simulation' / 'bad',
                session)
        with pytest.raises(ValueError, match='独立运行目录'):
            MODULE.validate_output_paths(MODULE.OFFICIAL_RANDOM_ROOT, session)


def test_preflight_report_rejects_seed_or_control_drift():
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / 'preflight.json'
        payload = {
            'passed': True,
            'traffic_checked': True,
            'expected_scenario_seed': '20260809',
            'control_source': 'keyboard',
            'expected_localization_provenance': 'lidar_imu_slam',
        }
        path.write_text(json.dumps(payload), encoding='utf-8')
        MODULE.validate_preflight_report(
            path, seed='20260809', control_source='keyboard',
            localization_provenance='lidar_imu_slam')
        with pytest.raises(ValueError, match='SEED'):
            MODULE.validate_preflight_report(
                path, seed='20260810', control_source='keyboard',
                localization_provenance='lidar_imu_slam')
        with pytest.raises(ValueError, match='控制源'):
            MODULE.validate_preflight_report(
                path, seed='20260809', control_source='navigation',
                localization_provenance='lidar_imu_slam')
