"""人工巡检运动覆盖统计与正式门禁离线测试。"""

import math
from pathlib import Path
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
PERCEPTION_ROOT = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
sys.path.insert(0, str(PERCEPTION_ROOT))

from hazardwalker_perception.patrol_coverage import (  # noqa: E402
    PatrolCoverageTracker,
    validate_patrol_coverage,
)


def test_tracker_accumulates_real_path_span_and_vertical_change():
    tracker = PatrolCoverageTracker()
    assert tracker.update(0.0, 0.0, 0.0, 1.0)
    assert tracker.update(1.0, 0.0, 0.0, 2.0)
    assert tracker.update(1.0, 1.0, 2.6, 3.0)
    snapshot = tracker.snapshot()
    assert snapshot['sample_count'] == 3
    assert snapshot['accepted_segment_count'] == 2
    assert snapshot['planar_path_length_m'] == pytest.approx(2.0)
    assert snapshot['planar_span_m'] == pytest.approx(math.sqrt(2.0))
    assert snapshot['vertical_span_m'] == pytest.approx(2.6)


def test_time_reversal_and_slam_jump_cannot_inflate_coverage():
    tracker = PatrolCoverageTracker(max_planar_step_m=1.5)
    tracker.update(0.0, 0.0, 0.0, 1.0)
    assert not tracker.update(10.0, 0.0, 0.0, 2.0)
    assert not tracker.update(10.5, 0.0, 0.0, 1.5)
    assert tracker.update(10.5, 0.0, 0.0, 3.0)
    snapshot = tracker.snapshot()
    assert snapshot['rejected_sample_count'] == 2
    assert snapshot['planar_path_length_m'] == pytest.approx(0.5)
    assert snapshot['planar_span_m'] == pytest.approx(0.5)


def test_non_finite_sample_is_rejected():
    tracker = PatrolCoverageTracker()
    assert not tracker.update(float('nan'), 0.0, 0.0, 1.0)
    assert tracker.snapshot()['sample_count'] == 0


def test_formal_gate_rejects_stationary_long_recording_and_accepts_patrol():
    stationary = {
        'sample_count': 1000,
        'planar_path_length_m': 0.0,
        'planar_span_m': 0.0,
    }
    errors = validate_patrol_coverage(stationary)
    assert any('平面路程' in item for item in errors)
    assert any('覆盖跨度' in item for item in errors)
    assert validate_patrol_coverage({
        'sample_count': 100,
        'planar_path_length_m': 12.0,
        'planar_span_m': 5.0,
    }) == []


def test_tracker_parameters_and_gate_payload_fail_closed():
    with pytest.raises(ValueError):
        PatrolCoverageTracker(max_planar_step_m=0.0)
    assert validate_patrol_coverage(None)


def test_patrol_coverage_runtime_contract_is_wired_into_launch_and_bag():
    setup_text = (
        REPO_ROOT / 'ros2_ws/src/hazardwalker_perception/setup.py'
    ).read_text(encoding='utf-8')
    launch_text = (
        REPO_ROOT
        / 'ros2_ws/src/hazardwalker_bringup/launch/official_simenv_business.launch.py'
    ).read_text(encoding='utf-8')
    runner_text = (
        REPO_ROOT / 'scripts/run_official_slam_exploration.py'
    ).read_text(encoding='utf-8')

    assert 'patrol_coverage_node' in setup_text
    assert "executable='patrol_coverage_node'" in launch_text
    assert 'start_evidence_recorder:=' in runner_text
    assert 'evidence_output_dir:=' in runner_text
