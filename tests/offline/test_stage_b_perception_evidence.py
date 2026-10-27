"""B阶段部分可见主动复查证据的结构、指标和轨迹唯一性回归。"""

import csv
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
EXPERIMENT = 'official_simenv_20260730_active_multiview_reobservation'
DIRECTORY = (
    ROOT / 'reports' / 'perception' / 'simulation' / '3d_native' / EXPERIMENT
)
RECORD_DIRECTORY = ROOT / 'reports' / 'perception' / 'test_records' / EXPERIMENT


def test_stage_b_active_reobservation_has_six_complete_passing_cases():
    summary = json.loads(
        (DIRECTORY / 'summary.json').read_text(encoding='utf-8'),
    )
    cases = json.loads(
        (DIRECTORY / 'cases.json').read_text(encoding='utf-8'),
    )
    with (DIRECTORY / 'testing_record_perception.csv').open(
            encoding='utf-8', newline='') as handle:
        rows = list(csv.DictReader(handle))

    assert summary['delivery_stage'] == '20260730'
    assert summary['case_count'] == summary['pass_count'] == 6
    assert summary['fail_count'] == 0
    assert summary['target_candidate_recall'] == 1.0
    assert summary['confirmed_false_positive_count'] == 0
    assert summary['confirmed_duplicate_count'] == 0
    assert summary['max_observed_localization_error_m'] <= 1.0
    assert summary['official_score_eligible'] is False
    assert summary['official_json_written'] is False
    assert len(cases) == len(rows) == 6
    assert all(case['result'] == 'pass' for case in cases)
    assert all(case['initial_partial_count'] >= 1 for case in cases)
    assert all(case['initial_strict_count'] == 0 for case in cases)
    assert all(case['final_confirmed_count'] == 1 for case in cases)
    assert all(case['actual_motion_count'] >= 2 for case in cases)
    assert all(case['max_localization_error_m'] <= 1.0 for case in cases)


def test_stage_b_each_case_starts_partial_and_finishes_one_confirmed_track():
    cases = json.loads(
        (DIRECTORY / 'cases.json').read_text(encoding='utf-8'),
    )
    for case in cases:
        case_id = case['case_id']
        snapshots = sorted(
            (DIRECTORY / 'snapshots').glob(case_id + '_view*_snapshot.json'),
        )
        assert len(snapshots) >= 3
        first = json.loads(snapshots[0].read_text(encoding='utf-8'))
        final = json.loads(snapshots[-1].read_text(encoding='utf-8'))
        assert first['detections_2d']
        assert first['detections_2d'][0]['requires_reobservation'] is True
        assert not [
            item for item in first['hazards']
            if item['status'] == 'confirmed'
        ]
        confirmed = [
            item for item in final['hazards']
            if item['status'] == 'confirmed'
        ]
        assert len(final['hazards']) == len(confirmed) == 1
        assert confirmed[0]['view_bearing_span_deg'] >= 25.0


def test_stage_b_artifacts_and_test_record_mirror_are_complete():
    assert (DIRECTORY / 'README.md').is_file()
    assert (DIRECTORY / 'summary.json').is_file()
    assert (DIRECTORY / 'cases.csv').is_file()
    assert (DIRECTORY / 'cases.json').is_file()
    assert (
        DIRECTORY / 'images' /
        'active_partial_reobservation_annotated_collage.png'
    ).is_file()

    snapshot_count = len(list((DIRECTORY / 'snapshots').glob('*_snapshot.json')))
    assert snapshot_count >= 6 * 3
    assert len(list((DIRECTORY / 'images').glob('*_raw.png'))) == snapshot_count
    assert len(list((DIRECTORY / 'images').glob('*_annotated.png'))) == snapshot_count
    assert len(list((DIRECTORY / 'images').glob('*_depth_mm.png'))) == snapshot_count

    for name in (
        'testing_record_perception.csv',
        'testing_record_perception.json',
    ):
        assert (DIRECTORY / name).read_bytes() == (
            RECORD_DIRECTORY / name
        ).read_bytes()
