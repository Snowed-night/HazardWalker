"""A阶段官方规格静态感知证据的结构和指标回归。"""

import csv
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = ROOT / 'reports' / 'perception' / 'simulation' / '3d_native'


def _load(experiment):
    directory = SIM_ROOT / experiment
    summary = json.loads((directory / 'summary.json').read_text(encoding='utf-8'))
    cases = json.loads((directory / 'cases.json').read_text(encoding='utf-8'))
    with (directory / 'testing_record_perception.csv').open(
            encoding='utf-8', newline='') as handle:
        rows = list(csv.DictReader(handle))
    return directory, summary, cases, rows


def _assert_complete_evidence(directory):
    """固定提交所需的图片、深度、记录和可追溯运行日志。"""
    assert (directory / 'README.md').is_file()
    assert (directory / 'summary.json').is_file()
    assert (directory / 'testing_record_perception.json').is_file()
    assert len(list((directory / 'images').glob('*_raw.png'))) == 5
    assert len(list((directory / 'images').glob('*_annotated.png'))) == 5
    assert len(list((directory / 'images').glob('*_depth_mm.png'))) == 5
    assert len(list((directory / 'logs').glob('*_detector.txt'))) == 5


def test_stage_a_base_localization_has_five_complete_low_error_cases():
    directory, summary, cases, rows = _load(
        'official_simenv_20260725_red_ball_3d_localization',
    )

    assert summary['delivery_stage'] == '20260725'
    assert summary['localization_evaluation_frame'] == 'base'
    assert summary['case_count'] == summary['pass_count'] == 5
    assert summary['fail_count'] == 0
    assert summary['target_candidate_recall'] == 1.0
    assert summary['confirmed_duplicate_count'] == 0
    assert summary['max_observed_localization_error_m'] <= 1.0
    assert summary['official_json_written'] is False
    assert len(cases) == len(rows) == 5
    assert all(item['localization_status'] == 'ok' for item in cases)
    assert all(item['localization_prediction_count'] == 1 for item in cases)
    _assert_complete_evidence(directory)


def test_stage_a_official_distractors_have_zero_confirmed_false_positives():
    directory, summary, cases, rows = _load(
        'official_simenv_20260725_official_distractor_rejection',
    )

    assert summary['delivery_stage'] == '20260725'
    assert summary['case_count'] == summary['pass_count'] == 5
    assert summary['fail_count'] == 0
    assert summary['target_case_count'] == 3
    assert summary['target_candidate_recall'] == 1.0
    assert summary['confirmed_false_positive_count'] == 0
    assert summary['confirmed_duplicate_count'] == 0
    assert summary['official_json_written'] is False
    assert len(cases) == len(rows) == 5
    assert all(item['final_confirmed_count'] == 0 for item in cases)
    assert cases[0]['initial_partial_count'] >= 1  # 红方块只保留黄色复查候选。
    assert cases[1]['initial_strict_count'] == 0   # 绿球不是红色目标。
    _assert_complete_evidence(directory)
