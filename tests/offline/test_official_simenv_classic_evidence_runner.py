"""官方 Gazebo Classic 证据执行器的离线逻辑测试。"""

import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

from official_simenv_classic_evidence_cases import build_suite
import run_official_simenv_classic_evidence as runner


def _snapshot(*, strict=0, partial=0, confirmed=0, position=None):
    detections = ([{'requires_reobservation': False}] * strict
                  + [{'requires_reobservation': True}] * partial)
    hazards = [{'status': 'confirmed', 'position': position or [0.0, 0.0, 0.15]}] * confirmed
    return {'detections_2d': detections, 'hazards': hazards}


def test_partial_case_accepts_reobserve_candidate_without_claiming_confirmation():
    case = build_suite('partial_visibility', (0.0, 0.0, 0.15))[1]
    row = runner._evaluate_case(case, [_snapshot(partial=1)], [], 0.2)

    assert row['result'] == 'pass'
    assert row['initial_partial_count'] == 1
    assert row['final_confirmed_count'] == 0


def test_multiview_non_sphere_requires_two_real_lateral_moves_and_no_confirmation():
    case = build_suite('active_multiview', (0.0, 0.0, 0.15))[3]
    motions = [{'translation_m': 0.25}, {'translation_m': 0.22}]
    row = runner._evaluate_case(case, [_snapshot(strict=1)] * 3, motions, 0.2)

    assert case.metadata['is_red_sphere_target'] is False
    assert row['actual_lateral_move_count'] == 2
    assert row['result'] == 'pass'


def test_multi_ball_requires_exact_confirmed_count_not_raw_candidate_count():
    """多球粘连不能用候选数量凑数；多出的 confirmed 轨迹必须令案例失败。"""
    case = build_suite('multi_ball_clutter', (0.0, 0.0, 0.15))[0]
    motions = [{'translation_m': 0.25}, {'translation_m': 0.22}]
    row = runner._evaluate_case(case, [_snapshot(strict=3, confirmed=3)] * 3, motions, 0.2)

    assert len(case.expected_sphere_positions) == 2
    assert row['result'] == 'fail'
    assert '完全相等' in row['criterion']


def test_motion_evidence_refuses_gazebo_truth_topics():
    """正式多视角证据不能用官方明确禁用的 Gazebo 真值里程计充数。"""
    assert runner._read_legal_motion('/hw/odom', {}) is None
    assert runner._read_legal_motion('/Odometry_gazebo', {}) is None
    assert runner._read_legal_motion('/ground_truth/base_w', {}) is None


def test_fixture_center_projects_camera_forward_in_ground_plane():
    """动态夹具坐标只用于生成临时模型，且应跟随相机的水平前向。"""
    center = runner._project_camera_forward_center(
        (10.0, -2.0, 1.0), (0.0, 0.0, 0.0, 1.0), 1.5, 0.15,
    )

    assert center == (11.5, -2.0, 0.15)


def test_background_complexity_does_not_count_the_red_ball_edge():
    with tempfile.TemporaryDirectory() as temporary:
        tmp_path = Path(temporary)
        image = np.full((80, 120, 3), 120, dtype=np.uint8)
        cv2.circle(image, (60, 40), 20, (0, 0, 255), thickness=-1)
        cv2.imwrite(str(tmp_path / 'raw.png'), image)

        ratio = runner._background_edge_ratio({'raw_image': 'raw.png'}, tmp_path)

    assert ratio == 0.0


def test_localization_case_reports_error_after_snapshot_not_as_runtime_input():
    case = build_suite('complex_localization', (0.0, 0.0, 0.15))[0]
    truth = case.expected_sphere_positions[0]
    row = runner._evaluate_case(
        case, [_snapshot(strict=1, confirmed=1, position=[truth[0] + 0.02, truth[1], truth[2]])], [], 0.2,
    )

    assert row['localized_truth_count'] >= 1
    assert row['mean_localization_error_m'] != ''
