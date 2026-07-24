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


def _snapshot(*, strict=0, partial=0, confirmed=0, position=None, frame='real_sense'):
    detections = ([{'requires_reobservation': False}] * strict
                  + [{'requires_reobservation': True}] * partial)
    hazards = [{
        'status': 'confirmed',
        'position': position or [0.0, 0.0, 0.15],
        'position_frame_id': frame,
    }] * confirmed
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


def test_motion_evidence_accepts_map_to_base_slam_tf():
    """主动复查可直接读取 SLAM 的 map→base，不再把命令积分里程计当真实位姿。"""

    class Result:
        stdout = (
            'Translation: [0.125, -0.250, 0.000]\n'
            'Rotation: in Quaternion [0.000, 0.000, 0.100, 0.995]\n'
            'Rotation: in RPY (radian) [0.000, -0.000, 0.200]\n'
        )

    original_run = runner._run
    runner._run = lambda *args, **kwargs: Result()
    try:
        pose = runner._read_legal_motion('tf://map/base', {})

        assert pose == {'x': 0.125, 'y': -0.25, 'z': 0.0, 'yaw_rad': 0.2}
        assert runner._read_legal_motion('tf://odom/base', {}) is None
    finally:
        runner._run = original_run


def test_controlled_lateral_reobservation_keeps_omnidirectional_semantics():
    """A1 的侧移复查必须保持 linear.y，避免前进弧线把目标推到下边缘。"""

    poses = iter([
        {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw_rad': 0.0},
        {'x': 0.02, 'y': -0.14, 'z': 0.0, 'yaw_rad': -0.01},
    ])
    calls = []

    original_read = runner._read_legal_motion
    original_run = runner._run
    original_sleep = runner.time.sleep
    runner._read_legal_motion = lambda *_args, **_kwargs: next(poses)
    runner._run = lambda command, *_args, **_kwargs: calls.append(command)
    runner.time.sleep = lambda *_args: None
    try:
        motion = runner._execute_reobservation_action(
            'move_right', 1.5, 'tf://map/base', {},
            lateral_speed_mps=0.3,
            forward_speed_mps=0.25,
            turn_speed_radps=0.45,
        )

        assert motion['command'] == {
            'linear_x': 0.0,
            'linear_y': -0.3,
            'angular_z': 0.0,
        }
        assert motion['translation_m'] > 0.14
        assert any('y: -0.300' in part for part in calls[0])
    finally:
        runner._read_legal_motion = original_read
        runner._run = original_run
        runner.time.sleep = original_sleep


def test_ros1_setup_prefix_supports_formal_and_isolated_workspace_paths():
    """证据执行器应兼容正式镜像路径和隔离验收副本，且缺失时显式失败。"""

    prefix = runner._ros1_setup_prefix()

    assert '/home/ros/simenv_ws/devel/setup.bash' in prefix
    assert '/home/ros/Guoyulun/Competition/SimEnv/devel/setup.bash' in prefix
    assert 'exit 3' in prefix


def test_fixture_center_projects_camera_forward_in_ground_plane():
    """动态夹具坐标只用于生成临时模型，且应跟随相机的水平前向。"""
    center = runner._project_camera_forward_center(
        (10.0, -2.0, 1.0), (0.0, 0.0, 0.0, 1.0), 1.5, 0.15,
    )

    assert center == (11.5, -2.0, 0.15)


def test_camera_aligned_fixture_rotates_canonical_forward_and_truth_about_center():
    """规范 +Y 遮挡几何应随相机 +X 前向整体旋转，目标中心保持不变。"""

    center = (2.0, 3.0, 0.15)
    case = build_suite('active_partial_reobservation', center)[0]
    yaw = runner._fixture_yaw_from_camera((0.0, 0.0, 0.0, 1.0))
    aligned = runner._yaw_align_case(case, center, yaw)

    assert abs(yaw + np.pi / 2.0) < 1e-8
    assert aligned.expected_sphere_positions[0] == center
    assert '<pose>' in aligned.sdf
    assert aligned.metadata['fixture_camera_alignment_yaw_rad'] == round(yaw, 8)


def test_yaw_alignment_rotates_offset_truth_without_exposing_it_to_runtime():
    center = (1.0, 1.0, 0.15)
    case = build_suite('multi_ball_clutter', center)[0]
    aligned = runner._yaw_align_case(case, center, np.pi / 2.0)
    original = case.expected_sphere_positions[0]
    rotated = aligned.expected_sphere_positions[0]

    assert np.allclose(
        rotated,
        (
            center[0] - (original[1] - center[1]),
            center[1] + (original[0] - center[0]),
            original[2],
        ),
    )
    assert 'danger_truth' not in aligned.sdf


def test_background_complexity_does_not_count_the_red_ball_edge():
    with tempfile.TemporaryDirectory() as temporary:
        tmp_path = Path(temporary)
        image = np.full((80, 120, 3), 120, dtype=np.uint8)
        cv2.circle(image, (60, 40), 20, (0, 0, 255), thickness=-1)
        cv2.imwrite(str(tmp_path / 'raw.png'), image)

        ratio = runner._background_edge_ratio({'raw_image': 'raw.png'}, tmp_path)

    assert ratio == 0.0


def test_active_sequence_requires_complex_start_and_majority_but_allows_closeup_end():
    ratios = [0.0045, 0.0038, 0.0028, 0.0029, 0.0037, 0.0052]

    assert runner._background_complexity_pass(
        'active_partial_reobservation', ratios, 0.0035,
    )
    assert not runner._background_complexity_pass(
        'active_partial_reobservation',
        [0.0030, *ratios[1:]],
        0.0035,
    )
    assert runner._background_complexity_pass(
        'active_partial_reobservation',
        [*ratios[:-1], 0.0030],
        0.0035,
    )


def test_lost_candidate_recovery_is_bounded_and_reuses_observed_direction():
    motions = [
        {'action': 'move_left'},
        {'action': 'turn_right'},
        {'action': 'move_left'},
    ]

    assert runner._bounded_lost_candidate_recovery(motions, 0) == 'turn_right'
    assert runner._bounded_lost_candidate_recovery(motions, 1) == 'turn_right'
    assert runner._bounded_lost_candidate_recovery(motions, 2) == ''
    assert runner._bounded_lost_candidate_recovery(
        [{'action': 'move_right'}], 0,
    ) == 'move_right'


def test_static_sequence_requires_every_background_frame_to_pass():
    assert runner._background_complexity_pass(
        'red_ball_3d_localization', [0.004, 0.0042], 0.0035,
    )
    assert not runner._background_complexity_pass(
        'red_ball_3d_localization', [0.004, 0.003], 0.0035,
    )


def test_rerun_output_uses_canonical_historical_directory_and_shared_run_id():
    """有效截图和测试表必须回到既有五目录，不能再散落成新顶层目录。"""

    result_dir = runner._suite_output_dir(
        Path('reports/perception/simulation/3d_native'),
        'red_objects',
        '20260718_seed42',
    )
    record_dir = runner._test_record_output_dir(
        Path('reports/perception/test_records'),
        'red_objects',
        '20260718_seed42',
    )

    assert result_dir.as_posix().endswith(
        'official_simenv_20260710_extended_red_object_stress/reruns/20260718_seed42'
    )
    assert record_dir.as_posix().endswith(
        'official_simenv_20260710_extended_red_object_stress/reruns/20260718_seed42'
    )


def test_stage_a_output_uses_fixed_delivery_label_not_actual_run_date():
    result_dir = runner._suite_output_dir(
        Path('reports/perception/simulation/3d_native'),
        'red_ball_3d_localization',
        '20260720_seed42',
    )
    record_dir = runner._test_record_output_dir(
        Path('reports/perception/test_records'),
        'red_ball_3d_localization',
        '20260720_seed42',
    )

    assert result_dir.as_posix().endswith(
        'official_simenv_20260725_red_ball_3d_localization'
    )
    assert record_dir.as_posix().endswith(
        'official_simenv_20260725_red_ball_3d_localization'
    )


def test_stage_b_output_uses_fixed_20260730_delivery_label():
    result_dir = runner._suite_output_dir(
        Path('reports/perception/simulation/3d_native'),
        'active_partial_reobservation',
        '20260723_isolated01',
    )
    record_dir = runner._test_record_output_dir(
        Path('reports/perception/test_records'),
        'active_partial_reobservation',
        '20260723_isolated01',
    )

    assert result_dir.as_posix().endswith(
        'official_simenv_20260730_active_multiview_reobservation'
    )
    assert record_dir.as_posix().endswith(
        'official_simenv_20260730_active_multiview_reobservation'
    )


def test_stage_b_requires_partial_real_motion_single_confirmation_and_world_position():
    case = build_suite(
        'active_partial_reobservation', (0.0, 0.0, 0.15),
    )[0]
    truth = case.expected_sphere_positions[0]
    snapshots = [
        _snapshot(partial=1, frame='world'),
        _snapshot(strict=1, frame='world'),
        _snapshot(strict=1, confirmed=1, position=list(truth), frame='world'),
    ]
    motions = [
        {
            'action': 'move_left',
            'translation_m': 0.24,
            'yaw_change_deg': 0.0,
            'motion_evidence_status': 'legal_slam',
        },
        {
            'action': 'move_right',
            'translation_m': 0.23,
            'yaw_change_deg': 0.0,
            'motion_evidence_status': 'legal_slam',
        },
    ]
    context = {
        'truth_frame_id': 'fixture_world',
        'evaluation_frame_id': 'world',
        'world_from_evaluation': {
            'translation': (0.0, 0.0, 0.0),
            'quaternion': (0.0, 0.0, 0.0, 1.0),
        },
    }

    passed = runner._evaluate_case(
        case, snapshots, motions, 0.2, localization_context=context,
    )
    no_motion = runner._evaluate_case(
        case, snapshots, [], 0.2, localization_context=context,
    )
    duplicate_final = _snapshot(
        strict=1, confirmed=2, position=list(truth), frame='world',
    )
    duplicate = runner._evaluate_case(
        case,
        snapshots[:-1] + [duplicate_final],
        motions,
        0.2,
        localization_context=context,
    )

    assert passed['result'] == 'pass'
    assert passed['actual_motion_count'] == 2
    assert passed['action_sequence'] == ['move_left', 'move_right']
    assert no_motion['result'] == 'fail'
    assert duplicate['result'] == 'fail'
    assert duplicate['final_confirmed_count'] == 2


def test_rerun_output_rejects_untraceable_run_id():
    """缺少日期或批次标识的截图目录不可进入规范归档。"""

    try:
        runner._suite_output_dir(Path('reports'), 'red_objects', 'latest')
    except ValueError as error:
        assert 'YYYYMMDD' in str(error)
    else:
        raise AssertionError('untraceable run_id should be rejected')


def test_fixed_stage_output_cannot_mix_old_and_new_files_without_explicit_replace():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        target = root / 'official_simenv_20260725_red_ball_3d_localization'
        target.mkdir()
        (target / 'old.png').write_bytes(b'old')

        try:
            runner._prepare_suite_output(root, target, replace_existing=False)
        except FileExistsError as error:
            assert '--replace-stage-output' in str(error)
        else:
            raise AssertionError('non-empty stage output should be protected')

        runner._prepare_suite_output(root, target, replace_existing=True)
        assert not target.exists()


def test_case_detector_cleanup_reaps_the_whole_ros2_run_process_group():
    """每例必须回收 ros2 CLI 及其节点子进程，防止重复检测器污染后续案例。"""

    source = (SCRIPTS_DIR / 'run_official_simenv_classic_evidence.py').read_text(
        encoding='utf-8',
    )

    assert 'start_new_session=True' in source
    assert 'os.killpg(process.pid, signal.SIGTERM)' in source
    assert 'os.killpg(process.pid, signal.SIGKILL)' in source


def test_isolated_container_reset_failure_is_fail_closed():
    """容器复位脚本失败时不得继续下一例并把残留模型当新场景。"""

    assert runner._reset_isolated_container(
        'python -c "import sys; sys.exit(7)"', {},
    ) is False


def test_localization_case_reports_error_after_snapshot_not_as_runtime_input():
    case = build_suite('red_ball_3d_localization', (0.0, 0.0, 0.15))[0]
    truth = case.expected_sphere_positions[0]
    row = runner._evaluate_case(
        case,
        [_snapshot(strict=1, confirmed=1, position=[truth[0] + 0.02, truth[1], truth[2]])],
        [],
        0.2,
        localization_context={
            'truth_frame_id': 'fixture_world',
            'evaluation_frame_id': 'real_sense',
            'world_from_evaluation': {
                'translation': (0.0, 0.0, 0.0),
                'quaternion': (0.0, 0.0, 0.0, 1.0),
            },
        },
    )

    assert row['localized_truth_count'] >= 1
    assert row['mean_localization_error_m'] != ''
    assert row['result'] == 'pass'


def test_localization_frame_mismatch_fails_closed():
    case = build_suite('red_ball_3d_localization', (0.0, 0.0, 0.15))[0]
    row = runner._evaluate_case(
        case,
        [_snapshot(strict=1, confirmed=1, frame='map')],
        [],
        0.2,
        localization_context={
            'truth_frame_id': 'fixture_world',
            'evaluation_frame_id': 'real_sense',
            'world_from_evaluation': {
                'translation': (0.0, 0.0, 0.0),
                'quaternion': (0.0, 0.0, 0.0, 1.0),
            },
        },
    )

    assert row['result'] == 'fail'
    assert row['localization_status'] == 'frame_mismatch'


def test_localization_enforces_one_meter_maximum_and_exact_one_to_one_count():
    case = build_suite('red_ball_3d_localization', (0.0, 0.0, 0.15))[0]
    context = {
        'truth_frame_id': 'fixture_world',
        'evaluation_frame_id': 'real_sense',
        'world_from_evaluation': {
            'translation': (0.0, 0.0, 0.0),
            'quaternion': (0.0, 0.0, 0.0, 1.0),
        },
    }
    too_far = runner._evaluate_case(
        case,
        [_snapshot(strict=1, confirmed=1, position=[1.2, 0.0, 0.15])],
        [],
        0.2,
        localization_context=context,
    )
    duplicate = _snapshot(strict=1, confirmed=1)
    duplicate['hazards'].append(dict(duplicate['hazards'][0]))
    duplicated = runner._evaluate_case(
        case, [duplicate], [], 0.2, localization_context=context,
    )

    assert too_far['result'] == 'fail'
    assert too_far['max_localization_error_m'] > 1.0
    assert duplicated['result'] == 'fail'
    assert duplicated['localization_unmatched_prediction_count'] == 1


def test_official_distractor_suite_never_treats_candidate_as_false_alarm():
    red_cube_only = build_suite(
        'official_distractor_rejection', (0.0, 0.0, 0.15),
    )[0]
    row = runner._evaluate_case(
        red_cube_only, [_snapshot(partial=1, confirmed=0)], [], 0.2,
    )

    assert row['result'] == 'pass'
    assert row['final_confirmed_count'] == 0
