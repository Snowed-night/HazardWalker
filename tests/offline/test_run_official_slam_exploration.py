"""官方 SLAM/探索运行器的离线合同测试。"""

import importlib.util
import json
import math
from pathlib import Path
import tempfile
import subprocess
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'run_official_slam_exploration.py'
SPEC = importlib.util.spec_from_file_location('slam_exploration_runner', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_launch_command_uses_unique_managed_control_and_legal_slam_inputs():
    command = MODULE.build_launch_command(
        Path('/tmp/nav-run'), scenario_seed='20260823', code_version='abc')
    joined = ' '.join(str(item) for item in command)
    assert 'start_command_mux:=false' in joined
    assert 'start_slam:=true' in joined
    assert 'start_pointcloud_map:=false' in joined
    assert 'start_navigation:=true' in joined
    assert 'slam_backend:=cartographer' in joined
    assert 'slam_dimension:=2d' in joined
    assert 'localization_provenance:=lidar_imu_slam' in joined
    assert 'navigation_linear_speed:=0.45' in joined
    assert 'navigation_minimum_linear_speed:=0.30' in joined
    assert 'navigation_start_paused:=true' in joined
    assert 'localization_command_motion_scale:=0.88' in joined
    assert 'mission_time_budget_s:=600.000' in joined
    assert 'strict_room_inspection:=false' in joined
    assert 'start_perception:=false' in joined
    assert 'start_evidence_recorder:=false' in joined
    assert 'perception_output_frame:=odom' in joined
    assert 'nav_record_dir:=/tmp/nav-run/navigation' in joined
    assert 'slam_monitor_output_dir:=/tmp/nav-run/slam' in joined
    assert 'pointcloud_map_output_dir:=/tmp/nav-run/slam_3d' in joined
    assert 'start_slam_video:=true' in joined
    assert 'slam_video_output:=/tmp/nav-run/video/slam_exploration.mp4' in joined
    assert '/hw/odom' not in joined

    multifloor = ' '.join(MODULE.build_launch_command(
        Path('/tmp/nav-run'), scenario_seed='20260823', code_version='abc',
        mission_time_budget_s=900.0,
        target_floors=(0, 1, 2), per_floor_exploration_s=120.0,
        simenv_container='simenv_ros1_test'))
    assert 'target_floors:=[0,1,2]' in multifloor
    assert 'per_floor_exploration_s:=120.000' in multifloor
    assert 'mission_time_budget_s:=900.000' in multifloor
    assert 'manual_elevator_assist:=true' in multifloor
    assert 'automatic_elevator_entry:=true' in multifloor
    assert 'localization_provenance:=lidar_imu_slam+public_floor_action' in multifloor
    assert 'simenv_container:=simenv_ros1_test' in multifloor

    strict = ' '.join(MODULE.build_launch_command(
        Path('/tmp/nav-run'), scenario_seed='20260823', code_version='abc',
        strict_room_inspection=True,
        world_from_map=(0.0, 1.403, math.pi / 2.0)))
    assert 'strict_room_inspection:=true' in strict
    assert 'start_perception:=true' in strict
    assert 'start_evidence_recorder:=true' in strict
    assert 'perception_output_frame:=odom' in strict
    assert 'perception_parameter_file:=' in strict
    assert '/config/perception.yaml' in strict
    assert 'official_hazard_source_frame:=odom' in strict
    assert 'official_world_from_map_y:=1.403000' in strict
    assert 'official_world_from_map_yaw:=1.570796' in strict
    assert 'official_floor_height_m:=2.600000' in strict
    assert 'official_sphere_center_height_m:=0.150000' in strict
    assert 'strict_room_clearance_m:=0.600000' in strict
    assert 'official_result_path:=/tmp/nav-run/detected_danger.json' in strict

    perception_only = ' '.join(MODULE.build_launch_command(
        Path('/tmp/nav-run'), scenario_seed='20260823', code_version='abc',
        enable_perception=True,
        strict_room_inspection=False,
        world_from_map=(0.0, 1.403, math.pi / 2.0)))
    assert 'start_perception:=true' in perception_only
    assert 'start_evidence_recorder:=true' in perception_only
    assert 'strict_room_inspection:=false' in perception_only
    assert 'perception_parameter_file:=' in perception_only
    assert 'official_hazard_source_frame:=odom' in perception_only


def test_map_origin_uses_actual_public_ingress_before_slam_start():
    origin = MODULE.map_origin_after_straight_ingress(
        0.0, -2.2, math.pi / 2.0, 3.603)
    assert origin[0] == pytest.approx(0.0, abs=1e-9)
    assert origin[1] == pytest.approx(1.403, abs=1e-9)
    assert origin[2] == pytest.approx(math.pi / 2.0)
    with pytest.raises(ValueError, match='不得为负'):
        MODULE.map_origin_after_straight_ingress(0.0, -2.2, 0.0, -0.1)

    curved = MODULE.map_origin_after_relative_ingress(
        0.0, -2.2, math.pi / 2.0,
        3.5, -0.4, math.radians(7.5),
    )
    assert curved[0] == pytest.approx(0.4, abs=1e-9)
    assert curved[1] == pytest.approx(1.3, abs=1e-9)
    assert curved[2] == pytest.approx(math.radians(97.5), abs=1e-9)


def test_preflight_loads_real_perception_yaml_before_motion():
    contract = MODULE.validate_perception_mission_config(
        ROOT / 'config' / 'perception.yaml')
    assert contract['parameters'] == {
        'confirm_observation_count': 1,
        'confirm_distinct_views': 1,
        'min_spherical_views_for_confirm': 1,
        'reject_non_spherical_tracks': False,
        'emit_partial_candidates': True,
    }
    assert len(contract['sha256']) == 64


def test_preflight_rejects_clearance_smaller_than_runtime_a1_footprint():
    original_run = MODULE.subprocess.run
    MODULE.subprocess.run = lambda *args, **kwargs: SimpleNamespace(
        returncode=0,
        stdout="'[[0.42,0.38],[0.42,-0.38],"
               "[-0.45,-0.38],[-0.45,0.38]]'\n",
        stderr='')
    try:
        contract = MODULE.validate_navigation_clearance_contract(
            'simenv_ros1_test', 0.60)
        assert contract['footprint_radius_m'] == pytest.approx(
            math.hypot(0.45, 0.38))
        with pytest.raises(RuntimeError, match='小于 A1 footprint'):
            MODULE.validate_navigation_clearance_contract(
                'simenv_ros1_test', 0.50)
    finally:
        MODULE.subprocess.run = original_run

    three_dimensional = ' '.join(MODULE.build_launch_command(
        Path('/tmp/nav-run'), scenario_seed='20260823', code_version='abc',
        enable_3d_map=True))
    assert 'start_pointcloud_map:=true' in three_dimensional
    assert 'slam_dimension:=3d' in three_dimensional


def test_target_floor_parser_preserves_order_and_rejects_duplicates():
    assert MODULE.parse_target_floors('0, 2, 1') == (0, 2, 1)
    assert MODULE.parse_target_floors('') == ()
    with pytest.raises(ValueError, match='不得重复'):
        MODULE.parse_target_floors('0,1,1')


def test_runner_bootstraps_its_own_ros2_overlay_before_parsing_arguments():
    source = SCRIPT.read_text(encoding='utf-8')
    main_source = source.split('def main()', 1)[1]
    assert main_source.index('ensure_workspace_overlay()') < main_source.index(
        'argparse.ArgumentParser')
    assert 'HAZARDWALKER_OVERLAY_BOOTSTRAPPED' in source
    assert 'source "$1"; source "$2"' in source
    assert "'pkg', 'executables', 'hazardwalker_perception'" in source
    assert 'scan_imu_localizer_node' in source


def test_git_state_ignores_runtime_products_but_detects_source_changes():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        subprocess.run(['git', 'init', '-q'], cwd=root, check=True)
        subprocess.run(
            ['git', 'config', 'user.email', 'test@example.com'],
            cwd=root, check=True)
        subprocess.run(
            ['git', 'config', 'user.name', 'Test'], cwd=root, check=True)
        source = root / 'scripts' / 'runner.py'
        source.parent.mkdir()
        source.write_text('stable\n', encoding='utf-8')
        subprocess.run(['git', 'add', '.'], cwd=root, check=True)
        subprocess.run(
            ['git', 'commit', '-qm', 'baseline'], cwd=root, check=True)

        generated = (
            root / 'ros2_ws/src/hazardwalker_platform/generated_building'
            / 'world.sdf')
        generated.parent.mkdir(parents=True)
        generated.write_text('runtime\n', encoding='utf-8')
        (root / 'install').mkdir()
        (root / 'install/cache').write_text('runtime\n', encoding='utf-8')
        assert MODULE.read_git_state(root)['dirty'] is False

        source.write_text('changed\n', encoding='utf-8')
        assert MODULE.read_git_state(root)['dirty'] is True


def test_official_evaluation_runs_only_from_explicit_post_run_files():
    command = MODULE.build_official_evaluation_command(
        Path('/tmp/truth.json'),
        Path('/tmp/detected.json'),
        Path('/tmp/evaluation.json'),
    )

    assert command[0] == 'python3'
    assert command[1].endswith('evaluate_danger.py')
    assert command[-6:] == [
        '--truth-file', '/tmp/truth.json',
        '--detected-file', '/tmp/detected.json',
        '--output-file', '/tmp/evaluation.json',
    ]
    source = SCRIPT.read_text(encoding='utf-8')
    assert source.index('stop_process_group(process)') < source.index(
        "manifest['evaluation'] = evaluate_completed_run(")


def test_navigation_acceptance_requires_all_four_strict_rooms_per_floor():
    with tempfile.TemporaryDirectory() as temporary:
        output = Path(temporary)
        navigation = output / 'navigation'
        navigation.mkdir()
        rows = []
        for floor in (0, 1, 2):
            for sector in ('far_left', 'far_right', 'near_left', 'near_right'):
                rows.append({
                    'floor': floor,
                    'sector': sector,
                    'phase': 'completed',
                    'reason': 'deterministic_loop_and_strict_inspection',
                    'obstacle_count': 4,
                    'inspection_goal_count': 12,
                    'inspection_completed_count': 12,
                    'visibility_coverage_ratio': 0.96,
                    'required_visibility_coverage_ratio': 0.95,
                    'visibility_target_cell_count': 400,
                    'visibility_covered_cell_count': 384,
                })
        (navigation / 'room_coverage.jsonl').write_text(
            ''.join(json.dumps(row) + '\n' for row in rows),
            encoding='utf-8')
        (navigation / 'failures.jsonl').write_text('', encoding='utf-8')

        result = MODULE.validate_navigation_acceptance(
            output, (0, 1, 2), strict_room_inspection=True)
        assert result['passed'] is True
        assert all(
            item['completed_room_count'] == 4
            for item in result['floors'].values())


def test_navigation_acceptance_rejects_missing_room_or_fake_capture():
    with tempfile.TemporaryDirectory() as temporary:
        output = Path(temporary)
        navigation = output / 'navigation'
        navigation.mkdir()
        rows = [{
            'floor': 0,
            'sector': sector,
            'phase': 'completed',
            'reason': 'deterministic_loop_and_strict_inspection',
            'obstacle_count': 4,
            'inspection_goal_count': 12,
            'inspection_completed_count': 12,
            'visibility_coverage_ratio': 0.96,
            'required_visibility_coverage_ratio': 0.95,
            'visibility_target_cell_count': 400,
            'visibility_covered_cell_count': 384,
        } for sector in ('far_left', 'far_right', 'near_left')]
        (navigation / 'room_coverage.jsonl').write_text(
            ''.join(json.dumps(row) + '\n' for row in rows),
            encoding='utf-8')
        (navigation / 'failures.jsonl').write_text('', encoding='utf-8')
        with pytest.raises(RuntimeError, match='3/4'):
            MODULE.validate_navigation_acceptance(
                output, (0,), strict_room_inspection=True)

        rows.append({
            'floor': 0,
            'sector': 'near_right',
            'phase': 'completed',
            'reason': 'deterministic_loop_and_strict_inspection',
            'obstacle_count': 4,
            'inspection_goal_count': 12,
            'inspection_completed_count': 11,
            'visibility_coverage_ratio': 0.96,
            'required_visibility_coverage_ratio': 0.95,
            'visibility_target_cell_count': 400,
            'visibility_covered_cell_count': 384,
        })
        (navigation / 'room_coverage.jsonl').write_text(
            ''.join(json.dumps(row) + '\n' for row in rows),
            encoding='utf-8')
        with pytest.raises(RuntimeError, match='11/12'):
            MODULE.validate_navigation_acceptance(
                output, (0,), strict_room_inspection=True)


def test_runner_stops_immediately_on_explicit_navigation_failure():
    source = SCRIPT.read_text(encoding='utf-8')
    assert "observer.latest_state in ('FINISHED', 'FAILED')" in source
    assert "observer.latest_state == 'FAILED'" in source
    assert 'navigation/failures.jsonl' in source
    assert "Clock, '/clock', self._on_clock" in source
    assert 'args.mission_time_budget_sec + 30.0' in source
    assert '超过仿真任务预算 30 秒' in source
    assert 'if not group_exists()' in source
    assert 'launch leader 可能先因缺失可执行文件退出' in source


def test_adapter_status_parser_and_output_contract_fail_closed():
    payload = MODULE.parse_adapter_status(
        'data: "prefix {\\"scenario_seed\\": \\"20260823\\"} suffix"')
    assert payload['scenario_seed'] == '20260823'
    assert MODULE.parse_adapter_status(
        '{"scenario_seed": "20260823"}')['scenario_seed'] == '20260823'
    with pytest.raises(ValueError, match='不含 JSON'):
        MODULE.parse_adapter_status('no payload')

    with tempfile.TemporaryDirectory() as temporary:
        outside = Path(temporary) / 'diagnostic'
        assert MODULE.validate_output_dir(
            outside, git_dirty=True, allow_dirty_diagnostic=True,
        ) == 'diagnostic'
        with pytest.raises(ValueError, match='正式仿真拒绝'):
            MODULE.validate_output_dir(
                outside, git_dirty=True, allow_dirty_diagnostic=False)


def test_preflight_allows_control_odom_but_rejects_ground_truth_tf():
    source = SCRIPT.read_text(encoding='utf-8')
    assert "adapter.get('enable_odom_relay') is not True" in source
    assert '赛事 DWA 控制要求平台转发只读 /hw/odom' in source
    assert "adapter.get('enable_odom_tf_relay') is not False" in source
    assert '禁止平台把 Gazebo odom 转发为 odom→base TF' in source
    assert "adapter.get('enable_pointcloud_relay') is not True" in source


def test_main_entrance_uses_public_service_and_requires_open_ack():
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(
            returncode=0, stdout='accepted: True\nstate: "open"\n', stderr='')

    original = MODULE.subprocess.run
    try:
        MODULE.subprocess.run = fake_run
        result = MODULE.open_main_entrance('simenv_ros1_test')
        assert result['accepted'] is True
        assert calls[0][0][:3] == ['docker', 'exec', 'simenv_ros1_test']
        assert '/set_door_state main_entrance true' in calls[0][0][-1]

        MODULE.subprocess.run = lambda *args, **kwargs: SimpleNamespace(
            returncode=0, stdout='accepted: False\nstate: missing\n', stderr='')
        with pytest.raises(RuntimeError, match='开门服务失败'):
            MODULE.open_main_entrance('simenv_ros1_test')
    finally:
        MODULE.subprocess.run = original


def test_default_ingress_clears_the_complete_a1_footprint_from_door_frame():
    source = SCRIPT.read_text(encoding='utf-8')
    assert 'distance_m: float = 3.6' in source
    assert "'--entrance-distance-m', type=float, default=3.6" in source
    assert 'and travelled >= float(distance_m)' in source
    assert 'and node.front_clearance >= 0.80' in source
    assert "'distance_clearance_fallback'" in source


def test_pointcloud_save_must_ack_before_launch_shutdown():
    original = MODULE.run_ros2_cli
    try:
        MODULE.run_ros2_cli = (
            lambda *args, **kwargs: 'response: success=True, message=ok')
        assert MODULE.save_pointcloud_map()['success'] is True

        MODULE.run_ros2_cli = (
            lambda *args, **kwargs: 'response: success=False, message=empty')
        with pytest.raises(RuntimeError, match='未成功'):
            MODULE.save_pointcloud_map()
    finally:
        MODULE.run_ros2_cli = original

    source = SCRIPT.read_text(encoding='utf-8')
    completion = source.split(
        "if observer.latest_state in ('FINISHED', 'FAILED'):", 1)[1].split(
            'if process.poll() is not None:', 1)[0]
    assert completion.index('if args.enable_3d_map:') < completion.index(
        'save_pointcloud_map()')
    assert "'reason': '2d_slam_profile'" in completion


def test_runner_marks_ros_context_shutdown_as_interrupted():
    source = SCRIPT.read_text(encoding='utf-8')
    assert 'rclpy.executors.ExternalShutdownException' in source
    assert "manifest['status'] = 'interrupted'" in source


def test_first_person_recording_is_container_local_and_converted_to_mp4():
    source = SCRIPT.read_text(encoding='utf-8')
    assert 'image:=/real_sense/rgb/image_raw' in source
    assert '_fps:=5 _codec:=MJPG' in source
    assert "'libx264'" in source
    assert "'pkill', '-INT'" in source
    assert MODULE._safe_run_slug('测试 run/01') == 'run_01'


def test_slam_starts_at_public_spawn_before_ingress_and_navigation_release():
    source = SCRIPT.read_text(encoding='utf-8')
    ingress = source.split('def perform_entrance_ingress', 1)[1].split(
        'def write_handoff', 1)[0]
    assert "'/hw/scan'" in ingress
    assert "'/hw/control/navigation_cmd_vel'" in ingress
    assert "'/hazardwalker/slam/odometry'" in ingress
    assert "'/hw/odom'" not in ingress
    main_source = source.split('def main()', 1)[1]
    assert main_source.index('validate_perception_mission_config(') < main_source.index(
        'open_main_entrance(container)')
    assert main_source.index('validate_navigation_clearance_contract(') < main_source.index(
        'open_main_entrance(container)')
    assert main_source.index('read_absolute_trunk_imu_yaw()') < main_source.index(
        'command = build_launch_command(')
    assert main_source.index('command = build_launch_command(') < main_source.index(
        'process = subprocess.Popen(')
    assert main_source.index('process = subprocess.Popen(') < main_source.index(
        'wait_for_slam_bootstrap()')
    assert main_source.index('wait_for_slam_bootstrap()') < main_source.index(
        'perform_entrance_ingress(')
    assert main_source.index('perform_entrance_ingress(') < main_source.index(
        'release_navigation_after_ingress()')
    assert 'start_temporary_localizer=False' in main_source
    assert "args.enable_perception or args.strict_room_inspection" in main_source
    assert "manifest['status'] == 'complete' and perception_enabled" in main_source
    assert "f'command_motion_scale:={A1_EXECUTION_SCALE:.2f}'" in source

    with pytest.raises(ValueError, match='必须为正数'):
        MODULE.perform_entrance_ingress(distance_m=0.0)


def test_entrance_structure_detector_distinguishes_lobby_and_door_frame():
    angle_min = -math.pi
    angle_increment = 2.0 * math.pi / 360.0
    lobby = [6.0] * 360

    def set_sector(values, low_deg, high_deg, value):
        for index in range(len(values)):
            degree = math.degrees(angle_min + index * angle_increment)
            if low_deg <= degree <= high_deg:
                values[index] = value

    set_sector(lobby, 70.0, 115.0, 1.2)
    set_sector(lobby, -115.0, -70.0, 3.2)
    set_sector(lobby, -70.0, -20.0, 7.5)
    assert MODULE.entrance_lobby_structure_detected(
        lobby, angle_min, angle_increment)

    deep_lobby = list(lobby)
    set_sector(deep_lobby, 70.0, 115.0, 4.4)
    set_sector(deep_lobby, -115.0, -70.0, 1.75)
    assert MODULE.entrance_lobby_structure_detected(
        deep_lobby, angle_min, angle_increment)

    exterior = list(lobby)
    exterior[:80] = [math.inf] * 80
    assert not MODULE.entrance_lobby_structure_detected(
        exterior, angle_min, angle_increment)

    door_frame = list(lobby)
    set_sector(door_frame, -15.0, 15.0, 2.4)
    assert not MODULE.entrance_lobby_structure_detected(
        door_frame, angle_min, angle_increment)

    seen, streak = MODULE.update_entrance_structure_state(
        False, 0, 2.4, False)
    assert seen is True and streak == 0
    for _ in range(3):
        seen, streak = MODULE.update_entrance_structure_state(
            seen, streak, 6.0, False)
    assert seen is True and streak == 3


def test_runner_writes_and_enforces_post_run_slam_physical_alignment():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        trajectory = root / 'trajectory.jsonl'
        rows = []
        for index in range(20):
            rows.append({
                'ros_sec': float(index),
                'x': index * 0.2,
                'y': 0.0,
                'yaw_deg': 0.0,
                'official_x': 3.0 + index * 0.2,
                'official_y': -2.0,
                'official_yaw_deg': 0.0,
            })
        trajectory.write_text(
            ''.join(json.dumps(row) + '\n' for row in rows),
            encoding='utf-8')
        output = root / 'slam' / 'physical_alignment.json'
        metrics = MODULE.evaluate_slam_physical_alignment(
            trajectory, output, p95_limit_m=0.1, max_limit_m=0.2)
        assert metrics['accepted'] is True
        assert metrics['max_error_m'] == pytest.approx(0.0, abs=1e-9)
        assert json.loads(output.read_text(encoding='utf-8'))['accepted'] is True

    source = SCRIPT.read_text(encoding='utf-8')
    assert "'slam_physical_alignment': None" in source
    assert "output_dir / 'slam' / 'physical_alignment.json'" in source
    assert "not manifest['slam_physical_alignment']['accepted']" in source


def test_runner_waits_for_perception_result_before_stopping_launch():
    source = SCRIPT.read_text(encoding='utf-8')
    completion = source.split(
        "if observer.latest_state in ('FINISHED', 'FAILED'):", 1)[1].split(
            'if process.poll() is not None:', 1)[0]
    assert "output_dir / 'detected_danger.json'" in completion
    assert completion.index('wait_for_nonempty_file(') < completion.rindex('break')

    with tempfile.TemporaryDirectory() as temporary:
        target = Path(temporary) / 'detected.json'
        target.write_text('{}\n', encoding='utf-8')
        evidence = MODULE.wait_for_nonempty_file(target, timeout_sec=0.2)
        assert evidence['size_bytes'] > 0
