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
    assert 'navigation_linear_speed:=2.00' in joined
    assert 'navigation_minimum_linear_speed:=1.20' in joined
    assert 'mission_time_budget_s:=600.000' in joined
    assert 'strict_room_inspection:=false' in joined
    assert 'start_perception:=false' in joined
    assert 'start_evidence_recorder:=false' in joined
    assert 'perception_output_frame:=map' in joined
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
        strict_room_inspection=True))
    assert 'strict_room_inspection:=true' in strict
    assert 'start_perception:=true' in strict
    assert 'start_evidence_recorder:=true' in strict
    assert 'perception_output_frame:=world' in strict
    assert 'official_result_path:=/tmp/nav-run/detected_danger.json' in strict

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


def test_entrance_ingress_precedes_slam_and_uses_only_public_inputs():
    source = SCRIPT.read_text(encoding='utf-8')
    ingress = source.split('def perform_entrance_ingress', 1)[1].split(
        'def write_handoff', 1)[0]
    assert "'/hw/scan'" in ingress
    assert "'/hw/control/navigation_cmd_vel'" in ingress
    assert "'/hazardwalker/slam/odometry'" in ingress
    assert "'/hw/odom'" not in ingress
    main_source = source.split('def main()', 1)[1]
    assert main_source.index('open_main_entrance(container)') < main_source.index(
        'perform_entrance_ingress(')
    assert main_source.index('perform_entrance_ingress(') < main_source.index(
        'command = build_launch_command(')

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
