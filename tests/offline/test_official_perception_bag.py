"""官方人工巡检 rosbag 录制与安全回放契约测试。"""

import importlib.util
from datetime import datetime, timedelta, timezone
import hashlib
import json
import sqlite3
import tempfile
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'official_perception_bag.py'
SPEC = importlib.util.spec_from_file_location('official_perception_bag', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_record_topics_cover_reproducible_rgbd_localization_and_control():
    required = {
        '/hw/camera/image_raw',
        '/hw/camera/depth_image',
        '/hw/camera/camera_info',
        '/hw/camera/depth_camera_info',
        '/hw/scan', '/hw/trunk_imu',
        '/tf', '/tf_static',
        '/hazardwalker/slam/odometry',
        '/hazardwalker/slam/localization_provenance',
        '/map',
        '/hw/platform/official_simenv_adapter_status',
        '/hw/cmd_vel',
        '/hw/perception/hazard_detections',
        '/hw/perception/view_recommendation',
    }
    assert required <= set(MODULE.RECORD_TOPICS)
    assert all('truth' not in topic.lower() for topic in MODULE.RECORD_TOPICS)
    assert '/hw/map' not in MODULE.RECORD_TOPICS
    assert set(MODULE.REQUIRED_RECORD_TOPICS) <= set(MODULE.RECORD_TOPICS)
    command = MODULE.build_record_command(
        Path('/tmp/example-bag'), MODULE.RECORD_TOPICS)
    assert '--use-sim-time' in command
    assert command.count('--topics') == 1


def test_formal_record_preflight_requires_every_requested_data_class():
    required = set(MODULE.REQUIRED_RECORD_TOPICS)
    assert {
        '/clock',
        '/hw/camera/image_raw',
        '/hw/camera/depth_image',
        '/hw/camera/camera_info',
        '/hw/camera/depth_camera_info',
        '/hw/scan', '/hw/trunk_imu',
        '/tf', '/tf_static',
        '/hazardwalker/slam/odometry',
        '/hazardwalker/slam/localization_provenance', '/map',
        '/hw/platform/official_simenv_adapter_status',
        '/hw/control/status',
        '/hw/control/assist_status',
        '/hw/cmd_vel',
        '/hw/perception/hazard_detections',
        '/hw/perception/view_recommendation',
    } <= required
    assert MODULE.REQUIRED_ANY_TOPIC_GROUPS == {
        'control_source': (
            '/hw/control/keyboard_cmd_vel',
            '/hw/control/navigation_cmd_vel',
        ),
    }


def test_keyboard_or_navigation_can_supply_the_same_recording_control_contract():
    common = set(MODULE.REQUIRED_RECORD_TOPICS)
    assert MODULE.find_missing_topic_requirements(
        common | {'/hw/control/keyboard_cmd_vel'}) == []
    assert MODULE.find_missing_topic_requirements(
        common | {'/hw/control/navigation_cmd_vel'}) == []
    missing = MODULE.find_missing_topic_requirements(common)
    assert len(missing) == 1
    assert missing[0].startswith('control_source:one_of(')


def test_topic_list_parser_ignores_non_topic_diagnostics():
    topics = MODULE.parse_topic_list(
        'warning: daemon restarted\n/hw/camera/image_raw\n\n/tf\n')
    assert topics == {'/hw/camera/image_raw', '/tf'}


def test_runtime_localization_provenance_echo_parser_handles_field_and_yaml():
    assert MODULE.parse_localization_provenance_echo(
        'lidar_imu_slam\n---\n') == 'lidar_imu_slam'
    assert MODULE.parse_localization_provenance_echo(
        "warning: daemon restarted\n"
        "data: 'lidar_imu_slam+public_floor_action'\n---\n"
    ) == 'lidar_imu_slam+public_floor_action'


def test_runtime_localization_provenance_cli_uses_exact_transient_qos(monkeypatch):
    """当前 Jazzy/FastDDS 需要完整 QoS 才能取得 transient-local 缓存。"""
    captured = {}

    def fake_check_output(command, **kwargs):
        captured['command'] = command
        captured['kwargs'] = kwargs
        return 'lidar_imu_slam\n---\n'

    monkeypatch.setattr(MODULE.subprocess, 'check_output', fake_check_output)

    assert MODULE.read_runtime_localization_provenance(3.0) == 'lidar_imu_slam'
    command = captured['command']
    assert command[command.index('--qos-durability') + 1] == 'transient_local'
    assert command[command.index('--qos-reliability') + 1] == 'reliable'
    assert command[command.index('--qos-history') + 1] == 'keep_last'
    assert command[command.index('--qos-depth') + 1] == '1'
    assert captured['kwargs']['timeout'] == 3.0


def test_patrol_coverage_echo_parser_ignores_diagnostics():
    payload = MODULE.parse_patrol_coverage_echo(
        'warning: daemon restarted\n'
        "data: '{\"sample_count\": 40, \"planar_path_length_m\": 9.0, "
        "\"planar_span_m\": 3.5}'\n---\n")
    assert payload['sample_count'] == 40
    assert MODULE.parse_patrol_coverage_echo('data: not-json\n') == {}


def _managed_adapter_status():
    return {
        'adapter': 'rosbridge_ros2',
        'managed_lifecycle': True,
        'lifecycle_container': 'simenv_ros1_hazard_platform',
        'enable_cmd_vel_relay': True,
        'enable_gui_overlay_relay': True,
        'gui_assist_request_topic': '/hazardwalker/gui/assist_request',
        'enable_image_relay': True,
        'image_throttle_rate_ms': 200,
        'enable_clock_relay': True,
        'clock_throttle_rate_ms': 20,
        'enable_pointcloud_relay': False,
        'enable_livox_imu_relay': False,
        'enable_trunk_imu_relay': True,
        'enable_odom_relay': False,
        'odom_throttle_rate_ms': 20,
        'enable_tf_relay': True,
        'tf_throttle_rate_ms': 20,
        'scan_throttle_rate_ms': 50,
        'scan_self_filter_range_m': 0.4,
        'sources': {'rgb': '/real_sense/rgb/image_raw'},
        'received': {'rgb': 123},
    }


def test_adapter_status_parser_and_contract_ignore_dynamic_counters():
    status = _managed_adapter_status()
    echoed = "warning: harmless\ndata: '" + json.dumps(status) + "'\n---\n"
    parsed = MODULE.parse_adapter_status_echo(echoed)
    assert parsed == status
    before = MODULE.adapter_contract_snapshot(parsed)
    status['received']['rgb'] = 456
    status['forwarded_cmd_count'] = 9
    assert MODULE.adapter_contract_snapshot(status) == before


def test_adapter_contract_rejects_unmanaged_or_changed_dataflow():
    status = _managed_adapter_status()
    status['managed_lifecycle'] = False
    try:
        MODULE.adapter_contract_snapshot(status)
    except ValueError as exc:
        assert 'auto_docker.sh' in str(exc)
    else:
        raise AssertionError('未受管适配器必须被拒绝')

    before = _managed_adapter_status()
    after = _managed_adapter_status()
    after['enable_image_relay'] = False
    assert (MODULE.adapter_contract_snapshot(before)
            != MODULE.adapter_contract_snapshot(after))

    slow = _managed_adapter_status()
    slow['image_throttle_rate_ms'] = 500
    try:
        MODULE.adapter_contract_snapshot(slow)
    except ValueError as exc:
        assert '250 ms' in str(exc)
    else:
        raise AssertionError('过慢图像桥接必须被正式录包合同拒绝')


def test_default_replay_does_not_publish_historical_control_or_detections():
    command = MODULE.build_replay_command(Path('/tmp/example-bag'))
    joined = ' '.join(command)
    assert '/hw/camera/image_raw' in command
    assert '/map' in command
    assert '/hazardwalker/slam/localization_provenance' in command
    assert '--clock' in command
    assert '/clock' not in command
    assert '/hw/cmd_vel' not in command
    assert '/hw/perception/hazard_detections' not in command
    assert '/hw/replay' not in joined


def test_audit_replay_remaps_every_output_away_from_live_topics():
    command = MODULE.build_replay_command(
        Path('/tmp/example-bag'), include_audit_topics=True)
    joined = ' '.join(command)
    assert '/hw/cmd_vel:=/hw/replay/cmd_vel' in joined
    assert (
        '/hw/perception/hazard_detections:='
        '/hw/replay/perception/hazard_detections'
    ) in joined
    assert (
        '/hw/platform/official_simenv_adapter_status:='
        '/hw/replay/platform/official_simenv_adapter_status'
    ) in joined
    assert command.count('--remap') == 1
    remap_index = command.index('--remap')
    topics_index = command.index('--topics')
    assert topics_index > remap_index
    assert len(command[remap_index + 1:topics_index]) == len(
        MODULE.AUDIT_REPLAY_TOPICS)


def test_non_positive_replay_rate_is_rejected():
    try:
        MODULE.build_replay_command(Path('/tmp/example-bag'), rate=0.0)
    except ValueError:
        pass
    else:
        raise AssertionError('非正回放倍率必须被拒绝')


def test_replay_segment_is_explicitly_bounded_in_command():
    command = MODULE.build_replay_command(
        Path('/tmp/example-bag'),
        rate=2.0,
        start_offset_sec=2400.0,
        playback_duration_sec=500.0,
    )
    assert command[command.index('--start-offset') + 1] == '2400.0'
    assert command[command.index('--playback-duration') + 1] == '500.0'


def test_negative_replay_segment_bounds_are_rejected():
    with pytest.raises(ValueError, match='起始偏移'):
        MODULE.build_replay_command(
            Path('/tmp/example-bag'), start_offset_sec=-1.0,
        )
    with pytest.raises(ValueError, match='片段时长'):
        MODULE.build_replay_command(
            Path('/tmp/example-bag'), playback_duration_sec=-1.0,
        )


def test_localization_recompute_replays_raw_sensors_without_old_pose_or_tf():
    command = MODULE.build_replay_command(
        Path('/tmp/example-bag'), recompute_localization=True)
    assert '/hw/scan' in command and '/hw/trunk_imu' in command
    assert '/hazardwalker/slam/odometry' not in command
    assert '/hazardwalker/slam/localization_provenance' not in command
    assert '/tf' not in command and '/tf_static' not in command


def test_recorded_sqlite_message_counts_are_summed_across_bag_splits():
    with tempfile.TemporaryDirectory() as temporary:
        bag_dir = Path(temporary)
        for index, image_count in enumerate((2, 3)):
            database = sqlite3.connect(str(bag_dir / f'part_{index}.db3'))
            database.executescript(
                'CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT);'
                'CREATE TABLE messages ('
                'id INTEGER PRIMARY KEY, topic_id INTEGER);'
                "INSERT INTO topics(id, name) VALUES "
                "(1, '/hw/camera/image_raw'), (2, '/tf');"
            )
            database.executemany(
                'INSERT INTO messages(topic_id) VALUES (1)',
                [()] * image_count,
            )
            database.execute('INSERT INTO messages(topic_id) VALUES (2)')
            database.commit()
            database.close()

        assert MODULE.inspect_sqlite_bag_message_counts(bag_dir) == {
            '/hw/camera/image_raw': 5,
            '/tf': 2,
        }


def test_recorded_sqlite_statistics_report_full_split_time_span():
    with tempfile.TemporaryDirectory() as temporary:
        bag_dir = Path(temporary)
        for index, timestamps in enumerate(((1_000_000_000, 3_000_000_000),
                                            (5_000_000_000, 7_000_000_000))):
            database = sqlite3.connect(str(bag_dir / f'part_{index}.db3'))
            database.executescript(
                'CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT);'
                'CREATE TABLE messages ('
                'id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER);'
                "INSERT INTO topics(id, name) VALUES "
                "(1, '/hw/camera/image_raw');"
            )
            database.executemany(
                'INSERT INTO messages(topic_id, timestamp) VALUES (1, ?)',
                [(timestamp,) for timestamp in timestamps],
            )
            database.commit()
            database.close()

        statistics = MODULE.inspect_sqlite_bag_statistics(bag_dir)
        assert statistics['message_counts'] == {
            '/hw/camera/image_raw': 4}
        assert statistics['first_timestamp_ns'] == 1_000_000_000
        assert statistics['last_timestamp_ns'] == 7_000_000_000
        assert statistics['duration_sec'] == 6.0


def test_bag_content_fingerprint_changes_when_any_file_changes():
    with tempfile.TemporaryDirectory() as temporary:
        bag_dir = Path(temporary)
        database = bag_dir / 'part.db3'
        metadata = bag_dir / 'metadata.yaml'
        database.write_bytes(b'first')
        metadata.write_text('version: 1\n', encoding='utf-8')
        first = MODULE.fingerprint_bag_directory(bag_dir)
        database.write_bytes(b'second')
        second = MODULE.fingerprint_bag_directory(bag_dir)
        assert first['sha256'] != second['sha256']
        assert [item['relative_path'] for item in second['files']] == [
            'metadata.yaml', 'part.db3']


def _complete_manifest(duration_sec=120.0):
    counts = {topic: 1 for topic in MODULE.REQUIRED_RECORD_TOPICS}
    counts['/hw/control/keyboard_cmd_vel'] = 1
    preflight_bytes = _preflight_bytes()
    return {
        'status': 'complete',
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
        'bag_validation': {
            'status': 'passed',
            'message_counts': counts,
            'duration_sec': duration_sec,
            'content_fingerprint_sha256': 'b' * 64,
            'files': [{
                'relative_path': 'part.db3',
                'size_bytes': 1,
                'sha256': 'c' * 64,
            }],
        },
        'patrol_coverage': {
            'status': 'passed',
            'minimum_samples': MODULE.MINIMUM_PATROL_COVERAGE_SAMPLES,
            'minimum_path_length_m': MODULE.MINIMUM_PATROL_PATH_LENGTH_M,
            'minimum_planar_span_m': MODULE.MINIMUM_PATROL_PLANAR_SPAN_M,
            'metrics': {
                'sample_count': 100,
                'planar_path_length_m': 12.0,
                'planar_span_m': 5.0,
                'vertical_span_m': 2.6,
            },
            'errors': [],
        },
    }


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


def _write_complete_bag(bag_dir):
    (bag_dir.parent / 'live_chain_preflight.json').write_bytes(
        _preflight_bytes())
    topics = list(MODULE.REQUIRED_RECORD_TOPICS) + [
        '/hw/control/keyboard_cmd_vel']
    database = sqlite3.connect(str(bag_dir / 'part.db3'))
    database.executescript(
        'CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT);'
        'CREATE TABLE messages ('
        'id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER);'
    )
    for topic_id, topic in enumerate(topics, start=1):
        database.execute(
            'INSERT INTO topics(id, name) VALUES (?, ?)', (topic_id, topic))
        database.execute(
            'INSERT INTO messages(topic_id, timestamp) VALUES (?, ?)',
            (topic_id, 1_000_000_000))
        database.execute(
            'INSERT INTO messages(topic_id, timestamp) VALUES (?, ?)',
            (topic_id, 121_000_000_000))
    database.commit()
    database.close()
    return topics


def test_formal_record_contract_requires_seed_and_legal_localization():
    MODULE.validate_record_contract('20260803', 'lidar_imu_slam')
    for seed, provenance in (
            ('', 'lidar_imu_slam'), ('20260803', 'unverified')):
        try:
            MODULE.validate_record_contract(seed, provenance)
        except ValueError:
            pass
        else:
            raise AssertionError('不可追溯录制元数据必须被拒绝')


def test_completed_manifest_rejects_stationary_patrol_coverage():
    manifest = _complete_manifest()
    manifest['patrol_coverage']['metrics'].update({
        'planar_path_length_m': 0.0,
        'planar_span_m': 0.0,
    })
    errors = MODULE.validate_completed_session_manifest(manifest)
    assert any('平面路程' in item for item in errors)
    assert any('覆盖跨度' in item for item in errors)


def test_formal_record_rejects_dirty_code_but_diagnostic_can_opt_in():
    state = {
        'commit': 'abc', 'dirty': True,
        'dirty_entries': ['config/perception.yaml'],
    }
    for skip_preflight, allow_dirty in ((False, True), (True, False)):
        try:
            MODULE.validate_git_state_for_record(
                state,
                skip_topic_preflight=skip_preflight,
                allow_dirty_worktree=allow_dirty,
            )
        except ValueError as exc:
            assert '正式巡检拒绝未提交' in str(exc)
        else:
            raise AssertionError('正式巡检不能接受未提交代码')
    MODULE.validate_git_state_for_record(
        state,
        skip_topic_preflight=True,
        allow_dirty_worktree=True,
    )


def test_live_preflight_report_must_be_recent_passed_and_same_provenance():
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / 'preflight.json'
        now = datetime.now(timezone.utc)
        payload = {
            'passed': True,
            'failures': [],
            'traffic_checked': True,
            'control_source': 'keyboard',
            'expected_localization_provenance': 'lidar_imu_slam',
            'git': {'branch': 'feature/test', 'commit': 'abc', 'dirty': False},
            'generated_at_utc': (now - timedelta(seconds=5)).isoformat(),
        }
        path.write_text(json.dumps(payload), encoding='utf-8')
        result = MODULE.validate_live_preflight_report(
            path,
            expected_localization_provenance='lidar_imu_slam',
            maximum_age_sec=30.0,
            expected_git_commit='abc',
            now_utc=now,
        )
        assert result['passed'] is True
        assert len(result['sha256']) == 64
        assert 4.0 <= result['age_sec_at_record_start'] <= 6.0

        payload['generated_at_utc'] = (
            now - timedelta(seconds=31)).isoformat()
        path.write_text(json.dumps(payload), encoding='utf-8')
        try:
            MODULE.validate_live_preflight_report(
                path,
                expected_localization_provenance='lidar_imu_slam',
                maximum_age_sec=30.0,
                expected_git_commit='abc',
                now_utc=now,
            )
        except ValueError as exc:
            assert '过期' in str(exc)
        else:
            raise AssertionError('过期实时预检报告必须被拒绝')

        payload['generated_at_utc'] = now.isoformat()
        payload['git']['dirty'] = True
        path.write_text(json.dumps(payload), encoding='utf-8')
        try:
            MODULE.validate_live_preflight_report(
                path,
                expected_localization_provenance='lidar_imu_slam',
                maximum_age_sec=30.0,
                expected_git_commit='abc',
                now_utc=now,
            )
        except ValueError as exc:
            assert '未提交代码' in str(exc)
        else:
            raise AssertionError('脏代码预检报告必须被拒绝')


def test_completed_manifest_is_revalidated_instead_of_trusting_passed_flag():
    manifest = _complete_manifest()
    assert MODULE.validate_completed_session_manifest(manifest) == []

    manifest['bag_validation']['message_counts']['/map'] = 0
    errors = MODULE.validate_completed_session_manifest(manifest)
    assert any('/map' in error for error in errors)


def test_completed_manifest_rejects_too_short_patrol():
    errors = MODULE.validate_completed_session_manifest(
        _complete_manifest(MODULE.MINIMUM_PATROL_DURATION_SEC - 0.1))
    assert any('巡检时长' in error for error in errors)


def test_completed_manifest_rejects_unproven_runtime_localization_source():
    manifest = _complete_manifest()
    manifest['runtime_localization_provenance'] = (
        'lidar_imu_slam+public_floor_action')
    errors = MODULE.validate_completed_session_manifest(manifest)
    assert any('运行时定位来源' in error for error in errors)


def test_completed_manifest_requires_messages_from_preflight_selected_control():
    manifest = _complete_manifest()
    counts = manifest['bag_validation']['message_counts']
    counts['/hw/control/keyboard_cmd_vel'] = 0
    counts['/hw/control/navigation_cmd_vel'] = 5
    errors = MODULE.validate_completed_session_manifest(manifest)
    assert any('所选控制源' in error for error in errors)


def test_session_payload_is_read_from_sqlite_and_must_match_manifest():
    with tempfile.TemporaryDirectory() as temporary:
        bag_dir = Path(temporary)
        topics = _write_complete_bag(bag_dir)
        manifest = _complete_manifest()
        manifest['bag_validation'].update({
            'message_counts': {topic: 2 for topic in topics},
            'first_timestamp_ns': 1_000_000_000,
            'last_timestamp_ns': 121_000_000_000,
        })
        fingerprint = MODULE.fingerprint_bag_directory(bag_dir)
        manifest['bag_validation'].update({
            'content_fingerprint_sha256': fingerprint['sha256'],
            'files': fingerprint['files'],
        })
        assert MODULE.validate_session_bag_payload(bag_dir, manifest) == []

        (bag_dir.parent / 'live_chain_preflight.json').write_bytes(b'changed')
        errors = MODULE.validate_session_bag_payload(bag_dir, manifest)
        assert any('预检报告副本哈希' in error for error in errors)
        (bag_dir.parent / 'live_chain_preflight.json').write_bytes(
            _preflight_bytes())

        altered = json.loads(_preflight_bytes().decode('utf-8'))
        altered['control_source'] = 'navigation'
        altered_bytes = json.dumps(altered, sort_keys=True).encode('utf-8')
        (bag_dir.parent / 'live_chain_preflight.json').write_bytes(altered_bytes)
        manifest['live_chain_preflight']['sha256'] = hashlib.sha256(
            altered_bytes).hexdigest()
        errors = MODULE.validate_session_bag_payload(bag_dir, manifest)
        assert any('control_source' in error for error in errors)

        (bag_dir.parent / 'live_chain_preflight.json').write_bytes(
            _preflight_bytes())
        manifest['live_chain_preflight']['sha256'] = hashlib.sha256(
            _preflight_bytes()).hexdigest()

        manifest['bag_validation']['message_counts']['/map'] = 99
        errors = MODULE.validate_session_bag_payload(bag_dir, manifest)
        assert any('消息计数' in error for error in errors)


def test_missing_or_empty_bag_is_not_accepted_as_recorded_data():
    with tempfile.TemporaryDirectory() as temporary:
        try:
            MODULE.inspect_sqlite_bag_message_counts(Path(temporary))
        except ValueError:
            pass
        else:
            raise AssertionError('没有数据库的 rosbag 必须判为无效')
