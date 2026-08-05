"""人工巡检实时链只读预检器离线测试。"""

import importlib.util
import io
import json
from pathlib import Path
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'verify_perception_live_chain.py'
SPEC = importlib.util.spec_from_file_location('verify_live_chain', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _endpoint(name):
    return {'node_name': name, 'node_namespace': '/'}


class _HealthResponse:
    """模拟 urllib 响应，确保健康检查只解析 JSON 且正确传递超时。"""

    def __init__(self, payload: bytes):
        self._stream = io.BytesIO(payload)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        return False

    def read(self):
        return self._stream.read()


def _valid_snapshot(control_source='keyboard'):
    topics = {}
    expected = dict(MODULE.TOPIC_TYPES)
    source = MODULE.CONTROL_SOURCE_TOPICS[control_source]
    expected[source] = 'geometry_msgs/msg/Twist'
    if control_source == 'navigation':
        expected.update(MODULE.NAVIGATION_TOPIC_TYPES)
    for topic, type_name in expected.items():
        topics[topic] = {
            'types': [type_name],
            'publishers': [_endpoint('generic_publisher')],
            'subscribers': [_endpoint('generic_subscriber')],
        }
    topics['/hw/cmd_vel']['publishers'] = [
        _endpoint('hazardwalker_command_mux')]
    topics[source]['subscribers'] = [_endpoint('hazardwalker_command_mux')]
    for topic in MODULE.PLATFORM_RELAY_TOPICS:
        topics[topic]['publishers'] = [
            _endpoint('hazardwalker_official_rosbridge_adapter')]
    topics['/map']['publishers'] = [
        _endpoint('hazardwalker_cartographer_occupancy_grid')]
    if control_source == 'navigation':
        topics['/hw/perception/hazard_detections']['subscribers'].append(
            _endpoint('frontier_explorer_node'))
    for topic, node in (
        ('/hw/control/assist_cmd_vel', 'hazardwalker_assist_alignment'),
        ('/hw/control/status', 'hazardwalker_command_mux'),
        ('/hw/control/assist_status', 'hazardwalker_assist_alignment'),
        ('/hw/platform/official_simenv_adapter_status',
         'hazardwalker_official_rosbridge_adapter'),
        ('/hazardwalker/slam/localization_provenance',
         'hazardwalker_scan_imu_localizer'),
        ('/hazardwalker/slam/odometry',
         'hazardwalker_scan_imu_localizer'),
        ('/hw/perception/hazard_detections', 'hsv_detector_node'),
        ('/hw/perception/view_recommendation', 'hsv_detector_node'),
        ('/hw/perception/patrol_coverage',
         'hazardwalker_patrol_coverage'),
    ):
        topics[topic]['publishers'] = [_endpoint(node)]
    return {
        'topics': topics,
        'services': list(MODULE.REQUIRED_SERVICES),
        'traffic_checked': True,
        'sample_sec': 3.0,
        'message_counts': {
            topic: 20 for topic in (
                list(MODULE.REQUIRED_LIVE_TOPICS)
                + (list(MODULE.NAVIGATION_TOPIC_TYPES)
                   if control_source == 'navigation' else []))},
        'latest_string_payloads': {
            '/hazardwalker/slam/localization_provenance': 'lidar_imu_slam',
            '/hw/perception/patrol_coverage': (
                '{"sample_count": 0, "planar_path_length_m": 0.0, '
                '"planar_span_m": 0.0}'),
            '/hw/platform/official_simenv_adapter_status': (
                '{"managed_lifecycle": true, '
                '"lifecycle_container": "simenv_ros1_hazard_platform", '
                '"enable_cmd_vel_relay": true, '
                '"enable_gui_overlay_relay": true, '
                '"image_throttle_rate_ms": 200, '
                '"gui_assist_request_topic": '
                '"/hazardwalker/gui/assist_request"}')},
        'first_person_health': {
            'frame': {
                'ready': True,
                'frame_age_sec': 0.1,
                'ros_stamp_sec': 123.0,
            },
            'overlay': {
                'perception': {
                    'stamp_sec': 123.02,
                    'image_width': 640,
                    'image_height': 480,
                    'detections_2d': [],
                    'view_recommendation': {
                        'action': 'continue_exploring'},
                },
                'state_age_sec': {
                    'perception': 0.1, 'control': 0.2, 'assist': 0.2},
            },
        },
    }


def test_valid_keyboard_chain_passes_without_requiring_keypress_traffic():
    snapshot = _valid_snapshot('keyboard')
    assert MODULE.evaluate_snapshot(snapshot, 'keyboard') == []


def test_formal_preflight_requires_clean_versioned_code():
    assert MODULE.evaluate_git_state(
        {'commit': 'abc', 'dirty': False}, graph_only=False) == []
    failures = MODULE.evaluate_git_state(
        {'commit': '', 'dirty': True}, graph_only=False)
    assert any('Git 提交' in item for item in failures)
    assert any('未提交代码' in item for item in failures)
    assert MODULE.evaluate_git_state(
        {'commit': '', 'dirty': True}, graph_only=True) == []


def test_fetch_first_person_health_reads_json_without_control_request():
    calls = []

    def fake_urlopen(request, timeout):
        calls.append((request.full_url, request.get_header('Accept'), timeout))
        return _HealthResponse(
            b'{"frame":{"ready":true},"overlay":{"state_age_sec":{}}}')

    original_urlopen = MODULE.urlopen
    MODULE.urlopen = fake_urlopen
    try:
        result = MODULE.fetch_first_person_health(
            'http://127.0.0.1:6082/healthz', timeout_sec=1.25)
    finally:
        MODULE.urlopen = original_urlopen

    assert result['frame']['ready'] is True
    assert calls == [
        ('http://127.0.0.1:6082/healthz', 'application/json', 1.25)]


def _assert_invalid_health_payload(payload):
    original_urlopen = MODULE.urlopen
    MODULE.urlopen = lambda request, timeout: _HealthResponse(payload)
    try:
        with pytest.raises(ValueError, match='第一人称健康接口'):
            MODULE.fetch_first_person_health('http://127.0.0.1:6082/healthz')
    finally:
        MODULE.urlopen = original_urlopen


def test_fetch_first_person_health_rejects_non_object_payload():
    _assert_invalid_health_payload(b'[]')


def test_fetch_first_person_health_rejects_invalid_json():
    _assert_invalid_health_payload(b'not-json')


def test_navigation_source_uses_same_mux_contract():
    snapshot = _valid_snapshot('navigation')
    assert MODULE.evaluate_snapshot(snapshot, 'navigation') == []


def test_runtime_localization_provenance_must_match_expected_source():
    snapshot = _valid_snapshot()
    failures = MODULE.evaluate_snapshot(
        snapshot, 'keyboard',
        'lidar_imu_slam+public_floor_action')
    assert any('定位来源与本轮预检声明不一致' in item for item in failures)


def test_legal_odometry_rejects_wrong_or_duplicate_publisher():
    snapshot = _valid_snapshot()
    snapshot['topics']['/hazardwalker/slam/odometry']['publishers'] = [
        _endpoint('gazebo_truth_relay'), _endpoint('stale_localizer')]
    failures = MODULE.evaluate_snapshot(
        snapshot, 'keyboard', 'lidar_imu_slam')
    assert any('恰好一个合法定位发布者' in item for item in failures)


def test_navigation_without_frontier_reobservation_subscription_is_rejected():
    snapshot = _valid_snapshot('navigation')
    snapshot['topics']['/hw/perception/hazard_detections']['subscribers'] = [
        _endpoint('dynamic_detection_recorder_node')]
    failures = MODULE.evaluate_snapshot(snapshot, 'navigation')
    assert any('Frontier' in item or 'frontier_explorer_node' in item
               for item in failures)


def test_multiple_cmd_vel_publishers_and_dead_rgb_fail_closed():
    snapshot = _valid_snapshot()
    snapshot['topics']['/hw/cmd_vel']['publishers'].append(
        _endpoint('rogue_navigation'))
    snapshot['message_counts']['/hw/camera/image_raw'] = 0
    failures = MODULE.evaluate_snapshot(snapshot, 'keyboard')
    assert any('恰好一个发布者' in item for item in failures)
    assert any('采样窗口无消息：/hw/camera/image_raw' in item
               for item in failures)


def test_formal_preflight_rejects_non_realtime_rgbd_and_detection_rates():
    snapshot = _valid_snapshot()
    for topic in MODULE.MINIMUM_LIVE_RATE_HZ:
        snapshot['message_counts'][topic] = 1
    failures = MODULE.evaluate_snapshot(snapshot, 'keyboard')
    assert any('实时频率不足：/hw/camera/image_raw' in item
               for item in failures)
    assert any('实时频率不足：/hw/perception/hazard_detections' in item
               for item in failures)


def test_duplicate_platform_adapter_and_missing_map_traffic_fail_closed():
    snapshot = _valid_snapshot()
    snapshot['topics']['/hw/scan']['publishers'].append(
        _endpoint('stale_rosbridge_adapter'))
    snapshot['message_counts']['/map'] = 0
    failures = MODULE.evaluate_snapshot(
        snapshot, 'keyboard', 'lidar_imu_slam')
    assert any('/hw/scan' in item and '恰好一个平台适配器' in item
               for item in failures)
    assert any('采样窗口无消息：/map' in item for item in failures)


def test_wrong_detector_owner_and_missing_assist_service_are_rejected():
    snapshot = _valid_snapshot()
    snapshot['topics']['/hw/perception/hazard_detections'][
        'publishers'] = [_endpoint('stale_detector')]
    snapshot['services'].remove('/hw/control/assist_align/start')
    failures = MODULE.evaluate_snapshot(snapshot, 'keyboard')
    assert any('发布者错误' in item for item in failures)
    assert any('缺少辅助复查服务' in item for item in failures)


def test_adapter_with_control_relay_disabled_is_rejected():
    snapshot = _valid_snapshot()
    snapshot['latest_string_payloads'][
        '/hw/platform/official_simenv_adapter_status'] = (
        '{"managed_lifecycle": true, '
        '"lifecycle_container": "simenv_ros1_hazard_platform", '
        '"enable_cmd_vel_relay": false, '
        '"enable_gui_overlay_relay": true, '
        '"image_throttle_rate_ms": 200, '
        '"gui_assist_request_topic": '
        '"/hazardwalker/gui/assist_request"}')
    failures = MODULE.evaluate_snapshot(snapshot, 'keyboard')
    assert any('未启用' in item and 'cmd_vel' in item for item in failures)


def test_missing_or_stale_first_person_stream_is_rejected():
    snapshot = _valid_snapshot()
    snapshot['first_person_health']['frame']['frame_age_sec'] = 3.0
    snapshot['first_person_health']['overlay']['state_age_sec'][
        'perception'] = None
    failures = MODULE.evaluate_snapshot(snapshot, 'keyboard')
    assert any('相机帧已超时' in item for item in failures)
    assert any('perception' in item and '超时' in item for item in failures)


def test_first_person_overlay_must_match_current_frame_and_full_contract():
    snapshot = _valid_snapshot()
    snapshot['first_person_health']['overlay']['perception'][
        'stamp_sec'] = 124.0
    snapshot['first_person_health']['overlay']['perception'].pop(
        'view_recommendation')
    failures = MODULE.evaluate_snapshot(snapshot, 'keyboard')
    assert any('未同步' in item for item in failures)
    assert any('载荷不完整' in item for item in failures)


def test_disabled_gui_relay_is_rejected_for_formal_live_chain():
    snapshot = _valid_snapshot()
    snapshot['latest_string_payloads'][
        '/hw/platform/official_simenv_adapter_status'] = (
        '{"managed_lifecycle": true, '
        '"lifecycle_container": "simenv_ros1_hazard_platform", '
        '"enable_cmd_vel_relay": true, '
        '"enable_gui_overlay_relay": false, '
        '"image_throttle_rate_ms": 200}')
    failures = MODULE.evaluate_snapshot(snapshot, 'keyboard')
    assert any('GUI 状态与辅助请求转发' in item for item in failures)


def test_unmanaged_adapter_is_rejected_for_formal_live_chain():
    snapshot = _valid_snapshot()
    snapshot['latest_string_payloads'][
        '/hw/platform/official_simenv_adapter_status'] = (
        '{"managed_lifecycle": false, '
        '"lifecycle_container": null, '
        '"enable_cmd_vel_relay": true, '
        '"enable_gui_overlay_relay": true, '
        '"image_throttle_rate_ms": 200, '
        '"gui_assist_request_topic": '
        '"/hazardwalker/gui/assist_request"}')
    failures = MODULE.evaluate_snapshot(snapshot, 'keyboard')
    assert any('auto_docker.sh' in item for item in failures)
    assert any('生命周期所属容器' in item for item in failures)


def test_slow_image_bridge_is_rejected_before_formal_patrol():
    snapshot = _valid_snapshot()
    status = json.loads(snapshot['latest_string_payloads'][
        '/hw/platform/official_simenv_adapter_status'])
    status['image_throttle_rate_ms'] = 500
    snapshot['latest_string_payloads'][
        '/hw/platform/official_simenv_adapter_status'] = json.dumps(status)
    failures = MODULE.evaluate_snapshot(snapshot, 'keyboard')
    assert any('图像桥接过慢' in item for item in failures)


def test_missing_adapter_status_payload_is_rejected():
    snapshot = _valid_snapshot()
    snapshot['latest_string_payloads'].pop(
        '/hw/platform/official_simenv_adapter_status')
    failures = MODULE.evaluate_snapshot(snapshot, 'keyboard')
    assert any('缺少平台适配器实时状态载荷' in item for item in failures)


def test_unknown_control_source_is_rejected():
    with pytest.raises(ValueError, match='未知控制源'):
        MODULE.evaluate_snapshot(_valid_snapshot(), 'automatic_magic')
