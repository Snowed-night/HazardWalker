"""官方 SimEnv ROS1 双向适配映射的离线契约测试。"""

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PLATFORM_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform'
if str(PLATFORM_SRC) not in sys.path:
    sys.path.insert(0, str(PLATFORM_SRC))

from hazardwalker_platform.official_simenv_mapping import (  # noqa: E402
    HW_CMD_VEL,
    OFFICIAL_CMD_VEL,
    OFFICIAL_RGB_TOPIC_CANDIDATES,
    OFFICIAL_ROS1_TO_HW,
    TF_PASSTHROUGH,
    source_to_destination,
    validate_mapping,
)


def test_official_ros1_required_sensor_mapping_is_complete():
    mapping = source_to_destination()
    assert mapping['/hazardwalker/odom'].destination == '/hw/odom'
    assert mapping['/real_sense/rgb/image_raw'].destination == '/hw/camera/image_raw'
    assert mapping['/real_sense/depth/image_raw'].destination == '/hw/camera/depth_image'
    assert mapping['/real_sense/rgb/camera_info'].destination == '/hw/camera/camera_info'
    assert all(item.destination.startswith('/hw/') for item in OFFICIAL_ROS1_TO_HW)
    assert '/camera/image_raw' in OFFICIAL_RGB_TOPIC_CANDIDATES


def test_control_and_tf_contract_do_not_hide_platform_difference():
    assert HW_CMD_VEL == '/hw/cmd_vel'
    assert OFFICIAL_CMD_VEL == '/cmd_vel'
    assert HW_CMD_VEL != OFFICIAL_CMD_VEL
    assert {(item.source, item.destination) for item in TF_PASSTHROUGH} == {
        ('/tf', '/tf'), ('/tf_static', '/tf_static')}


def test_official_mapping_is_internally_valid():
    assert validate_mapping() == ()


def test_rosbridge_control_relay_defaults_to_safe_and_uses_wall_clock_watchdog():
    source = (REPO_ROOT / 'scripts' / 'official_simenv_rosbridge_ros2_adapter_node.py').read_text(
        encoding='utf-8')
    assert "declare_parameter('enable_cmd_vel_relay', False)" in source
    assert 'time.monotonic()' in source
    assert "declare_parameter('rgb_topic'" in source
    assert "declare_parameter('ros1_odom_topic', '/hazardwalker/odom')" in source
    assert "'forwarded_cmd_count'" in source
    assert "rosbridge_host_header" in source
    assert 'ExternalShutdownException' in source


def test_direct_ros1_control_verifier_requires_exclusive_session_and_records_odometry():
    source = (REPO_ROOT / 'scripts' / 'verify_official_simenv_ros1_direct_control.sh').read_text(
        encoding='utf-8')
    assert 'OFFICIAL_SIMENV_EXCLUSIVE_SESSION' in source
    assert '/Odometry_gazebo' in source
    assert 'forward_at_least_1m' in source
    assert 'summary.json' in source


def test_rosbridge_fragment_contract_is_bounded_and_adapter_keeps_image_bytes():
    protocol = (PLATFORM_SRC / 'hazardwalker_platform' / 'rosbridge_protocol.py').read_text(encoding='utf-8')
    adapter = (REPO_ROOT / 'scripts' / 'official_simenv_rosbridge_ros2_adapter_node.py').read_text(encoding='utf-8')
    assert 'FragmentAssembler' in protocol and 'timeout_sec' in protocol
    assert "'fragment_size': 60000" in adapter
    assert 'base64.b64decode' in adapter
    assert "'/hw/cmd_vel'" in adapter and "'/cmd_vel'" in adapter
    assert "for name in ('x', 'y', 'z', 'w')" in adapter
    assert "for name in ('x', 'y', 'z'):" in adapter
    assert "tf2_msgs/TFMessage" in adapter
    assert "'/tf_static'" in adapter
    assert 'tf_odom_consistency_tolerance_m' in adapter
    assert 'dropped_inconsistent_tf' in adapter
    assert 'tf_throttle_rate_ms' in adapter
    assert 'odom_throttle_rate_ms' in adapter
    assert 'topic == self.ros1_odom_topic' in adapter
    assert "declare_parameter('world_frame', 'world')" in adapter
    # 官方 rosbridge 把不同图像订阅的 fragment 都标成 id=0，必须隔离 RGB/深度接收连接。
    assert 'def _receive_image_loop' in adapter
    assert 'self._image_threads' in adapter
    detector = (REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception' / 'hazardwalker_perception' /
                'hsv_detector_node.py').read_text(encoding='utf-8')
    # 官方规则禁止把 Gazebo 真值定位当比赛输入；感知默认等待合法 SLAM 的 map TF，
    # 未验证定位即使产生候选也不能被结果层导出为 world 危险源。
    assert "declare_parameter('output_frame', 'map')" in detector
    assert "declare_parameter('localization_provenance', 'unverified')" in detector


def test_official_full_stack_requires_an_exclusive_simenv_session():
    preflight = (REPO_ROOT / 'scripts' / 'check_official_simenv_exclusive_session.sh').read_text(encoding='utf-8')
    stack = (REPO_ROOT / 'scripts' / 'run_official_simenv_ros1_ros2_stack.sh').read_text(encoding='utf-8')
    direct = (REPO_ROOT / 'scripts' / 'verify_official_simenv_ros1_direct_control.sh').read_text(encoding='utf-8')
    # 文案可以说明 docker stop/rm，但脚本本体不能调用这两个破坏性命令。
    assert '\ndocker stop ' not in preflight and '\ndocker rm ' not in preflight
    assert '--require-exclusive' in preflight
    assert 'check_official_simenv_exclusive_session.sh' in stack
    assert 'check_official_simenv_exclusive_session.sh' in direct
    legacy_launcher = (REPO_ROOT / 'scripts' / 'run_official_simenv_ros1_adapter.sh').read_text(encoding='utf-8')
    assert 'run_official_simenv_rosbridge_adapter.sh' in legacy_launcher


def test_official_business_launch_never_starts_fake_platform_by_default():
    source = (REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_bringup' / 'launch' /
              'official_simenv_business.launch.py').read_text(encoding='utf-8')
    assert "DeclareLaunchArgument('start_navigation', default_value='false')" in source
    assert "package='hazardwalker_platform'" not in source
    compile(source, 'official_simenv_business.launch.py', 'exec')


def test_official_perception_world_export_requires_explicit_legal_slam_contract():
    """默认不得把平台里程计包装成可提交的世界坐标。"""

    source = (REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_bringup' / 'launch' /
              'official_simenv_business.launch.py').read_text(encoding='utf-8')
    assert "DeclareLaunchArgument('perception_output_frame', default_value='map')" in source
    assert "DeclareLaunchArgument('localization_provenance', default_value='unverified')" in source
    assert "'output_frame': perception_output_frame" in source
    assert "'localization_provenance': localization_provenance" in source


def test_official_minimal_navigation_consumes_stable_hw_odom():
    source = (REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav' / 'hazardwalker_nav' /
              'waypoint_patrol_node.py').read_text(encoding='utf-8')
    assert "Odometry, '/hw/odom'" in source
    assert "'/hw/Odometry_gazebo'" not in source


def test_stack_keeps_adapter_alive_while_business_launch_runs():
    source = (REPO_ROOT / 'scripts' / 'run_official_simenv_ros1_ros2_stack.sh').read_text(
        encoding='utf-8')
    assert 'ADAPTER_PID=$!' in source
    assert 'bash "$ROOT/scripts/run_official_simenv_rosbridge_adapter.sh" &' in source
    assert 'trap cleanup_adapter EXIT INT TERM' in source
    # 业务 launch 会派生多个节点；必须拥有独立进程组并在退出时整体回收，
    # 否则下一次联调会残留多个 /hw/cmd_vel 发布者。
    assert 'setsid ros2 launch hazardwalker_bringup official_simenv_business.launch.py' in source
    assert 'kill -- "-$BUSINESS_PID"' in source
    assert 'set +u\nsource /opt/ros/jazzy/setup.bash' in source
    assert 'ros2 launch hazardwalker_bringup official_simenv_business.launch.py' in source


def test_legacy_json_bridge_fails_closed_in_official_profile():
    source = (REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform' / 'hw_bridge.py').read_text(
        encoding='utf-8')
    assert 'HAZARDWALKER_ENABLE_LEGACY_JSON_BRIDGE' in source
    assert 'run_official_simenv_rosbridge_adapter.sh' in source
