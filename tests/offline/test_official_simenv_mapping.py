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
    assert mapping['/Odometry_gazebo'].destination == '/hw/odom'
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
    assert "'/Odometry_gazebo'" in source
    assert "'forwarded_cmd_count'" in source


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
    legacy_launcher = (REPO_ROOT / 'scripts' / 'run_official_simenv_ros1_adapter.sh').read_text(encoding='utf-8')
    assert 'run_official_simenv_rosbridge_adapter.sh' in legacy_launcher


def test_official_business_launch_never_starts_fake_platform_by_default():
    source = (REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_bringup' / 'launch' /
              'official_simenv_business.launch.py').read_text(encoding='utf-8')
    assert "DeclareLaunchArgument('start_navigation', default_value='false')" in source
    assert "package='hazardwalker_platform'" not in source
    compile(source, 'official_simenv_business.launch.py', 'exec')


def test_stack_keeps_adapter_alive_while_business_launch_runs():
    source = (REPO_ROOT / 'scripts' / 'run_official_simenv_ros1_ros2_stack.sh').read_text(
        encoding='utf-8')
    assert 'ADAPTER_PID=$!' in source
    assert 'trap cleanup_adapter EXIT INT TERM' in source
    assert 'ros2 launch hazardwalker_bringup official_simenv_business.launch.py' in source
