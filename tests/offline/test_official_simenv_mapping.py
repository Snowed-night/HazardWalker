"""官方 SimEnv ROS1 双向适配映射的离线契约测试。"""

import sys
import xml.etree.ElementTree as ET
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


def test_platform_package_manifest_is_valid_and_declares_clock_dependency():
    manifest = ET.parse(
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform' / 'package.xml',
    ).getroot()
    assert manifest.tag == 'package'
    assert 'rosgraph_msgs' in {
        node.text for node in manifest.findall('exec_depend')
    }


def test_navigation_manifest_declares_scan_and_slam_runtime_dependencies():
    manifest = ET.parse(
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav' / 'package.xml',
    ).getroot()
    dependencies = {
        node.text for node in manifest.findall('exec_depend')
    }
    assert {'sensor_msgs', 'slam_toolbox', 'cartographer_ros'} <= dependencies

    setup_source = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav' / 'setup.py'
    ).read_text(encoding='utf-8')
    assert "glob.glob('config/*.lua')" in setup_source


def test_official_laserscan_profile_disables_legacy_type_conflict_and_is_horizontal():
    platform_root = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform' / 'src' /
        'unitree_guide'
    )
    launch_path = (
        platform_root / 'unitree_guide' / 'unitree_guide' /
        'launch' / 'multi_floor_gazeboSim.launch'
    )
    robot_path = (
        platform_root / 'unitree_ros' / 'robots' / 'a1_description' /
        'xacro' / 'robot.xacro'
    )
    gazebo_path = robot_path.with_name('gazebo.xacro')
    parsed = {}
    for xml_path in (launch_path, robot_path, gazebo_path):
        parsed[xml_path] = ET.parse(xml_path)

    launch_source = launch_path.read_text(encoding='utf-8')
    robot_source = robot_path.read_text(encoding='utf-8')
    gazebo_source = gazebo_path.read_text(encoding='utf-8')
    assert 'name="enable_legacy_pointcloud2livox" default="false"' in launch_source
    assert 'if="$(arg enable_legacy_pointcloud2livox)"' in launch_source
    assert 'link name="laser_scan"' in robot_source
    assert 'joint name="laser_scan_joint"' in robot_source
    assert 'rpy="0 0 0"' in robot_source
    assert '<pose>0 0 0 0 -0.785 0</pose>' in gazebo_source
    assert '<frameName>laser_scan</frameName>' in gazebo_source
    ray_sensor = parsed[gazebo_path].getroot().find(
        ".//sensor[@name='laser_livox']")
    assert ray_sensor is not None
    assert len(ray_sensor.findall('ray')) == 1
    assert ray_sensor.find('plugin/ray') is None
    assert ray_sensor.findtext('ray/scan/horizontal/samples') == '360'
    assert ray_sensor.findtext('ray/scan/horizontal/min_angle') == '${-M_PI}'
    assert ray_sensor.findtext('ray/scan/horizontal/max_angle') == '${M_PI}'
    assert ray_sensor.findtext('ray/range/min') == '0.1'
    assert ray_sensor.findtext('ray/range/max') == '40.0'


def test_official_controller_discovers_split_cuda_runtime_libraries():
    auto_source = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform' / 'auto.sh'
    ).read_text(encoding='utf-8')
    patch_source = (
        REPO_ROOT / 'patches' /
        'official_simenv_controller_runtime_compat_20260718.patch'
    ).read_text(encoding='utf-8')
    for token in (
        'gazebo-11/plugins',
        'GAZEBO_PLUGIN_PATH',
        'nvidia/cublas/lib',
        'nvidia/cuda_runtime/lib',
        'nvidia/nvtx/lib',
    ):
        assert token in auto_source
        assert token in patch_source
    assert 'export LD_LIBRARY_PATH=' in auto_source


def test_official_headless_startup_reuses_display_and_cleans_only_stale_lock():
    """容器 restart 后不能因陈旧 X 锁静默丢失 RGB-D 插件。"""

    for relative_path in ('ros2_ws/src/hazardwalker_platform/auto.sh',):
        source = (REPO_ROOT / relative_path).read_text(encoding='utf-8')
        assert 'SIMENV_HEADLESS_DISPLAY' in source
        assert 'display_is_ready()' in source
        assert 'xdpyinfo -display "$DISPLAY_VALUE"' in source
        assert 'kill -0 "$LOCK_PID"' in source
        assert 'rm -f "$DISPLAY_LOCK" "$DISPLAY_SOCKET"' in source
        assert '> "$XVFB_LOG" 2>&1 &' in source
        assert 'if [ "$DISPLAY_READY" != "1" ]' in source
        assert 'pkill Xvfb' not in source
        assert '&>/dev/null' not in source


def test_official_container_has_bounded_memory_and_no_oom_restart_loop():
    """长时间 gzserver 异常膨胀只能影响本容器，不能拖垮共享主机。"""

    compose = (
        PLATFORM_SRC / 'docker' / 'docker-compose.yml'
    ).read_text(encoding='utf-8')
    wrapper = (
        PLATFORM_SRC / 'docker' / 'auto_noetic.sh'
    ).read_text(encoding='utf-8')
    assert 'mem_limit: ${SIMENV_MEMORY_LIMIT:-32g}' in compose
    assert 'memswap_limit: ${SIMENV_MEMORY_SWAP_LIMIT:-32g}' in compose
    assert 'restart: "${SIMENV_RESTART_POLICY:-no}"' in compose
    assert 'SIMENV_MEMORY_LIMIT=32g' in wrapper


def test_adapter_waits_for_declared_container_healthcheck_to_pass():
    """容器刚创建时的临时 health=none 不能触发适配器抢跑。"""

    source = (PLATFORM_SRC / 'auto_docker.sh').read_text(encoding='utf-8')
    assert '.Config.Healthcheck' in source
    assert '"$health_configured" == true && "$health" == healthy' in source
    assert '"$health_configured" == false && "$health" == none' in source


def test_clean_clone_bootstraps_ignored_unitree_runtime_assets_safely():
    source = (
        PLATFORM_SRC / 'docker' / 'bootstrap_runtime_assets.sh'
    ).read_text(encoding='utf-8')
    wrapper = (PLATFORM_SRC / 'auto_docker.sh').read_text(encoding='utf-8')
    assert 'OFFICIAL_SIMENV_SOURCE_ROOT' in source
    assert "-name '*.so'" in source
    assert "-name '*.a'" in source
    assert "-name '*.pt'" in source
    assert 'policy_act_inference_plane.pt' in source
    assert 'policy_act_inference_stair.pt' in source
    assert 'danger_truth' not in source
    assert 'bash "$RUNTIME_ASSET_BOOTSTRAP"' in wrapper
    ensure_runtime = wrapper.split('ensure_runtime() {', 1)[1].split('\n}', 1)[0]
    assert ensure_runtime.index('bash "$RUNTIME_ASSET_BOOTSTRAP"') < ensure_runtime.index(
        'if runtime_ready')


def test_container_health_requires_current_startup_ready_marker():
    startup = (PLATFORM_SRC / 'auto.sh').read_text(encoding='utf-8')
    compose = (
        PLATFORM_SRC / 'docker' / 'docker-compose.yml'
    ).read_text(encoding='utf-8')
    marker = '/tmp/hazardwalker-simenv-runtime-ready'
    assert f'RUNTIME_READY_FILE="{marker}"' in startup
    assert 'rm -f "$RUNTIME_READY_FILE"' in startup
    assert 'touch "$RUNTIME_READY_FILE"' in startup
    assert f'test -f {marker}' in compose


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
    assert "declare_parameter('enable_odom_relay', False)" in source
    assert "if self.enable_odom_relay:" in source
    assert "source.get('orientation', {})" in source
    assert 'time.monotonic()' in source
    assert "declare_parameter('rgb_topic'" in source
    assert "declare_parameter('ros1_odom_topic', '/hazardwalker/odom')" in source
    assert "'forwarded_cmd_count'" in source
    assert "rosbridge_host_header" in source


def test_gui_assist_request_can_only_call_bounded_ros2_services():
    """浏览器请求必须转成辅助服务调用，不能绕过控制仲裁发布速度。"""

    source = (
        REPO_ROOT / 'scripts' /
        'official_simenv_rosbridge_ros2_adapter_node.py'
    ).read_text(encoding='utf-8')
    assert "'/hazardwalker/gui/assist_request'" in source
    assert "Trigger, '/hw/control/assist_align/start'" in source
    assert "Trigger, '/hw/control/assist_align/cancel'" in source
    assert "action not in ('start', 'cancel')" in source
    assert 'self._pending_gui_assist_action = action' in source
    dispatch = source.split(
        'def _dispatch_gui_assist_request', 1)[1].split(
            'def _on_gui_assist_result', 1)[0]
    assert 'call_async(Trigger.Request())' in dispatch
    assert "'/cmd_vel'" not in dispatch
    assert 'ExternalShutdownException' in source
    assert "declare_parameter('enable_clock_relay', True)" in source
    assert "subscriptions.append((self.clock_topic, 'rosgraph_msgs/Clock'))" in source
    assert "declare_parameter('enable_pointcloud_relay', False)" in source
    assert "declare_parameter('enable_livox_imu_relay', False)" in source
    assert "declare_parameter('enable_trunk_imu_relay', True)" in source
    assert "declare_parameter('scan_throttle_rate_ms', 50)" in source
    assert "declare_parameter('imu_throttle_rate_ms', 20)" in source
    assert "declare_parameter('scan_self_filter_range_m', 0.40)" in source
    assert "'/hw/scan_raw'" in source
    assert 'filter_scan_self_returns(' in source
    assert "header.get('frame_id', 'laser_scan')" in source


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
    assert "declare_parameter('max_latest_tf_fallback_delta_sec', 0.10)" in detector
    assert 'Latest TF rejected' in detector
    assert "'tf_synchronized': self._last_tf_synchronized" in detector
    assert "localization_provenance not in ('', 'unverified')" in detector
    assert 'self.tracker.published_tracks()' in detector
    assert "'raw_surface_depth_m': raw_surface_depth_m" in detector
    assert "declare_parameter('max_rgb_depth_sync_delta_sec', 0.06)" in detector
def test_official_full_stack_requires_an_exclusive_simenv_session():
    preflight = (REPO_ROOT / 'scripts' / 'check_official_simenv_exclusive_session.sh').read_text(encoding='utf-8')
    stack = (REPO_ROOT / 'scripts' / 'run_official_simenv_ros1_ros2_stack.sh').read_text(encoding='utf-8')
    direct = (REPO_ROOT / 'scripts' / 'verify_official_simenv_ros1_direct_control.sh').read_text(encoding='utf-8')
    # 文案可以说明 docker stop/rm，但脚本本体不能调用这两个破坏性命令。
    assert '\ndocker stop ' not in preflight and '\ndocker rm ' not in preflight
    assert '--require-exclusive' in preflight
    assert '[[ "$NAME" == "$CONTAINER"' in preflight
    assert 'OFFICIAL_SIMENV_ALLOW_ISOLATED_PARALLEL' in preflight
    assert 'TARGET_ROS_MASTER' in preflight
    assert 'TARGET_GAZEBO_MASTER' in preflight
    assert 'check_official_simenv_exclusive_session.sh' in stack
    assert 'bash "$ROOT/scripts/check_official_simenv_exclusive_session.sh"' in stack
    assert 'check_official_simenv_exclusive_session.sh' in direct
    adapter_runner = (REPO_ROOT / 'scripts' / 'run_official_simenv_rosbridge_adapter.sh').read_text(
        encoding='utf-8')
    assert 'official_simenv_rosbridge_ros2_adapter_node.py' in adapter_runner


def test_official_navigation_opens_main_entrance_via_public_service_only():
    source = (
        REPO_ROOT / 'scripts' / 'run_official_simenv_ros1_ros2_stack.sh'
    ).read_text(encoding='utf-8')

    assert "'/set_door_state'" in source
    assert "'main_entrance', True" in source
    assert 'response.accepted' in source
    assert 'danger_truth' not in source


def test_official_business_launch_never_starts_fake_platform_by_default():
    source = (REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_bringup' / 'launch' /
              'official_simenv_business.launch.py').read_text(encoding='utf-8')
    assert "DeclareLaunchArgument('start_navigation', default_value='false')" in source
    assert "DeclareLaunchArgument('start_slam', default_value='false')" in source
    assert "DeclareLaunchArgument('slam_backend', default_value='cartographer')" in source
    assert "DeclareLaunchArgument('start_legal_localization', default_value='true')" in source
    assert "'mission_time_budget_s': 600.0" in source
    assert "'minimum_return_reserve_s': 120.0" in source
    assert "'entry_ingress_depth_m': 6.0" in source
    assert "executable='scan_imu_localizer_node'" in source
    assert "'localization_provenance': localization_provenance" in source
    assert "package='hazardwalker_platform'" not in source
    assert "'online_async_launch.py'" in source
    assert "'autostart': 'true'" in source
    assert "executable='async_slam_toolbox_node'" not in source
    assert 'def _launch_slam_toolbox(context, slam_config):' in source
    assert "get_package_share_directory('slam_toolbox')" in source
    assert (
        "slam_launch = os.path.join(\n"
        "        get_package_share_directory('slam_toolbox'),"
    ) not in source
    assert (
        "'start_slam=true and slam_backend=slam_toolbox require '"
        in source
    )
    assert "package='cartographer_ros'" in source
    assert "executable='cartographer_node'" in source
    assert "executable='cartographer_occupancy_grid_node'" in source
    assert "executable='depth_to_scan_node'" not in source
    assert "('scan', '/hw/scan')" in source
    assert "('scan_1', '/hw/scan')" not in source
    assert "('scan_2', '/hw/depth_scan')" not in source
    assert "('odom', '/hazardwalker/slam/odometry')" in source
    assert "'publish_tf': publish_legal_tf_parameter" in source
    assert 'OpaqueFunction(' in source
    assert "get_package_share_directory('cartographer_ros')" in source
    assert 'except PackageNotFoundError as exc:' in source
    assert "'cartographer_ros; configure the official stack Cartographer prefix.'" in source
    assert (
        "builtin_dir = os.path.join(\n"
        "        get_package_share_directory('cartographer'),"
    ) not in source
    assert "'cartographer',\n        'configuration_files'" in source
    assert 'shutil.copytree(builtin_dir, runtime_dir, dirs_exist_ok=True)' in source
    assert "ParameterValue(scenario_seed, value_type=str)" in source
    assert "ParameterValue(code_version, value_type=str)" in source
    assert "DeclareLaunchArgument('use_sim_time', default_value='true')" in source
    assert "'navigation_cmd_vel_topic', default_value='/hw/cmd_vel'" in source
    assert source.count("'cmd_vel_topic': navigation_cmd_vel_topic") == 2
    assert source.count("'use_sim_time': sim_time_parameter") >= 5
    assert "LaunchConfiguration('use_sim_time').perform(context)" in source
    assert "'use_sim_time': use_sim_time" in source
    assert source.count('condition=IfCondition(start_navigation)') == 2

    cartographer_config = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav' / 'config'
        / 'cartographer_official_2d.lua'
    ).read_text(encoding='utf-8')
    assert 'num_laser_scans = 1' in cartographer_config
    assert 'TRAJECTORY_BUILDER_2D.max_range = 8.0' in cartographer_config
    assert "LaunchConfigurationEquals(\n                    'nav_mode', expected_value='frontier')" in source
    assert "LaunchConfigurationEquals(\n                    'nav_mode', expected_value='waypoint')" in source
    assert 'PythonExpression' in source
    assert "'goal_tolerance_m': 0.25" in source
    cartographer_config = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav' / 'config' /
        'cartographer_official_2d.lua'
    ).read_text(encoding='utf-8')
    assert 'POSE_GRAPH.constraint_builder.max_constraint_distance = 1.5' in cartographer_config
    assert 'POSE_GRAPH.constraint_builder.min_score = 0.72' in cartographer_config
    assert 'POSE_GRAPH.constraint_builder.global_localization_min_score = 0.90' in cartographer_config
    compile(source, 'official_simenv_business.launch.py', 'exec')


def test_unified_control_launch_keeps_navigation_behind_command_mux():
    """键盘、导航和辅助对准必须共享唯一底盘输出，且默认不启动导航。"""

    source = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_bringup' / 'launch' /
        'official_simenv_control_interface.launch.py'
    ).read_text(encoding='utf-8')
    assert "executable='command_mux_node'" in source
    assert "executable='assist_alignment_node'" in source
    assert "DeclareLaunchArgument('start_navigation', default_value='false')" in source
    assert "'navigation_cmd_vel_topic': (" in source
    assert "'/hw/control/navigation_cmd_vel'" in source
    assert "'fallback_mode': LaunchConfiguration('control_mode')" in source
    compile(source, 'official_simenv_control_interface.launch.py', 'exec')


def test_legacy_simenv_demo_is_a_safe_official_business_wrapper():
    """旧 launch 名称不能重新引入 PR #33 的 TF 合并和默认控制。"""

    source = (REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_bringup' / 'launch' /
              'simenv_demo.launch.py').read_text(encoding='utf-8')
    assert "'official_simenv_business.launch.py'" in source
    assert "DeclareLaunchArgument('start_navigation', default_value='false')" in source
    assert "DeclareLaunchArgument('start_slam', default_value='false')" in source
    assert "DeclareLaunchArgument('slam_backend', default_value='cartographer')" in source
    assert "'start_navigation': start_navigation" in source
    assert "'start_slam': start_slam" in source
    assert "'slam_backend': slam_backend" in source
    assert "DeclareLaunchArgument('exploration_timeout_s', default_value='540.0')" in source
    assert "'exploration_timeout_s': exploration_timeout_s" in source
    assert "start_evidence_recorder': 'false'" in source
    assert "executable='async_slam_toolbox_node'" not in source
    assert "('/tf', '/hw/tf')" not in source
    assert "('/tf_static', '/hw/tf')" not in source
    assert "package='hazardwalker_platform'" not in source
    compile(source, 'simenv_demo.launch.py', 'exec')


def test_legacy_start_simenv_fails_before_starting_old_relay():
    """旧一键脚本仅允许 stop，不得再运行无门禁的 relay。"""

    source = (REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform' / 'scripts' /
              'start_simenv.sh').read_text(encoding='utf-8')
    assert 'run_official_simenv_ros1_ros2_stack.sh' in source
    assert '此启动入口已弃用' in source
    assert source.index('exit 2') < source.index('# 1. 启动 Docker ROS1')


def test_nav_pr33_report_is_marked_historical_and_forbids_old_tf_path():
    """保留优化路径时也必须阻止成员复制历史真值/TF 命令。"""

    source = (REPO_ROOT / 'docs' / 'groups' / 'nav' /
              'phase5.2-progress-20260717.md').read_text(encoding='utf-8')
    assert '[!CAUTION]' in source
    assert '历史排障记录，不是当前启动手册' in source
    assert 'run_official_simenv_ros1_ros2_stack.sh' in source
    assert '不得复制执行' in source


def test_official_perception_world_export_requires_explicit_legal_slam_contract():
    """默认不得把平台里程计包装成可提交的世界坐标。"""

    source = (REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_bringup' / 'launch' /
              'official_simenv_business.launch.py').read_text(encoding='utf-8')
    assert "DeclareLaunchArgument('perception_output_frame', default_value='map')" in source
    assert "DeclareLaunchArgument('localization_provenance', default_value='unverified')" in source
    assert "DeclareLaunchArgument('exploration_timeout_s', default_value='540.0')" in source
    assert "exploration_timeout_s = LaunchConfiguration('exploration_timeout_s')" in source
    assert "'exploration_timeout_s': exploration_timeout_parameter" in source
    assert "ParameterValue(\n        exploration_timeout_s, value_type=float" in source
    assert "'exploration_timeout_s': 540.0" not in source
    assert "DeclareLaunchArgument('perception_parameter_file', default_value='')" in source
    assert "'perception_output_frame').perform(context)" in source
    assert "'localization_provenance').perform(context)" in source
    assert 'flatten_perception_config' in source


def test_scan_imu_localizer_publishes_configured_runtime_provenance():
    source = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception' /
        'hazardwalker_perception' / 'scan_imu_localizer_node.py'
    ).read_text(encoding='utf-8')
    assert (
        "declare_parameter('localization_provenance', 'lidar_imu_slam')"
        in source
    )
    assert 'String(data=self.localization_provenance)' in source
    assert "'visual_inertial_slam'" not in source


def test_official_minimal_navigation_consumes_stable_hw_odom():
    source = (REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav' / 'hazardwalker_nav' /
              'waypoint_patrol_node.py').read_text(encoding='utf-8')
    assert "Odometry, '/hw/odom'" in source
    assert "'/hw/Odometry_gazebo'" not in source
    assert "'/hw/nav/diagnostic_state'" in source
    assert "'/hw/nav/state'" not in source


def test_container_owns_adapter_lifecycle_and_stack_reuses_it():
    source = (REPO_ROOT / 'scripts' / 'run_official_simenv_ros1_ros2_stack.sh').read_text(
        encoding='utf-8')
    assert '由 auto_docker.sh up 管理的官方适配器' in source
    assert 'bash "$ROOT/scripts/run_official_simenv_rosbridge_adapter.sh" &' not in source
    assert 'trap cleanup_business EXIT INT TERM' in source
    # 业务 launch 会派生多个节点；必须拥有独立进程组并在退出时整体回收，
    # 否则下一次联调会残留多个 /hw/cmd_vel 发布者。
    assert 'setsid ros2 launch hazardwalker_bringup official_simenv_control_interface.launch.py' in source
    assert 'kill -TERM -- "-$BUSINESS_PID"' in source
    assert 'set +u\nsource /opt/ros/jazzy/setup.bash' in source
    assert 'OFFICIAL_SIMENV_CARTOGRAPHER_PREFIX' in source
    assert 'OFFICIAL_SIMENV_CARTOGRAPHER_LIBRARY_PATH' in source
    assert 'ros2 pkg prefix cartographer_ros' in source
    assert 'share/cartographer_ros' in source
    assert 'share/cartographer_ros_msgs/local_setup.bash' in source
    assert 'export HAZARDWALKER_ROOT="$ROOT"' in source
    assert 'Frontier 导航要求本轮显式 start_slam=true' in source
    assert 'OFFICIAL_SIMENV_AUTO_STOP_ON_FINISHED' in source
    assert 'OFFICIAL_SIMENV_STACK_TIMEOUT_SEC' in source
    assert 'ros2 topic echo /hw/mission/state --field data --once' in source
    assert 'RESULT_MTIME >= RUN_START_EPOCH' in source
    assert 'kill -KILL -- "-$BUSINESS_PID"' in source
    assert 'for _ in {1..150}; do' in source
    assert 'ADAPTER_PID=' not in source
    assert 'ros2 launch hazardwalker_bringup official_simenv_control_interface.launch.py' in source
    assert 'LAUNCH_ARGS+=("control_mode:=navigation")' in source
    assert '统一控制模式必须为 navigation' in source
    assert 'ros2 topic echo /clock --once' in source
    assert 'CLOCK_NSEC=' in source
    assert 'CLOCK_NS > LAST_CLOCK_NS' in source
    assert 'LAST_CLOCK_NS="$CLOCK_NS"' in source
    assert '连续递增的 /clock' in source
    assert "grep -qx '/hazardwalker_official_rosbridge_adapter'" in source
    assert 'start_navigation=true 但控制适配未显式开启' in source
    assert '"enable_cmd_vel_relay": true' in source
    assert '"managed_lifecycle": true' in source
    assert 'NAV_MODE=frontier' in source
    assert 'PERCEPTION_OUTPUT_FRAME=map' in source
    assert 'LOCALIZATION_PROVENANCE=unverified' in source
    assert '正式一键任务只允许 nav_mode=frontier' in source
    assert 'perception_output_frame=world' in source
    assert '白名单内的合法 SLAM localization_provenance' in source
    assert '必须开启证据记录并提供 SEED、代码版本和输出目录' in source

    adapter_runner = (
        REPO_ROOT / 'scripts' / 'run_official_simenv_rosbridge_adapter.sh'
    ).read_text(encoding='utf-8')
    assert '-p use_sim_time:=false' in adapter_runner
    assert '-p enable_clock_relay:=true' in adapter_runner
    assert '-p managed_lifecycle:="$MANAGED_LIFECYCLE"' in adapter_runner
    assert '-p lifecycle_container:="$LIFECYCLE_CONTAINER"' in adapter_runner

    slam_config = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav' / 'config' /
        'slam_toolbox_online_async.yaml'
    ).read_text(encoding='utf-8')
    assert 'restamp_tf: false' in slam_config
    assert 'min_laser_range: 0.40' in slam_config


def test_compose_starts_the_complete_official_ros1_entry():
    """Docker 必须调用含中继/rosbridge/控制门禁的唯一正式入口。"""

    source = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform' / 'docker' /
        'docker-compose.yml'
    ).read_text(encoding='utf-8')
    assert '${SIMENV_HOST_PATH:-..}:/home/ros/simenv_ws' in source
    assert 'test -x ./auto.sh' in source
    assert 'exec ./auto.sh' in source
    assert 'START_CONTROLLER: ${START_CONTROLLER:-1}' in source
    assert 'SIMENV_AUTO_RL: ${SIMENV_AUTO_RL:-1}' in source
    assert 'SIMENV_HEADLESS_MODE: ${SIMENV_HEADLESS_MODE:-move_base}' in source
    assert 'START_ROSBRIDGE: ${START_ROSBRIDGE:-1}' in source
    assert 'START_ODOM_RELAY: ${START_ODOM_RELAY:-1}' in source
    assert 'healthcheck:' in source
    assert 'pgrep -x junior_ctrl' in source
    # healthcheck 直接检查 /cmd_vel 的正式订阅者；rosnode ping 会在 Gazebo 初始化
    # 窗口中产生瞬态误报，不能作为容器存活判据。
    assert 'rostopic info /cmd_vel' in source
    assert '/unitree_gazebo_servo' in source
    assert 'tail -f /dev/null' not in source
    assert 'START_ROS1_DYNAMIC_BRIDGE' not in source
    assert 'ros1_bridge.sh' not in source

    dockerfile = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform' / 'docker' /
        'Dockerfile'
    ).read_text(encoding='utf-8')
    assert 'ros-noetic-rosbridge-server' in dockerfile
    assert 'expect' in dockerfile
    assert 'ros-foxy-ros1-bridge' not in dockerfile

    entry = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform' / 'auto.sh'
    ).read_text(encoding='utf-8')
    assert 'scripts/rosbridge_odom_relay.py' in entry
    assert 'rosbridge_websocket.launch' in entry
    assert r'\[HEADLESS_FSM\].*mode=move_base.*auto_rl=[01]' in entry
    assert 'SIMENV_HEADLESS_MODE="${SIMENV_HEADLESS_MODE:-move_base}"' in entry
    assert 'rostopic info /cmd_vel' in entry
    assert 'Timed out waiting for /gazebo/unpause_physics.' in entry
    assert 'Timed out waiting for /unitree_gazebo_servo to subscribe /cmd_vel.' in entry
    assert 'Switched from fixed stand to RL' in entry
    assert 'Controller physical /cmd_vel probe passed:' in entry
    assert 'scripts/controller_motion_probe.py' in entry
    assert 'CONTROLLER_PROBE_MIN_DISPLACEMENT_M' in entry
    assert 'CONTROLLER_PROBE_MAX_DISPLACEMENT_M' in entry
    assert 'CONTROLLER_PROBE_MIN_BASE_HEIGHT_M' in entry
    assert 'CONTROLLER_RL_SETTLE_SEC' in entry
    assert 'UNITREE_CTRL_DT="${UNITREE_CTRL_DT:-0.004}"' in entry
    assert 'wait "$LAUNCH_PID"' in entry

    docker_wrapper = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform' /
        'auto_docker.sh'
    ).read_text(encoding='utf-8')
    assert 'image)' in docker_wrapper
    assert 'auto_noetic.sh" image' in docker_wrapper
    assert 'source is newer than devel/lib/unitree_guide/junior_ctrl' in docker_wrapper
    assert "./auto_docker.sh build force" in docker_wrapper
    assert 'manage_adapter start' in docker_wrapper
    assert 'manage_adapter stop' in docker_wrapper
    assert docker_wrapper.index('manage_adapter stop') < docker_wrapper.index(
        'exec "$ROOT/docker/auto_noetic.sh" down')
    assert 'wait_for_container_ready' in docker_wrapper

    adapter_runner = (
        REPO_ROOT / 'scripts' /
        'run_official_simenv_rosbridge_adapter.sh'
    ).read_text(encoding='utf-8')
    assert 'OFFICIAL_SIMENV_IMAGE_THROTTLE_RATE_MS:-200' in adapter_runner

    adapter_manager = (
        REPO_ROOT / 'scripts' / 'manage_official_simenv_rosbridge_adapter.sh'
    ).read_text(encoding='utf-8')
    assert 'PID_FILE=' in adapter_manager
    assert 'SIGNATURE_FILE=' in adapter_manager
    assert 'adapter_signature()' in adapter_manager
    assert 'flock -x 9' in adapter_manager
    assert 'setsid bash "$RUNNER"' in adapter_manager
    assert 'stop_adapter' in adapter_manager
    assert 'node_visible' in adapter_manager
    assert 'single_node_visible' in adapter_manager
    assert "grep -cx '/hazardwalker_official_rosbridge_adapter'" in adapter_manager
    assert 'ROS2 node: duplicate count=' in adapter_manager
    assert 'kill -TERM "$pid"' in adapter_manager
    assert 'kill -KILL "$pid"' in adapter_manager
    assert '同一 ROS 域仍有其他会话的适配器' in adapter_manager
    assert 'external-node domain=' in adapter_manager

    lifecycle_test = (
        REPO_ROOT / 'tests' / 'runtime' /
        'verify_official_simenv_adapter_lifecycle.sh'
    ).read_text(encoding='utf-8')
    assert 'adapter lifecycle start/idempotent/status/stop' in lifecycle_test
    assert 'duplicate ROS2 node did not block adapter startup' in lifecycle_test
    assert 'external same-domain adapter fails closed' in lifecycle_test
    assert 'auto_docker up/down ordering and startup rollback' in lifecycle_test
    assert 'container-up,adapter-start,adapter-stop,container-down' in lifecycle_test

    relay = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform' / 'scripts' /
        'rosbridge_odom_relay.py'
    ).read_text(encoding='utf-8')
    assert "default='/Odometry_gazebo'" in relay
    assert "default='/hazardwalker/odom'" in relay


def test_rl_controller_has_safe_cmd_vel_watchdog_and_thread_lifecycle():
    """RL 控制器需安全读取速度，且线程重入、时钟冻结和退出均不得留下旧状态。"""

    controller_root = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform' / 'src' /
        'unitree_guide' / 'unitree_guide' / 'unitree_guide'
    )
    rl_source = (
        controller_root / 'src' / 'FSM' / 'State_RL_test.cpp'
    ).read_text(encoding='utf-8')
    base_source = (
        controller_root / 'src' / 'FSM' / 'FSMState.cpp'
    ).read_text(encoding='utf-8')
    base_header = (
        controller_root / 'include' / 'FSM' / 'FSMState.h'
    ).read_text(encoding='utf-8')
    rl_header = (
        controller_root / 'include' / 'FSM' / 'State_RL_test.h'
    ).read_text(encoding='utf-8')
    move_base_source = (
        controller_root / 'src' / 'FSM' / 'State_move_base.cpp'
    ).read_text(encoding='utf-8')
    fsm_source = (
        controller_root / 'src' / 'FSM' / 'FSM.cpp'
    ).read_text(encoding='utf-8')

    assert 'void State_RL::run(){' in rl_source
    assert 'nh.subscribe<geometry_msgs::Twist>("/cmd_vel",1' in rl_source
    assert 'command_age_sec <= 0.5' in rl_source
    assert 'std::chrono::steady_clock::now()' in rl_source
    assert 'infer_thread_running.store(State_RL::RUNNING)' in rl_source
    assert 'infer_thread.reset(new std::thread' in rl_source
    assert 'stopWorkerThreads();' in rl_source
    assert 'amp_obs_thread && amp_obs_thread->joinable()' in rl_source
    assert 'std::lock_guard<std::mutex>' in rl_source
    assert 'std::lock_guard<std::mutex>' in base_source
    assert 'std::isfinite(msg->linear.x)' in base_source
    assert 'std::mutex cmd_vel_mutex_' in base_header
    assert 'std::chrono::steady_clock::time_point received_at' in base_header
    assert 'std::atomic<uint8_t> infer_thread_running' in rl_header
    assert 'std::unique_ptr<std::thread> amp_obs_thread' in rl_header
    assert '_vx(0.0), _vy(0.0), _wz(0.0)' in move_base_source
    assert '[HEADLESS_FSM] mode=' in fsm_source
    assert 'Switched from ' in fsm_source
    assert 'hasFreshMotionCommand()' in fsm_source
    assert 'return FSMStateName::RL;' in fsm_source
