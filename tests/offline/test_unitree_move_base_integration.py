"""赛事随附宇树 move_base 局部避障适配合同测试。"""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
PLATFORM = ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform'
UNITREE = (
    PLATFORM / 'src' / 'unitree_guide' / 'unitree_guide' /
    'unitree_move_base'
)


def test_image_installs_upstream_move_base_without_rebuilding_large_layers():
    source = (PLATFORM / 'docker' / 'Dockerfile').read_text(encoding='utf-8')
    assert 'ros-noetic-move-base' in source
    assert source.index('COPY install_libtorch.sh') < source.rindex(
        'ros-noetic-move-base')


def test_unitree_move_base_launch_uses_isolated_velocity_output():
    path = UNITREE / 'launch' / 'hazardwalker_move_base.launch'
    root = ET.parse(path).getroot()
    node = root.find("node[@pkg='move_base']")
    assert node is not None
    remap = node.find("remap[@from='cmd_vel']")
    assert remap is not None
    assert remap.attrib['to'] == '$(arg cmd_vel_topic)'
    text = path.read_text(encoding='utf-8')
    assert '/hazardwalker/unitree_move_base/cmd_vel' in text


def test_costmap_reuses_mid360_scan_and_unitree_footprint():
    config = yaml.safe_load((
        UNITREE / 'config' / 'hazardwalker_costmap_common_params.yaml'
    ).read_text(encoding='utf-8'))
    assert config['observation_sources'] == 'livox_scan'
    assert config['livox_scan']['topic'] == '/livox/scan_projection'
    assert config['livox_scan']['data_type'] == 'LaserScan'
    assert config['footprint'] == [
        [0.42, 0.38], [0.42, -0.38], [-0.45, -0.38], [-0.45, 0.38],
    ]
    assert config['inflation_radius'] == 0.10
    assert config['footprint_padding'] == 0.02


def test_local_planner_is_upstream_trajectory_planner_dwa():
    config = yaml.safe_load((
        UNITREE / 'config' / 'hazardwalker_base_local_planner_params.yaml'
    ).read_text(encoding='utf-8'))
    assert config['base_local_planner'] == (
        'base_local_planner/TrajectoryPlannerROS')
    planner = config['TrajectoryPlannerROS']
    assert planner['dwa'] is True
    assert planner['holonomic_robot'] is True
    assert planner['heading_scoring'] is True
    assert planner['max_vel_x'] == 0.90
    assert planner['min_vel_x'] == 0.55
    assert planner['max_vel_y'] == 0.55
    assert planner['max_vel_theta'] == 2.4
    assert planner['min_in_place_vel_theta'] == 1.40
    assert planner['acc_lim_x'] >= 8.0
    assert planner['acc_lim_y'] >= 8.0
    assert planner['acc_lim_theta'] >= 8.0
    lateral_samples = [
        float(value) for value in planner['y_vels'].split(',')]
    assert all(
        value == 0.0 or abs(value) >= 0.30
        for value in lateral_samples)
    assert planner['pdist_scale'] == 1.4
    assert planner['gdist_scale'] == 2.8
    assert planner['occdist_scale'] >= 0.05


def test_adapted_profile_does_not_require_missing_three_camera_topics():
    text = '\n'.join(
        path.read_text(encoding='utf-8')
        for path in (UNITREE / 'config').glob('hazardwalker_*.yaml')
    )
    for missing_topic in (
        '/cam1/point_cloud_face',
        '/cam3/point_cloud_left',
        '/cam4/point_cloud_right',
    ):
        assert missing_topic not in text


def test_frontier_delegates_local_avoidance_to_unitree_move_base():
    source = (
        ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav' /
        'hazardwalker_nav' / 'frontier_explorer_node.py'
    ).read_text(encoding='utf-8')
    assert "declare_parameter('local_planner_backend', 'direct')" in source
    assert "self._local_planner_backend == 'unitree_move_base'" in source
    assert 'self._follow_path_with_unitree_move_base()' in source
    assert 'self.unitree_move_base_goal_pub.publish(message)' in source
    assert 'transform_planar_goal_to_robot_frame' in source
    assert "self.get_parameter('base_frame').value" in source
    assert 'map->odom TF unavailable' not in source
    assert 'self._unitree_move_base_last_goal_map' in source
    assert "'unitree_move_base_corridor_lookahead_m', 3.0" in source
    assert "'unitree_move_base_corridor_goal_change_m', 0.80" in source
    assert 'goal_key[0] - self._unitree_move_base_last_goal_map[0]' in source
    assert "'use_official_odom_for_corridor_control', False" in source
    assert "'official_corridor_center_x_m', 0.0" in source
    assert "'use_official_odom_for_room_control', False" in source
    assert "'official_near_room_y_m', 14.865" in source
    assert "'official_far_room_y_m', 28.895" in source
    assert "outgoing_frame = str(self.get_parameter('odom_frame').value)" in source
    assert 'on_official_control_odom' in source
    assert '_official_room_goal_from_map' in source
    assert 'use_official_room_goal' in source
    assert "startswith('room_door:')" in source
    assert "'center_lateral'" in source
    assert "'center_longitudinal'" in source
    assert "'enter_door'" in source
    assert 'self._official_room_door_center_y = official_y' in source
    assert "'use_official_odom_for_return_control', False" in source
    assert "'official_home_y_m', -2.2" in source
    assert 'use_official_return_goal' in source
    assert 'Arrived at official physical home.' in source
    assert "'use_official_odom_for_elevator_control', False" in source
    assert "'official_elevator_cabin_x_m', 2.70" in source
    assert '_handle_official_floor_transition' in source
    assert "self._floor_transition_phase = 'riding'" in source
    assert 'Official elevator exit completed' in source
    assert 'self._cancel_unitree_move_base()' in source
    assert 'Unitree move_base command unavailable or stale' in source
    assert "'unitree_move_base_direct_fallback_s', 12.0" in source
    assert 'existing A* path with lidar clearance fallback' in source
    assert 'def _follow_path_with_direct_backend(' in source
    assert "'room_approach', 'room_cross', 'room_loop'," in source
    assert "'room_inspection', 'room_exit'" in source
    assert "self._local_planner_backend = 'direct'" in source
    assert 'self._local_planner_backend = saved_backend' in source
    assert 'and not goal_changed):' in source
    assert 'refresh_due = (' not in source
    assert "declare_parameter('unitree_move_base_cmd_timeout_s', 3.00)" in source
    assert 'Unitree move_base goal updated:' in source
    assert 'ParameterUninitializedException' in source


def test_official_room_completion_requires_physical_loop_and_holds_heading():
    source = (
        ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav' /
        'hazardwalker_nav' / 'frontier_explorer_node.py'
    ).read_text(encoding='utf-8')
    assert 'official_room_pending = False' in source
    assert 'if (not official_room_pending' in source
    assert 'if not official_room_pending' in source
    assert 'physical_room_loop_is_valid(' in source
    assert 'without shrinking or accepting it' in source
    assert 'and not official_room_mode' in source
    assert 'deterministic_room_hold_heading_during_loop' in source
    assert 'physical_yaw = self._deterministic_room_hold_yaw' in source


def test_rosbridge_keeps_unitree_speed_isolated_until_frontier_and_mux():
    source = (
        ROOT / 'scripts' /
        'official_simenv_rosbridge_ros2_adapter_node.py'
    ).read_text(encoding='utf-8')
    assert "'/hazardwalker/unitree_move_base/cmd_vel'" in source
    assert "'/hw/control/unitree_move_base_cmd_vel'" in source
    assert "'/hw/navigation/unitree_move_base_goal'" in source
    assert "'/hw/navigation/unitree_move_base_control'" in source
    assert "str(message.data).strip().lower() != 'cancel'" in source
    assert "'secs': 0" in source
    assert "'nsecs': 0" in source
    assert 'target=self._receive_unitree_cmd_loop' in source
    assert 'def _receive_unitree_cmd_loop(self):' in source
    assert "'id': 'hw:unitree_move_base_cmd'" in source
    assert 'Unitree move_base 独立速度连接中断' in source


def test_platform_lifecycle_starts_and_health_checks_unitree_move_base():
    auto = (PLATFORM / 'auto.sh').read_text(encoding='utf-8')
    host = (PLATFORM / 'auto_docker.sh').read_text(encoding='utf-8')
    compose = (
        PLATFORM / 'docker' / 'docker-compose.yml'
    ).read_text(encoding='utf-8')
    launch = (
        PLATFORM / 'src' / 'unitree_guide' / 'unitree_guide'
        / 'unitree_move_base' / 'launch' / 'hazardwalker_move_base.launch'
    ).read_text(encoding='utf-8')
    assert 'START_UNITREE_MOVE_BASE="${START_UNITREE_MOVE_BASE:-0}"' in auto
    assert 'AUTO_OPEN_MAIN_ENTRANCE="${AUTO_OPEN_MAIN_ENTRANCE:-1}"' in auto
    assert 'rosservice call /set_door_state main_entrance true' in auto
    assert 'Main entrance opened by /set_door_state' in auto
    assert 'roslaunch unitree_move_base hazardwalker_move_base.launch' in auto
    assert '/opt/ros/noetic/lib/move_base/move_base' in auto
    assert 'pgrep -x move_base' in auto
    assert 'MOVE_BASE_LAUNCH_PID=$!' in auto
    assert 'OFFICIAL_SIMENV_ENABLE_UNITREE_MOVE_BASE_BRIDGE' in host
    assert ': "${ENABLE_LIDAR:=true}"' in host
    assert ': "${ENABLE_LIVOX_3D:=true}"' in host
    assert 'START_UNITREE_MOVE_BASE=1 requires ENABLE_LIDAR=true.' in auto
    move_base_guard = auto.split(
        'if [ "$START_UNITREE_MOVE_BASE" = "1" ]; then', 1,
    )[1].split(
        'echo "Starting Unitree upstream move_base', 1,
    )[0]
    assert '[ "$ENABLE_LIVOX_3D" != "true" ]' not in move_base_guard
    assert 'UNITREE_MOVE_BASE_SCAN_TOPIC="/scan"' in auto
    assert 'scan_topic:="$move_base_scan_topic"' in auto
    assert 'rosrun laser_filters scan_to_scan_filter_chain' in auto
    assert 'hazardwalker_unitree_scan_filter' in auto
    assert 'UNITREE_MOVE_BASE_FILTERED_SCAN_TOPIC' in auto
    scan_filter = (
        PLATFORM / 'config' / 'unitree_scan_filter.yaml'
    ).read_text(encoding='utf-8')
    assert 'laser_filters/LaserScanBoxFilter' in scan_filter
    assert 'box_frame: base' in scan_filter
    assert 'min_x: -0.48' in scan_filter
    assert 'max_x: 0.45' in scan_filter
    costmap = (
        UNITREE / 'config' / 'hazardwalker_costmap_common_params.yaml'
    ).read_text(encoding='utf-8')
    assert '[[0.42, 0.38], [0.42, -0.38]' in costmap
    assert 'inflation_radius: 0.10' in costmap
    assert '<arg name="scan_topic" default="/livox/scan_projection"/>' in launch
    assert 'global_costmap/livox_scan/topic' in launch
    assert 'local_costmap/livox_scan/topic' in launch
    assert 'rosnode ping -c 1 /move_base' in compose
    assert 'pgrep -x move_base' in compose
    assert 'UNITREE_RL_FORCE_CPU: ${UNITREE_RL_FORCE_CPU:-1}' in compose
    robot_xacro = (
        PLATFORM / 'src' / 'unitree_guide' / 'unitree_ros' / 'robots' /
        'a1_description' / 'xacro' / 'robot.xacro'
    ).read_text(encoding='utf-8')
    assert robot_xacro.count('xyz="0.2 0 0.35"') == 2
    rl_source = (
        PLATFORM / 'src' / 'unitree_guide' / 'unitree_guide' /
        'unitree_guide' / 'src' / 'FSM' / 'State_RL_test.cpp'
    ).read_text(encoding='utf-8')
    assert 'std::getenv("UNITREE_RL_FORCE_CPU")' in rl_source
    assert 'cuda_available && !force_cpu' in rl_source


def test_formal_runner_requires_unitree_bridge_and_selects_backend():
    source = (
        ROOT / 'scripts' / 'run_official_slam_exploration.py'
    ).read_text(encoding='utf-8')
    assert "'local_planner_backend:=unitree_move_base'" in source
    assert "adapter.get('enable_unitree_move_base_bridge') is not True" in source
    assert "'rosnode ping -c 1 /move_base >/dev/null'" in source


def test_official_profile_reselects_frontier_after_dwa_no_progress():
    source = (
        ROOT / 'ros2_ws' / 'src' / 'hazardwalker_bringup' / 'launch' /
        'official_simenv_business.launch.py'
    ).read_text(encoding='utf-8')
    assert "'frontier_net_progress_timeout_s': 12.0" in source
    assert "'frontier_net_progress_distance_m': 0.20" in source
    assert "'goal_tolerance_m': 0.40" in source
    assert "'deterministic_room_route_enabled': True" in source
