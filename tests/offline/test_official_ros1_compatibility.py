"""官方 ROS1 Noetic 适配脚本的 Python 3.8 语法兼容性检查。

ROS1 Noetic 常用 Python 3.8，而主仓库 ROS2 开发机可能更新。这个测试不导入 rospy，
只用 Python AST 的 3.8 grammar 检查官方节点与其复用的纯函数能否被 Noetic 解析。
"""

import ast
import os
import math


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def test_official_ros1_node_and_pure_modules_parse_as_python38():
    """禁止把 Python 3.10 专属语法悄悄带进官方 ROS1 启动路径。"""
    paths = [
        'scripts/official_simenv_ros1_perception_node.py',
        'scripts/official_simenv_ros1_evidence_recorder.py',
        'scripts/official_simenv_lidar_imu_slam_node.py',
        'scripts/official_simenv_ros1_perception_sweep.py',
        'ros2_ws/src/hazardwalker_perception/hazardwalker_perception/red_ball_detector.py',
        'ros2_ws/src/hazardwalker_perception/hazardwalker_perception/localize_hazard.py',
        'ros2_ws/src/hazardwalker_perception/hazardwalker_perception/track_hazards.py',
        'ros2_ws/src/hazardwalker_perception/hazardwalker_perception/scan_imu_localization.py',
        'ros2_ws/src/hazardwalker_perception/hazardwalker_perception/active_view_policy.py',
        'ros2_ws/src/hazardwalker_perception/hazardwalker_perception/active_view_geometry.py',
        'ros2_ws/src/hazardwalker_perception/hazardwalker_perception/room_search_policy.py',
        'ros2_ws/src/hazardwalker_decision/hazardwalker_decision/result_builder.py',
        'ros2_ws/src/hazardwalker_decision/hazardwalker_decision/official_simenv_contract.py',
    ]
    for relative_path in paths:
        path = os.path.join(REPO_ROOT, relative_path)
        # 历史检测模块含 UTF-8 BOM；解释器会识别它，语法审计也应按同样规则读取。
        with open(path, encoding='utf-8-sig') as handle:
            ast.parse(handle.read(), filename=relative_path, feature_version=(3, 8))


def test_public_start_pose_can_transform_team_slam_coordinate_to_world():
    """官方公开起点位姿可用于坐标对齐，不能用 ground_truth 里程计替代。"""
    # 与 ROS1 节点采用相同的二维 yaw 变换：world 起点 (10, -2)，朝向 +90 度。
    start_x, start_y, start_yaw = 10.0, -2.0, math.pi / 2.0
    local_x, local_y = 2.0, 0.0
    world_x = math.cos(start_yaw) * local_x - math.sin(start_yaw) * local_y + start_x
    world_y = math.sin(start_yaw) * local_x + math.cos(start_yaw) * local_y + start_y

    assert round(world_x, 6) == 10.0
    assert round(world_y, 6) == 0.0


def test_official_ros1_perception_does_not_read_generated_scene_metadata():
    """正式算法仅接收显式公开出生点，不能从 manifest 旁路获得场景信息。"""
    path = os.path.join(REPO_ROOT, 'scripts', 'official_simenv_ros1_perception_node.py')
    launcher = os.path.join(REPO_ROOT, 'scripts', 'run_official_simenv_ros1_perception.sh')
    with open(path, encoding='utf-8') as handle:
        source = handle.read()
    with open(launcher, encoding='utf-8') as handle:
        launcher_source = handle.read()

    assert 'team_scene_info' not in source
    # 节点可在注释中声明禁用文件，但正式输入路径不应调用文件读取 API。
    assert '.read_text(' not in source
    assert "'~public_start_world_x', 0.0" in source
    assert '_public_start_world_x:=0.0' in launcher_source
    assert '_team_scene_info_path' not in launcher_source


def test_official_ros1_evidence_recorder_uses_legal_inputs_and_archives_result():
    """ROS1 正式证据入口须与 ROS2 记录器同样拒绝真值位姿。"""
    path = os.path.join(REPO_ROOT, 'scripts', 'official_simenv_ros1_evidence_recorder.py')
    with open(path, encoding='utf-8') as handle:
        source = handle.read()

    assert "'~image_topic': '/real_sense/rgb/image_raw'" in source
    assert "'~depth_topic': '/real_sense/depth/image_raw'" in source
    assert "'~detection_topic': '/hazardwalker/perception/hazard_detections'" in source
    assert "'~legal_pose_topic': ''" in source
    assert "Subscriber('/Odometry_gazebo'" not in source
    assert "Subscriber('/hazardwalker/odom'" not in source
    assert "shutil.copy2" in source
    assert "'trajectory.jsonl'" in source
    assert "'~save_context_frames': True" in source
    assert "'~context_image_save_interval_sec': 10.0" in source
    assert "'~max_context_images': 80" in source
    assert "'official random scene context (no candidate)'" in source
    assert "'~mission_state_topic': '/hazardwalker/mission/state'" in source
    assert "self.mission_completed = True" in source
    assert "'mission_not_completed'" in source


def test_official_perception_evidence_orchestrator_fails_closed_and_never_owns_cmd_vel():
    """正式编排只服务感知；场景独占、固定 SEED 和导航完成信号缺一不可。"""
    path = os.path.join(REPO_ROOT, 'scripts', 'run_official_simenv_perception_evidence.sh')
    with open(path, encoding='utf-8') as handle:
        source = handle.read()

    assert 'OFFICIAL_SIMENV_EXCLUSIVE_SESSION' in source
    assert 'OFFICIAL_SCENARIO_SEED' in source
    assert 'OFFICIAL_CODE_VERSION' in source
    assert 'OFFICIAL_MAX_RUNTIME_SEC' in source and '> 600' in source
    assert "_auto_activate_cmd_vel:=false" in source
    assert "Publisher('/cmd_vel'" not in source
    assert "grep -m 1 -F 'FINISHED'" in source
    assert "_run_mode:=official_random_scene" in source


def test_official_ros1_node_uses_the_current_depth_shape_api():
    """防止 ROS1 节点引用已废弃的深度形状函数而只通过语法检查。"""
    import sys

    package_root = os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_perception')
    if package_root not in sys.path:
        sys.path.insert(0, package_root)
    from hazardwalker_perception.localize_hazard import evaluate_sphere_depth_shape

    assert callable(evaluate_sphere_depth_shape)


def test_official_ros1_node_calls_depth_localizer_with_named_arguments():
    """深度定位参数顺序相近，强制使用关键字以防 RGB-D 实机回调传反。"""
    import ast

    path = os.path.join(REPO_ROOT, 'scripts', 'official_simenv_ros1_perception_node.py')
    with open(path, encoding='utf-8') as handle:
        tree = ast.parse(handle.read(), filename=path)
    calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'localize_bbox_from_depth_image'
    ]
    assert len(calls) == 1
    assert not calls[0].args
    assert {
        item.arg for item in calls[0].keywords
    } >= {
        'bbox', 'intrinsics', 'depth_image', 'camera_to_output',
        'camera_axis_convention',
    }


def test_official_ros1_node_converts_optical_depth_axis_to_gazebo_camera_link():
    """官方 CameraInfo 是光学投影，world TF 却连接 X 向前的 real_sense 链路。"""
    node_path = os.path.join(REPO_ROOT, 'scripts', 'official_simenv_ros1_perception_node.py')
    launcher_paths = (
        os.path.join(REPO_ROOT, 'scripts', 'run_official_simenv_ros1_perception.sh'),
        os.path.join(REPO_ROOT, 'scripts', 'run_official_simenv_perception_evidence.sh'),
    )
    with open(node_path, encoding='utf-8') as handle:
        source = handle.read()

    assert "'~camera_axis_convention', 'gazebo_link_x_forward'" in source
    assert 'camera_axis_convention=self.camera_axis_convention' in source
    for launcher_path in launcher_paths:
        with open(launcher_path, encoding='utf-8') as handle:
            launcher_source = handle.read()
        assert '_camera_axis_convention:=gazebo_link_x_forward' in launcher_source


def test_official_ros1_stability_reuses_axis_aware_view_geometry():
    """RealSense 是 X 前向，ROS1 稳定门不得再硬编码旋转矩阵第三列。"""

    path = os.path.join(REPO_ROOT, 'scripts', 'official_simenv_ros1_perception_node.py')
    with open(path, encoding='utf-8') as handle:
        source = handle.read()

    assert 'camera_pose_signature(transform, self.camera_axis_convention)' in source
    assert 'quantized_camera_view_id(' in source
    assert 'transform.rotation[0][2]' not in source
    assert 'transform.rotation[1][2]' not in source


def test_official_ros1_node_emits_navigation_owned_reobservation_requests():
    """感知节点只发布侧向复查建议，不可直接把候选变成 /cmd_vel 控制。"""
    path = os.path.join(REPO_ROOT, 'scripts', 'official_simenv_ros1_perception_node.py')
    with open(path, encoding='utf-8') as handle:
        source = handle.read()

    assert 'choose_active_view_action' in source
    assert 'plan_lateral_reobservation' in source
    assert '/hazardwalker/perception/reobservation_request' in source
    assert "'view_id': self._stable_view_id if camera_stable else ''" in source
    assert "Publisher(self.reobserve_topic, String" in source
    assert "Publisher('/cmd_vel'" not in source


def test_official_ros1_result_export_requires_legal_slam_and_multiview_sphere_evidence():
    """ROS1 正式入口也必须与 ROS2 入口保持同样的 fail-closed 约束。"""
    path = os.path.join(REPO_ROOT, 'scripts', 'official_simenv_ros1_perception_node.py')
    with open(path, encoding='utf-8') as handle:
        source = handle.read()

    assert "'~localization_provenance', 'unverified'" in source
    assert 'min_spherical_views_for_confirm=2' in source
    assert 'require_legal_localization=True' in source
    assert 'require_multiview_sphere_evidence=True' in source
    assert "self.official_result_frame = 'world'" in source
    assert 'expected_frame=self.official_result_frame' in source
    assert "item['localization_provenance'] = self.localization_provenance" in source
    assert 'self.tracker.published_tracks()' in source
    assert "'localization_ready': bool(localization_ready)" in source
    assert "'localization_provenance': self.localization_provenance" in source
    assert "'stamp_sec': round(float(stamp_sec), 6)" in source


def test_official_ros1_uses_raw_depth_for_independent_diameter_evidence():
    """尺寸证据不能复用由已知球半径反推的球心深度。"""

    path = os.path.join(REPO_ROOT, 'scripts', 'official_simenv_ros1_perception_node.py')
    with open(path, encoding='utf-8') as handle:
        source = handle.read()

    assert 'estimate_depth_from_bbox(' in source
    assert 'bbox, raw_surface_depth_m, self.camera_intrinsics' in source
    assert "item['raw_surface_depth_m']" in source


def test_official_joy_activation_sequence_requires_stand_settle_then_cmd_vel():
    """正式自主运行不依赖人工按键，且不能跳过站立稳定阶段。"""
    import sys

    package_root = os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_decision')
    if package_root not in sys.path:
        sys.path.insert(0, package_root)
    from hazardwalker_decision.official_simenv_contract import activation_command

    timing = (1.0, 0.8, 4.0, 0.8)
    assert activation_command(0.5, *timing) == ('waiting_for_controller', None)
    assert activation_command(1.2, *timing) == ('standing', 1)
    assert activation_command(2.0, *timing) == ('settling', None)
    assert activation_command(6.0, *timing) == ('switching_to_cmd_vel', 3)
    assert activation_command(7.0, *timing) == ('ready', None)


def test_legal_lidar_imu_localizer_never_uses_gazebo_truth_inputs():
    """定位入口只能依赖官方允许的激光与 IMU，不能把 Gazebo 状态包装成 SLAM。"""
    path = os.path.join(REPO_ROOT, 'scripts', 'official_simenv_lidar_imu_slam_node.py')
    with open(path, encoding='utf-8') as handle:
        source = handle.read()

    assert "'/scan'" in source and "'/trunk_imu'" in source
    assert "'/hazardwalker/slam/odometry'" in source
    assert "'~slam_base_frame', 'slam_base'" in source
    assert "'~camera_frame', 'real_sense'" in source
    assert '/Odometry_gazebo' in source  # 文档禁止说明必须存在。
    assert "rospy.get_published_topics()" in source
    assert "self.scan_topic, PointCloud2, self._on_point_cloud" in source
    assert "self.scan_topic, LaserScan, self._on_laser_scan" in source
    assert "point_cloud2.read_points(" in source
    assert "point_cloud_xyz_to_base_points" in source
    assert "Subscriber(self.imu_topic, Imu" in source
    assert "'~floor_index_topic', '/hazardwalker/navigation/floor_index'" in source
    assert 'floor_index_to_elevation' in source
    assert 'message.pose.pose.position.z = self.floor_elevation_m' in source
    assert "Subscriber('/Odometry_gazebo'" not in source
    assert "Subscriber('/hazardwalker/odom'" not in source


def test_ros2_legal_localizer_uses_only_public_scan_imu_and_floor_action():
    path = os.path.join(
        REPO_ROOT,
        'ros2_ws',
        'src',
        'hazardwalker_perception',
        'hazardwalker_perception',
        'scan_imu_localizer_node.py',
    )
    with open(path, encoding='utf-8') as handle:
        source = handle.read()

    assert "declare_parameter('scan_topic', '/hw/scan')" in source
    assert "declare_parameter('imu_topic', '/hw/trunk_imu')" in source
    assert "declare_parameter('cmd_vel_topic', '/hw/cmd_vel')" in source
    assert 'motion_prior_base=motion_prior' in source
    assert "'/hazardwalker/navigation/floor_index'" in source
    assert "TransformBroadcaster(self)" in source
    assert '/hw/odom' in source  # 文件头的显式禁止说明必须存在。
    assert "create_subscription(\n            Odometry" not in source
    assert '/Odometry_gazebo' in source  # 文件头的显式禁止说明必须存在。


def test_perception_sweep_is_autonomous_exclusive_and_truth_safe():
    """入口环视只能用 IMU 闭环，且与导航发布者冲突时必须停车。"""
    path = os.path.join(
        REPO_ROOT, 'scripts', 'official_simenv_ros1_perception_sweep.py',
    )
    with open(path, encoding='utf-8') as handle:
        source = handle.read()

    assert "'~exclusive_session', False" in source
    assert "'~scenario_seed', ''" in source
    assert "'~code_version', ''" in source
    assert "'~imu_topic', '/trunk_imu'" in source
    assert "rospy.Publisher(self.cmd_vel_topic, Twist" in source
    assert 'getSystemState()' in source
    assert "'foreign_cmd_vel_publisher'" in source
    assert 'self._stop()' in source
    assert "'official_score_eligible': False" in source
    assert "'scenario_seed': self.scenario_seed" in source
    assert "'code_version': self.code_version" in source
    assert "'truth_or_layout_inputs_used': False" in source
    assert '/Odometry_gazebo' not in source
    assert '/ground_truth/' not in source
    assert 'danger_truth' not in source
    assert 'scene_manifest' not in source


def test_ros1_perception_launchers_explicitly_include_noetic_python3_packages():
    """官方容器缺失 PYTHONPATH 时，启动器仍须能导入 rospy/cv_bridge。"""
    for relative_path in (
            'scripts/run_official_simenv_lidar_imu_slam.sh',
            'scripts/run_official_simenv_ros1_perception.sh'):
        path = os.path.join(REPO_ROOT, relative_path)
        with open(path, encoding='utf-8') as handle:
            source = handle.read()
        assert 'ROS_PYTHON_DIST_PACKAGES="/opt/ros/noetic/lib/python3/dist-packages"' in source
        assert 'export PYTHONPATH="$ROS_PYTHON_DIST_PACKAGES:' in source
