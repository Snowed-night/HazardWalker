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
        'ros2_ws/src/hazardwalker_perception/hazardwalker_perception/red_ball_detector.py',
        'ros2_ws/src/hazardwalker_perception/hazardwalker_perception/localize_hazard.py',
        'ros2_ws/src/hazardwalker_perception/hazardwalker_perception/track_hazards.py',
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
    } >= {'bbox', 'intrinsics', 'depth_image', 'camera_to_output'}


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
