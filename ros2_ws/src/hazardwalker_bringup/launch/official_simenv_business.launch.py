"""官方 SimEnv 的 ROS2 业务层启动入口。

所属组：系统集成/平台组。负责人：姜晨。
文件作用：只启动 ROS2 业务节点，假定官方 ROS1 容器及双向适配层已提供稳定 /hw/*；绝不启动
fake_platform_node 或 Gazebo Harmonic，避免把占位平台接入官方比赛 profile。

导航组 (2026-07-17 更新):
- 新增 SLAM Toolbox 在线异步建图节点
- 新增 frontier_explorer_node 自主探索（替代固定航点 waypoint_patrol_node）
- 保留 waypoint_patrol_node 作为诊断回退

验证方式：先运行 scripts/run_official_simenv_rosbridge_adapter.sh，再 ros2 launch 本文件。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """按显式开关组合业务节点，官方 profile 不引入任何模拟平台节点。"""

    start_perception = LaunchConfiguration('start_perception')
    start_decision = LaunchConfiguration('start_decision')
    start_navigation = LaunchConfiguration('start_navigation')
    start_slam = LaunchConfiguration('start_slam')
    nav_mode = LaunchConfiguration('nav_mode')
    perception_output_frame = LaunchConfiguration('perception_output_frame')
    localization_provenance = LaunchConfiguration('localization_provenance')

    nav_pkg = get_package_share_directory('hazardwalker_nav')
    slam_config = os.path.join(nav_pkg, 'config', 'slam_toolbox_online_async.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('start_perception', default_value='true'),
        DeclareLaunchArgument('start_decision', default_value='true'),
        DeclareLaunchArgument('start_navigation', default_value='true'),
        DeclareLaunchArgument('start_slam', default_value='true'),
        # nav_mode: 'frontier' (自主探索，默认) 或 'waypoint' (固定航点诊断)
        DeclareLaunchArgument('nav_mode', default_value='frontier'),
        DeclareLaunchArgument('perception_output_frame', default_value='map'),
        DeclareLaunchArgument('localization_provenance',
                              default_value='lidar_imu_slam'),

        # ---- SLAM Toolbox (在线异步建图) ----
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config],
            condition=IfCondition(start_slam),
        ),

        # ---- 感知: HSV 红色危险源检测 ----
        Node(
            package='hazardwalker_perception',
            executable='hsv_detector_node',
            name='hsv_detector_node',
            output='screen',
            parameters=[{
                'camera_axis_convention': 'gazebo_link_x_forward',
                'output_frame': perception_output_frame,
                'localization_provenance': localization_provenance,
            }],
            condition=IfCondition(start_perception),
        ),

        # ---- 决策: 任务状态机 ----
        Node(
            package='hazardwalker_decision',
            executable='mission_state_machine_node',
            name='mission_state_machine_node',
            output='screen',
            condition=IfCondition(start_decision),
        ),

        # ---- 导航: Frontier 自主探索 (默认) ----
        Node(
            package='hazardwalker_nav',
            executable='frontier_explorer_node',
            name='frontier_explorer_node',
            output='screen',
            parameters=[{
                'exploration_timeout_s': 540.0,
                'min_frontier_size': 10,
                'goal_tolerance_m': 0.8,
                'linear_speed': 0.35,
                'angular_speed': 0.8,
            }],
            condition=IfCondition(start_navigation),
        ),

        # ---- 导航: 固定航点巡检 (诊断回退，仅 nav_mode=waypoint 时启用) ----
        Node(
            package='hazardwalker_nav',
            executable='waypoint_patrol_node',
            name='waypoint_patrol_node',
            output='screen',
            parameters=[{'minimum_turn_speed': 0.45}],
            condition=IfCondition(
                LaunchConfiguration('start_navigation_waypoint', default='false')),
        ),
    ])
