"""SimEnv Docker ROS1 + HazardWalker 启动文件。

启动: ros2 launch hazardwalker_bringup simenv_demo.launch.py

数据流:
  Docker ROS1 → ros1_bridge (或 hw_bridge pipe) → /hw/* 话题
  SLAM Toolbox → /map + map→odom tf
  Frontier Explorer → /hw/cmd_vel + /hw/nav/state
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('hazardwalker_nav')
    slam_config = os.path.join(nav_pkg, 'config', 'slam_toolbox_online_async.yaml')

    return LaunchDescription([
        # ---- SLAM Toolbox (在线异步建图) ----
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config],
            remappings=[
                ('/tf', '/hw/tf'),
                ('/tf_static', '/hw/tf'),
            ],
        ),

        # ---- 感知: HSV 红色危险源检测 ----
        Node(
            package='hazardwalker_perception',
            executable='hsv_detector_node',
            name='hsv_detector_node',
            output='screen',
        ),

        # ---- 导航: Frontier 自主探索 (替代固定航点巡检) ----
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
            remappings=[
                ('/tf', '/hw/tf'),
                ('/tf_static', '/hw/tf'),
            ],
        ),

        # ---- 决策: 任务状态机 ----
        Node(
            package='hazardwalker_decision',
            executable='mission_state_machine_node',
            name='mission_state_machine_node',
            output='screen',
        ),
    ])
