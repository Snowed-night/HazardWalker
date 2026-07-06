"""SimEnv Docker ROS1 + HazardWalker 启动文件。

启动: ros2 launch hazardwalker_bringup simenv_demo.launch.py

不启动 fake_platform_node, 传感器数据来自 Docker ROS1 → hw_topic_relay → /hw/*
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hazardwalker_perception',
            executable='hsv_detector_node',
            name='hsv_detector_node',
            output='screen',
        ),
        Node(
            package='hazardwalker_nav',
            executable='waypoint_patrol_node',
            name='waypoint_patrol_node',
            output='screen',
        ),
        Node(
            package='hazardwalker_decision',
            executable='mission_state_machine_node',
            name='mission_state_machine_node',
            output='screen',
        ),
    ])
