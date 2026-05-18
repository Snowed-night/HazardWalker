from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hazardwalker_platform',
            executable='fake_platform_node',
            name='fake_platform_node',
            output='screen',
        ),
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
