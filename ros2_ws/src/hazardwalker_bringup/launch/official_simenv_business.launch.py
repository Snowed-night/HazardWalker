"""官方 SimEnv 的 ROS2 业务层启动入口。

所属组：系统集成/平台组。负责人：姜晨。
文件作用：只启动 ROS2 业务节点，假定官方 ROS1 容器及双向适配层已提供稳定 /hw/*；绝不启动
fake_platform_node 或 Gazebo Harmonic，避免把占位平台接入官方比赛 profile。
当前边界：默认不启动固定航点巡检。它仅可用于接口诊断，不能替代导航组的自主探索实现。
验证方式：先运行 scripts/verify_official_simenv_ros1_adapter.sh，再 ros2 launch 本文件。
"""

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
    return LaunchDescription([
        DeclareLaunchArgument('start_perception', default_value='true'),
        DeclareLaunchArgument('start_decision', default_value='true'),
        # 当前 waypoint_patrol_node 是固定航点诊断节点，正式自主探索接入前保持关闭。
        DeclareLaunchArgument('start_navigation', default_value='false'),
        Node(
            package='hazardwalker_perception',
            executable='hsv_detector_node',
            name='hsv_detector_node',
            output='screen',
            condition=IfCondition(start_perception),
        ),
        Node(
            package='hazardwalker_decision',
            executable='mission_state_machine_node',
            name='mission_state_machine_node',
            output='screen',
            condition=IfCondition(start_decision),
        ),
        Node(
            package='hazardwalker_nav',
            executable='waypoint_patrol_node',
            name='waypoint_patrol_node',
            output='screen',
            # 官方 A1 对低角速度 RL 指令存在实测死区；仅官方 profile 设置下限。
            parameters=[{'minimum_turn_speed': 0.45}],
            condition=IfCondition(start_navigation),
        ),
    ])
