"""最小 demo 启动文件。

所属组：集成/平台组。
文件作用：
- 一次性启动平台、感知、导航和决策四个最小节点。
- 作为当前最小闭环的统一入口。

后续扩展方式：
- 如果增加 Gazebo 或官方平台适配，可以在这里拆成 `gazebo_minimal.launch.py`、`official_minimal.launch.py` 等入口。
- 这份 launch 目前只负责把最小 demo 串起来，不写复杂条件分支。
"""
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
