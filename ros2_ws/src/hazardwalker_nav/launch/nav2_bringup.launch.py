"""Nav2 导航栈启动文件 —— HazardWalker 诊断 / 手动导航。

所属组：导航组。
使用场景：
  - 诊断模式独立启动：ros2 launch hazardwalker_nav nav2_bringup.launch.py
  - 与 frontier 探索互补：frontier 决策“去哪”，Nav2 负责“怎么走”
  - 手动导航：通过 RVIZ "2D Goal Pose" 发送目标点

前置条件：
  - SLAM Toolbox 已运行并发布 /map
  - scan_imu_localizer_node 已运行并发布 odom→base TF
  - /hw/scan 可用

不启动：
  - AMCL（SLAM Toolbox mapping 模式直接提供 map→odom）
  - map_server（地图由 SLAM /map 话题动态提供）

坐标系：map → odom → base（不使用 base_link / base_footprint）
控制输出：/hw/cmd_vel

验证方式：
  ros2 topic list | grep -E '/(local|global)_costmap|plan|cmd_vel_nav'
  在 RVIZ 中设置 "2D Goal Pose" 测试 NavigateToPose
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    pkg_dir = get_package_share_directory('hazardwalker_nav')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='使用仿真时钟')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='自动激活 lifecycle 节点')

    # ---- Lifecycle 节点 ----
    # lifecycle_manager 在 autostart=true 时按依赖顺序自动 configure→activate。
    # 节点按 costmap→planner→controller→bt→behaviors→waypoint 顺序激活。

    # Nav2 内部 cmd_vel 流：controller_server → cmd_vel → velocity_smoother → cmd_vel_smoothed
    # 最终输出映射到 /hw/cmd_vel，由官方适配器转发给 Docker 中的机器人。
    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    planner_server = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    behavior_server = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    waypoint_follower = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # velocity_smoother 接收 controller_server 的原始 cmd_vel，平滑后输出到 /hw/cmd_vel
    velocity_smoother = LifecycleNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_smoothed', '/hw/cmd_vel')],
    )

    lifecycle_manager = LifecycleNode(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': [
                'controller_server',
                'planner_server',
                'bt_navigator',
                'behavior_server',
                'waypoint_follower',
                'velocity_smoother',
            ]},
        ],
    )

    return LaunchDescription([
        declare_sim_time,
        declare_autostart,
        controller_server,
        planner_server,
        bt_navigator,
        behavior_server,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
    ])
