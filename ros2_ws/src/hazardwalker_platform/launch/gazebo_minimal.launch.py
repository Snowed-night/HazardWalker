# -*- coding: utf-8 -*-
"""
HazardWalker Phase 1 — Gazebo 最小仿真启动文件。

用途：
  一键启动 Gazebo Harmonic 场景 + ros_gz_bridge + 算法节点，
  替换 fake_platform_node 的人造数据为真实仿真数据。

用法（仅限 Ubuntu / ROS 2 Jazzy）：
  ros2 launch hazardwalker_platform gazebo_minimal.launch.py

参数：
  headless  : 不启动 GUI（默认 false，即显示 GUI）

Windows 说明：
  本文件检测到非 Linux 环境时，只打印提示信息而不崩溃，
  方便在 Windows 上做语法检查和文件校验。
"""

import os
import sys
import platform

# ============================================================================
# 平台检测：非 Linux 环境直接退出（不抛异常，方便 Windows 上做语法检查）
# ============================================================================
_IS_LINUX = platform.system() == 'Linux'

if not _IS_LINUX:
    print(
        '\n'
        '╔══════════════════════════════════════════════════════════════╗\n'
        '║  [gazebo_minimal.launch.py] 当前运行在 Windows 上            ║\n'
        '║                                                              ║\n'
        '║  此启动文件需要 Ubuntu + ROS 2 Jazzy + Gazebo Harmonic。     ║\n'
        '║  Windows 上仅可做语法检查，实际仿真请 SSH 到主力机 hxbl。  ║\n'
        '║                                                              ║\n'
        '║  SSH 登录后运行：                                            ║\n'
        '║    ros2 launch hazardwalker_platform gazebo_minimal.launch.py║\n'
        '╚══════════════════════════════════════════════════════════════╝\n'
    )
    # 在 Windows 上正常退出，不报错
    sys.exit(0)

# ============================================================================
# 以下是 Linux 专用代码：所有 ROS 2 / Gazebo 导入放在此处
# ============================================================================

try:
    from launch import LaunchDescription
    from launch.actions import (
        DeclareLaunchArgument,
        ExecuteProcess,
        LogInfo,
        RegisterEventHandler,
        Shutdown,
    )
    from launch.conditions import IfCondition, LaunchConfigurationEquals
    from launch.event_handlers import OnProcessExit
    from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare
except ImportError as exc:
    print(f'[ERROR] ROS 2 launch 模块未找到: {exc}')
    print('请确认在 ROS 2 Jazzy 环境中运行此脚本。')
    sys.exit(1)


def generate_launch_description():
    """生成 Gazebo 最小仿真启动描述。"""

    # ------------------------------------------------------------------
    # 路径：世界文件 + 模型目录 + 桥接配置
    # ------------------------------------------------------------------
    pkg_share = FindPackageShare('hazardwalker_platform')

    world_file = PathJoinSubstitution([
        pkg_share, 'worlds', 'hazardwalker_minimal.sdf'
    ])

    models_dir = PathJoinSubstitution([
        pkg_share, 'models'
    ])

    bridge_config = PathJoinSubstitution([
        pkg_share, 'config', 'ros_gz_bridge.yaml'
    ])

    # ------------------------------------------------------------------
    # 启动参数
    # ------------------------------------------------------------------
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='是否以 headless 模式运行 Gazebo（不启动 GUI）',
    )

    # ------------------------------------------------------------------
    # Gazebo 仿真
    # ------------------------------------------------------------------
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            '-r',                                           # 启动后自动运行
            '-v', '2',                                      # 日志级别
            world_file,
        ],
        output='screen',
        additional_env={
            # 将 models/ 加入 Gazebo 资源路径，使 <include> 能找到模型
            'GZ_SIM_RESOURCE_PATH': models_dir,
        },
        condition=LaunchConfigurationEquals('headless', 'false'),
    )

    gz_sim_headless = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            '-r',
            '-s',                                           # headless 模式
            '--headless-rendering',                         # 无 GUI 时仍启用相机渲染
            '-v', '2',
            world_file,
        ],
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': models_dir,
        },
        condition=LaunchConfigurationEquals('headless', 'true'),
    )

    # ------------------------------------------------------------------
    # ros_gz_bridge —Gazebo 话题 → /hw/* 内部接口
    # ------------------------------------------------------------------
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[
            {
                'config_file': bridge_config,
            }
        ],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # ------------------------------------------------------------------
    # /hw/cmd_vel → /cmd_vel 转发
    # 导航组发布 /hw/cmd_vel，Gazebo DiffDrive 插件订阅 /cmd_vel。
    # 通过 ros_gz_bridge 中已配置的 /cmd_vel 桥接实现连接。
    # 此处为冗余说明，实际 remap 在 bridge YAML 中完成。
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # 算法节点（与最小 demo 一致的四个节点）
    # ------------------------------------------------------------------
    perception_node = Node(
        package='hazardwalker_perception',
        executable='hsv_detector_node',
        name='hsv_detector_node',
        output='screen',
    )

    nav_node = Node(
        package='hazardwalker_nav',
        executable='waypoint_patrol_node',
        name='waypoint_patrol_node',
        output='screen',
    )

    decision_node = Node(
        package='hazardwalker_decision',
        executable='mission_state_machine_node',
        name='mission_state_machine_node',
        output='screen',
    )

    # ------------------------------------------------------------------
    # 启动顺序提示
    # ------------------------------------------------------------------
    log_start = LogInfo(
        msg='[gazebo_minimal] 正在启动 Gazebo 仿真 + 全链路算法节点...'
    )

    return LaunchDescription([
        headless_arg,
        log_start,
        gz_sim,
        gz_sim_headless,
        gz_bridge,
        perception_node,
        nav_node,
        decision_node,
    ])
