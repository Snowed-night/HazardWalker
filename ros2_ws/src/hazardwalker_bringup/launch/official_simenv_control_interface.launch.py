"""官方业务栈可切换控制接口。

文件作用：
- 在不改动感知、GUI、记录和导航算法的前提下统一键盘、导航、辅助对准；
- 导航只写入独立输入源，``command_mux_node`` 是唯一 ``/hw/cmd_vel`` 发布者；
- 默认不启动导航和 SLAM，避免无参数运行时抢占共享官方容器。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """组合统一控制层和原官方业务栈。"""

    bringup_share = get_package_share_directory('hazardwalker_bringup')
    business_launch = os.path.join(
        bringup_share, 'launch', 'official_simenv_business.launch.py')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('control_mode', default_value='keyboard'),
        DeclareLaunchArgument('start_command_mux', default_value='true'),
        DeclareLaunchArgument('start_assist_alignment', default_value='true'),
        DeclareLaunchArgument('start_navigation', default_value='false'),
        DeclareLaunchArgument('start_slam', default_value='false'),
        DeclareLaunchArgument('start_pointcloud_map', default_value='false'),
        DeclareLaunchArgument('start_slam_video', default_value='false'),
        DeclareLaunchArgument('slam_backend', default_value='cartographer'),
        DeclareLaunchArgument('slam_dimension', default_value='3d'),
        DeclareLaunchArgument('start_perception', default_value='true'),
        DeclareLaunchArgument('start_decision', default_value='false'),
        DeclareLaunchArgument('start_legal_localization', default_value='true'),
        DeclareLaunchArgument('start_evidence_recorder', default_value='false'),
        DeclareLaunchArgument('nav_mode', default_value='frontier'),
        DeclareLaunchArgument(
            'local_planner_backend', default_value='direct'),
        DeclareLaunchArgument('localization_provenance', default_value='unverified'),
        DeclareLaunchArgument('perception_output_frame', default_value='map'),
        DeclareLaunchArgument('perception_parameter_file', default_value=''),
        DeclareLaunchArgument('exploration_timeout_s', default_value='540.0'),
        DeclareLaunchArgument('mission_time_budget_s', default_value='600.0'),
        DeclareLaunchArgument('navigation_linear_speed', default_value='1.20'),
        DeclareLaunchArgument(
            'navigation_minimum_linear_speed', default_value='0.80'),
        DeclareLaunchArgument('target_floors', default_value='[]'),
        DeclareLaunchArgument('per_floor_exploration_s', default_value='150.0'),
        DeclareLaunchArgument('manual_elevator_assist', default_value='true'),
        DeclareLaunchArgument(
            'automatic_elevator_entry', default_value='true'),
        DeclareLaunchArgument(
            'strict_room_inspection', default_value='false'),
        DeclareLaunchArgument('strict_room_clearance_m', default_value='0.60'),
        DeclareLaunchArgument(
            'simenv_container', default_value='simenv_ros1_hazard_platform'),
        DeclareLaunchArgument('evidence_output_dir', default_value=''),
        DeclareLaunchArgument('nav_record_dir', default_value=''),
        DeclareLaunchArgument('slam_monitor_output_dir', default_value=''),
        DeclareLaunchArgument('pointcloud_map_output_dir', default_value=''),
        DeclareLaunchArgument('slam_video_output', default_value=''),
        DeclareLaunchArgument('test_record_dir', default_value=''),
        DeclareLaunchArgument('scenario_seed', default_value=''),
        DeclareLaunchArgument('code_version', default_value=''),
        DeclareLaunchArgument(
            'official_result_path', default_value='results/detected_danger.json'),
        DeclareLaunchArgument('official_hazard_source_frame', default_value='map'),
        DeclareLaunchArgument('official_world_from_map_x', default_value='0.0'),
        DeclareLaunchArgument('official_world_from_map_y', default_value='0.0'),
        DeclareLaunchArgument('official_world_from_map_yaw', default_value='0.0'),
        DeclareLaunchArgument('official_floor_height_m', default_value='2.6'),
        DeclareLaunchArgument(
            'official_sphere_center_height_m', default_value='0.15'),

        Node(
            package='hazardwalker_platform',
            executable='command_mux_node',
            name='hazardwalker_command_mux',
            output='screen',
            parameters=[{
                'default_mode': LaunchConfiguration('control_mode'),
                # 控制看门狗和 20 Hz 转发必须使用墙钟。官方 Gazebo 在复杂
                # 楼宇中实时倍率可能低于 1；若跟随 /clock，控制频率会按同样
                # 比例下降，造成键盘延迟、转向卡顿和超时停车不及时。
                'use_sim_time': False,
            }],
            condition=IfCondition(LaunchConfiguration('start_command_mux')),
        ),
        Node(
            package='hazardwalker_platform',
            executable='assist_alignment_node',
            name='hazardwalker_assist_alignment',
            output='screen',
            parameters=[{
                # 状态心跳通常可精确恢复接管前模式；若启动早期尚未收到心跳，
                # 也必须回到本轮声明的键盘/导航/停止模式，不能硬退回键盘。
                'fallback_mode': LaunchConfiguration('control_mode'),
                # 辅助对准的超时同属安全控制合同，不得被仿真倍率拖慢。
                'use_sim_time': False,
            }],
            condition=IfCondition(
                LaunchConfiguration('start_assist_alignment')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(business_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'start_navigation': LaunchConfiguration('start_navigation'),
                'start_slam': LaunchConfiguration('start_slam'),
                'start_pointcloud_map': LaunchConfiguration(
                    'start_pointcloud_map'),
                'start_slam_video': LaunchConfiguration('start_slam_video'),
                'slam_backend': LaunchConfiguration('slam_backend'),
                'slam_dimension': LaunchConfiguration('slam_dimension'),
                'start_perception': LaunchConfiguration('start_perception'),
                'start_decision': LaunchConfiguration('start_decision'),
                'start_legal_localization': LaunchConfiguration(
                    'start_legal_localization'),
                'start_evidence_recorder': LaunchConfiguration(
                    'start_evidence_recorder'),
                'nav_mode': LaunchConfiguration('nav_mode'),
                'local_planner_backend': LaunchConfiguration(
                    'local_planner_backend'),
                'localization_provenance': LaunchConfiguration(
                    'localization_provenance'),
                'perception_output_frame': LaunchConfiguration(
                    'perception_output_frame'),
                'perception_parameter_file': LaunchConfiguration(
                    'perception_parameter_file'),
                'exploration_timeout_s': LaunchConfiguration(
                    'exploration_timeout_s'),
                'mission_time_budget_s': LaunchConfiguration(
                    'mission_time_budget_s'),
                'navigation_linear_speed': LaunchConfiguration(
                    'navigation_linear_speed'),
                'navigation_minimum_linear_speed': LaunchConfiguration(
                    'navigation_minimum_linear_speed'),
                'target_floors': LaunchConfiguration('target_floors'),
                'per_floor_exploration_s': LaunchConfiguration(
                    'per_floor_exploration_s'),
                'manual_elevator_assist': LaunchConfiguration(
                    'manual_elevator_assist'),
                'automatic_elevator_entry': LaunchConfiguration(
                    'automatic_elevator_entry'),
                'strict_room_inspection': LaunchConfiguration(
                    'strict_room_inspection'),
                'strict_room_clearance_m': LaunchConfiguration(
                    'strict_room_clearance_m'),
                'simenv_container': LaunchConfiguration('simenv_container'),
                'evidence_output_dir': LaunchConfiguration(
                    'evidence_output_dir'),
                'nav_record_dir': LaunchConfiguration('nav_record_dir'),
                'slam_monitor_output_dir': LaunchConfiguration(
                    'slam_monitor_output_dir'),
                'pointcloud_map_output_dir': LaunchConfiguration(
                    'pointcloud_map_output_dir'),
                'slam_video_output': LaunchConfiguration(
                    'slam_video_output'),
                'test_record_dir': LaunchConfiguration('test_record_dir'),
                'scenario_seed': LaunchConfiguration('scenario_seed'),
                'code_version': LaunchConfiguration('code_version'),
                'official_result_path': LaunchConfiguration(
                    'official_result_path'),
                'official_hazard_source_frame': LaunchConfiguration(
                    'official_hazard_source_frame'),
                'official_world_from_map_x': LaunchConfiguration(
                    'official_world_from_map_x'),
                'official_world_from_map_y': LaunchConfiguration(
                    'official_world_from_map_y'),
                'official_world_from_map_yaw': LaunchConfiguration(
                    'official_world_from_map_yaw'),
                'official_floor_height_m': LaunchConfiguration(
                    'official_floor_height_m'),
                'official_sphere_center_height_m': LaunchConfiguration(
                    'official_sphere_center_height_m'),
                'navigation_cmd_vel_topic': (
                    '/hw/control/navigation_cmd_vel'),
            }.items(),
        ),
    ])
