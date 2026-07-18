"""已弃用名称的官方 SimEnv 安全兼容入口。

所属组：系统集成/平台组。
文件作用：保留旧的 ``simenv_demo.launch.py`` 命令兼容性，但不再复制平台、SLAM、
感知和导航节点定义；所有业务节点统一交给 ``official_simenv_business.launch.py``。

安全边界：
- 不把 ``/tf`` 与 ``/tf_static`` 重映射或合并到 ``/hw/tf``。
- SLAM 由正式业务入口显式选择 Cartographer 或 slam_toolbox；本兼容入口默认关闭。
- 导航默认关闭，只有调用方在独占会话中显式传入 ``start_navigation:=true`` 才运行。
- 平台数据必须来自正式 rosbridge 适配器；本入口不会启动旧 ``hw_topic_relay``。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """把旧命令安全转发到唯一的官方业务 launch。"""

    business_launch = os.path.join(
        get_package_share_directory('hazardwalker_bringup'),
        'launch',
        'official_simenv_business.launch.py',
    )
    start_perception = LaunchConfiguration('start_perception')
    start_decision = LaunchConfiguration('start_decision')
    start_navigation = LaunchConfiguration('start_navigation')
    start_slam = LaunchConfiguration('start_slam')
    slam_backend = LaunchConfiguration('slam_backend')
    start_legal_localization = LaunchConfiguration('start_legal_localization')
    perception_output_frame = LaunchConfiguration('perception_output_frame')
    localization_provenance = LaunchConfiguration('localization_provenance')
    exploration_timeout_s = LaunchConfiguration('exploration_timeout_s')

    return LaunchDescription([
        DeclareLaunchArgument('start_perception', default_value='true'),
        DeclareLaunchArgument('start_decision', default_value='true'),
        DeclareLaunchArgument('start_navigation', default_value='false'),
        DeclareLaunchArgument('start_slam', default_value='false'),
        DeclareLaunchArgument('slam_backend', default_value='cartographer'),
        DeclareLaunchArgument('start_legal_localization', default_value='true'),
        DeclareLaunchArgument('perception_output_frame', default_value='map'),
        DeclareLaunchArgument('localization_provenance', default_value='unverified'),
        DeclareLaunchArgument('exploration_timeout_s', default_value='540.0'),
        LogInfo(msg=(
            '[DEPRECATED] simenv_demo.launch.py 仅为兼容入口；'
            '正式运行请使用 scripts/run_official_simenv_ros1_ros2_stack.sh。'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(business_launch),
            launch_arguments={
                'start_perception': start_perception,
                'start_decision': start_decision,
                'start_navigation': start_navigation,
                'start_slam': start_slam,
                'slam_backend': slam_backend,
                'start_legal_localization': start_legal_localization,
                'start_evidence_recorder': 'false',
                'perception_output_frame': perception_output_frame,
                'localization_provenance': localization_provenance,
                'exploration_timeout_s': exploration_timeout_s,
            }.items(),
        ),
    ])
