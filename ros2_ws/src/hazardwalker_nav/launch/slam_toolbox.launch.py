"""启动 SLAM Toolbox 在线异步建图。

用法: ros2 launch hazardwalker_nav slam_toolbox.launch.py

前置条件:
  - /hw/scan (LaserScan) — 由 official_simenv_rosbridge_ros2_adapter_node 提供
  - odom → base tf — 由合法 scan/IMU 里程计生成，不读取 Gazebo 真值
  - /clock — 由官方适配器从 ROS1 转发，整套业务统一使用仿真时间
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('hazardwalker_nav')
    config_path = os.path.join(pkg_dir, 'config', 'slam_toolbox_online_async.yaml')
    upstream_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py',
    )

    # slam_toolbox 必须完成 configure→activate 生命周期，不能只创建普通 Node。
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upstream_launch),
            launch_arguments={
                'slam_params_file': config_path,
                'autostart': 'true',
                'use_sim_time': 'true',
            }.items(),
        ),
    ])
