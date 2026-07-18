"""启动 SLAM Toolbox 在线异步建图。

用法: ros2 launch hazardwalker_nav slam_toolbox.launch.py

前置条件:
  - /hw/scan (LaserScan) — 由 official_simenv_rosbridge_ros2_adapter_node 提供
  - odom → base_link tf — 由适配器从 /hazardwalker/odom 真值生成，发布到 /tf
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('hazardwalker_nav')
    config_path = os.path.join(pkg_dir, 'config', 'slam_toolbox_online_async.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[config_path],
        # 适配器已直接发布 /tf，无需重映射
    )

    return LaunchDescription([slam_node])
