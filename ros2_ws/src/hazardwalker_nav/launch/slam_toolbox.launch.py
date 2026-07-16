"""启动 SLAM Toolbox 在线异步建图。

用法: ros2 launch hazardwalker_nav slam_toolbox.launch.py

前置条件:
  - /hw/scan (LaserScan) 有实时数据
  - odom → base_link tf 已发布 (来自 hw_bridge /hw/tf 或 ros1_bridge)
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
        # 将 tf 来源重映射到 /hw/tf (pipe bridge 发布)
        remappings=[
            ('/tf', '/hw/tf'),
            ('/tf_static', '/hw/tf'),
        ],
    )

    return LaunchDescription([slam_node])
