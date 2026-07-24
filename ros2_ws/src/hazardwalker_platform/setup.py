"""hazardwalker_platform 打包配置。

所属组：平台组。
文件作用：
- 让 ROS 2/colcon 能安装 `hazardwalker_platform` 包。
- 注册平台适配相关 console script。
- 安装仿真场景、模型、配置和启动文件到 share 目录。

当前入口：
- `fake_platform_node`：最小 demo 使用的平台占位节点。
- `keyboard_control_node`：负责人维护的 /hw/cmd_vel 安全键盘控制工具。
- `gazebo_adapter_node`：[Phase 2] Gazebo 平台适配节点（占位，待实现）。

Phase 1 安装内容：
- worlds/hazardwalker_minimal.sdf → share/hazardwalker_platform/worlds/
- models/{red_ball,simple_robot}/model.sdf → share/hazardwalker_platform/models/
- config/ros_gz_bridge.yaml → share/hazardwalker_platform/config/
- launch/gazebo_minimal.launch.py → share/hazardwalker_platform/launch/
"""
import glob
import os
from setuptools import find_packages, setup

package_name = 'hazardwalker_platform'

# 收集模型文件：models/ 下所有 .sdf 文件
_model_files = glob.glob('models/**/*.sdf', recursive=True)
_model_entries = []
for f in _model_files:
    dest_dir = os.path.join(f'share/{package_name}', os.path.dirname(f))
    _model_entries.append((dest_dir, [f]))

# 收集场景文件
_world_files = glob.glob('worlds/**/*.sdf', recursive=True)
_world_entries = []
for f in _world_files:
    dest_dir = os.path.join(f'share/{package_name}', 'worlds')
    _world_entries.append((dest_dir, [f]))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/gazebo_minimal.launch.py']),
        (f'share/{package_name}/config', ['config/ros_gz_bridge.yaml']),
    ] + _model_entries + _world_entries,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HazardWalker Team',
    maintainer_email='todo@example.com',
    description='Platform adapters for HazardWalker — Gazebo, official platform, and minimal demo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_platform_node = hazardwalker_platform.fake_platform_node:main',
            'keyboard_control_node = hazardwalker_platform.keyboard_control_node:main',
        ],
    },
)
