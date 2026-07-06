"""hazardwalker_platform 打包配置。

所属组：平台组。
文件作用：
- 让 ROS 2/colcon 能安装 `hazardwalker_platform` 包。
- 注册平台适配相关 console script。
- 安装仿真场景、模型、配置和启动文件到 share 目录。

当前入口：
- `fake_platform_node`：最小 demo 使用的平台占位节点。
- Gazebo 最小仿真通过 `gazebo_minimal.launch.py` 启动。

后续扩展：
- 新增 Gazebo 或官方平台节点时，需要在 `entry_points` 中补对应命令。
"""
import glob
import os
from setuptools import find_packages, setup

package_name = 'hazardwalker_platform'

# 该包提供平台适配节点，核心是把外部传感器/控制接口转换成 `/hw/*` 内部接口。

# 收集模型文件：models/ 下所有 .sdf 和 model.config 文件
_model_files = (
    glob.glob('models/**/*.sdf', recursive=True)
    + glob.glob('models/**/model.config', recursive=True)
)
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
    description='Platform adapters for HazardWalker — fake, Gazebo, and official platform layers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_platform_node = hazardwalker_platform.fake_platform_node:main',
        ],
    },
)
