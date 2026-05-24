"""hazardwalker_bringup 打包配置。

所属组：系统集成组。
文件作用：
- 让 ROS 2/colcon 能安装 `hazardwalker_bringup` 包。
- 把 `launch/*.launch.py` 安装到 share 目录，供 `ros2 launch` 调用。

当前入口：
- `launch/minimal_demo.launch.py`：最小 demo 启动文件。

后续扩展：
- 新增 launch 文件后，只要放在 `launch/` 目录中，就会被当前 `glob('launch/*.launch.py')` 自动安装。
"""
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hazardwalker_bringup'

# 该包只负责 launch 和系统组合，不放复杂算法。

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HazardWalker Team',
    maintainer_email='todo@example.com',
    description='Bringup launch files for HazardWalker.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
