"""hazardwalker_nav 打包配置。

所属组：导航组。
文件作用：
- 让 ROS 2/colcon 能安装 `hazardwalker_nav` 包。
- 注册导航相关 console script 并安装启动/配置文件。

入口：
- `waypoint_patrol_node`：固定航点巡检节点（诊断/回退）。
- `frontier_explorer_node`：Frontier 自主探索节点（正式）。
"""
import glob

from setuptools import find_packages, setup

package_name = 'hazardwalker_nav'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch',
         glob.glob('launch/*.launch.py')),
        (f'share/{package_name}/config',
         glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HazardWalker Team',
    maintainer_email='todo@example.com',
    description='Navigation nodes for HazardWalker: frontier exploration and SLAM integration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_patrol_node = hazardwalker_nav.waypoint_patrol_node:main',
            'frontier_explorer_node = hazardwalker_nav.frontier_explorer_node:main',
        ],
    },
)
