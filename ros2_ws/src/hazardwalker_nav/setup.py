"""hazardwalker_nav 打包配置。

所属组：导航组。
文件作用：
- 让 ROS 2/colcon 能安装 `hazardwalker_nav` 包。
- 注册导航相关 console script。

当前入口：
- `waypoint_patrol_node`：固定航点巡检节点。

后续扩展：
- 新增 Nav2 封装或 Frontier 节点时，需要在 `entry_points` 中补对应命令。
"""
from setuptools import find_packages, setup

package_name = 'hazardwalker_nav'

# 该包放导航与航点控制相关节点，后续可逐步替换为 Nav2 或 Frontier 逻辑。

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HazardWalker Team',
    maintainer_email='todo@example.com',
    description='Navigation nodes for HazardWalker.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_patrol_node = hazardwalker_nav.waypoint_patrol_node:main',
        ],
    },
)
