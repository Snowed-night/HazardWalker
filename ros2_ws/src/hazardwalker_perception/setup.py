"""hazardwalker_perception 打包配置。

所属组：感知组。
文件作用：
- 让 ROS 2/colcon 能安装 `hazardwalker_perception` 包。
- 注册感知相关 console script。

当前入口：
- `hsv_detector_node`：HSV 红球检测节点。

后续扩展：
- 新增三维定位、跟踪或调试图像节点时，需要在 `entry_points` 中补对应命令。
"""
from setuptools import find_packages, setup

package_name = 'hazardwalker_perception'

# 该包放红球检测、定位和感知相关节点，离线函数与 ROS 节点都归入这里。

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
    description='Hazard perception nodes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hsv_detector_node = hazardwalker_perception.hsv_detector_node:main',
        ],
    },
)
