"""hazardwalker_perception 打包配置。

所属组：感知组。
文件作用：
安装感知包并注册 `hsv_detector_node` 入口。
新增定位、跟踪或调试图像节点时，在 `entry_points` 中补入口。
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
            'dynamic_detection_recorder_node = hazardwalker_perception.dynamic_detection_recorder_node:main',
        ],
    },
)
