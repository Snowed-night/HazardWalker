"""hazardwalker_platform 打包配置。

所属组：平台组。
文件作用：
- 让 ROS 2/colcon 能安装 `hazardwalker_platform` 包。
- 注册平台适配相关 console script。

当前入口：
- `fake_platform_node`：最小 demo 使用的平台占位节点。

后续扩展：
- 新增 Gazebo 或官方平台节点时，需要在 `entry_points` 中补对应命令。
"""
from setuptools import find_packages, setup

package_name = 'hazardwalker_platform'

# 该包提供平台适配节点，核心是把外部传感器/控制接口转换成 `/hw/*` 内部接口。

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
    description='Platform adapters for HazardWalker.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_platform_node = hazardwalker_platform.fake_platform_node:main',
        ],
    },
)
