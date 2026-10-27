"""安装官方仿真平台适配包，注册键盘控制和速度仲裁入口。"""
from setuptools import find_packages, setup

package_name = 'hazardwalker_platform'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', ['config/unitree_scan_filter.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HazardWalker Team',
    maintainer_email='todo@example.com',
    description='Official SimEnv adapters and command arbitration for HazardWalker.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control_node = hazardwalker_platform.keyboard_control_node:main',
            'command_mux_node = hazardwalker_platform.command_mux_node:main',
            'assist_alignment_node = hazardwalker_platform.assist_alignment_node:main',
        ],
    },
)
