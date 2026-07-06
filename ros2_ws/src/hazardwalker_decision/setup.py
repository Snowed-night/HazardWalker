"""hazardwalker_decision 打包配置。

所属组：决策组。
文件作用：
- 让 ROS 2/colcon 能安装 `hazardwalker_decision` 包。
- 注册决策相关 console script。

当前入口：
- `mission_state_machine_node`：任务状态机和结果写入节点。

后续扩展：
- 新增完整 FSM、目标选择或重观察节点时，需要在 `entry_points` 中补对应命令。
"""
from setuptools import find_packages, setup

package_name = 'hazardwalker_decision'

# 该包放任务状态机、结果汇总和后续决策逻辑。

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
    description='Mission decision nodes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_state_machine_node = hazardwalker_decision.mission_state_machine_node:main',
        ],
    },
)
