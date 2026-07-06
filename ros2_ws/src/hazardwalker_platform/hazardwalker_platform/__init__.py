
"""hazardwalker_platform 包初始化文件。

所属组：平台组。
文件作用：
- 标记 `hazardwalker_platform` 是一个 Python 包。
- 平台适配节点放在独立模块中，例如 `fake_platform_node.py`。

当前职责：
- 不执行任何平台初始化逻辑。
- 避免 import 时自动创建 ROS 节点。

后续扩展：
- 如果增加 Gazebo 或官方平台适配节点，应新增独立模块，不写在本文件中。
"""
