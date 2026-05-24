
"""hazardwalker_bringup 包初始化文件。

所属组：系统集成组。
文件作用：
- 标记 `hazardwalker_bringup` 是一个 Python 包。
- 本包主要存放 launch 文件和系统组合入口，不在 `__init__.py` 中写启动逻辑。

当前职责：
- 保持包可被 ROS 2 Python 打包工具识别。
- 一键启动逻辑放在 `launch/minimal_demo.launch.py` 和 `scripts/run_minimal_demo.sh`。

后续扩展：
- 如果需要共享 launch 辅助函数，可新增独立 Python 模块，例如 `launch_utils.py`。
- 不建议在这里 import 具体节点，避免启动时产生隐式副作用。
"""
