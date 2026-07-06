# hazardwalker_bringup

启动包，负责一键启动、参数加载和系统组合。

## 当前职责

- 启动 launch 文件
- 加载参数
- 组合平台、感知、导航和决策节点
- 提供最小 demo 入口

## 当前文件

- `launch/minimal_demo.launch.py`：最小闭环启动入口。

## 后续扩展

- 后续可再增加 `gazebo_minimal.launch.py`、`official_minimal.launch.py`、`full_system.launch.py`。
