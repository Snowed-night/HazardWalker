# hazardwalker_bringup

启动包，负责一键启动、参数加载和系统组合。

## 当前职责

- 启动 launch 文件
- 加载参数
- 组合平台、感知、导航和决策节点
- 提供最小 demo 入口

## 当前文件

- `launch/minimal_demo.launch.py`：最小闭环启动入口。
- `launch/official_simenv_business.launch.py`：官方 ROS1 SimEnv 已由平台适配器提供 `/hw/*` 后的 ROS2
  业务入口；不启动 fake 平台或 Harmonic，固定航点导航默认关闭。

## 后续扩展

- 官方场景先执行 `scripts/run_official_simenv_rosbridge_adapter.sh` 和逐段验收，再运行
  `ros2 launch hazardwalker_bringup official_simenv_business.launch.py`；详见
  `docs/guidebook/官方SimEnv平台环境使用手册.md`。
