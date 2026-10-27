# 运行工具

日常使用根目录的 `check.sh`、`build.sh`、`run.sh`，完整操作见[快速开始](../docs/guidebook/quickstart.md)。

| 文件 | 职责 |
| --- | --- |
| `reproduction/verify.py` | 源码与环境检查 |
| `reproduction/prepare_assets.py` | 导入官方策略与 SDK 运行资源 |
| `reproduction/build.sh`、`build_noetic.py` | 构建 ROS 2 和 ROS 1 工作区 |
| `reproduction/run.py` | 单次实验启动、资源检查与结果核验 |
| `run_stable_three_floor_experiment.sh` | 编排仿真、业务栈和收尾 |
| `run_official_slam_exploration.py` | 整场任务运行及记录 |
| `manage_official_simenv_*.sh` | 平台适配器和命令仲裁的生命周期 |
| `run_official_simenv_*.sh` | 适配器与命令仲裁的底层启动 |
| `official_simenv_rosbridge_ros2_adapter_node.py` | ROS 1 / ROS 2 数据与控制桥接 |
| `evaluate_result.py` | 危险源结果匹配 |
| `reproduction_check_result.py` | 完整三层任务验收 |
| `run_offline_tests.py` | 核心算法及接口测试入口 |
