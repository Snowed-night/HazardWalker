# HazardWalker

HazardWalker 是面向 DG-202602 “基于四足机器人的危险源自主搜索与识别技术”赛题的工程仓库。

目标是在未知多层楼栋仿真环境中，实现四足机器人自主探索、红色球体危险源识别、三维定位、结果输出和自主返航。

## Repository Structure

分支约定：

```text
main                 稳定版本，只放可演示、可提交版本
dev                  日常集成版本
feature/<group>-<topic> 平台、导航、感知、决策、测试等功能开发
docs/<topic>         文档、报告、答辩材料开发
fix/xxx              Bug 修复
```

当前项目结构：

```text
HazardWalker/
├─ docs/                 项目文档、环境指南、各组资料和实验记录
├─ ros2_ws/              ROS 2 工作空间
│  └─ src/               ROS 2 功能包源码
├─ scripts/              构建、启动和离线测试脚本
├─ config/               话题、坐标系和算法参数配置
├─ tests/                离线测试和后续集成测试
├─ data_samples/         小型样例数据
├─ tools/                工具说明
└─ reports/              报告、实验记录和演示材料
```

## ROS 包

```text
hazardwalker_platform     平台适配，统一仿真/官方平台接口
hazardwalker_nav          SLAM、导航、自主探索和返航
hazardwalker_perception   红球检测、相机雷达融合和三维定位
hazardwalker_decision     任务状态机、重观察和返航约束
hazardwalker_bringup      一键启动和系统集成
hazardwalker_msgs         自定义消息定义
```

## 开发规则

日常开发基于 `dev` 创建功能分支，通过 Pull Request 合并，不直接在 `main` 上开发。文档入口见 `docs/README.md`。

各组成员修改文件内容时，需要同步检查并更新相关 README：

- 修改项目总体结构、运行方式、分支规则或公共约定时，同步更新根目录 `README.md`。
- 修改某个 ROS 2 包、脚本、配置、测试或文档目录时，同步更新对应目录下的 `README.md`。
- 如果确认 README 不需要改动，需要在提交说明或 PR 中写明原因。

官方 SimEnv（ROS1 Noetic + Gazebo Classic）与早期本地 Gazebo Harmonic 方案不可混用：业务节点始终使用
`/hw/*`，当前官方接入经容器 `rosbridge_websocket` 与 ROS2 适配器完成，不依赖容器内的 `ros1_bridge dynamic_bridge`。
官方运行、控制和 RGB-D 验收说明见
[`docs/guidebook/官方SimEnv平台环境使用手册.md`](docs/guidebook/官方SimEnv平台环境使用手册.md)。

新增代码文件时，需要在文件首部注明本文件的作用，建议包含所属小组、文件职责、当前实现边界和验证方式。重要模块、重要类和重要函数需要有简明注释或 docstring，说明输入、输出、关键逻辑和后续扩展点。注释应帮助成员理解代码，不写重复代码本身含义的空泛说明。

## 早期最小 demo（历史基线）

仓库保留最小闭环 demo，用于本地接口和启动流程回归；它不是官方比赛环境、未知楼宇探索或最终评测方案。`fake_platform_node`、固定航点与本地 Harmonic 配置不得替代当前官方 SimEnv 链路。

当前官方平台操作以 [`docs/guidebook/官方SimEnv平台环境使用手册.md`](docs/guidebook/官方SimEnv平台环境使用手册.md) 为准；早期设计与文件说明见 [`docs/history/project_early/`](docs/history/project_early/)。

相关文件：

- `docs/development/最小demo实现.md`
- `ros2_ws/src/hazardwalker_bringup/launch/minimal_demo.launch.py`
- `ros2_ws/src/hazardwalker_platform/hazardwalker_platform/fake_platform_node.py`
- `ros2_ws/src/hazardwalker_perception/hazardwalker_perception/red_ball_detector.py`
- `ros2_ws/src/hazardwalker_perception/hazardwalker_perception/hsv_detector_node.py`
- `ros2_ws/src/hazardwalker_nav/hazardwalker_nav/waypoint_controller.py`
- `ros2_ws/src/hazardwalker_nav/hazardwalker_nav/waypoint_patrol_node.py`
- `ros2_ws/src/hazardwalker_decision/hazardwalker_decision/result_builder.py`
- `ros2_ws/src/hazardwalker_decision/hazardwalker_decision/mission_state_machine_node.py`
- `scripts/run_minimal_demo.sh`
- `scripts/run_offline_tests.py`
- `scripts/evaluate_result.py`
- `tests/offline/`

## License
This project is licensed under the Apache License 2.0. See [LICENSE](./LICENSE) for details.

