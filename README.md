# HazardWalker

HazardWalker 是面向 DG-202602 “基于四足机器人的危险源自主搜索与识别技术”赛题的工程仓库。

目标是在未知多层楼栋仿真环境中，实现四足机器人自主探索、红色球体危险源识别、三维定位、结果输出和自主返航。

## Repository Structure

当前已有分支结构：

```text
main                 稳定版本，只放可演示、可提交版本
dev                  日常集成版本
feature/platform     平台与仿真开发
feature/nav          导航探索开发
feature/perception   感知定位开发
feature/test         测试脚本开发
docs/report          文档材料开发
fix/xxx              Bug 修复
```

当前项目结构：

```text
HazardWalker/
├─ docs/                 项目背景、初期开发总结、最小 demo 实现
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
hazardwalker_decision     任务状态机、NBV、重观察和返航约束
hazardwalker_bringup      一键启动和系统集成
hazardwalker_msgs         自定义消息定义
```

## 开发规则

日常开发基于 `dev` 创建功能分支，通过 Pull Request 合并，不直接在 `main` 上开发。文档以 `docs/项目背景介绍.md`、`docs/初期项目开发总结.md`、`docs/最小demo实现.md` 为主。

## 当前最小 demo

仓库当前正在实现最小闭环 demo，目标是先把平台、感知、导航、决策和结果输出串起来。

相关文件：

- `docs/最小demo实现.md`
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

