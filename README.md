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
├─ docs/                 项目文档、接口规范、会议纪要、周报和实验记录
├─ ros2_ws/              ROS 2 工作空间
│  └─ src/               ROS 2 功能包源码
├─ scripts/              环境安装、仿真启动、测试统计等脚本
├─ config/               传感器、导航、感知、决策等配置文件
├─ tests/                单元测试、集成测试和回归测试
├─ data_samples/         小型样例数据，不存放大 rosbag 或视频
├─ tools/                工具说明，不直接提交大型二进制工具
└─ reports/              技术报告、自测试报告、PPT、视频脚本等材料
```

## Planned ROS Packages

```text
hazardwalker_platform     平台适配，统一仿真/官方平台接口
hazardwalker_nav          SLAM、导航、自主探索和返航
hazardwalker_perception   红球检测、相机雷达融合和三维定位
hazardwalker_decision     任务状态机、NBV、重观察和返航约束
hazardwalker_bringup      一键启动和系统集成
hazardwalker_msgs         自定义消息、服务和动作定义
```

## Development Rule

日常开发基于 `dev` 创建功能分支，通过 Pull Request 合并，不直接在 `main` 上开发。具体提交流程教程放在HazardWalker\docs\guidebook\git提交手册.md里。

## License
This project is licensed under the Apache License 2.0. See [LICENSE](./LICENSE) for details.

