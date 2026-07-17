# HazardWalker 文档目录

本文档说明 `docs/` 下的资料组织方式。后续新增文档优先放入对应目录，避免根目录堆积。

## 目录结构

| 目录 | 内容 |
|---|---|
| `overview/` | 赛题背景、项目定位、评分指标等长期稳定信息 |
| `development/` | 初期开发总结、最小 demo、阶段路线和近期任务 |
| `environment/` | 主力机、SSH、远程桌面、GPU、共享目录等环境说明 |
| `groups/` | 平台、导航、感知、决策、集成测试、文档组资料 |
| `guidebook/` | Git、提交流程、协作规范 |
| `meeting_notes/` | 会议纪要 |
| `experiments/` | 实验计划、实验记录、失败案例 |

## 重点入口

- [项目背景介绍](overview/项目背景介绍.md)
- [DG-202602 项目总结](overview/DG-202602项目总结.md)
- [HazardWalker 技术选型与总体方向](overview/HazardWalker技术选型与总体方向.md)
- [项目初期开发](development/项目初期开发.md)
- [最小 demo 实现](development/最小demo实现.md)
- [主力机环境使用指南](environment/主力机环境使用指南.md)
- [远程桌面使用指南](environment/远程桌面使用指南.md)
- [官方 SimEnv ROS1/ROS2 双向适配整改](environment/官方SimEnv_ROS1_ROS2双向适配整改_20260714.md)
- [Gazebo 远程 GUI 分离操作指南](Gazebo远程GUI分离操作指南.md)
- [各组文档索引](groups/README.md)
- [Git 提交手册](guidebook/git提交手册.md)

## 内容边界

- 赛题目标、评分和项目定位放 `overview/`。
- 后续开发计划、阶段任务和各组交付统一放 `development/项目初期开发.md` 或 `groups/`。
- 主力机账号、环境、SSH、RDP、GPU 和共享目录放 `environment/`。
- 不在仓库文档中保存明文密码、管理员凭据、RDP 网关密码或 GitHub token。
