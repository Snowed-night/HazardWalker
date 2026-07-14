# 官方原生 3D 感知证据索引

2026-07-10 终轮只保留以下五类有效证据；其它日期目录是历史基线，不属于本轮验收。

| 类别 | 正式目录 | 规模 | 结果 |
|---|---|---:|---:|
| 多球粘连 | official_simenv_20260710_rgbd_multi_ball_clutter | 10 例、47 球 | 10/10 |
| 部分可见 | official_simenv_20260710_rgbd_partial_visibility | 21 例 | 21/21 |
| 红色物品 | official_simenv_20260710_rgbd_red_objects | 24 种 | 24/24 协议通过 |
| 真实运动多视角 | official_simenv_20260710_rgbd_active_multiview | 20 例 | 20/20 |
| 复杂环境三维定位 | official_simenv_20260710_rgbd_complex_localization | 8 例、32 点 | 8/8、32/32 |

统一口径：

- 场景为官方 SimEnv ROS2 Harmonic 已生成复杂建筑，不是纯灰墙空场景，也不是物理实机。
- 黄色 reobserve 是待复查候选，不等于确认识别。
- 多视角只在机器人 Gazebo 世界位姿实际变化后计数。
- 2026-07-10 目录的唯一性、图片/快照/结构化文件和测试组表格由 scripts/validate_official_simenv_complex_evidence.py 校验。
- 权威结论和限制见 reports/perception/docs/perception_progress_report_2026-07-10_evidence_matrix.md。
