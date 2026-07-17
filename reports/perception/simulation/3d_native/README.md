# 官方原生 3D 感知实验索引

本目录同时保留“历史内部回归”和后续重跑结果，用于可视化算法从候选、复查到确认的优化路径。
历史记录不是官方随机场景成绩，不能用其中的 `pass_count` 宣称比赛通过率或 SOTA。

| 类别 | 历史目录 | 可复用价值 | 已知限制 |
|---|---|---|---|
| 多球粘连 | `official_simenv_20260710_rgbd_multi_ball_clutter` | 展示粘连分割、漏分和多帧恢复 | 47 个目标仅 7 个完成确认 |
| 部分可见 | `official_simenv_20260710_rgbd_partial_visibility` | 展示不同可见比例下的候选与重观察边界 | 人工夹具，不能代表随机场景召回率 |
| 红色物品 | `official_simenv_20260710_rgbd_red_objects` | 展示非球体抑制与剩余疑难反例 | 单视角，圆锥端面、椭球等仍会产生候选 |
| 多视角 | `official_simenv_20260710_rgbd_active_multiview` | 展示主动复查的预期流程 | 旧视角语义审计不合格，不能计为有效多视角成绩 |
| 三维定位 | `official_simenv_20260710_rgbd_complex_localization` | 展示 RGB-D 反投影和误差计算 | 生成场景单视角评估，不是合法 SLAM 闭环 |

统一约定：

- 旧文件原样保留，结构化摘要额外写入 `official_score_eligible=false`。
- 黄色 `reobserve` 只表示待复查候选，不等于确认红球。
- 后续同类复跑放入对应目录的 `reruns/YYYYMMDD_<seed>/`，不得覆盖历史基线。
- 新结果必须记录固定 SEED、代码版本、合法定位来源、RGB-D、轨迹、确认过程和测试表。
- 只有通过独立视角语义、真值隔离和正式证据契约审计的重跑，才可另行标记为正式候选证据。
