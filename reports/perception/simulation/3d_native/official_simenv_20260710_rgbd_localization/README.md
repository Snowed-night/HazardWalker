# official_simenv_20260710_rgbd_localization

> **历史内部回归，禁止作为官方成绩。** 8 场 32 个点可用于复核 RGB-D 反投影与误差计算，
> 但均为生成场景单视角评估，没有合法 SLAM 多视角闭环。后续重跑放入 `reruns/YYYYMMDD_<seed>/`。

本目录只记录官方 SimEnv 原生复杂房间实验；保留墙体、门、家具、立柱、纵深和自然遮挡。

- 用例：8
- 通过：8
- 待复核：0
- 不读取 `danger_truth.json` 或私有布局真值。
- 多视角用例只有在 Gazebo 机器人世界位姿实际变化后才计为有效视角。
- 黄色 `reobserve` 是待复查候选，不等于确认识别。
