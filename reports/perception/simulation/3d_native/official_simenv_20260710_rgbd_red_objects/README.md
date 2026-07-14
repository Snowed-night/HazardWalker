# official_simenv_20260710_rgbd_red_objects

本目录只记录官方 SimEnv 原生复杂房间实验；保留墙体、门、家具、立柱、纵深和自然遮挡。

> 比赛最终目标仅为红色球体。本目录中的红色方块、圆柱、圆锥、椭球和不规则物
> 都是**干扰物压力测试**：它们可以产生 `reobserve` 候选以触发侧视，但绝不能进入
> `results/detected_danger.json`。

- 用例：24
- 通过：24
- 待复核：0
- 不读取 `danger_truth.json` 或私有布局真值。
- 多视角用例只有在 Gazebo 机器人世界位姿实际变化后才计为有效视角。
- 黄色 `reobserve` 是待复查候选，不等于确认识别。
