# 感知组仿真结果目录

本目录只保存 Gazebo/官方 SimEnv 相关检测效果，并按来源分为：

```text
simulation/
├─ 3d_native/    Gazebo/官方 SimEnv 原生 3D 相机截图
└─ 2d_derived/   基于官方环境截图或风格背景派生的二维压力测试
```

原生 3D 图用于证明真实 Gazebo 渲染链路下的效果；2D 派生图用于快速覆盖更多比例、数量和干扰边界。两类结果不能混为同一证据。

## 3D Native

阶段和类别索引见 `3d_native/README.md`。20260705、20260710、20260715 分目录保存；
来源不完整的再处理结果标记为 `provenance_uncertain`，不得冒充某次真实运行。

## 2D Derived

- `2d_derived/official_simenv_20260705_partial_red/`: 15% 到 95% 部分可见比例递进。
- `2d_derived/official_simenv_20260705_distractor_stress/`: 异色球、红色方块和杂波干扰。
- `2d_derived/official_simenv_20260705_multi_target_count/`: 1 到 10 个红球数量递进。
- `2d_derived/official_simenv_20260705_final_stress/`: 多红球、部分可见、干扰、弱光和噪声组合压力测试。
