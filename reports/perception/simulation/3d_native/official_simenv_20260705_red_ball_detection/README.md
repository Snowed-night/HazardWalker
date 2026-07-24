# official_simenv_20260705_red_ball_detection

官方 SimEnv 原生 Gazebo 3D 渲染验证结果。

## 为什么需要本目录

前面的 `partial_red`、`distractor_stress`、`multi_target_count`、`final_stress` 是基于官方 SimEnv 风格背景构造的二维压力测试，适合评估检测器边界，但不能单独证明 Gazebo 3D 原生渲染下效果一致。

本目录补充真正从官方 SimEnv Gazebo world 中相机 topic 取出的 3D 渲染图，图像未做二维合成。

## 内容

- `images/`: 原生 3D 截图检测标注图。
- `images/native_3d_validation_collage.png`: 7 张原生 3D 验证图总览。
- `cases.csv/json`: 每张图的条件和检测数量。
- `summary.json`: Gazebo 采集脚本输出的完整检测结果。
- `raw_frames.zip`: 原始 PPM 归档，不展开提交。

## 覆盖场景

- 官方生成红球完整可见视角：1 例。
- 官方生成红球 FOV 边缘部分可见视角：3 例。
- 官方红色方块干扰：1 例。
- 官方 world 内原生 spawn 的多红球+干扰物：2 例。

说明：`native_spawned_*` 案例不是二维绘制，而是在官方 Gazebo world 中通过 `/gazebo/spawn_sdf_model` 生成球体/方块模型，再由 Gazebo 相机渲染。
