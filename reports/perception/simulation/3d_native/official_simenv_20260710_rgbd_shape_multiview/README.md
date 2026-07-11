# official_simenv_20260710_rgbd_shape_multiview

官方 SimEnv 原生 RGB-D 红色形状与多视角确认前置抑制实验。固定相机下先验证“哪些候选绝不能进入 confirmed”；随后才由真实运动平台提供第二视角。

## 覆盖与结果

共 8 例，全部通过：红球正例、红色立方体、长方体、圆柱端面、圆柱侧面、红色平板、红球+圆柱端面、红球+平板。

- 圆柱端面出现圆形红色 2D 投影，但 `depth_shape.status=flat`、`confirmation_eligible=false`，不会写入确认轨迹。
- 圆柱侧面、立方体、长方体和平板均无严格候选；其中平板由 2D shape/extent 在深度步骤前拒绝，这同样满足不误确认要求。
- 混合场景仅保留红球的严格候选；圆柱端面仍被平面深度抑制。

总览见 `images/shape_multiview_collage.png`，逐例状态在 `snapshots/`、`cases.csv/json` 和测试组镜像中可复核。

注意：本目录验证了连续多视角确认的“第一道门”——红色非球体不能凭单帧进入轨迹。真实第二视角控制验证见同日期的 `official_simenv_20260710_dynamic_view_trial`，当前平台尚未提供足够的相机位姿变化。
