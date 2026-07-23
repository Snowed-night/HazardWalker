# official_simenv_20260705_extended_red_object_stress

官方 SimEnv 原生 Gazebo 3D 红色非球体干扰实验。用于判断红色立方体误检是个例还是当前 HSV+形状筛选方案的系统性风险。

## 内容

- `images/`: 原生 Gazebo 3D 截图检测标注图。
- `images/collage.png`: 总览拼图。
- `cases.csv/json`: 每个案例的条件、检测数量和图片路径。
- `summary.json`: 汇总指标。
- `raw_frames.zip`: 原始 PPM 归档，不展开提交。

## 摘要

- 案例数：6
- TP/FP/FN：2/2/0
- 总览图：`images/collage.png`
