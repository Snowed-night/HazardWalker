# 官方原生 3D 感知实验索引

本目录按交付阶段和统一实验类别保存可控专项结果。历史记录用于展示算法优化路径，
不是官方随机场景成绩；只有 `reports/perception/official_random/` 才能放未经修改的
官方 `auto.sh` 随机场景结果。

## 统一类别

- `red_ball_detection`：官方规格红球候选检测。
- `red_ball_3d_localization` / `rgbd_localization`：RGB-D 三维定位。
- `official_distractor_rejection`：只测试官方红色立方体和绿色球体。
- `partial_visibility`：部分可见候选边界。
- `active_multiview_reobservation`：真实移动后的主动复查。
- `multi_ball_clutter`：多球粘连和重复计数压力。
- `extended_red_object_stress`：圆柱、圆锥、平板和不规则红物，仅作内部压力测试。

## 阶段目录

| 阶段 | 目录 | 性质与已知限制 |
|---|---|---|
| 20260705 | `official_simenv_20260705_red_ball_detection` | 历史原生3D检测链验证；原始压缩帧未进Git |
| 20260705 | `official_simenv_20260705_partial_visibility` | 历史前景遮挡梯度 |
| 20260705 | `official_simenv_20260705_multi_ball_clutter` | 历史多球视角与粘连记录 |
| 20260705 | `official_simenv_20260705_extended_red_object_stress` | 历史非球体内部压力测试 |
| 20260710 | `official_simenv_20260710_multi_ball_clutter` | 历史内部回归，确认率低 |
| 20260710 | `official_simenv_20260710_partial_visibility` | 人工夹具，不代表随机场景召回率 |
| 20260710 | `official_simenv_20260710_extended_red_object_stress` | 20+内部物品，超出官方干扰源范围 |
| 20260710 | `official_simenv_20260710_active_multiview_reobservation` | 旧视角语义审计不合格 |
| 20260710 | `official_simenv_20260710_rgbd_localization` | 历史跨帧误差口径失效，无合法SLAM闭环 |
| 20260715 | `official_simenv_20260715_partial_visibility` | 实际2026-07-19运行，9/21，通过与失败均保留 |
| 20260715 | `official_simenv_20260715_extended_red_object_stress` | 实际2026-07-19两批运行，9/24 |

每个历史目录新增 `provenance.json`，保存原目录、Git提交、真实运行日期和缺失项。
三套 `reference_20260705_regression` 的再处理时间与运行提交无法确定，已明确标为
`provenance_uncertain`，没有强行归入某次真实运行。

## 当前固定交付标签

- A阶段已完成：`official_simenv_20260725_red_ball_3d_localization`、
  `official_simenv_20260725_official_distractor_rejection`。5例base定位最大误差
  0.0644 m；5例官方干扰组合 confirmed 虚警为0。独立红球检测未出现回归，
  因此按目标约定没有重复新建 `official_simenv_20260725_red_ball_detection`。
- B阶段：`official_simenv_20260730_partial_visibility_reobservation`、
  `official_simenv_20260730_official_random_robustness`。其中真正的官方随机场景证据
  仍必须保存到 `reports/perception/official_random/`，B阶段目录只保存专项汇总和链接。

所有新目录必须同时含原图、标注图、`README.md`、`summary.json`、
`testing_record_perception.csv/json`、SEED、真实时间、Git版本、启动参数和失败说明。
黄色 `reobserve` 只是候选，不等于确认红球；局部相机/base坐标也不能写入官方
`detected_danger.json`。
