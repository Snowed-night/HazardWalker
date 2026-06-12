# 感知检测可视化结果

本目录用于保存红球检测离线案例生成脚本的输出。默认运行：

```bash
python scripts/generate_perception_cases.py
```

输出结构：

```text
reports/perception/result/
├─ cases/          自动生成的原始测试图，不提交 Git
├─ annotated/      带 bbox 和指标的标注图，不提交 Git
├─ figures/        汇报或论文使用的精选图和遮挡指标图
├─ summary.csv     每个案例的检测结果表，不提交 Git
├─ summary.json    每个案例的结构化结果，不提交 Git
├─ metrics_summary.csv   precision、recall、F1、top1 error、AP50 汇总，不提交 Git
└─ metrics_summary.json  结构化指标汇总，不提交 Git
```

当前保留并可提交的精选图：

```text
figures/perception_cases_collage.png
figures/occlusion_confidence.png
figures/occlusion_circularity.png
```

其中 `occlusion_confidence.png` 和 `occlusion_circularity.png` 来自 `summary.csv`
中的遮挡案例，用于说明当前 HSV + 形状筛选方法在 10% 到 70% 遮挡下的置信度和圆度变化。

实验字段说明：

| 字段 | 含义 |
|---|---|
| `case_id` | 案例名称 |
| `case_type` | 案例类型，例如颜色、遮挡、误检对照、复杂背景 |
| `shape` | 目标形状，例如球体、方块、不规则物体 |
| `color_type` | 颜色类型，例如正常红、暗红、低饱和红 |
| `occlusion_ratio` | 遮挡比例 |
| `expected_detected` | 预期是否检出 |
| `actual_detected` | 实际是否检出 |
| `confidence` | 检测置信度 |
| `circularity` | 圆度指标 |
| `aspect_ratio` | bbox 短边与长边比例 |
| `extent` | 轮廓面积与 bbox 面积比例 |
| `gt_bbox` | 合成案例中的真值 bbox，负样本为空 |
| `iou` | 预测 bbox 与真值 bbox 的交并比 |

## 检测指标说明

`metrics_summary.csv` 和 `metrics_summary.json` 由合成案例自动计算，当前使用 IoU >= 0.5 判断一次检测是否命中真值目标。

| 指标 | 含义 |
|---|---|
| `precision` | 检出的候选中有多少是真正命中红球的结果 |
| `recall` | 应检红球中有多少被成功检出 |
| `f1_score` | precision 和 recall 的调和平均 |
| `false_positive_rate` | 负样本中被误检的比例 |
| `miss_rate` | 应检红球的漏检比例 |
| `top1_error` | 单图最高置信预测的错误率，等于 1 - accuracy |
| `ap50` | IoU 阈值为 0.5 时的平均精度 |

这些指标只代表当前可控合成案例集，不等价于真实比赛精度。真实仿真评估需要在官方或 Gazebo 环境中提供真值危险源位置和 bbox 后再计算。
