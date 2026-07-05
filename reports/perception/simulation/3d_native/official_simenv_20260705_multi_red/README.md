# official_simenv_20260705_multi_red

官方 SimEnv 多红球检测展示结果。

## 内容

- `selected_images/`: 每个官方红球自动选择最佳可见视角后的标注图。
- `official_simenv_multi_red_demo.mp4`: 4 个官方红球检测结果串联视频。
- `selected_cases.csv/json`: 展示用 4 个最佳视角案例。
- `candidate_views.csv/json`: 20 个候选视角明细，用于分析遮挡、视角和环境影响。
- `candidate_images/`: 20 个候选视角标注图，其中 `pass/` 为 16 张成功图，`blocked_or_missed/` 为 4 张遮挡或未检出图。
- `candidate_images/candidate_views_collage.png`: 20 个候选视角总览拼图。
- `summary.json`: 汇总指标、真值、候选视角和检测框。
- `raw_attempts.zip`: 原始 PPM 归档，不展开存放。

## 结果

- 官方红球真值数量：4
- 最佳视角检测成功：4/4
- 候选视角检测成功：16/20
- 本地 Python 平均检测耗时：3.75 ms/帧

说明：官方红球在房间内分散布置，单一固定视角会受墙体、家具和视线方向影响。本次对每个红球测试南/北/西/东/中距五类候选视角，并保留最佳视角作为展示结果。
