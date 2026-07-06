# 感知组 reports 目录

本目录统一保存感知组可展示结果、测试记录和阶段材料。

## 目录结构

```text
reports/perception/
├─ simulation/       Gazebo 或官方仿真下的红球检测效果
├─ 2d_detection/     二维红球检测效果图、summary 和精选拼图
├─ test_records/     按测试组模板整理的感知专项表格
└─ docs/             感知组阶段汇报和说明材料
```

## 当前成果

```text
simulation/3d_native/official_simenv_20260703_basic/
simulation/3d_native/official_simenv_20260705_multi_red/
simulation/3d_native/official_simenv_20260705_native_3d_validation/
simulation/3d_native/official_simenv_20260705_3d_red_shape_distractors/
simulation/3d_native/official_simenv_20260705_3d_red_ball_occlusion/
simulation/2d_derived/official_simenv_20260705_partial_red/
simulation/2d_derived/official_simenv_20260705_distractor_stress/
simulation/2d_derived/official_simenv_20260705_multi_target_count/
simulation/2d_derived/official_simenv_20260705_final_stress/
2d_detection/synthetic_20260620_230333/
2d_detection/real_images_20260620_230333/
test_records/20260705_official_simenv_3d_red_shape_distractors/
test_records/20260705_official_simenv_3d_red_ball_occlusion/
test_records/20260705_official_simenv_stress_suite/
docs/perception_progress_report_2026-07-05.md
```

## 归档规范

感知组 reports 只允许使用以下一级目录：

- `simulation/`: Gazebo、官方 SimEnv、远程仿真环境截图、视频和检测汇总。
- `2d_detection/`: 离线二维合成图、实拍图检测结果。
- `test_records/`: 按测试组字段整理的 CSV/JSON 表格。
- `docs/`: 阶段进度、分析说明和交付备注。

不要在 `reports/perception/` 下新建 `official_simenv/`、`result/`、`real_result/` 等并列目录。官方仿真结果必须放在：

```text
reports/perception/simulation/2d_derived/official_simenv_<YYYYMMDD>_<purpose>/
reports/perception/simulation/3d_native/official_simenv_<YYYYMMDD>_<purpose>/
```

每个仿真结果目录建议包含：

```text
README.md
summary.json
selected_images/
selected_cases.csv
selected_cases.json
candidate_views.csv        # 多视角/多条件测试时需要
candidate_views.json       # 多视角/多条件测试时需要
candidate_images/          # 候选视角标注图，多条件测试时需要
partial_images/            # 部分可见红球标注图，部分遮挡/FOV 边缘测试时需要
*_demo.mp4                 # 可展示视频，体积小时可提交
raw_attempts.zip           # 原始 PPM/大体积原图归档，默认不提交
```

命名统一使用小写英文、下划线和日期，例如：

```text
official_simenv_multi_red_case_01_red00_west_detected.png
official_simenv_multi_red_demo.mp4
```

原始 `.ppm`、未筛选帧、临时视频帧不要散放在目录中；需要保留时压缩成 `raw_attempts.zip` 或 `raw_frames.zip`，并让 `.gitignore` 保持忽略。

凡是形成测试结论的结果，必须同步放一份测试组记录到：

```text
reports/perception/test_records/<YYYYMMDD>_<source>_<purpose>/
```

测试组记录至少包含：

- `testing_record_perception.csv`
- `testing_record_perception.json`

多条件测试还应包含：

- `selected_cases.csv/json`
- `candidate_views.csv/json`
- `comparison_with_previous_results.csv/json`

## 生成命令

二维合成检测案例：

```bash
python scripts/generate_perception_cases.py
```

实物图片检测案例：

```bash
python scripts/evaluate_real_red_ball_images.py --input-dir C:\Users\jiangchen\OneDrive\Desktop\red_ball
```

Gazebo 仿真截图：

```bash
python scripts/capture_red_ball_gallery.py
```

测试组表格优先放到：

```text
reports/perception/test_records/<timestamp>/
```
