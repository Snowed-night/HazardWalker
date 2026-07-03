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
simulation/20260614_171740/
2d_detection/synthetic_20260620_230333/
2d_detection/real_images_20260620_230333/
test_records/20260620_230333/
docs/perception_progress_report_2026-06-12.md
```

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
