# Reports

比赛材料目录。

建议存放：

- 可用素材
- 技术方案报告
- 自测试报告
- 答辩 PPT 草稿
- 演示视频脚本
- 专利和论文草稿

当前可生成的实验结果：

```bash
python scripts/generate_perception_cases.py
```

默认输出到：

```text
reports/perception/2d_detection/synthetic_<timestamp>/
```

实物红球图片评估：

```bash
python scripts/evaluate_real_red_ball_images.py --input-dir C:\Users\jiangchen\OneDrive\Desktop\red_ball
```

默认输出到：

```text
reports/perception/2d_detection/real_images_<timestamp>/
```

Gazebo 红球 gallery：

```bash
python scripts/capture_red_ball_gallery.py
```

默认输出到：

```text
reports/perception/simulation/<timestamp>/
```

测试组表格统一放到：

```text
reports/perception/test_records/<timestamp>/
```
