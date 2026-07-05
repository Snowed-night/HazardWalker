# 实物红球图片检测结果

本目录用于保存本地实物红球图片的检测结果。默认输入目录是：

```text
C:\Users\jiangchen\OneDrive\Desktop\red_ball
```

运行命令：

```bash
python scripts/evaluate_real_red_ball_images.py
```

输出命名规则：

```text
real_001
real_002
...
real_010
```

原始图片文件名不修改，统一编号只用于结果图和表格。`summary.csv` 中保留 `source_file` 字段，用于追溯原始图片来源。

输出结构：

```text
reports/perception/2d_detection/real_images_<timestamp>/
├─ annotated/                  多目标标注图，不提交 Git
├─ figures/
│  └─ real_red_ball_collage.png 可提交的汇报拼图
├─ summary.csv                  每张图片汇总表，不提交 Git
├─ detections.csv               每个检测框明细表，不提交 Git
└─ summary.json                 结构化汇总，不提交 Git
```

推荐制作的汇报图表：

| 图表 | 用途 |
|---|---|
| `real_red_ball_collage.png` | 展示 10 张实物图检测效果 |
| `detections.csv` | 做 confidence、circularity、extent 的参数统计表 |
