# 感知组文档

负责范围：

- HSV 红球检测基线。
- YOLO 或其他增强识别方法预研。
- `bbox + CameraInfo + 点云/深度 + TF` 三维定位。
- 多帧确认、空间去重和虚警压低。

近期重点：

1. 阅读 `red_ball_detector.py` 和 `hsv_detector_node.py`。
2. 明确当前 2D bbox 输出与最终三维坐标输出的差距。
3. 设计相对起点 xyz 的坐标转换链路。
4. 给出多帧确认和重复报警去重方案。
