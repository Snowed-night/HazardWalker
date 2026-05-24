# hazardwalker_perception

感知包，负责红色球体检测和三维定位。

## 当前职责

- 红色球体检测
- 三维定位
- 多帧确认
- 虚警抑制
- 危险源去重

## 当前最小节点

- `hsv_detector_node`：订阅 `/hw/camera/image_raw`，用 HSV 检测红色区域，并向 `/hw/perception/hazard_detections` 发布 JSON 字符串。

当前输出中的三维坐标是占位值，用于先打通链路。

## 后续扩展

- `red_ball_detector.py`：离线红球检测纯函数。
- `hsv_detector_node.py`：ROS 图像输入和检测输出。
- 后续可新增 `localize_hazard.py`、`track_hazards.py`、`debug_image.py`。
