# hazardwalker_perception

危险源感知定位包，负责红色球体检测和三维定位。

职责：

- RGB red ball detection
- LiDAR-assisted 3D localization
- multi-frame confirmation
- false alarm suppression
- hazard source de-duplication

## 当前最小节点

- `hsv_detector_node`：订阅 `/hw/camera/image_raw`，用 HSV 检测红色区域，并向 `/hw/perception/hazard_detections` 发布 JSON 字符串。

当前输出中的三维坐标是占位值，用于先打通链路。后续需要替换为相机检测 + 点云/深度 + TF 的真实三维定位。

## 后续替换方向

1. 将 HSV 检测函数拆成可离线测试的纯函数。
2. 接入 `/hw/camera/camera_info`。
3. 接入 `/hw/lidar/points`。
4. 使用 TF 将点云投影或转换到相机坐标系。
5. 估计红球中心三维坐标。
6. 增加多帧确认、空间聚类去重和虚警抑制。
