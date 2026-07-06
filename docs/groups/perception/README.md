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

## 当前红球检测策略

感知组当前第一版目标是优先降低误检，而不是尽可能检出所有红色区域。`red_ball_detector.py` 已从纯像素扫描升级为 OpenCV 形状筛选：

```text
RGB/BGR 图像
  -> HSV 红色阈值
  -> mask 形态学去噪
  -> findContours 连通域/轮廓
  -> 距离变换 + watershed 分离轻度粘连目标
  -> 面积过滤
  -> 圆度 circularity
  -> bbox 长宽比 aspect_ratio
  -> 轮廓面积 / bbox 面积 extent
  -> 输出一个或多个红球候选
```

当前验收用例：

- 正常红色圆形目标应被检测。
- 暗红低亮度目标应被过滤。
- 低饱和度偏灰红目标应被过滤。
- 红色方块和两类红色不规则物体应被当作误检过滤。
- 10%、20%、30%、50% 遮挡红球应尽量检测。
- 70% 遮挡红球先按保守策略拒绝，后续可结合多帧确认或重观察再处理。
- 多个分离红球和轻度粘连红球应输出多个候选。
- 阴影渐变、高亮背景和复杂背景作为仿真环境影响的边界案例。

## 当前技术选择

当前先选 OpenCV HSV + 轮廓 + watershed 的原因：

```text
1. 红色球体颜色特征强，HSV 基线可解释、调参快、无需训练数据。
2. 轮廓圆度、长宽比和 extent 能压低红色方块、细长物体等虚警。
3. watershed 是成熟的实例分离方法，能处理部分粘连多红球。
4. 该方案可以在主力机无 GPU 或官方平台刚发布时立即运行。
```

后续预留方案：

```text
YOLO 检测：适合复杂光照、遮挡和非理想红球纹理，但需要数据和权重管理。
实例分割：适合多红球粘连和局部可见目标，输出 mask 后仍可复用三维定位。
RGB-D 一致性过滤：用深度连续性、球面几何或点云聚类进一步压低虚警。
主动重观察：低置信度或遮挡目标交给决策/导航换视角确认。
```

代码接入点是 `red_ball_detector.py` 的 `DetectionBackend`。未来新增模型后端时，应继续输出 `RedBallDetection2D`，这样 `hsv_detector_node.py` 的三维定位、tracking 和 JSON 输出不需要重写。

## 可展示结果和测试组表格

本地先运行：

```bash
python scripts/generate_perception_cases.py
python scripts/run_offline_tests.py
```

合成案例输出：

```text
reports/perception/2d_detection/synthetic_<timestamp>/
```

测试组感知专项指标输出：

```text
reports/perception/test_records/<timestamp>/testing_record_perception.csv
reports/perception/test_records/<timestamp>/testing_record_perception.json
```

## 三维定位与多帧确认

当前已经新增两个不依赖 ROS 的纯函数模块，先把算法边界固定下来：

- `localize_hazard.py`：把 2D bbox、相机内参和深度信息转换为三维坐标。
- `track_hazards.py`：把多帧三维观测合并为稳定危险源轨迹，完成确认和去重。

三维定位的第一版输入输出：

```text
输入：bbox、CameraInfo K 内参、bbox ROI 深度图或固定深度、相机到输出坐标系的刚体变换
输出：position(x, y, z)、frame_id、depth_m、像素中心和使用的深度点数量
```

多帧确认的第一版规则：

```text
merge_distance_m 内的观测合并为同一危险源
observation_count 达到 confirm_observation_count 后标记 confirmed
missed_count 达到 reject_after_missed_count 后标记 rejected
```

当前 `hsv_detector_node.py` 已接入 `/hw/camera/camera_info`、`/hw/camera/depth_image` 和 TF。
当相机内参、有效深度和 `camera_link -> start` 等输出坐标系 TF 同时可用时，节点会输出三维 `hazards` 并交给多帧跟踪器确认和去重。
如果深度或 TF 不可用，节点只发布 `detections_2d` 调试字段，不再发布占位三维坐标。

下一步需要在主力机 Gazebo/官方平台适配层验证 RGB、深度图和 TF 是否对齐；点云 ROI 定位可作为深度图不可用时的后续备用方案。
