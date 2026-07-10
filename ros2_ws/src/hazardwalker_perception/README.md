# hazardwalker_perception

感知包，负责红色球体检测和三维定位。

## 当前职责

- 红色球体检测
- 三维定位
- 多帧确认
- 虚警抑制
- 危险源去重
- 动态实验记录与主动重观察建议

## 当前最小节点

- `hsv_detector_node`：订阅 `/hw/camera/image_raw`、`/hw/camera/camera_info` 和 `/hw/camera/depth_image`，用 HSV 检测红色区域，结合深度和 TF 输出危险源三维定位 JSON。

对官方半径 0.15 m 的标准红球，`sphere_radius_m` 默认启用球面前沿到球心的深度补偿；若接入未知尺寸或非球形目标，应将该参数设为 `0.0`。
- `dynamic_detection_recorder_node`：订阅 `/hw/camera/image_raw`、`/hw/odom` 和 `/hw/perception/hazard_detections`，写入逐帧记录、精选截图、`summary.json` 与测试组 CSV/JSON；发布建议动作到 `/hw/perception/view_recommendation`，但不直接控制机器人。

当前检测链路已经引入 OpenCV：

```text
图像 -> HSV 红色阈值 -> mask 形态学去噪 -> 轮廓/连通域
  -> 距离变换 + watershed 分离粘连红球
  -> 面积、圆度、长宽比、面积比例筛选
  -> 多目标 bbox
```

第一版策略优先压低误检：红色立方体投影和红色不规则物体会被过滤；10%-50% 遮挡的单个红球作为应检目标；70% 遮挡先按边界案例保守拒绝。

## 当前方案选择依据和扩展点

本阶段优先交付可展示结果，因此选择业界成熟、实现成本低、可解释的 OpenCV 方案作为基线：

```text
HSV 阈值：适合红色球体这种颜色目标，调参和排错直接。
形态学 + 轮廓：用于过滤噪声和输出 bbox。
圆度/长宽比/extent：用于过滤红色方块、细长物体和不规则红色干扰。
距离变换 + watershed：用于分离轻度粘连的多个红球。
深度 ROI 中位数 + CameraInfo + TF：用于从 2D bbox 得到相对起点三维坐标。
```

后续更强方案不直接塞进 ROS 节点，而是通过 `DetectionBackend` 接口接入：

```text
create_detection_backend("hsv_opencv")  当前默认后端
未来可新增：
  yolo_backend.py       目标检测模型，输出 RedBallDetection2D 列表
  segment_backend.py    实例分割模型，mask 转 bbox 后复用定位和 tracking
  rgbd_backend.py       颜色检测 + 深度/点云一致性过滤
```

这样三维定位、`HazardTracker` 多帧确认、`/hw/perception/hazard_detections` 输出格式不需要随检测算法重写。

当前节点只有在相机内参、有效深度和 TF 链都可用时才输出三维 `hazards`。
如果缺少深度或 TF，节点仍会发布 `detections_2d` 调试字段，但不会把占位坐标伪装成真实定位。

为处理官方 SimEnv 中的遮挡、圆柱干扰和单视角风险，检测输出分为两层：

```text
严格红球候选：通过原有 HSV + 形状筛选，才允许进入三维 tracking。
宽松复查候选：仅在局部可见/弱光下触发主动换视角，字段 requires_reobservation=true，不能单帧确认。
疑似合并候选：距离峰提示一个红色连通域内仍有多个中心时，字段 may_be_merged=true，同样只能横移复查，不能直接计作一个危险源。
```

定位可用时，`hsv_detector_node` 默认要求同一轨迹累积至少 3 次观测、来自至少 2 个离散相机视角才会标记为 `confirmed`。这会避免红色圆柱或圆盘在固定机位的连续帧中被直接确认为红球；视角标签由相机在输出坐标系的位置和朝向量化得到。

## 当前核心模块

- `red_ball_detector.py`：离线红球检测纯函数。
- `hsv_detector_node.py`：ROS 图像输入和检测输出。
- `localize_hazard.py`：根据 bbox、相机内参、深度图或固定深度反投影三维坐标，并支持刚体变换到 `start` 等输出坐标系。
- `track_hazards.py`：按三维距离合并多帧观测，达到观测次数阈值后确认危险源，长期丢失后拒绝。
- `detection_metrics.py`：根据真值 bbox、预测 bbox 和置信度计算 IoU、precision、recall、F1、top1 error 和 AP50。
- `active_view_policy.py`：根据候选框贴边、重叠、红色像素数、圆度、置信度和可选深度，输出靠近、横移、转向、调俯仰或保持观察建议。
- `dynamic_detection_records.py`：将连续帧记录汇总为不伪造真值指标的动态实验摘要和测试表记录。

## 可展示离线结果

本地没有 Gazebo 时先运行：

```bash
python scripts/generate_perception_cases.py
python scripts/run_offline_tests.py
```

`generate_perception_cases.py` 会覆盖正常红球、遮挡、多红球、粘连多红球、复杂背景、阴影/高亮环境和红色非球体干扰。
输出目录：

```text
reports/perception/2d_detection/synthetic_<timestamp>/
```

测试组表格可直接参考：

```text
reports/perception/test_records/<timestamp>/testing_record_perception.csv
reports/perception/test_records/<timestamp>/testing_record_perception.json
```

## 三维定位和去重边界

`localize_hazard.py` 和 `track_hazards.py` 当前都是纯 Python 算法模块，不直接依赖 ROS 消息类型。
ROS 节点当前已经接入 `CameraInfo`、对齐深度图和 TF。
点云 ROI 定位尚未接入，后续可在深度图不可用时作为备用定位来源。

当前三维定位流程：

```text
bbox 中心像素
  -> CameraInfo 内参反投影
  -> bbox ROI 深度中位数
  -> 相机坐标系三维点
  -> 刚体变换到 start/map 坐标系
```

当前跟踪去重流程：

```text
单帧三维观测
  -> 按 merge_distance_m 寻找近邻轨迹
  -> 加权平均更新位置
  -> observation_count 达到阈值后 confirmed
  -> missed_count 达到阈值后 rejected
```

下一步需要在主力机 Gazebo/官方平台适配层验证 `/hw/camera/depth_image` 是否与 RGB 图像对齐，以及 `camera_link -> start` 的 TF 链是否稳定。

## 动态检测记录与主动观察

动态记录节点只使用统一的 `/hw/*` 接口，因此官方 SimEnv 需要先由平台层完成以下适配：

```text
/real_sense/rgb/image_raw       -> /hw/camera/image_raw
/real_sense/rgb/camera_info     -> /hw/camera/camera_info
/real_sense/depth/image_raw     -> /hw/camera/depth_image
/Odometry_gazebo                -> /hw/odom
/tf、/tf_static                 -> /tf、/tf_static
```

平台适配完成后，从仓库根目录运行（路径按本次实验日期替换）：

```bash
ros2 run hazardwalker_perception dynamic_detection_recorder_node --ros-args \
  -p output_dir:=reports/perception/simulation/3d_native/official_simenv_20260710_dynamic_detection \
  -p test_record_dir:=reports/perception/test_records/20260710_official_simenv_dynamic_detection \
  -p scenario_name:=official_simenv_dynamic_detection
```

节点会产出：

```text
<output_dir>/frames.jsonl                 每次感知输出对应的候选、位姿、确认状态、动作建议
<output_dir>/selected_images/*.png        按最小时间间隔保存的证据帧
<output_dir>/summary.json                 动态帧数、候选数、确认数、平均置信度、建议动作统计
<test_record_dir>/testing_record_perception.csv
<test_record_dir>/testing_record_perception.json
```

未提供危险源真值时，测试记录会把识别率、漏检、虚警和定位误差相关字段保留为空；只有将 SimEnv 的 `danger_truth.json` 与运行时间/目标 ID 对齐后才能填写这些结论。

## 依赖

离线红球形状检测需要 OpenCV：

```bash
python -c "import cv2; print(cv2.__version__)"
```

主力机环境应安装 `python3-opencv`。Windows 本机可使用：

```powershell
python -m pip install opencv-python
```
