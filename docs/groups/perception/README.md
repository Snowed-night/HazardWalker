# 感知组文档

## 1. 我们负责什么

- **HSV 红球检测基线**：用颜色阈值从图像中检出红色危险球。
- **YOLO 或其他增强识别方法预研**：作为 HSV 的升级方案。
- **三维定位**：`bbox + CameraInfo + 点云/深度 + TF` → 世界坐标。
- **多帧确认、空间去重和虚警压低**：同一个红球不报多次，假阳要压下来。

## 2. 当前代码文件一览

| 文件 | 作用 | 位置 |
|---|---|---|
| `red_ball_detector.py` | 离线红球检测纯函数（OpenCV 形状筛选） | `ros2_ws/src/hazardwalker_perception/hazardwalker_perception/` |
| `hsv_detector_node.py` | ROS 节点，订阅图像+深度+TF，发布检测结果 | `ros2_ws/src/hazardwalker_perception/hazardwalker_perception/` |
| `localize_hazard.py` | 不依赖 ROS 的三维定位纯函数 | `ros2_ws/src/hazardwalker_perception/hazardwalker_perception/` |
| `track_hazards.py` | 多帧观测合并、确认与去重 | `ros2_ws/src/hazardwalker_perception/hazardwalker_perception/` |
| `test_red_ball_detector.py` | 离线测试，用构造图像验证检测逻辑 | `tests/offline/` |
| `perception.yaml` | 红球检测、定位、跟踪的参数配置 | `config/` |

## 3. 当前红球检测策略

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

## 4. 三维定位与多帧确认

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

## 5. 我应该做什么（当前阶段）

### 5.1 理解数据流

```
/hw/camera/image_raw (sensor_msgs/Image)
/hw/camera/camera_info (sensor_msgs/CameraInfo)  ← 已接入
/hw/camera/depth_image (sensor_msgs/Image)       ← 已接入
TF (camera_link → start)                         ← 已接入
        │
        ▼
  hsv_detector_node.on_image()
        │
        ├─→ detect_red_balls_rgb()   ← red_ball_detector.py（OpenCV 形状筛选）
        ├─→ localize_hazard()        ← localize_hazard.py（三维定位）
        └─→ track_hazards()          ← track_hazards.py（多帧确认）
        │
        ▼
  /hw/perception/hazard_detections (hazardwalker_msgs/HazardArray 或 JSON)
  /hw/perception/detections_2d      (深度/TF 不可用时的降级输出)
```

### 5.2 理解核心检测逻辑（red_ball_detector.py）

文件中的关键函数：

1. **`rgb_to_hsv_pixel(r, g, b)`**：把一个 RGB 像素手动转成 HSV，不依赖 OpenCV。
   - 输入：R, G, B 各 0-255
   - 输出：H(0-180), S(0-255), V(0-255)

2. **`is_red_hsv(h, s, v, ...)`**：判断一个 HSV 像素是否属于红色。
   - 红色比较特殊：hue 在 0° 和 360° 附近都算红，所以用两段区间：
     - 区间 1：H ∈ [0, 10]
     - 区间 2：H ∈ [170, 180]
   - 同时要求：S ≥ 80, V ≥ 80（排除太暗或太淡的像素）
   - **关键理解**：HSV 色相是个圆环，红色正好在 0°/360° 断开处。

3. **`detect_red_balls_rgb(data, width, height, ...)`**：OpenCV 形状筛选版检测。
   - 流程：HSV 阈值 → 形态学去噪 → 轮廓提取 → 面积/圆度/长宽比/extent 筛选
   - 输出：`List[RedBallDetection2D]`，按置信度降序排列
   - 如果红色像素数 < `min_area_px`（默认 80），返回 `None`（过滤噪声）

### 5.3 理解 HSV 检测节点（hsv_detector_node.py）

- **订阅**：
  - `/hw/camera/image_raw`（`sensor_msgs/Image`）— RGB 相机图像
  - `/hw/camera/camera_info`（`sensor_msgs/CameraInfo`）— 相机内参
  - `/hw/camera/depth_image`（`sensor_msgs/Image`）— 深度图
  - TF — `camera_link` 到 `start` 等坐标系的变换
- **发布**：
  - `/hw/perception/hazard_detections` — 三维检测结果（深度和 TF 可用时）
  - `/hw/perception/detections_2d` — 仅 2D 检测框（深度或 TF 不可用时，降级输出）
  - `/hw/perception/debug_image` — 调试用标注图像
- **参数**：从 ROS 参数服务器读取 `min_area_px`、检测形状阈值、定位和跟踪参数
- **注意**：当前仅在相机内参、有效深度和 TF 同时可用时才输出三维坐标，否则只输出 2D 调试结果（不再使用硬编码占位坐标）。

### 5.4 感知参数（config/perception.yaml）

```yaml
perception:
  red_ball_detector:
    lower_red_1: [0, 80, 80]    # 红色区间1下限 H,S,V
    upper_red_1: [10, 255, 255] # 红色区间1上限
    lower_red_2: [170, 80, 80]  # 红色区间2下限
    upper_red_2: [180, 255, 255] # 红色区间2上限
    min_area_px: 80              # 最小红色像素数，低于此数不算目标
    min_confidence: 0.5          # 最小置信度

  localization:
    use_point_cloud: true
    max_detection_range_m: 20.0  # 最远检测距离
    roi_padding_px: 8
    min_points_in_roi: 5         # 最少点云点数
    output_frame: start

  tracking:
    confirm_observation_count: 3 # 连续确认帧数
    reject_after_missed_count: 10
    merge_distance_m: 0.5        # 合并距离阈值
```

## 6. 怎么做（操作步骤）

### 6.1 跑通离线测试

```bash
cd HazardWalker
python scripts/run_offline_tests.py
```

预期输出：所有测试全部 PASS。

### 6.2 只跑感知组测试

```bash
cd HazardWalker
python -m pytest tests/offline/test_red_ball_detector.py -v
```

### 6.3 阅读代码的顺序

1. 先看 `docs/groups/perception/README.md`（本文件）→ 了解全景
2. 再看 `red_ball_detector.py` → 理解核心检测算法（OpenCV 形状筛选）
3. 接着看 `localize_hazard.py` → 理解三维定位纯函数
4. 然后看 `track_hazards.py` → 理解多帧确认与去重
5. 再看 `test_red_ball_detector.py` → 看测试怎么构造图像、怎么验证
6. 接着看 `hsv_detector_node.py` → 看 ROS 节点怎么串起全链路
7. 最后看 `config/perception.yaml` → 看有哪些可调参数

### 6.4 验证自己理解了

能回答以下问题就算过关：

- Q1：为什么红色需要两段 hue 区间，而不是一段？
- Q2：`min_area_px` 起什么作用？改成 0 会怎么样？
- Q3：圆形度 circularity 和长宽比 aspect_ratio 各筛掉什么？
- Q4：当前节点什么时候输出三维坐标，什么时候只输出 2D 结果？
- Q5：多帧确认中 confirmed 和 rejected 的触发条件分别是什么？

## 7. 后续要做什么（下一阶段）

### 7.1 检测增强

- BGR 输入测试
- 暗红色（V < 80）过滤测试
- 低饱和度（S < 80）过滤测试
- 小面积噪声过滤测试
- 红色方块 / 不规则物体误检过滤测试
- 不同遮挡率（10%~70%）的检测精度测试

### 7.2 三维定位验证

- 在主力机 Gazebo 环境验证 RGB、深度图和 TF 对齐
- 验证 `localize_hazard.py` 的坐标反投影精度
- 点云 ROI 定位作为深度图不可用时的备用方案

### 7.3 多帧确认与去重

- 验证 `track_hazards.py` 的确认/拒绝逻辑
- 验证 merge_distance_m 空间合并效果
- 降低虚警率

## 8. 话题接口说明

| 话题 | 消息类型 | 方向 | 说明 |
|---|---|---|---|
| `/hw/camera/image_raw` | `sensor_msgs/Image` | 订阅（平台组→感知组） | 相机 RGB 图像输入 |
| `/hw/camera/camera_info` | `sensor_msgs/CameraInfo` | 订阅（平台组→感知组） | 相机内参（3D 定位用） |
| `/hw/camera/depth_image` | `sensor_msgs/Image` | 订阅（平台组→感知组） | 深度图（3D 定位用） |
| `/hw/lidar/points` | `sensor_msgs/PointCloud2` | 订阅（平台组→感知组） | 点云（备用 3D 定位） |
| `/hw/perception/hazard_detections` | `HazardArray` 或 JSON | 发布（感知组→决策组） | 三维检测到的危险源 |
| `/hw/perception/detections_2d` | JSON | 发布（感知组→调试） | 2D 检测框（深度/TF 不可用时） |
| `/hw/perception/debug_image` | `sensor_msgs/Image` | 发布（感知组→调试） | 标注后的调试图像 |

## 9. 关键概念速查

| 概念 | 说明 |
|---|---|
| **HSV** | Hue(色相) / Saturation(饱和度) / Value(明度)，比 RGB 更适合颜色阈值 |
| **红色 hue 两段区间** | 红色在 hue 圆环上跨越 0°/360° 边界，hue∈[0,10]∪[170,180] |
| **bbox / 包围框** | 图像上目标的最小外接矩形 (x_min, y_min, x_max, y_max) |
| **circularity** | 圆度 = 4π × 面积 / 周长²，接近 1 为圆，区分红球和红色方块 |
| **aspect_ratio** | bbox 宽高比，筛掉长条形误检 |
| **extent** | 轮廓面积 / bbox 面积，筛掉空心或不规则误检 |
| **min_area_px** | 红色像素数下限，过滤图像噪声 |
| **CameraInfo** | 包含焦距、主点等内参，用于从像素坐标反算三维方向 |
| **TF** | ROS 坐标变换树，把相机坐标系转换到世界/start 坐标系 |
| **confirmed** | 同一目标连续 confirm_observation_count 帧检测到，标记为确认 |
| **rejected** | 已确认目标连续 reject_after_missed_count 帧未检测到，标记为拒绝 |
