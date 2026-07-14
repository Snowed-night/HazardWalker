# 感知组文档

## 1. 我们负责什么

- **HSV 红球检测基线**：用颜色阈值从图像中检出红色危险球。
- **YOLO 或其他增强识别方法预研**：作为 HSV 的升级方案。
- **三维定位**：`bbox + CameraInfo + 点云/深度 + TF` → 世界坐标。
- **多帧确认、空间去重和虚警压低**：同一个红球不报多次，假阳要压下来。

### 1.1 2026-07-12 当前实现口径

- 当前只保留五类复杂环境证据目录：多球粘连 10 例、部分可见 21 例、红色干扰物压力集 24 种、真实运动多视角 20 例、三维定位 8 例 32 点，共 83 例。比赛最终输出类别只有红色球体；圆柱、圆锥、方块等仅用于压低虚警，绝不作为危险源类别。2026-07-12 审计已使旧多视角 `20/20` 失效：视角标签曾将竖直起伏计入独立视角，稳定平台重跑前不能宣称五类均通过。完整结论、黄色框语义和未完成边界见 reports/perception/docs/perception_progress_report_2026-07-10_evidence_matrix.md。
- 单帧红色圆形只作为候选；正式红球确认要求三个稳定停靠视角，且相机到同一目标的水平视线跨度至少 25°。只在正面前后靠近不能确认，必须取得侧面复查。
- 对深度为 `flat` 的近圆候选，轨迹状态会直接标为 `needs_reobservation`（而不是普通
  `tentative`）；展示字段同样会置 `requires_reobservation=true`。这使圆柱正面在实际侧向复查前
  不会被误读成已识别红球。
- 轨迹联合检查三维位置、表观直径变异、最差侧视长宽比、深度曲率变异、相对目标的侧向视差，以及深度曲率/表观直径中位数区间 `[0.10, 0.30]`。
- 相机运动期间冻结轨迹证据；一个稳定停靠周期只锁定一个 `view_id`，避免慢速运动或量化抖动伪造多视角。
- Hough/分水岭从同一连通域拆出的圆只标为 `requires_reobservation`，必须在后续视角形成独立轮廓后才可累计确认，防止红色哑铃端部误报。
- 严格候选使用 `>=200 px`、圆度 `>=0.65`；更小或更不规则的红色区域仍可作为黄色复查候选，
  不能直接增加红球计数。该边界已在 7 月 5 日 21 例遮挡与 10 球粘连原生帧上复归。
- 局部球 5%–85% 左右遮挡采用候选/盲扫后左右括号复查；黄色框表示待复查，不等于识别成功。
- 可复现实验入口为 `scripts/run_official_simenv_complex_perception_matrix.py`，严格验收入口为 `scripts/validate_official_simenv_complex_evidence.py`。
- 官方 ROS1 节点已在真实容器中实测发布 RGB-D 候选、深度反投影和
  `/hazardwalker/perception/reobservation_request`；该请求只要求导航层执行侧向复查，绝不直接发布
  `/cmd_vel`。对有三维位置的左右横移候选，请求包含分段圆弧 waypoint 和朝向，可被导航层逐段
  避障执行并用 TF 证明最终视差。最新离线回归为 122 passed，正式复杂环境横移证据仍未完成。

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

### 3.1 Gazebo 仿真检测结果

运行命令：

```bash
python3 scripts/capture_red_ball_gallery.py
```

输出目录：`reports/perception/simulation/<timestamp>/`

最新结果：`reports/perception/simulation/20260614_171740/`

| 场景 | 是否检测到 | 检测框数量 | 是否完整红球 | 是否局部红球 | 红色像素数量 | 备注 |
|---|---|---|---:|---|---|---|---|
| center_full | ✓ | 1 | ✓ | ✗ | 5434 | 中心完整红球，置信度 0.955 |
| edge_partial | ✗ | 0 | ✗ | ✓ | 4497 | 右侧边缘裁剪，形状筛选拒绝（圆度过低） |
| top_partial | ✓ | 1 | ✗ | ✓ | 4153 | 顶部裁剪，置信度降至 0.757 |
| multi_visible | ⚠ 1/2 | 1 | ✓ | ✗ | 3339 | 2 个红球只检出 1 个（components 显示 2 个连通域） |

### 3.2 合成图像检测结果（全部 PASS）

运行命令：

```bash
python scripts/generate_perception_cases.py    # 生成测试图像
python -m pytest tests/offline/test_red_ball_detector.py -v   # 运行离线测试
```

输出目录：`reports/perception/2d_detection/synthetic_<timestamp>/`

最新结果：`reports/perception/2d_detection/synthetic_20260621_150028/`（共 17 例全部 PASS）

| 场景 | 是否检测到 | 检测框数量 | 是否完整红球 | 是否局部红球 | 红色像素数量 | 备注 |
|---|---|---|---:|---|---|---|---|
| normal_sphere | ✓ | 1 | ✓ | ✗ | 5521 | 正常红球，置信度 0.967，IoU 0.954 |
| bright_sphere | ✓ | 1 | ✓ | ✗ | 5521 | 亮红球，置信度 0.967，IoU 0.954 |
| dark_sphere | ✗ | 0 | — | — | 0 | 低亮度暗红过滤 ✓ |
| low_sat_sphere | ✗ | 0 | — | — | 0 | 低饱和度红过滤 ✓ |
| occ_10 | ✓ | 1 | ✗ | ✓ | 5282 | 10% 遮挡，IoU 0.964 |
| occ_20 | ✓ | 1 | ✗ | ✓ | 4757 | 20% 遮挡，IoU 0.962 |
| occ_30 | ✓ | 1 | ✗ | ✓ | 4183 | 30% 遮挡，IoU 0.960 |
| occ_50 | ✓ | 1 | ✗ | ✓ | 2802 | 50% 遮挡，IoU 0.954 |
| occ_70 | ✗ | 0 | — | — | 0 | 70% 遮挡保守拒绝 ✓ |
| red_cube | ✗ | 0 | — | — | 0 | 红色方块误检过滤 ✓ |
| red_triangle | ✗ | 0 | — | — | 0 | 红色三角形误检过滤 ✓ |
| red_elongated | ✗ | 0 | — | — | 0 | 红色细长物体误检过滤 ✓ |
| red_fragments | ✗ | 0 | — | — | 0 | 红色碎片干扰过滤 ✓ |
| complex_bg_sphere | ✓ | 1 | ✓ | ✗ | 5521 | 复杂背景下正常检出 |
| multi_three_separate | ✓ | 3 | ✓ | ✗ | 3771 | 三个分离红球全部检出 |
| multi_partial_mix | ✓ | 2 | ✗ | ✓ | 3911 | 多红球+局部遮挡混合 |
| multi_touching_pair | ✓ | 2 | ✗ | ✓ | 6330 | 粘连红球 watershed 分裂成功 |

### 3.3 真实图片检测结果

运行命令：

```bash
python scripts/evaluate_real_red_ball_images.py --input-dir <图片目录>
```

输出目录：`reports/perception/2d_detection/real_images_<timestamp>/`

最新结果：`reports/perception/2d_detection/real_images_20260620_230333/`（共 10 张）

| 场景 | 是否检测到 | 检测框数量 | 是否完整红球 | 是否局部红球 | 红色像素数量 | 备注 |
|---|---|---|---:|---|---|---|---|
| real_001 (alicja-radish) | ✗ | 0 | — | — | — | 图片为红色果蔬，非球体 |
| real_002 (elisariva-ball) | ✗ | 0 | — | — | — | 反光/纹理干扰 |
| real_003 (soccer-3568168) | ✗ | 0 | — | — | — | 足球纹理+环境干扰 |
| real_004 (frahyle-card) | ✓ | 1 | ✓ | ✗ | — | 红色球体正确检出，置信度 0.961 |
| real_005 (christmas-tree) | ✓ | 4 | ✓ | ✗ | — | 圣诞树红球，检出 4 个 |
| real_006 (red-ball-8398445) | ✗ | 0 | — | — | — | 光照/纹理干扰 |
| real_007 (red-bauble) | ✗ | 0 | — | — | — | 装饰球材质反光 |
| real_008 (pexels-ball) | ✗ | 0 | — | — | — | 足球非纯红 |
| real_009 (pixloger-sphere) | ✓ | 1 | ✗ | ✓ | — | 大红球但边缘超出画布 |
| real_010 (currants) | ✓ | 1 | ✗ | ✗ | — | 红色果实非球体，出现误检（圆度仅 0.602） |

> **真实图片总结**：10 张中 4 张有检出（40%），3 张正确检出红球，1 张误检（real_010 红醋栗）。HSV 检测器对纯红色球体识别良好，但对复杂纹理、光照变化和非球体红色物体敏感。

### 3.4 离线测试全部通过

```bash
python -m pytest tests/offline/ -v   # 64 passed
```

测试覆盖：
- 红球检测：颜色阈值、面积过滤、形状筛选、BGR 输入、遮挡边界、多候选、粘连分裂（10 项）
- 三维定位：相机内参、像素反投影、刚体变换、深度估计、ROI 采样（9 项）
- 多帧跟踪：距离计算、观测合并、确认/拒绝、新目标创建（6 项）
- 检测指标：IoU、Precision/Recall、mAP 等（5 项）
- 平台 Phase1：模型 SDF、URDF、World、Bridge 配置文件（22 项）
- 导航控制：waypoint 控制逻辑（4 项）
- 结果评估：mission result 校验（2 项）
- 其余测试（6 项）

测试记录表格（测试组格式）：

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
    reject_after_missed_count: 300
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

## 7. 当前状态与下一阶段任务

### 7.1 已完成 ✓

- [x] BGR 输入测试 — `test_detect_red_ball_bgr8_returns_bbox` ✅
- [x] 暗红色（V < 80）过滤测试 — `test_dark_red_low_value_is_rejected_for_all_shapes` ✅
- [x] 低饱和度（S < 80）过滤测试 — `test_low_saturation_red_is_rejected_for_all_shapes` ✅
- [x] 小面积噪声过滤测试 — `test_detect_red_ball_rgb_bytes_ignores_small_noise` ✅
- [x] 红色方块 / 不规则物体误检过滤测试 — `test_red_non_sphere_shapes_are_rejected` ✅
- [x] 不同遮挡率（10%~70%）检测精度测试 — `test_partially_occluded_red_balls_keep_expected_detection_boundary` ✅
- [x] 多红球候选输出 — `test_detect_red_balls_rgb_bytes_returns_multiple_candidates` ✅
- [x] 粘连红球分裂 — `test_touching_red_balls_can_be_split_into_multiple_candidates` ✅
- [x] Gazebo 仿真截图 — `capture_red_ball_gallery.py` 已跑通 ✅
- [x] 合成图像 17 例全部 PASS ✅
- [x] 真实图片 10 张评估完成 ✅
- [x] `localize_hazard.py` 离线测试 9/9 PASS ✅
- [x] `track_hazards.py` 离线测试 6/6 PASS ✅
- [x] 全部离线测试 64/64 PASS ✅

### 7.2 当前不能稳定检测什么

1. **边缘/局部裁剪红球**：Gazebo `edge_partial` 场景中，红球被右边缘裁剪后形状筛选拒绝（圆度过低）。`top_partial` 虽然检出但置信度降到 0.757。
2. **多红球场景漏检**：Gazebo `multi_visible` 有 2 个红球，components 找到 2 个红色连通域，但检测器只输出 1 个候选（第二个可能被形状阈值拒绝）。
3. **70% 以上遮挡**：按设计保守拒绝，需多帧重观察来补救。
4. **真实图片中非纯红球体**：有纹理的红球（如足球）、反光材质球（如圣诞装饰球）、暗光环境红球无法稳定检出。10 张真实图片中仅 3 张正确检出。
5. **红色非球体误检**：real_010（红醋栗果实）产生误检，圆度仅 0.602 但仍通过阈值。

### 7.3 下一阶段任务

1. **检测增强**：调整边缘/局部裁剪场景的形状阈值，降低 partial 漏检率
2. **多红球检测**：修复 `multi_visible` 场景中第二个红球的漏检，排查形状筛选阈值是否过严
3. **三维定位端到端验证**：在主力机 Gazebo 环境验证 RGB、深度图和 TF 对齐；验证 `localize_hazard.py` 坐标反投影精度
4. **多帧跟踪端到端验证**：验证 `track_hazards.py` 确认/拒绝逻辑在实际传感器数据上的表现
5. **降低真实场景误检**：结合深度信息或提高形状筛选严格度来压低假阳

### 7.4 后续三维定位需要平台组提供什么

1. **确认 RGB 图像、深度图和 TF 在主力机 Gazebo 中对齐可用** — 当前节点已订阅三个话题，但缺少端到端验证
2. **相机内参标定结果** — `/hw/camera/camera_info` 话题需要正确的 K 矩阵
3. **`camera_link → start` 等坐标系的 TF 变换** — 当前代码已接入 TF，需要确认发布正确
4. **深度图有效性确认** — 需要确认 `/hw/camera/depth_image` 的像素值与真实距离一致

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

## 10. 会议验收问答（2026-07-04 前）

### 1. 本组当前能跑什么？

- **离线红球检测**：HSV + OpenCV 形状筛选，支持 RGB/BGR 输入，可检测多个红球、分离粘连红球、过滤红色非球体误检
- **离线三维定位**：bbox 中心像素 + 相机内参 + 深度 → 世界坐标（纯函数，不依赖 ROS）
- **离线多帧跟踪**：三维观测合并、确认、去重、拒绝（纯函数，不依赖 ROS）
- **Gazebo 仿真截图**：启动 Gazebo 红球画廊世界，从 4 个相机 topic 截图并运行检测器
- **合成图像批量测试**：17 例（颜色/遮挡/误检/多球/光照），全部 PASS
- **真实图片批量评估**：10 张真实红球图片，生成 CSV/JSON 检测结果

### 2. 运行命令是什么？

```bash
# 离线测试（全部 64 项）
python -m pytest tests/offline/ -v

# 只测感知组
python -m pytest tests/offline/test_red_ball_detector.py tests/offline/test_localize_hazard.py tests/offline/test_track_hazards.py -v

# Gazebo 仿真截图（需要 Gazebo + Linux）
python3 scripts/capture_red_ball_gallery.py

# 合成图像检测评估
python scripts/generate_perception_cases.py
python scripts/evaluate_perception_results.py --results-dir reports/perception/2d_detection/synthetic_<timestamp>/

# 真实图片检测评估
python scripts/evaluate_real_red_ball_images.py --input-dir <图片目录>
```

### 3. 结果保存在哪里？

| 类型 | 路径 | 内容 |
|---|---|---|
| Gazebo 仿真截图 | `reports/perception/simulation/<timestamp>/` | PNG 截图 + summary.json |
| 合成图像检测 | `reports/perception/2d_detection/synthetic_<timestamp>/` | summary.json/csv + 拼图 + 指标 |
| 真实图片检测 | `reports/perception/2d_detection/real_images_<timestamp>/` | summary.json/csv + detections.csv |
| 测试组记录表格 | `reports/perception/test_records/<timestamp>/` | testing_record_perception.csv/json |
| 阶段报告 | `reports/perception/docs/` | perception_progress_report_*.md |

### 4. 截图、JSON、日志或表格是什么？

- **截图**：
  - Gazebo 4 个场景的标注框截图：`center_full_boxed.png`, `edge_partial_boxed.png`, `top_partial_boxed.png`, `multi_visible_boxed.png`
  - 合成图像拼图：`perception_cases_collage.png`
  - 真实图片拼图：`real_red_ball_collage.png`
- **JSON**：每个测试批次都有 `summary.json`（检测结果 + bbox + 置信度 + 形状指标）
- **CSV**：`testing_record_perception.csv`（测试组标准格式）、`detections.csv`、`metrics_summary.csv`
- **日志**：Gazebo 仿真运行日志在 console 输出

### 5. 当前最大问题是什么？

1. **缺少主力机 Gazebo 端到端验证**：RGB + 深度 + TF 对齐尚未在真实仿真环境中验证。离线纯函数测试全部通过（64/64），但 ROS 节点 (`hsv_detector_node.py`) 的整体管线在主力机上还没跑通过。
2. **边缘/局部裁剪漏检**：Gazebo `edge_partial` 场景中右侧裁剪的红球被形状筛选拒绝
3. **多红球漏检**：Gazebo `multi_visible` 场景中 2 个红球只检出 1 个
4. **真实图片检出率低**：10 张真实图片仅 3 张正确检出（30%），复杂纹理和光照下表现不佳

### 6. 下一步需要哪个组配合？

- **平台组**：
  - 确认主力机 Gazebo 环境中 `/hw/camera/image_raw`、`/hw/camera/camera_info`、`/hw/camera/depth_image` 三个话题正常发布
  - 确认 `camera_link → start` 的 TF 变换正确发布
  - 提供可用的 Gazebo 仿真环境账号或远程访问方式
- **决策组**：
  - 确认 `/hw/perception/hazard_detections` 话题的消息格式需求
  - 确认危险源三维坐标的坐标系约定（`start` 还是 `map`）
- **集成测试组**：
  - 提供感知组专项测试模板和记录表格位置
  - 协助在主力机上跑通感知 ROS 节点的端到端测试
