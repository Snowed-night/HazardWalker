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

对未贴边的完整球形候选，`use_sphere_projection_geometry` 默认根据 bbox 的投影半径反推球心深度，减少远距离场景中“固定加半径”的过补偿；贴边、遮挡或过小候选自动退回保守深度补偿。

`allow_latest_tf_fallback` 默认开启：相机帧时间比公开 world TF 新数毫秒时，节点会回退到最新变换而非丢弃整帧定位；对高速运动且要求严格时序的硬件平台可关闭它。
- `dynamic_detection_recorder_node`：订阅 `/hw/camera/image_raw` 和 `/hw/perception/hazard_detections`，写入逐帧记录、精选截图、`summary.json` 与测试组 CSV/JSON；可选订阅显式传入的合法 SLAM `Odometry` 话题，默认不记录 `/hw/odom`，避免将官方禁用的 Gazebo 真值桥接链路混入正式证据；发布建议动作到 `/hw/perception/view_recommendation`，但不直接控制机器人。

上述节点收到官方一键栈的正常停止信号时会安静收尾；记录节点仍先落盘 `frames.jsonl`、`summary.json` 和
测试表，避免把 ROS2 外部 shutdown 误报为感知崩溃。

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

## 官方随机场景联调契约

正式比赛运行时，本包不读取 `danger_truth`、场景布局、`/Odometry_gazebo` 或任何
`/ground_truth/*`。导航/定位组必须提供合法 SLAM 的 `world -> real_sense`（或可组合的
等价）TF，并使用下列一键业务参数明确声明来源：

~~~text
ros2 launch hazardwalker_bringup official_simenv_business.launch.py \
  perception_output_frame:=world \
  localization_provenance:=lidar_imu_slam
~~~

可替换为 `visual_inertial_slam`。未具备该契约时保留默认 `map/unverified`，最终结果构建器
会故意排除所有轨迹；这是防止调试真值坐标混入 `detected_danger.json` 的安全门，而不是故障。

### 官方 ROS1 的激光—IMU 定位入口

当前官方实例已公开 `/scan`、`/trunk_imu` 和静态 `base -> real_sense` 外参，但现有
`map -> odom` 来自 Gazebo 状态节点，不能使用。感知定位组提供了独立的
`official_simenv_lidar_imu_slam_node.py`：它只以扫描端点相关匹配和 IMU 朝向维护
`start -> slam_base -> real_sense`，并发布 `/hazardwalker/slam/odometry`；不订阅任何 Gazebo
里程计或真值话题。使用独立 `slam_base` 避免与官方已有 `odom -> base` 形成多父 TF 冲突。
多楼层高度使用官方生成器公开的固定层高 `2.6 m`，订阅
`/hazardwalker/navigation/floor_index`。该楼层号必须由导航在电梯服务确认成功或楼梯状态机
确认到层后发布，不能从 layout、manifest 或真值读取；换层时定位器清空旧楼层的二维匹配地图，
同时保留 x/y/yaw 连续值。

~~~text
# 先启动合法定位（不发送控制命令）
SIMENV_ROOT=/path/to/SimEnv \
  bash scripts/run_official_simenv_lidar_imu_slam.sh

# 再启动感知；参数显式声明定位来源，未启动上述节点则不会导出最终危险源。
SIMENV_ROOT=/path/to/SimEnv \
  bash scripts/run_official_simenv_ros1_perception.sh \
    _localization_provenance:=lidar_imu_slam+public_floor_action
~~~

这只是增量定位实现，正式评分前仍必须在官方随机 SEED 场景验证长程漂移、重定位和
`start -> base -> real_sense` TF 完整性，并验证导航不会提前或错误发布楼层号；它不替代
导航组的建图、探索或返航模块。

为处理官方 SimEnv 中的遮挡、圆柱干扰和单视角风险，检测输出分为两层：

```text
严格红球候选：通过原有 HSV + 形状筛选，才允许进入三维 tracking。
宽松复查候选：仅在局部可见/弱光下触发主动换视角，字段 requires_reobservation=true，不能单帧确认。
疑似合并候选：距离峰提示一个红色连通域内仍有多个中心时，字段 may_be_merged=true，同样只能横移复查，不能直接计作一个危险源。
```

定位可用时，`hsv_detector_node` 默认要求同一轨迹累积至少 3 次有效观测、来自至少 3 个离散相机视角，并且相机到候选的水平视线跨度至少 25°，才会标记为 `confirmed`。这会避免红色圆柱或圆盘在固定机位的连续帧中被直接确认为红球；视角标签由相机在输出坐标系的位置和朝向量化得到。

除上述门槛外，正式 profile 还要求至少两个独立视角各自产生 RGB-D 球面正证据，并将反投影
中位表观直径与题目标准值 `0.30 m` 对比（默认允许 35% 相对误差）。因此，红色圆柱正面、
红色圆锥端面、局部可见弧段、尺寸明显不符的红色圆物都只能提出复查请求；只有完整证据链才
允许导出为红球。

每个 RGB-D 视角的球面正证据不仅检查“中心比外环更近”，还分别计算水平、竖直和两条对角线
四个方向的深度曲率。四个方向都有凸曲率且最小/最大曲率比例不低于
`min_sphere_axis_curvature_ratio`（默认 `0.35`）时才记为 `spherical`；只在一个方向弯曲的
任意角度的圆柱侧面或弧形板标为 `anisotropic`，只可触发侧向复查。方向有效深度不足时标为 `unknown`，
不会误充球面正证据，也不会永久拒绝可能被遮挡的真实红球。

官方 SimEnv 的 RGB 与深度经 rosbridge 独立到达，节点只会在二者时间戳差不超过
`max_rgb_depth_sync_delta_sec`（默认 0.15 s）时使用深度做球形判别和三维反投影。超出窗口时该帧仅保留 RGB 候选，不能把旧深度当作当前物体的反证或错误坐标。

官方 Gazebo 的 `real_sense` TF 是 **X 前方的机体链路系**，而内参反投影结果是标准
ROS 光学系（右、下、前）。官方业务启动文件显式传入
`camera_axis_convention:=gazebo_link_x_forward` 完成坐标转换；其他符合 ROS 光学系的相机不要套用此参数。

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
/Odometry_gazebo                -> /hw/odom（官方正式感知/证据禁止使用）
/tf、/tf_static                 -> /tf、/tf_static
```

平台适配完成后，从仓库根目录运行（路径按本次实验日期替换）：

```bash
ros2 run hazardwalker_perception dynamic_detection_recorder_node --ros-args \
  -p output_dir:=reports/perception/official_random/seed_20260715_42 \
  -p test_record_dir:=reports/perception/test_records/official_random_seed_20260715_42 \
  -p scenario_name:=official_random_seed_20260715_42 \
  -p run_mode:=official_random_scene \
  -p scenario_seed:=20260715_42 \
  -p code_version:=<git-commit> \
  -p legal_pose_topic:=/hw/slam/odometry \
  -p localization_provenance:=lidar_imu_slam
```

节点会产出：

```text
<output_dir>/frames.jsonl                 每次感知输出对应的候选、位姿、确认状态、动作建议
<output_dir>/selected_images/*.png        按最小时间间隔保存的证据帧
<output_dir>/selected_depth/*.npy         与 RGB 时间差不超过 0.15 秒的米制深度帧
<output_dir>/trajectory.jsonl             调用方显式提供的合法 SLAM 位姿采样
<output_dir>/summary.json                 动态帧数、候选数、确认数、平均置信度、建议动作统计
<test_record_dir>/testing_record_perception.csv
<test_record_dir>/testing_record_perception.json
```

记录器默认 `run_mode=internal_regression`，其 `summary.json` 会明确拒绝作为正式证据。正式随机场景
运行必须显式传入固定 SEED、代码版本、合法 SLAM 位姿话题和 `lidar_imu_slam` 或
`visual_inertial_slam` 来源；否则仍会保留调试记录，但 `formal_evidence_eligible=false`。未提供受控案例
真值时，测试记录会把识别率、漏检、虚警和定位误差相关字段保留为空。受控夹具只能在**运行结束后**计算
内部回归指标；运行期检测、换视角策略和导航不得读取 `danger_truth.json` 或场景私有布局文件。

结束后使用与运行时隔离的结构校验器检查归档；它只读取已保存的文件和输出 JSON，不会读取场景
真值。通过后会生成 `independent_post_evaluation.json`；召回率、虚警率和定位误差仍须由主办方的
赛后评测程序独立给出。

~~~text
# 先把本场运行的最终输出复制到同一证据目录，保留可复核快照。
cp /path/to/SimEnv/results/detected_danger.json \
  reports/perception/official_random/seed_20260715_42/detected_danger.json

python scripts/validate_official_random_perception_evidence.py \
  reports/perception/official_random/seed_20260715_42 \
  --result-json reports/perception/official_random/seed_20260715_42/detected_danger.json
~~~

直接接入 ROS1 官方感知节点时，记录器也可以只更换输入话题，而不改变检测算法：

~~~text
-p image_topic:=/real_sense/rgb/image_raw \
-p depth_topic:=/real_sense/depth/image_raw \
-p detection_topic:=/hazardwalker/perception/hazard_detections
~~~

## 依赖

离线红球形状检测需要 OpenCV：

```bash
python -c "import cv2; print(cv2.__version__)"
```

主力机环境应安装 `python3-opencv`。Windows 本机可使用：

```powershell
python -m pip install opencv-python
```
