# HazardWalker 感知定位阶段进展汇报

汇报日期：2026-06-12

## 1. 项目目标与评分约束

HazardWalker 面向 DG-202602「基于四足机器人的危险源自主搜索与识别技术」赛题，目标是在未知多层楼栋仿真环境中完成：

1. 四足机器人自主探索未知环境。
2. 识别环境中的红色球体危险源。
3. 输出危险源相对起点的三维坐标。
4. 完成任务后自主返航。

当前需要重点对齐的评分约束：

```text
探索 + 返航平均耗时 <= 600 秒：探索效率满分
识别率 <= 60%：危险源识别概率直接 0 分
虚警率 > 10%：开始扣分
```

因此现阶段感知组策略不是盲目检出所有红色区域，而是优先压低虚警，在稳定识别红色球体的基础上逐步提高召回率。

## 2. 当前总体技术路线

项目当前采用的主线方案如下：

```text
Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic
Nav2 + SLAM Toolbox
Frontier 探索起步
OpenCV / HSV 红球检测基线
CameraInfo + 深度图/点云 + TF 三维定位
FSM 任务状态机
JSON / CSV 指标统计
```

工程接口约束：

```text
hazardwalker_platform 是唯一平台适配入口
算法模块只面向 /hw/* 内部接口
感知节点不直接依赖 Gazebo 或官方平台原始话题
```

## 3. 感知算法当前进展

红球检测已经从早期的 HSV 红色像素统计升级为多条件候选检测：

```text
RGB/BGR 图像
  -> HSV 红色阈值
  -> mask 形态学去噪
  -> findContours 轮廓/连通域
  -> 面积过滤
  -> 圆度 circularity
  -> bbox 长宽比 aspect_ratio
  -> 轮廓面积 / bbox 面积 extent
  -> 输出一个或多个红球候选
```

当前检测结果包含：

```text
bbox
confidence
red_pixel_count
circularity
aspect_ratio
extent
```

已支持多目标红球候选输出，并保留旧单目标接口兼容现有调用。

## 4. 三维定位与多帧确认

当前已经完成三维定位和多帧确认的算法基础模块：

| 模块 | 作用 |
|---|---|
| `localize_hazard.py` | 根据 bbox、CameraInfo 内参和深度信息反投影三维坐标 |
| `track_hazards.py` | 按三维距离合并多帧观测，完成确认和去重 |
| `detection_metrics.py` | 计算 IoU、precision、recall、F1、top1 error、AP50 等检测指标 |

三维定位流程：

```text
bbox 中心像素
  -> CameraInfo 内参反投影
  -> bbox ROI 深度中位数
  -> 相机坐标系三维点
  -> TF 刚体变换到 start/map 坐标系
```

多帧确认规则：

```text
merge_distance_m 内的观测合并为同一危险源
observation_count 达到 confirm_observation_count 后标记 confirmed
missed_count 达到 reject_after_missed_count 后标记 rejected
```

ROS 接口方面，`hsv_detector_node` 已接入：

```text
/hw/camera/image_raw
/hw/camera/camera_info
/hw/camera/depth_image
/tf
/hw/perception/hazard_detections
```

当相机内参、有效深度和 TF 链可用时，节点会输出三维 `hazards`。如果深度或 TF 不可用，只输出 `detections_2d` 调试字段，不再发布占位三维坐标。

## 5. 离线测试与可控实验结果

当前离线测试覆盖：

```text
红球 HSV + 形状检测
BGR/RGB 输入
暗红、低饱和红过滤
红色方块和不规则物体过滤
10%、20%、30%、50% 遮挡检测
70% 遮挡保守拒绝
多目标候选输出
三维定位反投影与坐标变换
多帧确认和空间去重
IoU、precision、recall、AP50 等指标计算
```

最新测试结果：

```text
python scripts/run_offline_tests.py
Offline tests: 35 passed, 0 failed

python -m pytest tests/offline
35 passed
```

合成可控案例实验：

```text
python scripts/generate_perception_cases.py
Generated 14 perception cases, 14/14 passed.
```

当前合成案例检测指标：

| 指标 | 当前结果 |
|---|---:|
| 样本数 | 14 |
| 真值红球样本 | 7 |
| true positive | 7 |
| false positive | 0 |
| false negative | 0 |
| true negative | 7 |
| precision | 1.0000 |
| recall | 1.0000 |
| F1-score | 1.0000 |
| top1 error | 0.0000 |
| AP50 | 1.0000 |

说明：上述指标来自可控合成案例，只用于验证当前算法逻辑和回归测试，不等价于真实比赛精度。真实仿真精度需要在 Gazebo/官方平台中结合真值危险源位置进一步评估。

## 6. 实物图泛化观察

使用 10 张本地实物红球图片进行泛化观察：

```text
python scripts/evaluate_real_red_ball_images.py --input-dir C:\Users\jiangchen\OneDrive\Desktop\red_ball
Evaluated 10 real images, total detections=7.
```

观察结论：

1. 当前方法对简单红色圆形目标有效。
2. 对纹理、反光、复杂背景、小目标和红色非球体仍有局限。
3. 实物图没有人工真值 bbox，因此只作为展示和泛化观察，不作为严格精度评测。
4. 后续比赛主线仍应回到 Gazebo/官方平台图像、深度图、点云和 TF。

## 7. 当前成果文件

核心代码：

```text
ros2_ws/src/hazardwalker_perception/hazardwalker_perception/red_ball_detector.py
ros2_ws/src/hazardwalker_perception/hazardwalker_perception/hsv_detector_node.py
ros2_ws/src/hazardwalker_perception/hazardwalker_perception/localize_hazard.py
ros2_ws/src/hazardwalker_perception/hazardwalker_perception/track_hazards.py
ros2_ws/src/hazardwalker_perception/hazardwalker_perception/detection_metrics.py
```

核心测试：

```text
tests/offline/test_red_ball_detector.py
tests/offline/test_localize_hazard.py
tests/offline/test_track_hazards.py
tests/offline/test_detection_metrics.py
```

汇报图：

```text
reports/perception/2d_detection/synthetic_20260620_230333/figures/perception_cases_collage.png
reports/perception/2d_detection/real_images_20260620_230333/figures/real_red_ball_collage.png
```

## 8. 主要风险

1. 当前三维定位已完成接口接入和离线验证，但真实 Gazebo/官方平台中仍需确认 RGB、深度图和 TF 是否严格对齐。
2. 当前定位优先使用深度图 ROI 中位数，点云 ROI 定位尚未接入。
3. 当前多目标 id 是空间跟踪 id，不是复杂视觉重识别 id。
4. 当前红球检测仍是传统视觉基线，遇到复杂光照、遮挡和非标准材质时可能漏检。
5. 实物图存在版权边界，原图不提交仓库，也不作为公开报告材料。

## 9. 下一步计划

近期优先级建议：

1. 在主力机 Gazebo/官方平台中验证 `/hw/camera/depth_image`、`/hw/camera/camera_info` 和 TF 链。
2. 用仿真场景中的红球真值位置评估三维定位误差。
3. 将多帧 confirmed hazards 接入决策 FSM，替换早期占位坐标。
4. 增加主动重观察逻辑：低置信或深度无效时调整视角再次观测。
5. 推进 Frontier/Nav2 探索与感知触发联动，逐步验证探索效率和危险源识别率。
6. 在稳定传统视觉基线后，再评估 YOLO、小目标检测或主动视觉方法是否值得接入主流程。
