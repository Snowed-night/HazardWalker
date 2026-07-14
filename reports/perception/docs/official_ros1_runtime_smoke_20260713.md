# 官方 ROS1 Noetic 运行时冒烟记录

日期：2026-07-13

## 目的

验证当前官方 ROS1 感知节点可以在真实 SimEnv 容器中订阅 RGB、深度、相机内参和 TF；本次
不启动控制器切换、不发布 `/cmd_vel`、不写官方 `results/`，也不读取真值文件。

## 环境与命令边界

- 容器：`simenv_ros1_hazard_platform`（ROS1 Noetic + Gazebo Classic）。
- 实测公开输入：`/real_sense/rgb/image_raw`、`/real_sense/depth/image_raw`、
  `/real_sense/rgb/camera_info`、`map -> real_sense` TF。
- 在容器 `/tmp/hw_official_trial/` 中临时运行
  `official_simenv_ros1_perception_node.py`，参数为 `world_frame=map`、
  `localization_frame=map`、`auto_activate_cmd_vel=false`。
- 输出仅写入 `/tmp/hw_official_trial/detected_danger.json`，测试结束后删除临时目录。

## 结果

空视场节点连续运行约 22 秒，成功发布：

```json
{"hazards": [], "detections_2d": [], "camera_stable": true, "output_frame": "map"}
```

随后以唯一名称在 Gazebo 中临时生成一颗半径 0.15 m 红球，运行节点但**不移动机器人**。实测
`/hazardwalker/perception/hazard_detections` 已产生以下关键证据：

```json
{
  "hazards": [{
    "status": "tentative",
    "observation_count": 6,
    "distinct_view_count": 1,
    "view_bearing_span_deg": 0.0,
    "evidence_status": "collecting_views"
  }],
  "detections_2d": [{
    "confidence": 0.9387,
    "depth_shape": {"status": "spherical", "curvature_m": 0.03564},
    "position": [0.0345, -2.1689, 0.0789],
    "position_frame_id": "map"
  }]
}
```

关闭节点后的临时官方格式输出为：

```json
{"exploration_time": 17.645, "detected_danger_sources": []}
```

这说明红球候选、RGB-D 反投影和跟踪已在真实官方容器中走通；也说明单一固定视角不会越过
正式上报门槛。模型随后删除，未改变比赛场景。运行时还发现并修正两个真实集成问题：节点曾调用
不存在的 `analyze_depth_shape`，且曾把深度定位函数的位置参数顺序传反；现已分别改为
`evaluate_sphere_depth_shape` 和具名参数调用，并新增离线 API/调用契约测试。同步器容差也从
0.12 s 调整为可配置的 0.25 s，以适应官方容器 RGB 与深度时间戳的实际抖动。

同一受控静止复测还收到主动观察请求：

```json
{
  "required_min_view_bearing_span_deg": 25.0,
  "recommendation": {
    "action": "move_right",
    "priority": 55,
    "reason": "单视角圆形仍可能是圆柱或圆锥端面；完成当前稳定帧后获取独立侧视再确认。"
  }
}
```

该话题是给导航层的语义请求，感知节点没有发布 `/cmd_vel`；因此这次静止复测不伪造“已完成多
视角”的结论。

升级后的同类复测进一步得到可执行但尚未执行的侧视几何计划：候选距相机约 1.576 m，节点为
`move_right` 生成两个 waypoint（累计 12.5°、25.0°），每段由导航层避障并保持相机朝向候选。
该计划的最后两维目标为 `[1.7839, -1.9325]`、yaw `-2.978744` rad。它证明官方实际候选能进入
侧视规划链路；由于本次没有让机器人移动，不能把 waypoint 当作已取得的视差证据。

同一位置还临时生成了半径 0.15 m、长度 0.50 m 的红色竖直圆柱（侧面可见）。真实官方节点发布
`{"hazards": [], "detections_2d": []}`，即未把侧面长方形圆柱作为红球候选或正式目标。该模型
同样已删除；它只覆盖“圆柱侧面”的负样本，**不**覆盖正面近圆端面，后者仍须以实际横移获得侧面
反证。

随后把同一圆柱转为轴向朝向相机，使端面近圆。它确实形成高置信二维候选（`confidence=0.933`、
`circularity=0.8916`），但深度形状为 `flat`，因此被明确标为
`requires_reobservation=true`、`confirmation_eligible=false`；轨迹状态为
`needs_reobservation/single_view_flat_or_non_spherical`，没有进入官方结果。请求优先级为 94，
要求向右侧向复查。该负样本说明前端不会把“看起来圆”包装为红球识别成功；在实际执行侧向
waypoint 后，才可验证它是否会由多视角证据拒绝。最后一次复测的实际消息确认了上述状态：
`status=needs_reobservation`、`evidence_status=single_view_flat_or_non_spherical`、
`requires_reobservation=true`，且 `image_requires_reobservation=false`，从而清楚地区分“二维近圆”
与“RGB-D 已阻止确认”。模型和临时目录均已删除。

## 结论与限制

本记录只证明 **官方 ROS1 RGB-D + CameraInfo + TF + 节点发布链路**及单视角“只候选、不上报”
的行为正确；这里为了验证接口而临时使用了 `map -> real_sense` TF，不能替代正式运行中团队
SLAM/定位输出，也没有以该受控模型生成任何五类正式效果或性能结论。它不证明红球检出率、圆柱
排除、多视角确认、三维定位误差或全屋搜索性能。上述指标仍须在受控复杂场景、实际横移观察且
不依赖 Gazebo 真值的正式五类实验中重跑。Gazebo 和 `junior_ctrl` 已核验仍正常运行，容器和主机
的 `/tmp/hw_official_trial` 临时文件均已清理。
