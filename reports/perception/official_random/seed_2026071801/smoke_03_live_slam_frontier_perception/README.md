# 官方随机场景 SEED=2026071801 烟测 03（在线 SLAM + Frontier）

状态：平台/建图/探索链路烟测，任务失败样本；仅保留为算法优化路径，不计入官方成绩。

## 运行范围

- 场景由官方 `auto_headless.sh` 以固定 `SEED=2026071801` 生成，三层、每层四个房间。
- 运行算法只订阅公开 RGB-D、相机内参、`/scan`、`/trunk_imu` 和 TF；适配器明确关闭
  Gazebo 里程计转发，导航、SLAM 和感知均未订阅真值位姿。
- 使用合法 scan/IMU 里程计、SLAM Toolbox 在线异步建图和 Frontier 探索，并同时记录真实
  复杂楼宇上下文 RGB-D。
- 本轮只验证底层链路能否实际运行，不包含多楼层切换、完整楼宇覆盖、目标复查、结果提交或返航。

## 真实结果

- SLAM Toolbox 生命周期达到 `active [3]`，`/map` 可订阅并保存为
  `slam/map.pgm`、`slam/map.png` 和 `slam/map.yaml`。
- 证据记录器采集 `484` 帧、`86` 个合法定位轨迹样本和 `5` 组真实 RGB-D 上下文。
- 运行期间没有红色候选或确认红球；记录器按合同写出
  `mission_not_completed`、`no_confirmed_red_ball_recorded`。
- Frontier 节点确实发出运动指令并触发过到达、重规划和卡住恢复；机器人发生真实位移。
- 当前地图存在明显射线拖影和累计畸变，说明“SLAM 节点已启动”不等于“SLAM 已达到可用精度”。
  该问题仍需统一 ROS 时间域、标定扫描坐标和继续压低 scan/IMU 里程计漂移。
- 容器停止时在 15 秒期限内未完全退出，Docker 最终记录 `Exited (137)`；不能据此宣称正式生命周期
  验收通过。

## 关键修复与限制

- 修复 rosbridge 把 LaserScan 中 `+inf` 序列化为 JSON `null` 后导致适配器崩溃的问题。
- 修复 SLAM Toolbox 被普通 `Node` 启动后停在 `unconfigured` 的问题，改用上游
  `online_async_launch.py` 并启用 lifecycle autostart。
- 修正正式 SLAM 基坐标为官方 URDF 中存在的 `base`，并禁止历史 JSON bridge 重发
  `/tf`、`/scan` 或转发控制。
- 通过只读 SimEnv 覆盖层临时禁用与官方 LaserScan 类型冲突的旧 PointCloud 转换节点，并补偿
  激光链路约 45° 的俯仰。正式仓库修复需以补丁和平台组审核落地，不能依赖临时覆盖层。

## 证据说明

- `selected_images/`、`selected_depth/`：真实运行 RGB-D 上下文。
- `frames.jsonl`、`trajectory.jsonl`：逐帧感知状态和合法定位轨迹。
- `summary.json`、`failure_reasons.json`、`test_records/`：结构化结果与失败口径。
- `slam/`：生命周期、地图话题信息及保存地图。
- `nav_slam.log`、`perception.log`、`adapter_status.yaml`：运行日志和适配器状态。

`summary.json` 中的 `formal_evidence_eligible=true` 仅表示输入来源合同没有被发现违反；
同一文件明确记录 `mission_completed=false`，因此本轮不是合格正式结果。
