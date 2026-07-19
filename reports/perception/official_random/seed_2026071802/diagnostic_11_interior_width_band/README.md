# diagnostic_11：入楼与红球主动复查失败复盘

本目录是官方 `auto.sh` 固定 `SEED=2026071802` 随机三层复杂楼宇的真实运行记录，
不是受控专项成绩。运行期只使用公开 RGB、深度、相机内参、雷达、IMU、合法
Cartographer SLAM、公开门控服务和 `/hw/cmd_vel`；未读取场景布局、危险源真值或
Gazebo ground truth。

## 运行结论

- 代码版本：`48a0b4922302d918fa6d2f57174c5e643d3965a4`
- Cartographer 持续建图并保存 `1362 × 1218 @ 0.05 m` 地图。
- Frontier 从入口进入建筑，沿走廊抵达约 `(31.4, 0.6)`，随后进入侧房间前沿。
- 共记录 1555 帧、243 条合法轨迹样本；151 帧出现红色球形候选，103 帧完成三维定位。
- 候选在完整、清晰视角下置信度约 `0.94–0.96`，多帧深度形状大多支持球面。
- 最终确认数为 0，未输出可提交的 `detected_danger.json`；本轮不能作为官方闭环成功证据。
- 本轮在任务完成前按排障需要停止，因此 `mission_completed=false`，未验证返航。

## 直接失败原因

主动复查动作使用墙钟计时，而仿真实时率约为 `0.15–0.20`。动作结束后的墙钟
2.5 秒只对应约 0.4–0.5 秒仿真时间，A1 尚未完全停稳，感知的稳定相机门禁没有
取得足够的新视角帧。与此同时，官方相机是 X 前向 link 坐标系，旧实现却按光学
Z 前向计算相机朝向，原地转动被错误记为同一 `yaw:0` 视角。

结果是候选虽然跨越四个平移量化位置，确认轨迹仍只有：

- `distinct_view_count=1`
- `view_bearing_span_deg=0`
- `evidence_status=collecting_views`

此外，RGB 与深度相差 50–100 ms 的运动帧大量产生 `flat/anisotropic` 假反证；
时间戳完全同步的 67 帧中有 63 帧呈 `spherical`，说明必须收紧 RGB-D 同步门槛。

## 后续修复

下一轮在同一 SEED 上验证以下修改：

1. 按 `camera_axis_convention` 使用真实 X/Z 前向轴计算稳定性和视角 ID。
2. 重观察动作改用 ROS 仿真时钟，确保停车阶段获得足够 RGB-D 帧。
3. 横移根据实时目标方位变化闭环停止，达到 25° 侧向视差后再停稳采样。
4. RGB-D 同步门槛由 150 ms 收紧为 20 ms，错帧不进入球面正/反证。

## 证据索引

- `diagnostic_11_contact_sheet.png`：从入楼、走廊覆盖到发现候选的关键帧总览。
- `selected_images/`：导航上下文与候选复查原图。
- `cartographer_map.png` / `cartographer_map.yaml`：本轮建图快照。
- `frames.jsonl`：逐帧候选、深度形状、视角、定位和复查建议。
- `trajectory.jsonl`：合法 SLAM 轨迹。
- `summary.json` / `failure_reasons.json`：结构化结论与失败原因。
- `full_stack.log`：Cartographer、Frontier、感知和复查状态原始日志，仅保存在本地及
  远程原始证据目录，不提交 Git。

原始 `selected_depth/*.npy` 单帧约 1.2 MB，仅保存在远程原始证据目录，不提交 Git。
