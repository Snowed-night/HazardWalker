# diagnostic_12：稳定视角与主动复查实机链路复盘

本目录是官方 `auto.sh` 固定 `SEED=2026071802` 随机三层复杂楼宇的真实运行记录，
不是受控专项成绩。运行期只使用公开 RGB、深度、相机内参、雷达、IMU、合法
Cartographer SLAM、公开门控服务和 `/hw/cmd_vel`；未读取场景布局、危险源真值或
Gazebo ground truth。

## 运行结论

- 代码版本：`730458c8e0c524d33f9bb2f75fd6ea90b0a258c8`。
- Cartographer 持续建图并保存 `1405 × 840 @ 0.05 m` 地图。
- Frontier 从入口进入复杂楼宇，经过长走廊、房间入口和多类非红色障碍物；共记录
  3926 帧、589 条合法 SLAM 轨迹样本。
- 145.043–163.843 秒间出现 21 帧红球候选。目标在遮挡物边缘部分可见，置信度范围
  为 `0.19–0.96`，算法没有把低质量单视角候选直接写成危险源。
- 导航实际执行了三轮有界复查：向右转动一次、向右横移两次，之后恢复 Frontier
  探索。修复后的视角标识覆盖 `yaw:60 → yaw:30` 及两个平移网格，证明相机 X 前向
  约定和仿真时钟动作链已真实生效。
- 探索窗口结束后状态真实进入 `RETURNING`。应用户要求停止当前仿真，返航尚未完成，
  因此 `mission_completed=false`，本轮不能作为官方闭环成功证据。
- 最终确认数为 0，没有生成可提交的 `detected_danger.json`；这是严格门禁的预期结果，
  不能将黄色 `reobserve` 框解释为“已检测确认”。

## 直接失败原因

RGB 与深度由两个独立 rosbridge WebSocket 到达。21 个候选帧使用到的最近深度均与
RGB 相差约 `49–101 ms`，超过正式门槛 `20 ms`，所以：

- `depth_synchronized=false`；
- 深度形状保持 `unknown`；
- 0 帧完成三维定位；
- 0 个轨迹满足多视角球面确认。

旧回调会在 RGB 先到时立即使用上一帧深度，导致严格门槛虽然避免了错确认，却会让
所有此类候选都无法定位。下一轮修复为“RGB 最多等待一帧，匹配时间戳深度随后到达
时再处理；若深度持续缺失则降级为不可确认候选”，不放宽 20 ms 门槛。

## 本轮验证到的改进

1. 官方复杂楼宇内 Cartographer、Frontier、RGB 感知和主动换视角可以同时运行。
2. 红球只露出窄小边缘时被标记为候选并触发复查，没有冒充已确认红球。
3. 主动复查使用真实导航控制权，动作有次数上限，完成后会继续探索。
4. 官方探索超时能够触发 `EXPLORING → RETURNING`，但完整返航仍待下一轮验证。
5. 严格 RGB-D 同步门禁有效拦截错帧三维证据。

## 证据索引

- `diagnostic_12_contact_sheet.png`：真实楼宇探索、候选发现和三轮换视角的关键帧总览。
- `selected_images/`：导航上下文与候选复查原图。
- `cartographer_map.png` / `cartographer_map.yaml`：本轮合法 SLAM 地图快照。
- `frames.jsonl`：逐帧候选、同步状态、视角和复查建议。
- `trajectory.jsonl`：合法 SLAM 轨迹。
- `summary.json` / `failure_reasons.json`：结构化结论与失败原因。
- `full_stack.log`：Cartographer、Frontier、感知、复查和返航状态原始日志，仅保存在
  本地及远程原始证据目录，不提交 Git。

原始 `selected_depth/*.npy` 单帧约 1.2 MB，仅保存在远程原始证据目录，不提交 Git。
