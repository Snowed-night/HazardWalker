# diagnostic_14：空间前沿退避与返航闭环复测

本目录是官方 `auto.sh` 固定 `SEED=2026071802` 三层随机复杂楼宇的真实运行记录，
不是受控专项成绩。运行期只使用公开 RGB、深度、相机内参、雷达、IMU、合法
Cartographer SLAM、公开门控服务和 `/hw/cmd_vel`；未读取场景布局、危险源真值或
Gazebo ground truth。

## 运行结论

- 代码版本：`a90286763d7b9857c8893a9b41a4678c068fa78d`。
- Cartographer 全程在线，保存 `1535 × 1368 @ 0.05 m` 地图；记录 5017 帧、
  755 条合法 SLAM 轨迹样本和约 76.37 m 的 `odom` 帧累计路径。
- Frontier 共选择 12 个目标、真实到达 3 个；6 次局部卡死和 9 次不可达规划均
  被空间盆地退避打断，没有再出现 diagnostic_13 对同一 5 cm 邻域连续重选 7 次。
- `(6.00,-8.00)` 邻域第二次失败时，退避从 45 仿真秒升级为 90 仿真秒；节点没有
  被固定完成宽限提前切走，随后改选其他区域并继续产生真实位移。
- 约 300 秒探索预算到达后自动切换 `EXPLORING → RETURNING`，从距 home 约
  17.2 m 处沿合法 SLAM 地图返航；没有无安全路径或返航看门狗告警，最终以
  `Distance=0.25m` 进入 `RETURNING → FINISHED`。
- 官方结果在仿真时间 416.0 秒写出，`mission_completed=true`，满足 600 秒总预算。

## 感知与 SLAM 限制

- 5017 帧均未出现红球候选，因此本轮不能证明 RGB-D 延迟配对、三维定位或多视角
  确认在真实候选上生效，也不能把 0 个结果解释成“场景没有红球”。
- 严格证据校验结果为 `structural_evidence_complete=false`，明确失败项是
  `no_candidate_to_review` 和 `no_confirmed_red_ball`；本轮只作为导航/SLAM 闭环
  优化证据，不作为完整感知成绩。
- 机器人主要沿长主走廊覆盖，房间横向搜索不足；固定种子的危险源区域没有进入
  有效视野。下一轮应让房间入口/横向近前沿优先于远端大前沿。
- 地图能支撑本轮去程与返航，但 `cartographer_map.png` 仍有明显长射线和自由区
  伪影，不能称为高质量全楼 SLAM；跨楼层状态与分层地图仍未实现。
- `trajectory.jsonl` 是 `odom` 帧，导航到家判定使用 `map` 帧，不能把两者坐标
  直接相减。到家证据以节点实时 `map→base` 距离和状态转换日志为准。

## 证据索引

- `diagnostic_14_contact_sheet.png`：复杂楼内探索、楼梯/门洞、长走廊和返航关键帧。
- `selected_images/`：每约 10 仿真秒保存的真实 RGB 上下文。
- `cartographer_map.png` / `cartographer_map.yaml`：最终合法 SLAM 地图快照。
- `frames.jsonl` / `trajectory.jsonl`：逐帧感知记录与合法 SLAM 轨迹。
- `summary.json` / `failure_reasons.json` / `detected_danger.json`：结构化结论。
- `evidence_validation.json`：严格校验的未通过项，防止把零候选闭环误当正式成绩。
- 测试组 CSV/JSON 位于对应 `reports/perception/test_records/` 目录。
- `full_stack.log`、原始 `.pgm` 和 `selected_depth/*.npy` 仅保存在本地及远程
  原始证据目录，不提交 Git。

本轮可证明单层 2D SLAM、复杂楼内移动、不可达前沿恢复、预算返航和结果写出已经
闭环；不能证明红球搜索成功，更不能证明三层完整任务完成。
