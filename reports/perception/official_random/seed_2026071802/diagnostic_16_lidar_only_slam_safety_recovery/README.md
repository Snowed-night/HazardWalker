# diagnostic_16：LiDAR-only SLAM 与持续转向中止诊断

本目录保存官方 `auto.sh` 固定 `SEED=2026071802` 三层随机复杂楼宇中的真实运行
记录，不是受控专项成绩，也不是完整官方闭环。运行期只使用公开 RGB、深度、相机
内参、雷达、IMU、合法 Cartographer SLAM、公开门控服务和 `/hw/cmd_vel`；未读取
场景布局、危险源真值或 Gazebo ground truth。

## 运行结论

- 代码版本：`5eefc70ea6512b2dcfa12b96da2705113bc33cda`。
- 正式 Cartographer 仅融合 360° `/hw/scan`，没有启动旧
  `depth_to_scan_node`；单雷达标准话题 `scan` 已正确映射，地图与 TF 正常发布。
- 首个近侧目标 `(3.63,-1.17)` 被真实到达；随后选择 `(8.33,1.73)`。机器人
  从入口移动到合法 `odom` 约 `(6.10,-0.66)`，说明 diagnostic_15 的“动作全部
  被门禁归零”死锁未复现。
- 保存 1915 帧 RGB 记录、306 条合法 SLAM 轨迹样本和
  `1154 × 1213 @ 0.05 m` 地图快照；没有红球候选，不能证明三维定位或多视角确认。
- 第二个目标后期持续发布角速度并发生小幅转动，但约 40 仿真秒没有有效净平移。
  普通卡死检测把角度变化误当作进展，安全门禁看门狗又只处理“所有动作被归零”，
  因此无法主动切换目标。确认该缺口后停止隔离实验。
- `summary.json` 明确记录 `mission_completed=false`；没有生成或伪造
  `detected_danger.json`。

## SLAM A/B 结论

停用 RGB-D 伪水平扫描后，地图仍保留接近传感器 30 m 量程半径的放射状端点。
这排除了“深度扫描是唯一根因”。进一步审计确认 Cartographer 的
`max_range=30 m` 会把雷达接近量程上限的有限值当成真实击中；同时适配器把
0.4 m 内机身遮挡错误改成 `+inf`，会产生虚假自由射线。下一轮将把有效击中距离
收紧到 8 m，并把机身遮挡改为 `NaN` 丢弃。

## 证据索引

- `diagnostic_16_contact_sheet.png`：入楼、侧向移动和持续对柱体/门边转向的连续视野。
- `selected_images/`：每约 10 仿真秒保存的 RGB 上下文。
- `cartographer_map.png` / `cartographer_map.yaml`：LiDAR-only 中止地图。
- `frames.jsonl` / `trajectory.jsonl`：逐帧感知与合法 SLAM 轨迹。
- `summary.json` / `failure_reasons.json`：未完成结论与可直接观察的失败信号。
- 测试组 CSV/JSON 位于对应 `reports/perception/test_records/` 目录。
- `full_stack.log`、原始 `.pgm` 和 `selected_depth/*.npy` 仅保存在本地及远程
  原始证据目录，不提交 Git。

本轮证明 LiDAR-only 单雷达链路可出图并产生真实移动，同时定位到 30 m 量程伪
端点和“持续转向但无净进展”两项缺口；不能证明任务完成或正式感知成绩。
