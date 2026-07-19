# diagnostic_15：近场房间前沿优先的中止诊断

本目录保存官方 `auto.sh` 固定 `SEED=2026071802` 三层随机复杂楼宇中的真实运行
记录，不是受控专项成绩，也不是完整官方闭环。运行期只使用公开 RGB、深度、相机
内参、雷达、IMU、合法 Cartographer SLAM、公开门控服务和 `/hw/cmd_vel`；未读取
场景布局、危险源真值或 Gazebo ground truth。

## 运行结论

- 代码版本：`363802eef16175bb0d13a0636376099e61d0a2f4`。
- 近场候选带生效：首个目标为 `(3.53,-0.17)`，真实到达后没有跳向十几米外的
  长走廊，而是选择侧向小前沿 `(4.59,-1.32)`；该前沿只有 3 个栅格，证明原先
  会提前删除小门洞候选的全局尺寸过滤已被移除。
- 保存 879 帧 RGB 记录、139 条合法 SLAM 轨迹样本和 `1231 × 1188 @ 0.05 m`
  地图快照；运行中没有红球候选，不能证明三维定位或多视角确认。
- 选择第二个目标后，局部扫描安全门禁持续把期望转向归零；约 26 仿真秒内机器
  人位置仅有厘米级变化，`/hw/cmd_vel` 实测持续为零。
- 原卡死检测仅统计门禁后的非零指令，因此没有把该目标标记为失败，形成“安全
  门禁静止死锁”。确认原因后主动停止隔离实验，没有等待 600 秒假装完成。
- `summary.json` 明确记录 `mission_completed=false`；没有生成或伪造
  `detected_danger.json`。

## SLAM 诊断

`cartographer_map.png` 出现大面积放射状弱占据拖影和伪前沿。只读审计确认正式
Cartographer 同时融合 `/hw/scan` 与 `/hw/depth_scan`，而后者把深度图中间
35%–65% 的整条竖直带用 10% 低分位压成二维扫描，未按相机高度过滤地面、楼梯
和天花板。这一投影会把非竖直表面误写成墙，是下一轮单独 A/B 的首要变量。

## 证据索引

- `diagnostic_15_contact_sheet.png`：入口、楼梯、门洞和静止死锁的连续真实视野。
- `selected_images/`：每约 10 仿真秒保存的 RGB 上下文。
- `cartographer_map.png` / `cartographer_map.yaml`：中止时合法 SLAM 地图。
- `frames.jsonl` / `trajectory.jsonl`：逐帧感知与合法 SLAM 轨迹。
- `summary.json` / `failure_reasons.json`：未完成结论与可直接观察的失败信号。
- 测试组 CSV/JSON 位于对应 `reports/perception/test_records/` 目录。
- `full_stack.log`、原始 `.pgm` 和 `selected_depth/*.npy` 仅保存在本地及远程
  原始证据目录，不提交 Git。

本轮只证明近场小前沿会进入正式选择链路，同时定位到安全门禁死锁和深度扫描地图
伪影；不能证明任务完成、红球召回率、虚警率或三维定位精度。
