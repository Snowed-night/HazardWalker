# diagnostic_17：有界雷达建图与近场前沿推进诊断

本目录保存官方 `auto.sh` 固定 `SEED=2026071802` 三层随机复杂楼宇中的真实运行
记录，不是受控专项成绩，也不是完整官方闭环。运行期只使用公开 RGB、深度、相机
内参、雷达、IMU、合法 Cartographer SLAM、公开门控服务和 `/hw/cmd_vel`；未读取
场景布局、危险源真值或 Gazebo ground truth。

## 运行结论

- 代码版本：`288f70d7fabe41e2508801d1273b40cd302195db`。
- Cartographer 的有效击中距离收紧为 8 m，0.4 m 内机身遮挡回波改为 `NaN`
  丢弃；仍然只融合官方 360° `/hw/scan`。
- 地图由上一轮的 `1154 × 1213` 收敛为 `413 × 416 @ 0.05 m`，30 m 量程
  伪圆环消失，楼内房间、走廊和门洞结构开始可辨。入口外仍有约 8 m 未击中射线
  形成的自由半圆，符合本轮配置，不等同于真实障碍物。
- 共保存 1810 帧 RGB 记录、263 条合法 SLAM 轨迹样本；10 次选择新前沿，
  首个 `(3.63,-0.17)` 被真实到达，近场目标抢占触发 2 次。
- 随后出现 5 次“几乎无净平移”的卡死。审计确认全局 A* 默认仅膨胀
  0.25 m，而局部安全门禁要求 0.45 m 净空，导致规划认为可通行、执行阶段持续
  拒绝；同一规划失败盆地还会在同一轮被重复计数。两项问题均已在下一代码版本修复。
- 没有红球进入有效视野，故没有候选、多视角确认或三维定位结果。
  `summary.json` 明确记录 `mission_completed=false`，没有生成或伪造
  `detected_danger.json`。

## A/B 结论

与 diagnostic_16 相比，本轮证明 8 m 有界雷达和机身回波丢弃能显著消除远距离
伪占用，并让前沿目标回到真实楼内。近场抢占策略也能从远目标主动切换到更近的
小前沿。当前主要阻断已从“SLAM 伪墙主导”收敛为“规划净空与执行净空不一致”，
因此下一轮保持同一 SEED，只验证统一 0.45 m 净空和失败盆地单次计数。

## 证据索引

- `diagnostic_17_contact_sheet.png`：入口、门洞、走廊和房间内推进的连续实景。
- `selected_images/`：每约 10 仿真秒保存的 RGB 上下文。
- `cartographer_map.png` / `cartographer_map.yaml`：8 m 有界雷达中止地图。
- `frames.jsonl` / `trajectory.jsonl`：逐帧感知与合法 SLAM 轨迹。
- `summary.json` / `failure_reasons.json`：未完成结论与可直接观察的失败信号。
- 测试组 CSV/JSON 位于对应 `reports/perception/test_records/` 目录。
- `full_stack.log`、原始 `.pgm` 和 `selected_depth/*.npy` 仅保存在本地及远程
  原始证据目录，不提交 Git。

本轮证明 SLAM 伪圆环问题已得到实质改善，并暴露统一净空的确定性缺口；不能证明
任务完成、红球识别成绩或正式评分。
