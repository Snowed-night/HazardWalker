# diagnostic_20：有界前沿失败与自动返航闭环

本目录保存官方 `auto.sh` 固定 `SEED=2026071802` 三层随机复杂楼宇中的真实运行
记录，不是受控专项成绩，也不是红球识别成功案例。运行期只使用公开 RGB、深度、
相机内参、雷达、IMU、合法 Cartographer SLAM、公开门控服务和 `/hw/cmd_vel`；
未读取场景布局、危险源真值或 Gazebo ground truth。

## 运行结论

- 代码版本：`01b5edebcb0dda27a16cc8f8721f620a157db8fe`。
- diagnostic_19 曾在一个重规划回调中批量封禁几十个前沿。本轮每次重规划最多
  封禁 4 个失败盆地：第一次达到上限时保留 90 个候选，下一次重规划立即找到
  115 栅格安全路径；全程 3 次达到上限，均未再出现整层候选一次清空。
- 机器人记录 3956 帧、594 条合法 SLAM 轨迹，轨迹采样路径长约 52.51 m，
  最远离起点约 11.39 m；自主选择 11 个前沿、到达 2 个，另有 3 个目标由
  净进展看门狗安全退避。
- 约 322 仿真秒进入 `RETURNING`，没有出现“无安全返航路径”；约
  338.5 秒自动进入 `FINISHED` 并由任务状态机写出官方格式
  `detected_danger.json`。
- 全程没有红色候选，因此正式结果为空列表。`summary.json` 真实记录
  `mission_completed=true`、`confirmed_hazard_count=0`；这只能证明覆盖/返航
  链路改善，不能证明红球召回率、主动多视角或三维定位。
- 成功轮次还暴露一个证据收尾缺陷：结果本来就在证据目录，记录器再次复制到自身
  会触发 `SameFileError`，使测试组 CSV/JSON 缺失。后续代码已跳过同路径复制；
  本目录测试表由同轮 `summary.json` 按记录器同一纯函数补写，数值可复算。

## 证据索引

- `diagnostic_20_contact_sheet.png`：入口、真实楼梯区域、走廊、房间边缘及返航
  阶段的 12 帧时间序列。
- `cartographer_map.png` / `cartographer_map.yaml`：自动完成时的合法单雷达地图。
- `frames.jsonl`：3956 帧逐帧感知记录，全部为 `continue_exploring`，无候选。
- `trajectory.jsonl`：594 条合法 SLAM 位姿，可复算覆盖和返航。
- `detected_danger.json`：状态机自动生成的官方格式空结果，不是人工伪造。
- `summary.json` / `failure_reasons.json`：任务完成、0 确认及唯一可观察失败原因。
- 测试组 CSV/JSON 位于对应 `reports/perception/test_records/` 目录。
- 完整 `selected_images/`、`selected_depth/*.npy`、`full_stack.log` 和原始
  `.pgm` 保存在本地及远程原始证据目录；Git 只提交精选图与结构化证据。

本轮证明有界失败保护在官方复杂环境中真实改善了覆盖连续性，并恢复自动返航；
没有进入危险源视野，不能计作感知识别成绩。下一轮需优先改善房间覆盖与楼层切换，
再在真实候选上验证有效横移和严格多视角球体确认。
