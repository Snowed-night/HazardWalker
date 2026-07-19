# diagnostic_21：前沿到达后的主动 RGB-D 环视

本目录保存官方 `auto.sh` 固定 `SEED=2026071802` 三层随机复杂楼宇中的真实运行
记录，不是受控摆拍成绩，也不是红球识别成功案例。运行期只使用公开 RGB、深度、
相机内参、雷达、IMU、合法 Cartographer SLAM、公开门控服务和 `/hw/cmd_vel`；
未读取场景布局、危险源真值或 Gazebo ground truth。

## 运行结论

- 代码版本：`5815e2b95ccdbba890117a91dc7b12b0cb554c0b`。
- 新增“到达前沿后先停稳并完成 360° RGB-D 环视，再继续探索”的主动观察流程。
  本轮自主选择 13 个前沿、到达 3 个，三次均真实开始并完成整圈环视；轨迹显示
  环视时位置基本不变、朝向连续跨越整圈，不是重复保存同一张图。
- 机器人记录 3795 帧、586 条合法 SLAM 轨迹，轨迹采样路径长约 52.47 m；
  覆盖坐标范围约 `x=[0.00, 4.03] m`、`y=[-7.42, 3.88] m`，最远离
  初始坐标约 8.36 m。地图快照为 `413×492`、`0.05 m/像素`。
- diagnostic_20 引入的有界前沿规划保护继续生效：本轮一次达到 4 次失败预算后
  保留 114 个候选，后续仍找到新路径；全程没有一次性批量清空前沿。
- 探索预算到达后自动进入 `RETURNING`，日志以实时地图坐标判定距 home 0.25 m，
  随后自动进入 `FINISHED` 并写出官方格式结果。`trajectory.jsonl` 保存的是
  独立 odom 轨迹，受 SLAM 回环坐标修正影响，不能用末条 odom 到原点的距离替代
  状态机的 map-frame home 门禁。
- 全程没有红色候选，因此正式结果为空列表。`summary.json` 真实记录
  `mission_completed=true`、`confirmed_hazard_count=0`；这轮只证明复杂楼宇内
  “探索—到达—主动环视—继续探索—返航—结果落盘”能够闭环，不能证明红球召回率、
  多视角球体确认或三维定位已经完成。

## 证据索引

- `diagnostic_21_contact_sheet.png`：入口、三处前沿环视的不同朝向、走廊、楼梯区域
  和返航阶段的 12 帧时间序列。
- `cartographer_map.png` / `cartographer_map.yaml`：自动完成时保存的合法单雷达地图。
- `frames.jsonl`：3795 帧逐帧感知记录，全部为 `continue_exploring`，无候选。
- `trajectory.jsonl`：586 条合法 SLAM 位姿，可复算路径长度和覆盖范围。
- `detected_danger.json`：状态机自动生成的官方格式空结果，不是人工伪造。
- `summary.json` / `failure_reasons.json`：任务完成、0 确认及唯一可直接观察失败原因。
- `independent_post_evaluation.json`：独立归档校验确认 RGB/深度样本配对完整，但因
  `no_candidate_to_review`、`no_confirmed_red_ball` 明确判定不能作为正式识别证据。
- 测试组 CSV/JSON 位于对应 `reports/perception/test_records/` 目录。
- 完整 `selected_images/`、`selected_depth/*.npy`、`full_stack.log` 和原始
  `.pgm` 保存在本地及远程原始证据目录；Git 只提交精选图与结构化证据。

本轮主动环视机制已在官方复杂场景中连续执行三次并保持自动返航，但探索路线没有进入
红球可见区域。下一步应提高房间内部和楼梯可达区域的覆盖率，并在真实红球候选上验证
有效横移、严格多视角球体确认与独立 RGB-D 三维定位。
