# official_simenv_20260710_dynamic_view_trial

官方 SimEnv 当前 `/hw/cmd_vel` 动态多视角能力的真实试验。临时生成标准红球后，连续运行感知节点，向机器人发布 `angular.z=0.2`、持续 2 秒的转向命令，并保存前后原生检测图与节点快照。

## 实测结论：平台控制阻塞

- 前后 `view_id` 都是 `pos:0.0:-2.0:0.4|yaw:90`。
- `distinct_view_count_after=1`，轨迹状态仍为 `tentative`。
- 独立 TF 检查中，世界相机位姿仅约 3 mm、0.004 rad 变化，远低于 2 秒转向应提供的有效第二视角。

因此当前实现正确地**没有**把单视角连续帧写成 confirmed；但也不能声称机器人已完成真实多视角识别或最佳视角搜索。平台需完成真实 A1/等效稳定控制迁移后，复用本目录脚本验证“候选 -> 真实转向/横移 -> 新 view_id -> distinct_view_count>=2 -> confirmed”。

前后图见 `images/before_after_collage.png`，原始快照和 `summary.json` 保留完整证据。
