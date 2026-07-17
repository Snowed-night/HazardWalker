# hazardwalker_decision

决策包，负责任务状态机、结果汇总和后续主动决策。

## 当前职责

- 任务状态机
- 结果汇总
- 返航约束
- 重观察入口预留
- 后续 NBV 入口预留

## 当前最小节点

- `mission_state_machine_node`：订阅 `/hw/nav/state` 和 `/hw/perception/hazard_detections`，发布 `/hw/mission/state`，并在任务结束时写出结果 JSON。

正式 SimEnv 运行还会写 `results/detected_danger.json`：只导出状态为 `confirmed`、
坐标系为 `world` 且经过空间去重的红球轨迹。黄色 `reobserve`、圆柱/圆锥等
非球体候选和 `start` 坐标一律不能进入该官方评分文件。官方运行需令感知节点使用
`output_frame:=world`，并通过正式 ROS1 接口映射输入，不可读取 `danger_truth.json`。

当前节点只做最小状态收集和结果写入。

官方一键栈停止时，节点会避免对已经关闭的 ROS2 上下文二次 shutdown；正式结果仍只由真实导航 `FINISHED`
触发，不能由进程退出伪造。

## 后续扩展

- `result_builder.py`：结果结构统一生成。
- `mission_state_machine_node.py`：状态转移和结果写入。
- 后续可新增 `state_machine.py`、`target_selection.py`、`belief_map.py`。
