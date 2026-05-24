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

当前节点只做最小状态收集和结果写入。

## 后续扩展

- `result_builder.py`：结果结构统一生成。
- `mission_state_machine_node.py`：状态转移和结果写入。
- 后续可新增 `state_machine.py`、`target_selection.py`、`belief_map.py`。
