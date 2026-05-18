# hazardwalker_decision

主动决策包，负责全局搜索、主动重观察和任务状态机。

职责：

- task state machine
- next-best-view search
- active re-observation
- return constraint
- exploration completion decision

## 当前最小节点

- `mission_state_machine_node`：订阅 `/hw/nav/state` 和 `/hw/perception/hazard_detections`，发布 `/hw/mission/state`，并在任务结束时写出结果 JSON。

当前节点只做最小状态收集和结果写入。后续应扩展为真正的任务状态机。

## 后续替换方向

1. 固定航点任务调度。
2. 完整状态机：`IDLE / EXPLORING / NAVIGATING / REOBSERVING / REPLANNING / RETURNING / FINISHED / FAILED`。
3. 接收感知模块的 `tentative / confirmed / rejected` 目标。
4. 加入重观察和返航时间约束。
5. 后期加入 NBV 和危险源信念地图。
