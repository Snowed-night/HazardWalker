# hazardwalker_msgs

自定义消息定义包。

## 当前文件

- `Hazard.msg`：单个危险源数据。
- `HazardArray.msg`：危险源数组。
- `MissionState.msg`：任务状态消息。

## 当前使用状态

第一阶段节点为了降低集成成本仍使用 `std_msgs/String` JSON 打通链路；稳定后逐步迁移到本包消息。

## 后续扩展

- 如果后续需要重观察目标或任务控制服务，再新增对应 `.srv` 文件。
