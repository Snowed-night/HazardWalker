# hazardwalker_msgs

自定义消息、服务和动作定义包。

后续可定义：

- Hazard.msg
- HazardArray.msg
- MissionState.msg
- ReobserveTarget.srv

第一阶段节点为了降低集成成本仍使用 `std_msgs/String` JSON 打通链路；稳定后逐步迁移到本包消息。
