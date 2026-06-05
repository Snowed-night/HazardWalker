# hazardwalker_msgs

自定义消息定义包。

## 当前文件

- `Hazard.idl`：单个危险源数据，当前 ROS 2 构建使用的接口文件。
- `HazardArray.idl`：危险源数组，当前 ROS 2 构建使用的接口文件。
- `MissionState.idl`：任务状态消息，当前 ROS 2 构建使用的接口文件。
- `Hazard.msg` / `HazardArray.msg` / `MissionState.msg`：保留为消息语义参考。

当前主力机仓库路径包含中文目录名，ROS 2 Jazzy 的 `.msg -> .idl` 适配链路在该路径下存在解析问题，因此本包直接使用 `.idl` 参与构建。

## 当前使用状态

第一阶段节点为了降低集成成本仍使用 `std_msgs/String` JSON 打通链路；稳定后逐步迁移到本包消息。

## 后续扩展

- 如果后续需要重观察目标或任务控制服务，再新增对应 `.srv` 文件。
