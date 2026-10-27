# hazardwalker_nav

导航包，负责固定航点、导航、探索和返航。

## 当前职责

- 固定航点巡检
- `Nav2` 入口预留
- Frontier 目标选择预留
- 卡死检测和返航控制

## 当前最小节点

- `waypoint_patrol_node`：订阅 `/hw/odom`，发布 `/hw/cmd_vel` 和 `/hw/nav/state`，用于第一阶段固定航点巡检和返航。

当前节点不是完整 `Nav2` 替代品，只用于最小闭环集成。

官方 SimEnv 一键栈收到正常停止信号时，节点会识别 ROS2 外部 shutdown 并安静退出；这不是航点完成，
也不能替代真实复杂楼宇中的自主探索验收。

## 后续扩展

- `waypoint_controller.py`：航点控制纯函数。
- `waypoint_patrol_node.py`：ROS 节点封装。
- 后续可新增 `frontier_explorer.py`、`nav2_goal_adapter.py`、`stuck_recovery.py`。
