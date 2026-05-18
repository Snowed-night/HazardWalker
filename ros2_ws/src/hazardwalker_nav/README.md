# hazardwalker_nav

导航探索包，负责未知环境建图、路径规划、自主探索和返航。

职责：

- SLAM / localization
- frontier exploration
- global and local planning
- stuck recovery
- return-home behavior

## 当前最小节点

- `waypoint_patrol_node`：订阅 `/hw/odom`，发布 `/hw/cmd_vel` 和 `/hw/nav/state`，用于第一阶段固定航点巡检和返航占位。

当前节点不是完整 Nav2 替代品，只用于最小闭环集成。

## 后续替换方向

1. 固定航点巡检。
2. Nav2 goal wrapper。
3. SLAM Toolbox 地图接入。
4. Frontier 提取和目标点评分。
5. 卡死检测、失败重规划和返航约束。
