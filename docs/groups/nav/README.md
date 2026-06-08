# 导航组文档

负责范围：

- 固定航点控制接口维护。
- SLAM Toolbox 建图和定位。
- Nav2 goal 接入、路径规划和局部避障。
- Frontier 探索目标生成。
- 返航触发、卡死判定和失败恢复。

近期重点：

1. 阅读 `waypoint_controller.py` 和 `waypoint_patrol_node.py`。
2. 明确固定航点到 Nav2 goal 的替换边界。
3. 给出 Frontier + SLAM Toolbox + Nav2 的最小接入方案。
4. 设计返航成功判定和卡死判定规则。
