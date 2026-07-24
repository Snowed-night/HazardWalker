# 平台与仿真组文档

平台组负责官方 SimEnv 接入、ROS1↔ROS2 适配、`/hw/*` 公共接口、传感器可用性与运行稳定性。

## 文档边界

- 全员日常启动、接入和安全边界以 [官方 SimEnv 平台环境使用手册](../../guidebook/官方SimEnv平台环境使用手册.md) 为准。
- 导航控制与 W/S/A/D/K 键盘验收以
  [官方 SimEnv 控制链路与键盘测试](../nav/官方SimEnv控制链路与键盘测试.md) 为准；该文档由负责人修改。
- 官方原始传感器、评测和接口资料保留在 [`ros2_ws/src/hazardwalker_platform/docs/`](../../../ros2_ws/src/hazardwalker_platform/docs/)，不得改写为团队真值或私有接口。
- 本目录仅保存平台组提交的设计、验收和整改证据；不重复维护一份“平台使用手册”。

## 历史记录

- [2026-07-14 官方 SimEnv ROS1↔ROS2 双向适配整改](history/官方SimEnv_ROS1_ROS2双向适配整改_20260714.md)：历史整改与验收记录，不代表当前共享环境已自动上线。
- [2026-07-10 ROS2 RGB-D 与 world 对齐修复](history/simenv_ros2_rgbd_world_alignment_fix_20260710.md)：Harmonic 临时部署记录，不适用于当前官方平台。

新增记录须说明 Git 版本、运行环境、输入输出话题、验证命令、通过范围和未解决风险。
