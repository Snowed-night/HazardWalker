# 外部平台候选补丁

负责人：姜晨。此目录保存对官方 SimEnv 外部源码的候选补丁，**不会**由 HazardWalker 自动应用到共享
平台。平台组必须在官方 SimEnv 独立副本中执行 `git apply --check`、备份、编译和回归；验证失败可用
`git apply -R` 回滚。

2026-07-14 的隔离 ROS1 验收已经证明传感器和 RL 控制链路可以工作，但运行中的官方副本还包含人工
诊断期改动（例如激光 `<ray>` SDF 层级和 headless 控制入口）。在将这些改动导出为可干净应用的上游
补丁前，**不得把下列旧候选文件当作完整复现方案**；应先在干净官方副本比对源码、导出精确差异，再编译。
验收证据和已知差异见
`reports/platform/official_simenv_ros1_ros2/20260714_ros1_clean_direct_acceptance/` 与
`docs/environment/官方SimEnv_ROS1_ROS2双向适配整改_20260714.md`。

- `official_simenv_cmd_vel_async_spinner.patch`：让 `IOROS` 的订阅执行器随对象存活。它只覆盖回调
  生命周期，不覆盖 headless RL/物理控制的全部条件。
- `official_simenv_headless_camera_and_cmd_vel.patch`：候选的 headless 渲染参数入口；实际验收还依赖
  Xvfb/Mesa、物理未暂停和正确传感器 SDF。
- `official_simenv_headless_auto_rl.patch`：显式 `SIMENV_AUTO_RL=1` 时让 headless 控制器进入已有 RL
  模式；默认关闭，仍必须以独占场景的里程计验证。

官方 profile 当前使用 ROS1 容器内 `rosbridge_websocket` 与 ROS2 主机适配器；旧版 JSON 管道和
`ros1_bridge dynamic_bridge` 都不是已验证的官方传输方案。控制验证必须以实际里程计和截图/视频为准。
