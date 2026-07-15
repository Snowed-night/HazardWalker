# 官方 SimEnv 一键业务栈生命周期验收

负责人：姜晨。

本轮在独占的 `simenv_run` 与 ROS 域 42 中，以
`bash scripts/run_official_simenv_ros1_ros2_stack.sh start_navigation:=true` 启动官方 ROS1→ROS2
适配器、感知、决策和导航。为验证可重复接入，测试前先仅回收负责人临时集成目录的遗留业务进程，
不触碰 `/home/hazard_nav` 的其他成员环境。

- 运行中有且仅有 1 个 `waypoint_patrol_node` 发布 `/hw/cmd_vel`；官方适配器是唯一订阅者。
- `/hw/odom` 有且仅有 1 个官方适配器发布者；`waypoint_patrol_node` 为订阅者。
- 真实 RGB 帧进入感知节点，日志记录 640×480 `rgb8`，并完成 `tf=True` 的处理。
- 对外层栈进程组发送 `SIGTERM` 后，域 42 中无节点、负责人临时目录下无适配器/导航/感知/决策残留。

本目录证明启动、数据和清理链路可复现，**不**证明固定航点已经完成有效导航，更不代表红球搜索、
多视角确认、三维定位或整场闭环完成。原始启动尾日志位于远程
`/tmp/hw_stack_lifecycle_20260715.log`，其中含节点启动与感知帧处理记录；该临时日志不提交仓库。
