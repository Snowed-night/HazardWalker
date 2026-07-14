# 官方容器只读预检

负责人：姜晨。执行日期：2026-07-14。环境：远程 `simenv_run` 容器。

## 观察结果

- ROS1 话题可见 `/Odometry_gazebo`、`/cmd_vel`、`/tf`、`/tf_static`；成功读取一帧里程计。
- `/unitree_gazebo_servo` 是 `/cmd_vel` 的真实订阅者，且节点发布 A1 各关节 `MotorCmd`，因此控制器链路
  的订阅端已经出现。
- `rostopic list` 中 `/camera/*` 与 `/real_sense/*` 数量为 0；不能启动 RGB-D 适配或三维定位验收。
- `/cmd_vel` 当时已有多路外部发布者。为避免干扰共享仿真，本次没有发送任何控制命令，故没有 1m 前进、转向
  或停止的真实运动结论。

## 结论

该预检仅证明“里程计与控制器订阅端可观测”，不证明机器人已经响应速度命令。下一次需由平台组独占或明确
协调场景后，按 `scripts/verify_official_simenv_ros1_adapter.sh --control` 执行 ROS1 直连 1m、转向、停止，
同时保存前后里程计、控制器日志和视频；相机问题应优先审查并验证 headless 渲染候选补丁。
