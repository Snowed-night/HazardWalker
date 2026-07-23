# 官方 SimEnv Docker 启动链路真实验收

日期：2026-07-23。范围：官方 ROS1 Noetic Docker、ROS1 rosbridge、ROS2 Jazzy 适配器；不包含 SLAM、导航、感知业务结论。

## 目的

验证正式 `auto_docker.sh up -> auto.sh` 链路不再依赖容器内手工安装或手工启动，并确认 RGB-D、诊断里程计、控制与 ROS2 `/hw/*` 可以在同一轮环境中工作。

## 环境与方法

- 无缓存重建镜像：`simenv_ros1:noetic-focal`，镜像 ID `61b42683046b`。
- 正式容器：`simenv_ros1_hazard_platform`；验收前独占检查仅发现该一个运行中的 SimEnv 容器。
- 启动入口：`auto.sh`。镜像内固化 `ros-noetic-rosbridge-server` 和 `expect`；启动时自动生成 `/Odometry_gazebo -> /hazardwalker/odom` 最新值中继。
- 控制器：`SIMENV_AUTO_RL=1`、`SIMENV_HEADLESS_MODE=move_base`。日志确认 `HEADLESS_FSM mode=move_base auto_rl=1` 和 `Switched from fixed stand to RL`。
- ROS2 端：Jazzy + `websocket-client` 独立 venv，连接 ROS1 `ws://127.0.0.1:9090`。验证时只启用本轮唯一适配器。

## 结果

- ROS1 节点存在：`/unitree_gazebo_servo`、`/hazardwalker_odom_relay`、`/rosbridge_websocket`、`/rosapi`。
- 原生 RGB-D 均有真实消息，`/real_sense/rgb/image_raw` 实测约 20 Hz；修复后的 `/hazardwalker/odom` 固定为 20.000 Hz。
- ROS2 实测收到 `/hw/odom`、`/hw/camera/image_raw`、`/hw/camera/depth_image`、两路内参、`/hw/scan`、`/hw/trunk_imu` 与 `/tf`。
- ROS2 零速度命令已穿透到 ROS1 `/cmd_vel`；控制订阅者唯一为 `/unitree_gazebo_servo`。
- 独占直连运动阈值验收：直行 `1.3557 m`、转向 `0.2341 rad`、已发送停止命令，均达到手册的动作阈值。

## 结论与边界

Docker 启动链路、ROS1↔ROS2 数据链路和 A1 控制链路通过本轮真实验收，可按更新后的手册供各组接入。此处的 `/hw/odom` 是平台诊断中继，不能作为正式 SLAM 位姿或比赛定位来源。本轮没有自动录像，故运动证据的媒体完整性仍需后续补录；该缺口不影响“平台链路可启动、可发布传感器、可控制”的结论。
