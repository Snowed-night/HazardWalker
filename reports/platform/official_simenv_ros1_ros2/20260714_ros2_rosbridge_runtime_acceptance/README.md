# 官方 SimEnv ROS2 rosbridge 运行验收

负责人：姜晨。平台组协作。

本目录保存宿主机 ROS2 Jazzy 与隔离官方 ROS1 SimEnv 的实际互通记录。ROS2 适配器运行在宿主机，
通过 Docker 映射的 `9091 -> 9090` 连接容器内 rosbridge；因 rosbridge 校验 Host 头，使用
`rosbridge_host_header=127.0.0.1:9090`。

## 已验证

- ROS1 `/Odometry_gazebo` 到 ROS2 `/hw/odom`，以及 RGB、深度、两组 CameraInfo 到 `/hw/camera/*`
  均收到完整尺寸和字节数。
- 在禁用高带宽图像订阅的隔离控制轮次中，ROS2 `/hw/cmd_vel` 经适配器到 ROS1 `/cmd_vel`，以 ROS2
  `/hw/odom` 记录前进 **1.0051 m**；状态中 `forwarded_cmd_count=1120`，最后发送零速度。
- 另一控制轮记录到 ROS2 转向 **0.2553 rad**；ROS1 端监控到非零/零速度与关节命令。
- 在**全量 RGB-D、两组内参与里程计同时转发**时，ROS2 `/hw/cmd_vel` 使 A1 连续移动
  **0.5118 m**；其后零速度的 2 s 漂移为 **0.0524 m**。该轮收到 RGB **120** 帧、深度
  **119** 帧，适配器无重连、无无效图像帧。原始 640×480 图像经过 rosbridge 以 **500 ms**
  节流，避免下一帧覆盖未完成的分片。

## 文件

- `hw_runtime_sensor_check.json`：完整 RGB-D、内参、里程计和适配器状态。
- `hw_forward_one_meter.json`：隔离 ROS2 `/hw/cmd_vel` 前进 1 m 与零速度尾迹。
- `hw_control_acceptance.json`：转向、零速度保持和控制转发计数。
- `hw_full_rgbd_soak_check.json`：修复分片边界后全量 RGB-D 空载连续 35 s 的实际 ROS2 接收与零错误日志检查。
- `hw_concurrent_rgbd_control_acceptance.json`：隔离容器内全量 RGB-D 与 ROS2 控制并发的里程计、
  帧数、适配状态和停止尾迹。

## 限制

全量 RGB-D 已完成空载 35 s 和与控制并发的实际回归。当前原始图像保守节流为 2 Hz，保证控制、
里程计与分片传输不互相打断；更高帧率、ROS2 导航/感知/决策完整闭环仍未运行，不能据此宣称任务完成。
