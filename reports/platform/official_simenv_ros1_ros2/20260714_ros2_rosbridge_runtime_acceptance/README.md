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

## 文件

- `hw_runtime_sensor_check.json`：完整 RGB-D、内参、里程计和适配器状态。
- `hw_forward_one_meter.json`：隔离 ROS2 `/hw/cmd_vel` 前进 1 m 与零速度尾迹。
- `hw_control_acceptance.json`：转向、零速度保持和控制转发计数。
- `hw_full_rgbd_soak_check.json`：修复分片边界后全量 RGB-D 空载连续 35 s 的实际 ROS2 接收与零错误日志检查。

## 限制

修复同一订阅 id 的新帧覆盖边界后，全量 RGB-D 已在空载连续 35 s 内无适配器重连/错误，并收到完整
图像；但全量 RGB-D 与控制同时长期运行仍需压力复测。本轮 1 m 控制为隔离控制验收，显式关闭图像
中继以避免把图像吞吐问题混入运动结论。尚未运行导航、感知和决策完整闭环。
