# 官方 SimEnv ROS1 隔离直连控制验收

负责人：姜晨。平台组协作。

本目录记录 2026-07-14 在独立 Docker 网络容器 `simenv_clean_acceptance` 中完成的官方 ROS1 原生验收。
隔离是必要条件：共享 host 网络中的另一个遗留容器持续向同一个 ROS master 发布
`/cmd_vel.angular.z=-0.8`，会使停止与变速实验失真。本目录的控制容器不接入该共享网络，且验证前
`/cmd_vel` 无外部消息。

## 结论

- ROS1 `/cmd_vel` 直接驱动官方 `junior_ctrl` 的 RL 控制入口，机器人里程计前进 **1.0014 m**。
- 原地转向达到 **0.2540 rad**。
- 发布零速度后的初始短暂策略尾迹不用于停止判定；收敛后，两个连续 3 s 窗口的位移均约
  **0.00031 m**、偏航变化均小于 **0.00018 rad**，表明零速度保持有效。
- ROS1 原生 RGB（`640×480 rgb8`）、深度（`640×480 32FC1`）、相机内参、里程计和 `/scan`
  均收到真实消息。该结论仅证明 ROS1 原生层；**尚不证明 ROS2 `/hw/*` 适配或业务闭环通过**。

## 文件

- `clean_ros1_direct_acceptance.json`：前进、转向和初始零速窗口的原始里程计数值。
- `clean_ros1_stop_hold.json`：零速度收敛后的两个连续保持窗口。
- `clean_ros1_sensor_contract.json`：RGB-D、内参、里程计和雷达的 ROS1 实际契约。
- `clean_ros1_rgb.png`、`clean_ros1_depth.png`：同一轮官方 RealSense 原生图像证据。
- `summary.json`、`testing_record_platform.csv`：供测试组汇总的结论和表格。

## 复现前提

1. 不要与其他使用 host 网络且指向同一 ROS master 的容器并行发布 `/cmd_vel`。
2. 先用 `START_CONTROLLER=0` 启动官方场景，再单独运行 `SIMENV_AUTO_RL=1` 的 `junior_ctrl`；
   不把未修复的 MoveBase 路径当作已验收入口。
3. 对外共享环境必须先清除旧发布者或为每次验收建立隔离网络，并记录控制发布者列表。
