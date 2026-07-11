# 算法接入接口

本文说明参赛算法常用的控制输入、状态输出和传感器接口。

## 控制器状态切换

`auto.sh` 默认以前台方式启动 `junior_ctrl`。该控制器仍遵循 Unitree 原有交互流程：

- 键盘输入 `2`：站立。
- 键盘输入 `6`：切换到 RL 模式。
- RL 模式下订阅 `/cmd_vel`。

如果 `CONTROLLER_FOREGROUND=0`，`junior_ctrl` 会在后台运行，键盘状态切换通常不可用，日志写入 `logs/junior_ctrl.log`。

## 最小控制接口

| 接口 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 机器人速度指令输入 |

`/cmd_vel` 在 RL 模式下生效。

## 常用状态与传感器接口

| 接口 | 类型 | 说明 |
|------|------|------|
| `/Odometry_gazebo` | `nav_msgs/Odometry` | 仿真里程计输出 |
| `/scan` | `sensor_msgs/PointCloud2` | Livox Mid-360 点云数据 |
| `/livox/imu` | `sensor_msgs/Imu` | Livox 内置 IMU |
| `/trunk_imu` | `sensor_msgs/Imu` | 机体 IMU |
| `/camera/image_raw` | `sensor_msgs/Image` | 前视 RGB 图像 |
| `/real_sense/depth/points` | `sensor_msgs/PointCloud2` | 深度相机点云 |

传感器安装位姿、完整话题和坐标系见 [传感器与 ROS 话题](sensors-and-topics.md)。

## 结果输出约束

参赛算法应输出 `results/detected_danger.json`。不应读取 `results/danger_truth.json`。结果格式和评分方法见 [结果格式与评估方法](evaluation.md)。

## 控制周期

`junior_ctrl` 原始控制周期为 `0.002 s`，即 500 Hz。当前 `auto.sh` 默认设置：

```bash
UNITREE_CTRL_DT=0.004
```

默认值即 250 Hz，通常更适合 Gazebo GUI、随机楼栋、传感器和 RL 推理同时运行的比赛场景。如机器性能充足，可显式恢复 500 Hz：

```bash
UNITREE_CTRL_DT=0.002 ./auto.sh
```
