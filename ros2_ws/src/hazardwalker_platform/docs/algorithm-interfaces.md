# 算法接入接口

本文说明参赛算法常用的控制输入、状态输出和传感器接口。

本轮控制链路与键盘规范修改人：负责人（姜晨）。

## 控制器状态切换

正式 `auto_docker.sh up` 通过 `auto.sh` 在后台启动 `junior_ctrl`，由镜像内的 `expect` 发送 Unitree 原有
交互序列并检查日志：

- `2`：站立。
- `6`：切换到 RL 模式。
- 正式模式先确认日志出现 `[HEADLESS_FSM] mode=move_base auto_rl=1`，再解除物理暂停；
  随后必须出现 `Switched from fixed stand to RL`，并由启动器的低速探针证明真实位移。
  `/cmd_vel` 的唯一订阅者只证明 ROS 图连通，不能单独作为控制生效证据。

需要人工排障时可显式设置 `CONTROLLER_FOREGROUND=1 SIMENV_AUTO_RL=0`，再直接执行 `./auto.sh`。该模式不属于
Docker 正式流程，也不能替代 RL 就绪验收。

## 最小控制接口

| 接口 | 类型 | 说明 |
|------|------|------|
| `/hw/cmd_vel` | `geometry_msgs/Twist` | 导航、键盘与业务层的唯一控制输入 |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS1 容器内控制输入，由适配器转发，不供业务节点直连 |

正式链路为 `/hw/cmd_vel → ROS2 适配器 → /cmd_vel → junior_ctrl(RL)`。`/cmd_vel`
只有在 RL 模式下生效，适配器必须显式设置 `enable_cmd_vel_relay=true`。同一轮只能有一个最终
速度发布者。

负责人维护的键盘工具：

```bash
ros2 run hazardwalker_platform keyboard_control_node
```

按键为 `w` 前进、`s` 后退、`a` 左转、`d` 右转、`k` 立即停止。工具带短时命令超时和退出
零速度；完整步骤见
[导航组控制链路与键盘测试](../../../../docs/groups/nav/官方SimEnv控制链路与键盘测试.md)。

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

`junior_ctrl` 当前验收控制周期为 `0.002 s`，即 500 Hz。`auto.sh` 默认设置：

```bash
UNITREE_CTRL_DT=0.002
```

`0.004 s` 曾使 RL 动作与仿真动力学响应不稳定，因此不再作为正式默认值。出现
`absoluteWait` warning 时先关闭 GUI、降低渲染或点云负载；修改控制周期后必须重新通过启动物理探针和完整控制验收。
