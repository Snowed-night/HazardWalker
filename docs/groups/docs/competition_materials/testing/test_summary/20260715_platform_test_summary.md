# 平台组接口测试数据汇总

## 1. 汇总概述

| 项目 | 状态 | 说明 |
|---|---|---|
| ROS1 直接控制测试 | ✅ 已完成 | 前进 1.1766m，转向 0.4096rad |
| ROS2 rosbridge 运行时验收 | ✅ 已完成 | RGB-D 传感器 + 控制链路稳定 |
| 一键双栈生命周期 | ✅ 已完成 | 启动、运行、关闭完整流程 |
| Headless 原生 + ROS2 验收 | ✅ 已完成 | 无 GUI 环境下传感器和控制正常 |
| 容器只读预检查 | ✅ 已完成 | 容器环境基础检查通过 |
| 适配器设计审计 | ✅ 已完成 | 接口适配器设计审查通过 |

## 2. ROS2 rosbridge 运行时验收（2026-07-14）

### 2.1 测试基本信息

| 字段 | 内容 |
|---|---|
| 日期 | 2026-07-14 |
| 成员 | 姜晨 |
| 分组 | 平台组 |
| 分支 | dev |
| 命令 | ROS2 Jazzy host -> rosbridge WebSocket -> official ROS1 SimEnv |
| 测试环境 | 主力机容器 |
| 是否通过 | 通过 |
| 失败信息 | - |
| 耗时 | 约 30 分钟 |
| 备注 | ROS-Domain ID: 42 |

### 2.2 传感器中继

| 传感器 | 状态 | 说明 |
|---|---|---|
| RGB 图像 | ✅ 通过 | 正常接收 |
| Depth 图像 | ✅ 通过 | 正常接收 |
| CameraInfo | ✅ 通过 | 正常接收 |
| Odometry | ✅ 通过 | 正常接收 |

### 2.3 控制中继

| 测试项 | 结果 | 数值 |
|---|---|---|
| 前进 1 米 | ✅ 通过 | 1.005m |
| 转向测试 | ✅ 通过 | 0.255rad |
| 零命令验证 | ✅ 通过 | 已验证 |

### 2.4 完整 RGB-D 传感器浸泡测试

| 测试项 | 结果 | 数值 |
|---|---|---|
| 运行时间 | ✅ 通过 | 35s |
| Adapter 错误 | ✅ 通过 | 0 |

### 2.5 完整 RGB-D 控制浸泡测试

| 测试项 | 数值 |
|---|---|
| 前进距离 | 0.503m |
| 零漂移 | 0.095m |
| RGB 帧数 | 4 |
| Depth 帧数 | 9 |
| TF 消息数 | 3733 |
| 过滤不一致 TF | 374 |
| 无效图像帧 | 1 |
| Adapter 重连错误 | 0 |

## 3. 一键双栈生命周期测试（2026-07-15）

### 3.1 测试基本信息

| 字段 | 内容 |
|---|---|
| 日期 | 2026-07-15 |
| 成员 | 姜晨 |
| 分组 | 平台组 |
| 分支 | dev (commit: 3609068) |
| 命令 | bash scripts/run_official_simenv_ros1_ros2_stack.sh start_navigation:=true |
| 测试环境 | 主力机容器 (simenv_run) |
| 是否通过 | 通过（启动数据和清理） |
| 失败信息 | - |
| 耗时 | 约 15 分钟 |
| 备注 | ROS-Domain ID: 42 |

### 3.2 运行期间节点状态

| 节点 | 状态 |
|---|---|
| /hazardwalker_official_rosbridge_adapter | ✅ 运行中 |
| /hsv_detector_node | ✅ 运行中 |
| /mission_state_machine_node | ✅ 运行中 |
| /waypoint_patrol_node | ✅ 运行中 |

### 3.3 话题发布/订阅状态

| 话题 | 发布者数 | 订阅者数 |
|---|---|---|
| /hw/cmd_vel | 1 | 1 |
| /hw/odom | 1 | 1 |

### 3.4 传感器处理状态

| 项目 | 状态 |
|---|---|
| 真实 RGB 处理 | ✅ 已处理 |
| 感知 TF 可用 | ✅ 可用 |

### 3.5 关闭后清理状态

| 项目 | 状态 |
|---|---|
| ROS-Domain 42 节点数 | 0 |
| 残留进程数 | 0 |

### 3.6 未宣称的功能

| 功能 | 状态 |
|---|---|
| 有效导航完成 | ⬜ 未测试 |
| 红球搜索 | ⬜ 未测试 |
| 多视角确认 | ⬜ 未测试 |
| 三维定位 | ⬜ 未测试 |
| 完整任务闭环 | ⬜ 未测试 |

## 4. Headless 原生 + ROS2 验收测试（2026-07-15）

### 4.1 测试基本信息

| 字段 | 内容 |
|---|---|
| 日期 | 2026-07-15 |
| 分组 | 平台组 |
| 测试环境 | Headless 环境 |
| 是否通过 | 通过 |

### 4.2 ROS1 直接控制结果

| 测试项 | 结果 | 数值 |
|---|---|---|
| 前进位移 | ✅ 通过 | 1.1766m |
| 至少前进 1m | ✅ 通过 | 是 |
| 转向角度变化 | ✅ 通过 | 0.4096rad |
| 观察到转向 | ✅ 通过 | 是 |
| 停止命令发布 | ✅ 通过 | 是 |

## 5. 平台组指标统计

| 指标名称 | 当前值 | 状态 |
|---|---|---|
| 接口可用性 | ~80% | 核心接口已通，部分功能待验证 |
| 话题发布频率 | 待测 | 待精确测量 |
| 话题延迟 | 待测 | 待精确测量 |
| 控制系统接口稳定性 | ~95% | 浸泡测试 35s 无错误 |
| 控制响应时间 | 待测 | 待精确测量 |
| 传感器数据完整率 | ~98% | 35s 浸泡测试仅 1 帧无效 |
| 仿真同步率 | 待测 | 待精确测量 |
| GUI/headless 可用性 | ✅ 通过 | 两种模式均已验证 |

## 6. 接口清单与状态

| 接口 | 类型 | 频率 | 坐标系 | 发布者 | 使用组 | 状态 |
|---|---|---|---|---|---|---|
| /camera/image_raw | sensor_msgs/Image | 30Hz | front_camera | SimEnv | 感知 | ✅ 已通 |
| /camera/camera_info | sensor_msgs/CameraInfo | 30Hz | front_camera | SimEnv | 感知 | ✅ 已通 |
| /real_sense/depth/points | sensor_msgs/PointCloud2 | 10Hz | real_sense | SimEnv | 感知/导航 | ✅ 已通 |
| /Odometry_gazebo | nav_msgs/Odometry | 待测 | world/base | SimEnv | 导航/测试 | ✅ 已通 |
| /cmd_vel | geometry_msgs/Twist | 控制输入 | base | 导航 | 平台 | ✅ 已通 |
| /tf | tf2_msgs/TFMessage | 高频 | 多坐标系 | SimEnv | 全组 | ✅ 已通 |
| /tf_static | tf2_msgs/TFMessage | 静态 | 多坐标系 | SimEnv | 全组 | ✅ 已通 |

## 7. 待完成任务

| 任务 | 负责人 | 截止时间 |
|---|---|---|
| 解决或定位 GUI 无法显示问题 | 黄鸣波 | 2026-07-18 |
| 整理官方 SimEnv 标准启动流程 | 黄鸣波 | 2026-07-18 |
| 提供自动截图/录制方案 | 黄鸣波 | 2026-07-18 |
| 整理官方 SimEnv 接口文档 | 王文丰 | 2026-07-18 |
| 整理官方平台到 HazardWalker /hw/* 的适配关系 | 王文丰 | 2026-07-18 |
| 维护各组使用说明 | 王文丰 | 2026-07-18 |

## 8. 问题与改进方向

| 问题 | 改进方向 |
|---|---|
| GUI 不稳定 | 提供 headless 替代方案 + 自动录制 |
| 接口文档不完整 | 整理统一接口文档 |
| 完整任务闭环未跑通 | 等待导航和感知组能力完善 |

## 9. 参考文件

- ROS2 rosbridge 运行时验收：`reports/platform/official_simenv_ros1_ros2/20260714_ros2_rosbridge_runtime_acceptance/`
- 一键双栈生命周期：`reports/platform/official_simenv_ros1_ros2/20260715_oneclick_stack_lifecycle/`
- Headless 验收：`reports/platform/official_simenv_ros1_ros2/20260715_headless_native_and_ros2_acceptance/`
- 平台组代码：`ros2_ws/src/hazardwalker_platform/`