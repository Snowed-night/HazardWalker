# Minimal Demo Development Guide

本文档说明第一阶段最小闭环脚手架的使用方式。当前代码不是最终算法版本，而是为了提前固定模块接口、启动方式和算法嵌入位置。

## 1. Current Minimal Nodes

当前最小链路包含：

| Package | Node | Role |
|---|---|---|
| `hazardwalker_platform` | `fake_platform_node` | 发布 `/hw/camera/image_raw`、`/hw/odom`、`/tf`，接收 `/hw/cmd_vel` |
| `hazardwalker_perception` | `hsv_detector_node` | 订阅图像，检测红色区域，发布危险源 JSON |
| `hazardwalker_nav` | `waypoint_patrol_node` | 发布 `/hw/cmd_vel`，模拟固定航点巡检和返航 |
| `hazardwalker_decision` | `mission_state_machine_node` | 订阅导航状态和检测结果，写出结果 JSON |
| `hazardwalker_bringup` | `minimal_demo.launch.py` | 启动最小链路 |

## 2. Build

在 Ubuntu 24.04 + ROS 2 Jazzy 环境中：

```bash
./scripts/build.sh
```

等价命令：

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## 3. Run

```bash
./scripts/run_minimal_demo.sh
```

或：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch hazardwalker_bringup minimal_demo.launch.py
```

运行结束后，结果文件写入：

```text
reports/run_results/<timestamp>_result.json
```

## 4. Algorithm Replacement Points

### Platform

后续 Gazebo 或官方平台接入时，替换：

```text
hazardwalker_platform/fake_platform_node.py
```

但保持输出接口不变：

```text
/hw/camera/image_raw
/hw/camera/camera_info
/hw/lidar/points
/hw/odom
/tf
/hw/cmd_vel
```

### Perception

当前 `hsv_detector_node` 只做最小 HSV 检测，并用简化坐标占位。后续应替换为：

```text
图像 HSV 检测
点云 ROI 提取
TF 坐标转换
多帧确认
空间去重
```

### Navigation

当前 `waypoint_patrol_node` 不是真正 Nav2。后续替换顺序：

```text
固定航点 -> Nav2 goal wrapper -> Frontier -> NBV
```

### Decision

当前状态机只收集结果。后续逐步加入：

```text
EXPLORING
REOBSERVING
REPLANNING
RETURNING
FAILED
```

## 5. Minimal Acceptance

验收标准见：

```text
docs/minimal_demo_acceptance.md
```

当前脚手架的作用是让团队先看到 topic、节点、launch 和结果文件如何串起来。真正的 Gazebo/官方平台接入后，应优先保持这些内部接口不变。
