# HazardWalker 仿真平台 — 接口测试报告

> 日期：2026-07-08 | Docker：`simenv_ros1:noetic-focal` | Gazebo 11

---

## 一、Docker ROS1 源接口

| 接口 | 类型 | 数据 | 说明 |
|------|------|------|------|
| `/Odometry_gazebo` | `nav_msgs/Odometry` | ✅ | 里程计 |
| `/trunk_imu` | `sensor_msgs/Imu` | ✅ | 躯干 IMU |
| `/livox/imu` | `sensor_msgs/Imu` | ✅ | Livox IMU |
| `/scan` | `sensor_msgs/LaserScan` | ✅ | 激光雷达（标准插件） |
| `/real_sense/rgb/image_raw` | `sensor_msgs/Image` | ✅ | RGB 图像 |
| `/real_sense/depth/image_raw` | `sensor_msgs/Image` | ✅ | 深度图像 |
| `/real_sense/depth/points` | `sensor_msgs/PointCloud2` | ✅ | 深度点云 |
| `/tf` | `tf2_msgs/TFMessage` | ✅ | 坐标变换 |
| `/cmd_vel` | `geometry_msgs/Twist` | ✅ | 速度控制 |

---

## 二、宿主机 `/hw/*` 桥接接口

| 接口 | 数据 | 说明 |
|------|------|------|
| `/hw/Odometry_gazebo` | ✅ | 里程计桥接正常 |
| `/hw/livox/imu` | ✅ | Livox IMU 桥接正常 |
| `/hw/scan` | ✅ | LaserScan 桥接正常 |
| `/hw/trunk_imu` | ✅ | 躯干 IMU 桥接正常 |
| `/hw/tf` | ✅ | TF 桥接正常 |
| `/hw/real_sense/rgb/image_raw` | ✅ | 桥接正常 |
| `/hw/real_sense/depth/image_raw` | ✅ | 桥接正常 |
| `/hw/real_sense/depth/points` | ✅ | 桥接正常 |
| `/hw/cmd_vel` | ✅ | 可发布控制 |

---

## 三、控制接口

| 接口 | 测试结果 |
|------|----------|
| `/set_door_state` | `accepted=True state=open` ✅ |
| `/call_elevator` | `accepted=True floor=0 state=idle` ✅ |
| `/hw/cmd_vel` | 可发布，机器人响应 ✅ |

---

## 四、桥接架构

```
Docker (ROS1 Noetic)              宿主机 (ROS2 Jazzy)
─────────────────────              ─────────────────────
/Odometry_gazebo ──┐
/trunk_imu ────────┤              ┌─→ /hw/Odometry_gazebo
/livox/imu ────────┤ docker_pipe  ├─→ /hw/trunk_imu
/scan ─────────────┼──→ JSON ──→  ├─→ /hw/livox/imu
/real_sense/* ─────┤  stdout      ├─→ /hw/scan
/tf ───────────────┘              ├─→ /hw/real_sense/*
                                  └─→ /hw/tf
```

---

## 五、使用方式

```bash
# 1. Docker 仿真
cd /home/hazard_platform/HazardWalker/ros2_ws/src/hazardwalker_platform
./auto_docker.sh up

# 2. 加载环境 + 启动桥接
cd /tmp
source /opt/ros/jazzy/setup.zsh
source ~/HazardWalker/ros2_ws/install/setup.zsh
nohup python3 ~/HazardWalker/ros2_ws/src/hazardwalker_platform/hw_bridge.py &>/tmp/hw_bridge.log &
sleep 3

# 3. 验证
ros2 topic list | grep /hw/
```

---

## 六、改动的文件

| 文件 | 改动 |
|------|------|
| `auto_noetic_headless.sh` | 添加 Xvfb 启动、移除 Docker 内 `/hw/` relay |
| `docker/ros1_bridge.sh` | 改用直接二进制路径 |
| `docker/Dockerfile` | 添加 xvfb 包 |
| `docker/docker-compose.yml` | 镜像切回 `noetic-focal` |
| `src/.../control_server.py` | 服务名去 `/hw/` 前缀 |
| `src/.../gazebo.xacro` | LiDAR 改用标准插件 |
| `hazardwalker_platform/hw_topic_relay_node.py` | 移除 `/scan`、`/livox/lidar2` |

新增：
| 文件 | 用途 |
|------|------|
| `docker_pipe.py` | Docker 内 JSON 管道 |
| `hw_bridge.py` | 宿主机桥接 |
| `scripts/hw_service_call.sh` | 门/电梯调用 |
