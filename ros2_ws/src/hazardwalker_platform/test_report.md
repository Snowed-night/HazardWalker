# HazardWalker 仿真平台 — 接口桥接测试报告

> 测试时间：2026-07-06  
> 测试环境：Docker `simenv_ros1_hazard_platform`（ROS1 Noetic）→ ros1_bridge（Foxy）→ 宿主机（ROS2 Jazzy）→ hw_topic_relay_node

---

## 一、测试架构

```
Docker (ROS1 Noetic)       ros1_bridge       Host (ROS2 Jazzy)     hw_topic_relay
─────────────────────      ───────────       ─────────────────     ───────────────
Gazebo → /Odometry_gazebo  ──→ bridge ──→   /Odometry_gazebo  ──→  /hw/Odometry_gazebo
Gazebo → /scan (PointCloud) → pointcloud2livox → /livox/Pointcloud2 ──→ bridge ──→ /hw/livox/Pointcloud2
Gazebo → /trunk_imu        ──→ bridge ──→   /trunk_imu        ──→  /hw/trunk_imu
Gazebo → /livox/imu        ──→ bridge ──→   /livox/imu        ──→  /hw/livox/imu
Gazebo → /tf               ──→ bridge ──→   /tf               ──→  /hw/tf
                                                                     /hw/cmd_vel  ← 算法发布
                                                                   hw_service_call.sh ← 门/电梯
```

---

## 二、Docker 内 ROS1 源接口测试

| 接口 | 类型 | 发布者 | 状态 | 说明 |
|------|------|--------|------|------|
| `/Odometry_gazebo` | `nav_msgs/Odometry` | `state_from_gazebo` | ✅ | 数据正常，100Hz |
| `/scan` | `sensor_msgs/PointCloud` | `gazebo` | ✅ | 旧类型，由 pointcloud2livox 转为 PointCloud2 |
| `/livox/Pointcloud2` | `sensor_msgs/PointCloud2` | `pointcloud2livox` | ✅ | 与 /scan 同一数据源，10Hz |
| `/livox/imu` | `sensor_msgs/Imu` | `gazebo` | ✅ | 数据正常，1000Hz |
| `/trunk_imu` | `sensor_msgs/Imu` | `gazebo` | ✅ | 数据正常，1000Hz |
| `/tf` | `tf2_msgs/TFMessage` | 多个节点 | ✅ | 数据正常 |
| `/cmd_vel` | `geometry_msgs/Twist` | — | ✅ | 话题存在 |
| `/set_door_state` | `building_generator_interfaces/SetDoorState` | `building_generator_classic_control` | ✅ | 可调用 |
| `/call_elevator` | `building_generator_interfaces/CallElevator` | `building_generator_classic_control` | ✅ | 可调用 |

### `/scan` 与 `/livox/Pointcloud2` 关系

```
Gazebo Livox 传感器
  ├─ 发布 /scan          (sensor_msgs/PointCloud, 旧类型)
  └─ pointcloud2livox 节点订阅 /scan
       └─ 发布 /livox/Pointcloud2  (sensor_msgs/PointCloud2, 标准类型)
```

**结论：两者是同一份数据**。`/scan` 是 Gazebo 原生 PointCloud 类型，ros1_bridge 无法桥接。`pointcloud2livox` 将其转为 PointCloud2 发布到 `/livox/Pointcloud2`，bridge 可正常桥接。**导航组使用 `/hw/livox/Pointcloud2` 即可。**

---

## 三、宿主机 ROS2 桥接接口测试

| 接口 | 发布者数 | 状态 | 说明 |
|------|----------|------|------|
| `/Odometry_gazebo` | 1 | ✅ | bridge 正常 |
| `/scan` | 0 | ❌ | PointCloud 类型无桥接模板 |
| `/livox/Pointcloud2` | 1 | ✅ | bridge 正常（替代 /scan） |
| `/livox/imu` | 1 | ✅ | bridge 正常 |
| `/trunk_imu` | 1 | ✅ | bridge 正常 |
| `/tf` | 1 | ✅ | bridge 正常 |
| `/cmd_vel` | — | ✅ | 可被 bridge 反向桥接 |

---

## 四、宿主机 `/hw/*` 中继接口测试

| 接口 | 发布者数 | 数据 | 状态 |
|------|----------|------|------|
| `/hw/Odometry_gazebo` | 3 | ✅ | ✅ 可用 |
| `/hw/livox/Pointcloud2` | 3 | ✅ | ✅ 可用（替代 /hw/scan） |
| `/hw/livox/imu` | 3 | ✅ | ✅ 可用 |
| `/hw/trunk_imu` | 3 | ✅ | ✅ 可用 |
| `/hw/tf` | 3 | ✅ | ✅ 可用 |
| `/hw/real_sense/rgb/image_raw` | 3 | ❌ | ⚠️ 源无数据（headless） |
| `/hw/real_sense/depth/image_raw` | 3 | ❌ | ⚠️ 源无数据（headless） |
| `/hw/real_sense/depth/points` | 3 | ❌ | ⚠️ 源无数据（headless） |
| `/hw/cmd_vel` | 可写入 | — | ✅ 测试通过 |

---

## 五、控制接口测试

### 速度控制 `/hw/cmd_vel`

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /hw/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.1}}'
```

**结果**：`publisher: publishing #1: Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.1))` ✅

### 门控服务

```bash
./scripts/hw_service_call.sh door main_entrance true
```

**结果**：`accepted=True state=open` ✅

### 电梯服务

```bash
./scripts/hw_service_call.sh elevator elevator_main 1 true
```

**结果**：`accepted=True floor=1 state=door_open` ✅

---

## 六、导航组可用接口汇总

| 用途 | 接口 | 类型 | 状态 |
|------|------|------|------|
| 里程计/定位 | `/hw/Odometry_gazebo` | topic | ✅ |
| LiDAR 点云 | `/hw/livox/Pointcloud2` | topic | ✅ |
| 躯干 IMU | `/hw/trunk_imu` | topic | ✅ |
| Livox IMU | `/hw/livox/imu` | topic | ✅ |
| 坐标变换 | `/hw/tf` | topic | ✅ |
| 速度控制 | `/hw/cmd_vel`（发布） | topic | ✅ |
| 开关门 | `hw_service_call.sh door` | 脚本 | ✅ |
| 呼叫电梯 | `hw_service_call.sh elevator` | 脚本 | ✅ |

---

## 七、已知问题

| # | 问题 | 影响 | 状态 |
|---|------|------|------|
| 1 | `/scan` PointCloud 类型无法桥接 | 不影响，用 `/hw/livox/Pointcloud2` 替代 | 已规避 |
| 2 | RealSense 相机无数据（headless Gazebo） | 视觉感知不可用 | 待修复 |
| 3 | `junior_ctrl` 缺 RL 模型文件 | `/cmd_vel` 无人响应 | 待模型文件 |

---

## 八、改动的文件清单

| 文件 | 改动 |
|------|------|
| `auto_noetic_headless.sh` | 删除 Docker 内 `/hw/` topic relay 块 |
| `docker/ros1_bridge.sh` | `set -u` → 移除；改用直接路径启动 bridge |
| `docker/scan_bridge.yaml` | **新增**：/scan 类型映射参数文件 |
| `src/.../control_server.py` | 服务名去 `/hw/` 前缀 |
| `scripts/hw_service_call.sh` | **新增**：宿主机门/电梯调用脚本 |
