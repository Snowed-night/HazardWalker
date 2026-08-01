# 导航组 Phase 5.3 SLAM 建图验证报告

> 日期：2026-07-22 | 分支：`feature/nav` | 提交：`d22f2aa` | 机器：hxbl | 账号：hazard_nav

---

## 1. 验证目标

在官方 SimEnv（Docker `simenv_ros1_hazard_platform`）上完成以下验证：

| 编号 | 目标 | 状态 |
|------|------|------|
| V1 | ROS1→ROS2 rosbridge 双向适配器正常启动并转发传感器数据 | ✅ |
| V2 | scan_imu_localizer 合法定位节点输出 odom→base TF | ✅ |
| V3 | SLAM Toolbox 在线异步建图产出 /map（OccupancyGrid） | ✅ |
| V4 | TF 链路 map→odom→base 完整闭合 | ✅ |
| V5 | 运动控制 /hw/cmd_vel → Docker /cmd_vel 链路 | ⏳ 待平台组修复控制器 |

---

## 2. 环境配置

| 项 | 值 |
|---|---|
| 机器 | hxbl (Ubuntu 24.04) |
| ROS 2 | Jazzy |
| 账号 | hazard_nav |
| ROS_DOMAIN_ID | 17 |
| Docker 容器 | simenv_ros1_hazard_platform |
| 容器内 ROS | Noetic + Gazebo Classic |
| 容器内 rosbridge | `/rosbridge_websocket` on ws://127.0.0.1:9090 |
| 分支 | `feature/nav` |
| 代码提交 | `d22f2aa` |

---

## 3. 架构与数据流

```
┌── Docker 容器 (simenv_ros1_hazard_platform) ──┐
│                                                │
│  Gazebo Classic                                │
│    │  Unitree A1 + Mid-360 LiDAR + RealSense   │
│    ▼                                            │
│  ROS1 话题                                      │
│    ├─ /scan (LaserScan, ~10Hz)                 │
│    ├─ /trunk_imu (Imu, ~1kHz)                  │
│    ├─ /hazardwalker/odom (Odometry, 20Hz)      │
│    ├─ /real_sense/rgb/image_raw                │
│    ├─ /real_sense/depth/image_raw              │
│    ├─ /tf, /tf_static                          │
│    └─ /clock                                    │
│         │                                       │
│    ┌────▼────────────┐                         │
│    │ rosbridge_ws    │ ws://127.0.0.1:9090     │
│    └─────────────────┘                         │
└──────────────────────┬──────────────────────────┘
                       │ WebSocket
┌──────────────────────▼──────────────────────────┐
│  ROS2 主机 (hxbl)                                │
│                                                  │
│  official_simenv_rosbridge_ros2_adapter_node     │
│    ├─ /hw/scan (LaserScan)                      │
│    ├─ /hw/trunk_imu (Imu)                       │
│    ├─ /hw/odom (Odometry)                       │
│    ├─ /hw/camera/image_raw                      │
│    ├─ /hw/camera/depth_image                    │
│    ├─ /hw/livox/imu                             │
│    ├─ /hw/lidar/points                          │
│    ├─ /tf, /tf_static                           │
│    └─ /clock                                     │
│         │                                        │
│    ┌────▼──────────────────┐                    │
│    │ scan_imu_localizer    │ scan+IMU 合法定位   │
│    │  → odom→base TF       │                    │
│    │  → /hazardwalker/slam/odometry             │
│    └───────────────────────┘                    │
│         │                                        │
│    ┌────▼──────────────────┐                    │
│    │ slam_toolbox          │ 在线异步建图        │
│    │  → /map (OccupancyGrid)                    │
│    │  → map→odom TF         │                    │
│    └───────────────────────┘                    │
│                                                  │
│  TF 链路: map → odom → base                      │
│  /hw/cmd_vel → rosbridge → Docker /cmd_vel       │
└──────────────────────────────────────────────────┘
```

---

## 4. 复现步骤

### 4.1 前置条件

- hxbl 上已登录 `hazard_nav`
- Docker 容器 `simenv_ros1_hazard_platform` 在运行
- 容器内 `/rosbridge_websocket` 在线
- 代码已切换到 `feature/nav` 分支并编译

```bash
# 确认容器
docker inspect -f '{{.State.Running}}' simenv_ros1_hazard_platform
# 输出: true

# 确认代码
cd ~/HazardWalker && git branch --show-current
# 输出: feature/nav
```

### 4.2 编译

```bash
cd ~/HazardWalker/ros2_ws
cd /tmp
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/HazardWalker/ros2_ws/install/setup.bash 2>/dev/null

cd ~/HazardWalker/ros2_ws
colcon build --symlink-install --packages-select \
  hazardwalker_platform hazardwalker_nav hazardwalker_perception \
  hazardwalker_decision hazardwalker_bringup
```

预期输出：

```text
Summary: 5 packages finished
```

### 4.3 启动 rosbridge 适配器（终端 1）

> ⚠️ Jazzy 的 `setup.bash` 有 CWD bug —— 必须 `cd /tmp` 再 source，否则报错：
> `no such file or directory: ./setup.sh`

```bash
cd /tmp
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/HazardWalker/ros2_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=17

python3 ~/HazardWalker/scripts/official_simenv_rosbridge_ros2_adapter_node.py \
  --ros-args \
  -p rosbridge_url:="ws://127.0.0.1:9090" \
  -p enable_cmd_vel_relay:=false \
  -p enable_image_relay:=true \
  -p enable_scan_relay:=true \
  -p enable_lidar_relay:=true \
  -p enable_tf_relay:=true 2>&1 | grep -v "\[DEBUG\]"
```

预计日志输出含 `[hazardwalker_official_rosbridge_adapter]`。

### 4.4 验证适配器输出（终端 2）

```bash
cd /tmp
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/HazardWalker/ros2_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=17

ros2 topic list | grep '/hw/'
```

预期输出（14 个话题）：

```text
/hw/camera/camera_info
/hw/camera/depth_camera_info
/hw/camera/depth_image
/hw/camera/image_raw
/hw/cmd_vel
/hw/lidar/points
/hw/livox/imu
/hw/odom
/hw/platform/official_simenv_adapter_status
/hw/scan
/hw/scan_raw
/hw/trunk_imu
```

### 4.5 启动 SLAM 业务栈（终端 2）

```bash
cd /tmp
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/HazardWalker/ros2_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=17

ros2 launch hazardwalker_bringup official_simenv_business.launch.py \
  start_slam:=true \
  slam_backend:=slam_toolbox \
  start_navigation:=false \
  start_perception:=true \
  start_decision:=true \
  use_sim_time:=true
```

预期输出：启动 `scan_imu_localizer_node`（日志含 `Legal scan/IMU odometry ready`）、`slam_toolbox`（lifecycle autostart → active）、`hsv_detector_node`、`mission_state_machine_node`。

### 4.6 验证 SLAM 建图（终端 3）

```bash
cd /tmp
source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/HazardWalker/ros2_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=17

# 1) 节点列表
ros2 node list

# 2) /map 频率
ros2 topic hz /map

# 3) /map 内容
timeout 10 ros2 topic echo /map --once 2>/dev/null | head -30

# 4) TF 链路
timeout 3 ros2 run tf2_ros tf2_echo odom base 2>/dev/null | head -10

# 5) SLAM 里程计
timeout 5 ros2 topic echo /hazardwalker/slam/odometry --once 2>/dev/null | head -20
```

---

## 5. 验证结果

### 5.1 节点列表

```
/hazardwalker_official_rosbridge_adapter
/hazardwalker_scan_imu_localizer
/hsv_detector_node
/mission_state_machine_node
/slam_toolbox
```

### 5.2 /map 输出

```
Type: nav_msgs/msg/OccupancyGrid
Publisher count: 1
Subscription count: 1
average rate: 0.500 Hz
  resolution: 0.05 m
  size: 346 × 282 cells
  frame_id: map
```

### 5.3 TF 链路

```
map ──→ odom ──→ base
 (SLAM)   (scan_imu_localizer)
  ✅       ✅
```

| TF 边 | 来源 | 状态 |
|--------|------|------|
| map→odom | slam_toolbox | ✅ 实时更新 |
| odom→base | scan_imu_localizer_node | ✅ 实时更新 |

### 5.4 里程计

```
frame_id: odom → child_frame_id: base
position: [0.196, -0.164, 0.000]
covariance[0] = 0.04 (tracking 模式)
```

### 5.5 SLAM 节点订阅

```
/slam_toolbox
  Subscribers:
    /clock      ✅
    /hw/scan    ✅
    /map        ✅
  Publishers:
    /map        ✅
    /tf         ✅ (map→odom)
```

---

## 6. 运动控制验证（部分完成）

### 6.1 控制链路验证

ROS2 `/hw/cmd_vel` → rosbridge → Docker `/cmd_vel` 链路已确认通：

```bash
# 终端 3 发布
ros2 topic pub --rate 5 /hw/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.35}, angular: {z: 0.0}}" &

# 终端 3 验证容器内接收
docker exec simenv_ros1_hazard_platform bash -lc \
  'source /opt/ros/noetic/setup.bash; timeout 3 rostopic echo /cmd_vel -n 1'
```

确认容器内收到 `linear.x: 0.35`。

### 6.2 当前阻塞

容器内 `junior_ctrl` 未订阅 `/cmd_vel`（`rostopic info /cmd_vel` → `Subscribers: None`），
导致 `/cmd_vel` 消息到达但 A1 不响应。控制器日志显示：

```
[HEADLESS_FSM] mode=move_base auto_rl=1
```

需要平台组（姜晨）确认控制器 RL 模式状态或重启控制器。

---

## 7. 已知问题与解决

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| Jazzy `setup.bash` 报 `no such file or directory: ./setup.sh` | Jazzy CWD bug | 必须 `cd /tmp` 再 source |
| `pip3 install` 报 `externally-managed-environment` | Ubuntu 24.04 PEP 668 | 加 `--break-system-packages` |
| `ros2 topic echo --once` 卡住 | `use_sim_time=true` 下时间戳不匹配 | 用 `timeout` 防护 |
| bash 适配器脚本静默退出 | `set -e` + setup.bash 报错 | 绕过脚本，直接 Python 启动 |
| `junior_ctrl` 不响应 /cmd_vel | 控制器不在 RL 模式 | **待平台组修复** |

---

## 8. 关键文件路径

| 文件 | 路径 |
|------|------|
| rosbridge 适配器 | `scripts/official_simenv_rosbridge_ros2_adapter_node.py` |
| 业务启动文件 | `ros2_ws/src/hazardwalker_bringup/launch/official_simenv_business.launch.py` |
| SLAM 参数 | `ros2_ws/src/hazardwalker_nav/config/slam_toolbox_online_async.yaml` |
| 合法定位器 | `ros2_ws/src/hazardwalker_perception/hazardwalker_perception/scan_imu_localizer_node.py` |
| Frontier 探索 | `ros2_ws/src/hazardwalker_nav/hazardwalker_nav/frontier_explorer_node.py` |

---

## 9. 下一步

1. **平台组修复控制器** → A1 响应 /cmd_vel
2. **手动控制测试** → 前进/转向/停止 验证狗实际运动
3. **Frontier 探索联调** → 启动 `start_navigation:=true nav_mode:=frontier`
4. **Nav2 安装** → 安装 ros-jazzy-navigation2 后开启诊断导航

---

> 验证人：hazard_nav @ hxbl
> 复核人：待定
> 更新：2026-07-22 15:50
