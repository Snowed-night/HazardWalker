# 导航组 Phase 5.2 进度总结

> 日期：2026-07-17 | 分支：`feature/nav` | 提交：`848e90a`

---

## 任务完成情况

| 任务 | 状态 | 说明 |
|------|------|------|
| 任务1：基础运动控制 | ✅ 完成 | 机器人实际移动确认，10s 位移 0.3m |
| 任务2：SLAM 建图 | ⏳ 就绪 | 配置和启动文件完成，/scan ranges 为空待解决 |
| 任务3：Frontier 探索 | ⏳ 就绪 | 代码完成，等 SLAM /map 输出后联调 |

---

## 数据链路

```
Docker simenv_ros1_hazard_nav (ROS1)
    │  /Odometry_gazebo, /scan, /tf, /livox/imu, /trunk_imu
    │
    ▼  hw_bridge.py (stdin pipe → ROS2 /hw/*)
    │
    ├─ /hw/Odometry_gazebo (500Hz) ──→ waypoint_patrol_node ──→ /hw/cmd_vel
    ├─ /hw/scan (6Hz) ──────────────→ SLAM Toolbox ──→ /map + map→odom TF
    ├─ odom→base_link TF ────────────→ SLAM Toolbox (scan matching)
    └─ /hw/cmd_vel ──────────────────→ Docker /cmd_vel → A1 机器人
```

### 已验证可用的话题

| 话题 | 频率 | 类型 |
|------|------|------|
| `/hw/Odometry_gazebo` | ~500Hz | nav_msgs/Odometry |
| `/hw/scan` | ~6Hz | sensor_msgs/LaserScan |
| `/hw/tf` | ~50Hz | tf2_msgs/TFMessage |
| `/hw/cmd_vel` | 10Hz | geometry_msgs/Twist |
| `/hw/nav/state` | 10Hz | std_msgs/String |
| `/hw/livox/imu` | ~100Hz | sensor_msgs/Imu |
| `/hw/trunk_imu` | ~100Hz | sensor_msgs/Imu |

---

## 新建文件

| 文件 | 作用 |
|------|------|
| `hazardwalker_nav/config/slam_toolbox_online_async.yaml` | SLAM 参数配置 |
| `hazardwalker_nav/launch/slam_toolbox.launch.py` | SLAM 启动文件 |
| `hazardwalker_nav/hazardwalker_nav/frontier_detector.py` | 前沿检测+A* 路径规划纯函数 |
| `hazardwalker_nav/hazardwalker_nav/frontier_explorer_node.py` | 5 状态机自主探索 ROS 节点 |

## 修改文件

| 文件 | 改动 |
|------|------|
| `hw_bridge.py` | 容器名→环境变量；时间戳修复；odom→base_link TF 生成；scan/tf 双发 |
| `docker_pipe.py` | scan 发送完整 ranges |
| `hw_topic_relay_node.py` | 新增 /scan→/hw/scan 转发 |
| `official_simenv_rosbridge_ros2_adapter_node.py` | 新增 scan/LiDAR/IMU WebSocket 转发 |
| `official_simenv_business.launch.py` | 新增 SLAM+frontier_explorer |
| `simenv_demo.launch.py` | 替换 waypoint→frontier+SLAM |
| `setup.py` + `package.xml` | v0.2.0 + tf2_ros/numpy 依赖 |

---

## 远程启动命令

```bash
# 1. 拉取代码
cd ~/HazardWalker && GIT_SSL_NO_VERIFY=1 git pull origin feature/nav

# 2. 编译
cd ~/HazardWalker/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  hazardwalker_nav hazardwalker_platform hazardwalker_bringup

# 3. 启动数据桥 (终端1)
PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$(find ~/HazardWalker/ros2_ws/install -name 'site-packages' -type d | tr '\n' ':')" \
LD_LIBRARY_PATH=/opt/ros/jazzy/lib \
ROS_DOMAIN_ID=12 SIMENV_CONTAINER=simenv_ros1_hazard_nav \
python3 ~/HazardWalker/ros2_ws/src/hazardwalker_platform/hw_bridge.py

# 4. 启动 SLAM (终端2)
LD_LIBRARY_PATH=/opt/ros/jazzy/lib ROS_DOMAIN_ID=12 \
/opt/ros/jazzy/lib/slam_toolbox/async_slam_toolbox_node \
  --ros-args --params-file ~/HazardWalker/ros2_ws/src/hazardwalker_nav/config/slam_toolbox_online_async.yaml \
  -r /tf:=/hw/tf

# 5. 启动导航 (终端3)
LD_LIBRARY_PATH=/opt/ros/jazzy/lib python3 /tmp/run_nav.py
```

`/tmp/run_nav.py` 见 [[#导航启动脚本]]

---

## 导航启动脚本

```python
import sys, os
sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')
sys.path.insert(0, os.path.expanduser(
    '~/HazardWalker/ros2_ws/install/hazardwalker_nav/lib/python3.12/site-packages'))
sys.path.insert(0, os.path.expanduser(
    '~/HazardWalker/ros2_ws/install/hazardwalker_platform/lib/python3.12/site-packages'))
os.environ['ROS_DOMAIN_ID'] = '12'

import rclpy
from hazardwalker_nav.waypoint_patrol_node import WaypointPatrolNode
from nav_msgs.msg import Odometry

rclpy.init()
node = WaypointPatrolNode()
node.destroy_subscription(node.odom_sub)
node.odom_sub = node.create_subscription(
    Odometry, '/hw/Odometry_gazebo', node.on_odom, 10)
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
```

---

## Frontier 探索状态机

```
INIT ──(地图就绪)──→ EXPLORING ──(无前沿)──→ RETURNING ──(到起点)──→ FINISHED
                       │    ↑                    │
                       │    └──(重观察完成)──────┘
                       ↓
                   REOBSERVING
                  (感知重观察请求)
```

---

## 已知问题

| 问题 | 影响 | 状态 |
|------|------|------|
| Docker 内 `/scan` ranges 为空 | SLAM 无法建图 | 待排查 |
| `ros2` 命令行 source 路径异常 | 无法用 ros2 run/launch | 临时 PYTHONPATH 绕过 |
| `waypoint_patrol_node` 包名 bug | 控制台脚本入口不可用 | `hazardwalker-nav`→`hazardwalker_nav` |
| RL 模型缺失 | 机器人稳定性受限 | 平台组 |
| 远程 hxbl 无 push 权限 | 只能本地推送 | 找管理员加 collaborator |
