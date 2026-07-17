# HazardWalker 仿真平台 — 历史远程使用手册

> 历史版本：v3.2 | 日期：2026-07-08 | 面向：导航组 / 感知组

> **重要更新（负责人：姜晨）**：本文件中的 `hw_bridge.py`、JSON 管道、
> `ros1_bridge dynamic_bridge`、`/hw/Odometry_gazebo` 等描述属于旧实现，不能用于当前官方
> SimEnv 验收；其中旧 JSON bridge 已被代码默认拒绝启动，因为它不完整转发 RGB-D，也不回传
> `/hw/cmd_vel`。官方 ROS1 profile 的唯一当前入口、稳定话题名、环境变量和验收顺序以
> [`docs/environment/官方SimEnv_ROS1_ROS2双向适配整改_20260714.md`](../../../docs/environment/官方SimEnv_ROS1_ROS2双向适配整改_20260714.md)
> 为准：ROS2 主机运行 rosbridge 适配器，输出 `/hw/odom`、`/hw/camera/image_raw`、
> `/hw/camera/depth_image` 等。保留本文件仅供追溯历史环境，不应执行下方的“第 3 步”。

---

## 一、启动平台

```bash
# 第 1 步：启动 Docker 仿真（约 60 秒）
cd /home/hazard_platform/HazardWalker/ros2_ws/src/hazardwalker_platform

# 仅传感器（无控制器）：
./auto_docker.sh up

# 传感器 + 机器人控制器（自动站立 + RL 模式，/cmd_vel 可用）：
START_CONTROLLER=1 ./auto_docker.sh up

# 固定场景种子：
SEED=77 ./auto_docker.sh up

# 第 2 步：加载 ROS2 环境（注意：shell 为 zsh，用 .zsh）
cd /tmp
source /opt/ros/jazzy/setup.zsh
source ~/HazardWalker/ros2_ws/install/setup.zsh

# 第 3 步：启动桥接（Docker ROS1 → 宿主机 ROS2 /hw/*）
pkill -f hw_bridge 2>/dev/null
nohup python3 ~/HazardWalker/ros2_ws/src/hazardwalker_platform/hw_bridge.py > /tmp/hw_bridge.log 2>&1 &
sleep 4

# 第 4 步：验证（应输出 9 个 /hw/ 话题）
ros2 topic list | grep /hw/
```

**停止**：`./auto_docker.sh down`

| 常用参数 | 默认 | 说明 |
|----------|------|------|
| `START_CONTROLLER=1` | 0 | 启动机器人控制器（自动 RL 模式，/cmd_vel 可用） |
| `SEED=77` | 随机 | 固定场景种子 |
| `FLOOR_COUNT=3` | 3 | 楼层数 |
| `ROOMS_PER_FLOOR=4` | 4 | 每层房间数 |

---

## 二、传感器接口（全部订阅接收）

### 2.1 `/hw/Odometry_gazebo` — 里程计

| 类型 | 频率 | 坐标系 |
|------|------|--------|
| `nav_msgs/msg/Odometry` | ~100 Hz | `odom` → `base_link` |

**输出示例**：
```
position:    {x: 3.128, y: -2.342, z: 0.138}
orientation: {x: -0.005, y: -0.002, z: 0.982, w: -0.189}
linear:      {x: -0.028, y: 0.0, z: 0.0}
angular:     {x: 0.0, y: 0.0, z: 0.0}
```

**订阅代码**：
```python
from nav_msgs.msg import Odometry
node.create_subscription(Odometry, '/hw/Odometry_gazebo', callback, 10)

def callback(msg):
    x = msg.pose.pose.position.x     # 世界 X (m)
    y = msg.pose.pose.position.y     # 世界 Y (m)
    z = msg.pose.pose.position.z     # 高程 (m)
    vx = msg.twist.twist.linear.x    # 前进速度 (m/s)
    wz = msg.twist.twist.angular.z   # 偏航角速度 (rad/s)
```

### 2.2 `/hw/scan` — 激光雷达

| 类型 | 频率 | FOV | 测距 | 坐标系 |
|------|------|-----|------|--------|
| `sensor_msgs/msg/LaserScan` | 10 Hz | 360° | 0.1~40m | `laser_livox` |

**输出示例**：
```
angle_min: 0.0, angle_max: 0.0, angle_increment: 0.0
range_min: 0.0, range_max: 0.0
ranges: [1.234, 1.567, ...]      # 360 个距离值 (m)
intensities: [0.0, 0.0, ...]
```

**订阅代码**：
```python
from sensor_msgs.msg import LaserScan
node.create_subscription(LaserScan, '/hw/scan', callback, 10)

def callback(msg):
    for i, r in enumerate(msg.ranges):
        if msg.range_min < r < msg.range_max:
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
```

### 2.3 `/hw/trunk_imu` — 躯干 IMU

| 类型 | 频率 | 坐标系 |
|------|------|--------|
| `sensor_msgs/msg/Imu` | ~100 Hz | `imu_link`（机器人质心） |

**输出示例**：
```
linear_acceleration: {x: -0.104, y: -0.046, z: 9.777}   # m/s², 静止时 z≈9.81
angular_velocity:    {x: -0.010, y: 0.116, z: -0.001}   # rad/s
orientation_covariance[0]: -1                            # 姿态无效
```

**订阅代码**：
```python
from sensor_msgs.msg import Imu
node.create_subscription(Imu, '/hw/trunk_imu', callback, 10)

def callback(msg):
    ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
    wx, wy, wz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
```

### 2.4 `/hw/livox/imu` — Livox 内置 IMU

| 类型 | 频率 | 坐标系 |
|------|------|--------|
| `sensor_msgs/msg/Imu` | ~100 Hz | `livox_imu_link` |

格式同 2.3 `/hw/trunk_imu`。

### 2.5 `/hw/real_sense/rgb/image_raw` — RGB 图像

| 类型 | 频率 | 分辨率 | 编码 | 坐标系 |
|------|------|--------|------|--------|
| `sensor_msgs/msg/Image` | ~20 Hz | 640×480 | `rgb8` | `real_sense` |

**输出示例**：
```
height: 480, width: 640, encoding: rgb8
step: 1920, data: <1920*480 bytes>
```

**订阅代码**：
```python
from sensor_msgs.msg import Image
import numpy as np
node.create_subscription(Image, '/hw/real_sense/rgb/image_raw', callback, 10)

def callback(msg):
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
```

### 2.6 `/hw/real_sense/depth/image_raw` — 深度图像

| 类型 | 频率 | 分辨率 | 编码 | 坐标系 |
|------|------|--------|------|--------|
| `sensor_msgs/msg/Image` | ~20 Hz | 640×480 | `32FC1`(米) | `real_sense` |

**输出示例**：
```
height: 480, width: 640, encoding: 32FC1
step: 2560, data: <2560*480 bytes>
```

**订阅代码**：
```python
node.create_subscription(Image, '/hw/real_sense/depth/image_raw', callback, 10)

def callback(msg):
    depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    # depth[y, x] = 距离 (m)，0 表示无效
```

### 2.7 `/hw/real_sense/depth/points` — 深度点云

| 类型 | 频率 | 分辨率 | 字段 | 坐标系 |
|------|------|--------|------|--------|
| `sensor_msgs/msg/PointCloud2` | ~20 Hz | 640×480 | x,y,z,rgb (FLOAT32) | `real_sense` |

**输出示例**：
```
height: 480, width: 640, point_step: 32, row_step: 20480
fields: [{name: x, offset: 0, datatype: 7}, {name: y, offset: 4, datatype: 7},
         {name: z, offset: 8, datatype: 7}, {name: rgb, offset: 16, datatype: 7}]
```

**订阅代码**：
```python
from sensor_msgs.msg import PointCloud2
import struct
node.create_subscription(PointCloud2, '/hw/real_sense/depth/points', callback, 10)

def callback(msg):
    for i in range(msg.width * msg.height):
        off = i * msg.point_step
        x, y, z, rgb = struct.unpack_from('ffff', msg.data, off)
```

### 2.8 `/hw/tf` — 坐标变换

| 类型 |
|------|
| `tf2_msgs/msg/TFMessage` |

**TF 树**：
```
odom → base_link → trunk → imu_link
                  ├→ laser_livox  (0.20, 0, 0.08)
                  └→ real_sense   (0.28, 0, 0.043)
```

```python
from tf2_ros import Buffer, TransformListener
tf_buffer = Buffer()
TransformListener(tf_buffer, node)
t = tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
x, y = t.transform.translation.x, t.transform.translation.y
```

---

## 三、控制接口

### 3.1 `/hw/cmd_vel` — 速度指令（发布）

| 类型 | 方向 |
|------|------|
| `geometry_msgs/msg/Twist` | 算法 → 机器人 |

**输入格式**：
```
linear.x  = 前进速度 (m/s)，正=前进
angular.z = 偏航角速度 (rad/s)，正=左转
```

**发布示例**：
```python
from geometry_msgs.msg import Twist
pub = node.create_publisher(Twist, '/hw/cmd_vel', 10)

# 前进 0.5 m/s
msg = Twist()
msg.linear.x = 0.5
pub.publish(msg)

# 左转 0.3 rad/s
msg = Twist()
msg.angular.z = 0.3
pub.publish(msg)

# 前进 + 转向
msg = Twist()
msg.linear.x = 0.3
msg.angular.z = -0.2
pub.publish(msg)
```

### 3.2 门控制

```bash
S=~/HazardWalker/ros2_ws/src/hazardwalker_platform/scripts/hw_service_call.sh

# 开大门
$S door main_entrance true

# 关大门
$S door main_entrance false

# 开 1 楼电梯门 / 2 楼电梯门
$S door elevator_floor_0 true
$S door elevator_floor_1 true
```

**Python 调用**：
```python
import subprocess
S = '/home/hazard_platform/HazardWalker/ros2_ws/src/hazardwalker_platform/scripts/hw_service_call.sh'

def open_door(door_id='main_entrance'):
    subprocess.run([S, 'door', door_id, 'true'], check=True)

def close_door(door_id='main_entrance'):
    subprocess.run([S, 'door', door_id, 'false'], check=True)
```

**返回示例**：`accepted=True state=open`

### 3.3 电梯控制

```bash
# 到 1 楼，不开门
$S elevator elevator_main 0 false

# 到 2 楼，开门
$S elevator elevator_main 1 true
```

**Python 调用**：
```python
def call_elevator(floor=0, open_doors=False):
    subprocess.run([S, 'elevator', 'elevator_main', str(floor),
                    str(open_doors).lower()], check=True)
```

**返回示例**：`accepted=True floor=1 state=door_open`

**典型电梯流程**：
```bash
$S elevator elevator_main 0 false    # 1. 叫到 1 楼
$S door elevator_floor_0 true        # 2. 开门进入
# ... 通过 /hw/cmd_vel 驶入轿厢 ...
$S door elevator_floor_0 false       # 3. 关门
$S elevator elevator_main 1 false    # 4. 到 2 楼
$S door elevator_floor_1 true        # 5. 开门离开
```

---

## 四、验证平台

```bash
source /opt/ros/jazzy/setup.zsh
source ~/HazardWalker/ros2_ws/install/setup.zsh

ros2 topic hz /hw/Odometry_gazebo       # ~100Hz
ros2 topic hz /hw/scan                  # 10Hz
ros2 topic hz /hw/trunk_imu             # ~100Hz
ros2 topic echo /hw/Odometry_gazebo --once | head -15
ros2 topic echo /hw/scan --once | head -10
```

---

## 五、接口速查表

| 接口 | 类型 | 频率 | 帧 ID | 方向 |
|------|------|------|-------|------|
| `/hw/Odometry_gazebo` | `nav_msgs/Odometry` | ~100Hz | odom→base | 订阅 |
| `/hw/scan` | `sensor_msgs/LaserScan` | 10Hz | laser_livox | 订阅 |
| `/hw/trunk_imu` | `sensor_msgs/Imu` | ~100Hz | imu_link | 订阅 |
| `/hw/livox/imu` | `sensor_msgs/Imu` | ~100Hz | livox_imu_link | 订阅 |
| `/hw/real_sense/rgb/image_raw` | `sensor_msgs/Image` | ~20Hz | real_sense | 订阅 |
| `/hw/real_sense/depth/image_raw` | `sensor_msgs/Image` | ~20Hz | real_sense | 订阅 |
| `/hw/real_sense/depth/points` | `sensor_msgs/PointCloud2` | ~20Hz | real_sense | 订阅 |
| `/hw/tf` | `tf2_msgs/TFMessage` | — | — | 订阅 |
| `/hw/cmd_vel` | `geometry_msgs/Twist` | — | — | 发布 |
| `hw_service_call.sh door` | — | — | — | 门控制 |
| `hw_service_call.sh elevator` | — | — | — | 电梯控制 |
