# HazardWalker 仿真平台 — 使用手册

> 版本：v2.0 | 日期：2026-07-06 | 面向：导航组 / 感知组

---

## 一、启动平台（按顺序执行）

```bash
# 第一步：启动 Docker 仿真环境
cd /home/hazard_platform/HazardWalker/ros2_ws/src/hazardwalker_platform
./auto_docker.sh up
# 等待约 60 秒（场景生成 + Gazebo + 传感器初始化）

# 第二步：加载 ROS2 环境
cd /tmp
source /opt/ros/jazzy/setup.zsh
source ~/HazardWalker/ros2_ws/install/setup.zsh

# 第三步：启动话题中继（Docker原始话题 → /hw/命名空间）
pkill -f hw_topic_relay 2>/dev/null
ros2 run hazardwalker_platform hw_topic_relay_node &
sleep 3

# 第四步：验证
ros2 topic list | grep /hw/
```

应输出 9 个话题：

```
/hw/Odometry_gazebo
/hw/livox/Pointcloud2
/hw/livox/imu
/hw/trunk_imu
/hw/tf
/hw/real_sense/rgb/image_raw
/hw/real_sense/depth/image_raw
/hw/real_sense/depth/points
/hw/cmd_vel
```

**停止平台**：

```bash
cd /home/hazard_platform/HazardWalker/ros2_ws/src/hazardwalker_platform
./auto_docker.sh down
```

**可选启动参数**：

```bash
SEED=77 ./auto_docker.sh up                     # 固定场景种子（可复现）
FLOOR_COUNT=5 ./auto_docker.sh up               # 5 层楼
ROOMS_PER_FLOOR=6 ./auto_docker.sh up           # 每层 6 个房间
DANGER_COUNT=2:4 ./auto_docker.sh up            # 2-4 个危险源
```

---

## 二、传感器数据接口

所有话题统一命名空间 `/hw/`，通过 `ros2 topic echo/list/hz` 查看。

### 里程计 `/hw/Odometry_gazebo`

| 属性 | 值 |
|------|-----|
| 消息类型 | `nav_msgs/msg/Odometry` |
| 发布频率 | ~100 Hz |
| 父坐标系 | `odom` |
| 子坐标系 | `base_link` |

消息结构：

```
pose.pose.position.x, y, z            # 位置 (m)
pose.pose.orientation.x, y, z, w      # 姿态四元数
twist.twist.linear.x, y, z            # 线速度 (m/s)，x=前进
twist.twist.angular.x, y, z           # 角速度 (rad/s)，z=偏航
```

订阅示例：

```python
from nav_msgs.msg import Odometry

def callback(msg: Odometry):
    x = msg.pose.pose.position.x    # 世界 X
    y = msg.pose.pose.position.y    # 世界 Y
    v = msg.twist.twist.linear.x    # 前进速度
    w = msg.twist.twist.angular.z   # 偏航角速度
```

### LiDAR 点云 `/hw/livox/Pointcloud2`

| 属性 | 值 |
|------|-----|
| 消息类型 | `sensor_msgs/msg/PointCloud2` |
| 发布频率 | ~10 Hz |
| 坐标系 | `laser_livox` |
| 传感器 | Livox Mid-360（360°水平，57°垂直，40m测距） |

消息结构：

```
height: 1                            # 无序点云
width: N                             # 点数
fields:
  {name: "x",         offset: 0,  datatype: FLOAT32}
  {name: "y",         offset: 4,  datatype: FLOAT32}
  {name: "z",         offset: 8,  datatype: FLOAT32}
  {name: "intensity", offset: 12, datatype: FLOAT32}
point_step: 16                       # 每点 16 字节
row_step: N * 16
data: 二进制点云数据
is_dense: true
```

解析示例：

```python
from sensor_msgs.msg import PointCloud2
import struct

def callback(msg: PointCloud2):
    pts = []
    for i in range(msg.width):
        off = i * msg.point_step
        x, y, z, intensity = struct.unpack_from('ffff', msg.data, off)
        pts.append((x, y, z, intensity))
```

### 躯干 IMU `/hw/trunk_imu`

| 属性 | 值 |
|------|-----|
| 消息类型 | `sensor_msgs/msg/Imu` |
| 发布频率 | ~1000 Hz |
| 坐标系 | `imu_link`（机器人质心） |

消息结构：

```
linear_acceleration.x, y, z      # 线加速度 (m/s²)，z 含重力 ~9.81
angular_velocity.x, y, z         # 角速度 (rad/s)
orientation_covariance[0] = -1   # 姿态无效
```

### Livox 内置 IMU `/hw/livox/imu`

| 属性 | 值 |
|------|-----|
| 消息类型 | `sensor_msgs/msg/Imu` |
| 发布频率 | ~1000 Hz |
| 坐标系 | `livox_imu_link`（雷达内部） |
| 结构 | 同 `/hw/trunk_imu` |

### 坐标变换 `/hw/tf`

| 属性 | 值 |
|------|-----|
| 消息类型 | `tf2_msgs/msg/TFMessage` |

TF 树结构：

```
odom ─→ base_link ─→ trunk ─→ imu_link
                 ├→ front_camera  (0.25, 0, 0.35)
                 ├→ laser_livox   (0.20, 0, 0.08)
                 └→ real_sense    (0.28, 0, 0.043)
```

坐标系约定：X=前方，Y=左侧，Z=上方（ENU）

查询示例：

```python
from tf2_ros import Buffer, TransformListener
tf = Buffer()
tf_listener = TransformListener(tf, node)
t = tf.lookup_transform('odom', 'base_link', rclpy.time.Time())
print(t.transform.translation.x, t.transform.translation.y)
```

---

## 三、控制接口

### 速度指令 `/hw/cmd_vel`

| 属性 | 值 |
|------|-----|
| 消息类型 | `geometry_msgs/msg/Twist` |
| 方向 | 算法 → 机器人 |

```
linear.x  = 前进速度 (m/s)，正=前进
angular.z = 偏航角速度 (rad/s)，正=左转
```

发布示例：

```python
from geometry_msgs.msg import Twist
pub = node.create_publisher(Twist, '/hw/cmd_vel', 10)
msg = Twist()
msg.linear.x = 0.5     # 前进 0.5 m/s
msg.angular.z = 0.1    # 左转 0.1 rad/s
pub.publish(msg)
```

---

## 四、门与电梯控制

通过 `hw_service_call.sh` 脚本调用。

### 开关门

```bash
SCRIPT=/home/hazard_platform/HazardWalker/ros2_ws/src/hazardwalker_platform/scripts/hw_service_call.sh

$SCRIPT door main_entrance true       # 开大门
$SCRIPT door main_entrance false      # 关大门
$SCRIPT door elevator_floor_0 true    # 开 1 楼电梯门
$SCRIPT door elevator_floor_1 true    # 开 2 楼电梯门
```

### 呼叫电梯

```bash
$SCRIPT elevator elevator_main 0 false    # 电梯到 1 楼，不开门
$SCRIPT elevator elevator_main 1 true     # 电梯到 2 楼，开门
```

- 楼层编号：`0`=1楼，`1`=2楼，`2`=3楼
- 电梯 ID：`elevator_main`

### 典型电梯流程

```bash
$SCRIPT elevator elevator_main 0 false     # 1. 叫电梯到 1 楼
$SCRIPT door elevator_floor_0 true         # 2. 开门进入
# ... 通过 /hw/cmd_vel 控制机器人驶入轿厢 ...
$SCRIPT door elevator_floor_0 false        # 3. 关门
$SCRIPT elevator elevator_main 1 false     # 4. 电梯到 2 楼
$SCRIPT door elevator_floor_1 true         # 5. 开门离开
```

### 代码调用

```python
import subprocess
S = '/home/hazard_platform/HazardWalker/ros2_ws/src/hazardwalker_platform/scripts/hw_service_call.sh'

subprocess.run([S, 'door', 'main_entrance', 'true'], check=True)
subprocess.run([S, 'elevator', 'elevator_main', '1', 'false'], check=True)
```

---

## 五、验证平台

### 快速检查

```bash
source /opt/ros/jazzy/setup.zsh
source ~/HazardWalker/ros2_ws/install/setup.zsh

ros2 topic hz /hw/Odometry_gazebo       # 应 ~100 Hz
ros2 topic hz /hw/livox/Pointcloud2     # 应 ~10 Hz
ros2 topic hz /hw/trunk_imu             # 应 ~1000 Hz
ros2 topic echo /hw/Odometry_gazebo --once | head -10
```

### 一键测试

```bash
cd /home/hazard_platform/HazardWalker/ros2_ws/src/hazardwalker_platform
bash test_bridge.sh
```

### 确认自己的代码正常

```bash
# 节点发布 cmd_vel 后，确认有发布者
ros2 topic info /hw/cmd_vel | grep Publisher

# 确认能收传感器数据
ros2 topic echo /hw/Odometry_gazebo --once | head -5

# 确认门控可用
./scripts/hw_service_call.sh door main_entrance true
# 期望：accepted=True state=open
```

---

## 六、接口速查表

| 接口 | 类型 | 频率 | 帧 | 方向 |
|------|------|------|-----|------|
| `/hw/Odometry_gazebo` | `nav_msgs/Odometry` | 100Hz | odom→base_link | 订阅 |
| `/hw/livox/Pointcloud2` | `sensor_msgs/PointCloud2` | 10Hz | laser_livox | 订阅 |
| `/hw/trunk_imu` | `sensor_msgs/Imu` | 1000Hz | imu_link | 订阅 |
| `/hw/livox/imu` | `sensor_msgs/Imu` | 1000Hz | livox_imu_link | 订阅 |
| `/hw/tf` | `tf2_msgs/TFMessage` | — | — | 订阅 |
| `/hw/cmd_vel` | `geometry_msgs/Twist` | — | — | 发布 |
| `hw_service_call.sh door` | — | — | — | 调用 |
| `hw_service_call.sh elevator` | — | — | — | 调用 |

---

## 七、已知限制

| 问题 | 影响 | 说明 |
|------|------|------|
| 相机无数据 | RealSense 话题存在但空 | Gazebo headless 模式 |
| 机器狗不能动 | `/hw/cmd_vel` 无人响应 | 缺 RL 模型文件 |
