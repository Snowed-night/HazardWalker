# HazardWalker 仿真平台 — 使用手册

> 版本：v2.1 | 日期：2026-07-06 | 面向：导航组 / 感知组

---

## 一、启动平台

```bash
# 第一步：启动 Docker 仿真环境
cd /home/hazard_platform/HazardWalker/ros2_ws/src/hazardwalker_platform
./auto_docker.sh up

# 第二步：加载 ROS2 环境
cd /tmp
source /opt/ros/jazzy/setup.zsh
source ~/HazardWalker/ros2_ws/install/setup.zsh

# 第三步：启动直接桥接（Docker ROS1 → 宿主机 ROS2 /hw/*）
pkill -f hw_bridge 2>/dev/null
nohup python3 /home/hazard_platform/HazardWalker/ros2_ws/src/hazardwalker_platform/hw_bridge.py > /tmp/hw_bridge.log 2>&1 &
sleep 4

# 第四步：验证
ros2 topic list | grep /hw/
```

应输出：

```
/hw/Odometry_gazebo
/hw/livox/imu
/hw/trunk_imu
/hw/tf
/hw/cmd_vel
/hw/real_sense/rgb/image_raw
/hw/real_sense/depth/image_raw
/hw/real_sense/depth/points
/hw/livox/Pointcloud2
```

**停止**：

```bash
cd /home/hazard_platform/HazardWalker/ros2_ws/src/hazardwalker_platform
./auto_docker.sh down
```

**可选参数**：

```bash
SEED=77 ./auto_docker.sh up                              # 固定场景
SEED=77 FLOOR_COUNT=5 ROOMS_PER_FLOOR=6 ./auto_docker.sh up  # 大场景
```

---

## 二、可用接口

### 传感器数据（订阅）

| 接口 | 类型 | 频率 | frame_id | 数据 |
|------|------|------|----------|------|
| `/hw/Odometry_gazebo` | `nav_msgs/msg/Odometry` | ~85 Hz | odom→base_link | ✅ |
| `/hw/trunk_imu` | `sensor_msgs/msg/Imu` | ~1000 Hz | imu_link | ✅ |
| `/hw/livox/imu` | `sensor_msgs/msg/Imu` | ~1000 Hz | livox_imu_link | ✅ |
| `/hw/tf` | `tf2_msgs/msg/TFMessage` | — | — | ✅ |
| `/hw/livox/Pointcloud2` | `sensor_msgs/msg/PointCloud2` | ~10 Hz | laser_livox | ❌ |
| `/hw/real_sense/*` | `sensor_msgs/Image/PointCloud2` | — | real_sense | ❌ |

### 控制（发布）

| 接口 | 类型 | 方向 |
|------|------|------|
| `/hw/cmd_vel` | `geometry_msgs/msg/Twist` | 算法 → 机器人 |

### 门/电梯

| 命令 | 说明 |
|------|------|
| `./scripts/hw_service_call.sh door {id} {true\|false}` | 开关门 |
| `./scripts/hw_service_call.sh elevator elevator_main {楼层} {true\|false}` | 呼叫电梯 |

---

## 三、消息格式

### 里程计 `/hw/Odometry_gazebo`

```
pose.pose.position.x, y, z        # 位置 (m)
pose.pose.orientation.x,y,z,w     # 姿态四元数
twist.twist.linear.x              # 前进速度 (m/s)
twist.twist.angular.z             # 偏航角速度 (rad/s)
```

```python
from nav_msgs.msg import Odometry
def cb(msg: Odometry):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    v = msg.twist.twist.linear.x
    w = msg.twist.twist.angular.z
```

### IMU `/hw/trunk_imu`、`/hw/livox/imu`

```
linear_acceleration.x, y, z    # 加速度 (m/s²)
angular_velocity.x, y, z       # 角速度 (rad/s)
orientation_covariance[0]=-1   # 姿态无效
```

```python
from sensor_msgs.msg import Imu
def cb(msg: Imu):
    ax = msg.linear_acceleration.x
    az = msg.linear_acceleration.z   # ~9.81 静止时
    wz = msg.angular_velocity.z
```

### 坐标变换 `/hw/tf`

```python
from tf2_ros import Buffer, TransformListener
tf = Buffer(); TransformListener(tf, node)
t = tf.lookup_transform('odom', 'base_link', rclpy.time.Time())
print(t.transform.translation.x, t.transform.translation.y)
```

TF 树：`odom → base_link → trunk → imu_link / laser_livox / real_sense`

### 速度控制 `/hw/cmd_vel`

```
linear.x  = 前进速度 (m/s)
angular.z = 偏航角速度 (rad/s)
```

```python
from geometry_msgs.msg import Twist
pub = node.create_publisher(Twist, '/hw/cmd_vel', 10)
m = Twist()
m.linear.x = 0.5; m.angular.z = 0.1
pub.publish(m)
```

---

## 四、验证平台

```bash
source /opt/ros/jazzy/setup.zsh
source ~/HazardWalker/ros2_ws/install/setup.zsh

# 频率检查
ros2 topic hz /hw/Odometry_gazebo     # ~85 Hz
ros2 topic hz /hw/trunk_imu           # ~1000 Hz

# 数据采样
ros2 topic echo /hw/Odometry_gazebo --once | head -15
ros2 topic echo /hw/trunk_imu --once | head -10

# 一键测试
cd /home/hazard_platform/HazardWalker/ros2_ws/src/hazardwalker_platform
bash test_bridge.sh
```

---

## 五、已知限制

| 问题 | 说明 |
|------|------|
| LiDAR + 相机无数据 | Gazebo headless 无 GPU 渲染，需 xvfb 方案 |
| 机器狗不能动 | 缺 RL 模型文件 |
