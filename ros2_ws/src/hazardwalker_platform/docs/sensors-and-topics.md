# 传感器与 ROS 话题

本文列出 Unitree A1 在仿真中的传感器安装位姿、常用 ROS 话题和坐标系。坐标系遵循 ROS 常用约定：X 向前，Y 向左，Z 向上。

## 传感器位姿

### 机载 IMU `imu_link`

| 属性 | 数值 |
|------|------|
| 父坐标系 | `trunk` |
| 安装位置 | 躯干中心 |
| 位姿 `xyz, rpy` | `(0.0, 0.0, 0.0)`, `(0.0, 0.0, 0.0)` |
| 质量 | 0.001 kg |

IMU 安装于机器人质心位置，与躯干坐标系重合。

### Livox Mid-360 `laser_livox`

| 属性 | 数值 |
|------|------|
| 父坐标系 | `base` |
| 安装位置 | 躯干前上方 |
| 位姿 `xyz, rpy` | `(0.2, 0.0, 0.08)`, `(0.0, 0.785, 0.0)` |
| 备注 | 绕 Y 轴倾斜 45 度，优化前方及地面点云覆盖 |

Livox 内置 IMU `livox_imu_link`：

- 父坐标系：`laser_livox`
- 相对位姿：`(-0.011, -0.02329, 0.04412)`, `(0.0, 0.0, 0.0)`

### RealSense D415 `real_sense`

| 属性 | 数值 |
|------|------|
| 父坐标系 | `base` |
| 安装位置 | 躯干最前端 |
| 位姿 `xyz, rpy` | `(0.28, 0.0, 0.043)`, `(0.0, 0.0, 0.0)` |
| 视觉朝向 | 绕 Z 轴旋转 90 度 |
| 质量 | 0.103 kg |

## 状态估计与真值

| 话题名称 | 消息类型 | 发布频率 | 说明 |
|---------|---------|---------|------|
| `/trunk_imu` | `sensor_msgs/Imu` | 1000 Hz | 躯干 IMU 数据 |
| `/livox/imu` | `sensor_msgs/Imu` | 1000 Hz | Livox 雷达内置 IMU |
| `/ground_truth/base_trunk` | `nav_msgs/Odometry` | 100 Hz | `base` 相对于 `trunk` 的真值位姿 |
| `/ground_truth/base_w` | `nav_msgs/Odometry` | 100 Hz | `base` 相对于 `world` 的真值位姿 |

## 足端状态真值

| 话题名称 | 消息类型 | 发布频率 | 说明 |
|---------|---------|---------|------|
| `/ground_truth/FL_foot` | `nav_msgs/Odometry` | 100 Hz | 左前足位姿与速度 |
| `/ground_truth/FR_foot` | `nav_msgs/Odometry` | 100 Hz | 右前足位姿与速度 |
| `/ground_truth/RL_foot` | `nav_msgs/Odometry` | 100 Hz | 左后足位姿与速度 |
| `/ground_truth/RR_foot` | `nav_msgs/Odometry` | 100 Hz | 右后足位姿与速度 |

## 足端接触力

| 话题名称 | 消息类型 | 发布频率 | 说明 |
|---------|---------|---------|------|
| `/FR_foot_contact` | `gazebo_msgs/ContactsState` | 100 Hz | 右前足接触力 |
| `/FL_foot_contact` | `gazebo_msgs/ContactsState` | 100 Hz | 左前足接触力 |
| `/RR_foot_contact` | `gazebo_msgs/ContactsState` | 100 Hz | 右后足接触力 |
| `/RL_foot_contact` | `gazebo_msgs/ContactsState` | 100 Hz | 左后足接触力 |

## 激光雷达

| 话题名称 | 消息类型 | 发布频率 | 说明 |
|---------|---------|---------|------|
| `/scan` | `sensor_msgs/PointCloud2` | 10 Hz | Livox Mid-360 点云数据 |
| `/livox/imu` | `sensor_msgs/Imu` | 1000 Hz | 雷达内置 IMU |

雷达参数：

- 水平 FOV：360 度
- 垂直 FOV：-5.22 度到 57.22 度
- 测距范围：0.1 m 到 40 m
- 分辨率：0.01 m
- 噪声：高斯噪声，标准差 0.005

## 视觉传感器

前视单目相机 `front_camera`：

| 话题名称 | 消息类型 | 发布频率 | 说明 |
|---------|---------|---------|------|
| `/camera/image_raw` | `sensor_msgs/Image` | 30 Hz | RGB 图像 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | 相机标定参数 |

相机参数：

- 分辨率：800 x 800
- 水平 FOV：1.396 rad，约 80 度
- 近/远裁剪面：0.02 m / 300 m
- 噪声：高斯噪声，标准差 0.007

RealSense D415 深度相机：

| 话题名称 | 消息类型 | 发布频率 | 说明 |
|---------|---------|---------|------|
| `/real_sense/rgb/image_raw` | `sensor_msgs/Image` | 10 Hz | RGB 图像 |
| `/real_sense/rgb/camera_info` | `sensor_msgs/CameraInfo` | 10 Hz | RGB 相机标定 |
| `/real_sense/depth/image_raw` | `sensor_msgs/Image` | 10 Hz | 深度图像 |
| `/real_sense/depth/camera_info` | `sensor_msgs/CameraInfo` | 10 Hz | 深度相机标定 |
| `/real_sense/depth/points` | `sensor_msgs/PointCloud2` | 10 Hz | 点云数据 |

相机参数：

- 分辨率：640 x 480
- 水平 FOV：60 度
- 近/远裁剪面：0.05 m / 8.0 m
- 点云最小距离：0.4 m

## 外部扰动

| 话题名称 | 消息类型 | 功能 |
|---------|---------|------|
| `/apply_force/trunk` | `geometry_msgs/Wrench` | 向躯干施加外力或力矩，用于扰动测试 |

## 坐标系

| 坐标系 ID | 父坐标系 | 说明 |
|----------|----------|------|
| `world` | - | Gazebo 世界坐标系 |
| `base` | `world` | 机器人基坐标系 |
| `trunk` | `base` | 浮动基座，接近质心 |
| `imu_link` | `trunk` | 机载 IMU |
| `laser_livox` | `base` | Livox 雷达 |
| `livox_imu_link` | `laser_livox` | 雷达内置 IMU |
| `real_sense` | `base` | RealSense 深度相机 |
| `front_camera` | - | 前视单目相机 |
| `FR_foot` / `FL_foot` / `RR_foot` / `RL_foot` | 各小腿 | 足端接触点 |

## 查看示例

```bash
# 查看 IMU 数据
rostopic echo /trunk_imu

# 可视化点云
rosrun rviz rviz
# 添加 PointCloud2 显示，订阅 /scan

# 查看深度图像
rosrun image_view image_view image:=/real_sense/depth/image_raw
```
