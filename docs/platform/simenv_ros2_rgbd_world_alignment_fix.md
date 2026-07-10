# SimEnv ROS2 RGB-D 与 world 坐标对齐修复说明

## 目的与边界

本文记录 2026-07-10 在主力机官方 SimEnv ROS2 Harmonic 部署中发现的两类平台问题及可复现修复。目标是让感知节点稳定取得 RGB、深度、相机内参和统一 TF，并将危险源坐标输出到 Gazebo `world` 坐标系。

本文只使用公开 ROS/Gazebo 话题和临时受控测试球；不得读取 `danger_truth.json`、world 文件、布局元数据或其他裁判真值文件。

## 已确认问题

1. 相机只桥接了 RGB，缺少 `CameraInfo` 与对齐深度图，导致三维反投影链路不完整。
2. relay 将 `/odometry` 的模型原点作为 `base_link`，但又将内部 SDF `base_link(z=0.15)` 与 `camera_link(z=0.32)` 相减，错误发布 `camera_offset_z_m=0.17`。实际 `/odometry` 对应模型原点，外参 z 必须为 `0.32`。
3. 当前 A1 控制器使 `Odometry_gazebo` 的转向与 Gazebo 模型公开世界姿态不一致；危险源 world 定位不能以该里程计为基准。
4. 重启脚本仅清理名称中包含 `simenv_hw_topic_relay` 的进程，实际 `ros2 run ... hw_topic_relay` 残留，可能制造重复发布者。

## 平台端变更

平台目录：`/home/hazard_test/Guoyulun/Competition/SimEnv_ROS2`。

### 1. 相机模型

文件：`ros2_ws/src/simenv_robot/models/a1_gazebo/model.sdf`

- 将相机安装到模型前上方：`camera_link` 使用 `x=0.32, z=0.32`，避免机身遮挡。
- 在同一 `camera_link` 增加深度相机，发布 `camera/depth_image`；RGB 与深度使用相同 640×480、HFOV 和裁剪面。

### 2. ros_gz_bridge

文件：`ros2_ws/src/simenv_bridge/config/ros_gz_bridge.yaml`

新增 GZ 到 ROS 映射：

```yaml
- ros_topic_name: "/simenv/camera/camera_info"
  gz_topic_name: "/camera/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS

- ros_topic_name: "/simenv/camera/depth_image"
  gz_topic_name: "/camera/depth_image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
```

### 3. `/hw/*` relay 与 TF

文件：`ros2_ws/src/simenv_sensors/simenv_sensors/hw_topic_relay_node.py`

- 转发相机内参到 `/hw/camera/camera_info`，深度到 `/hw/camera/depth_image`。
- 从里程计发布 `odom -> base_link` 动态 TF。
- 首帧 RGB 到达后发布 `base_link -> <camera_frame>` 静态 TF。
- 关键参数必须为：

```python
self.declare_parameter("camera_offset_x_m", 0.32)
self.declare_parameter("camera_offset_y_m", 0.0)
# /odometry child_frame 虽命名 base_link，实际使用模型原点。
self.declare_parameter("camera_offset_z_m", 0.32)
```

相机光学坐标到机器人前/左/上的旋转为四元数 `(-0.5, 0.5, -0.5, 0.5)`。

### 4. 重启清理

文件：`auto_ros2.sh`

在现有清理逻辑中补充：

```bash
pkill -f "simenv_sensors.*pointcloud2livox" || true
pkill -f "simenv_sensors.*hw_topic_relay" || true
```

随后重新构建：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/hazard_test/Guoyulun/Competition/SimEnv_ROS2/ros2_ws
colcon build --symlink-install --packages-select simenv_robot simenv_sensors
```

## world 坐标对齐

新增 `simenv_sensors/world_pose_tf_node.py`，通过 Gazebo Python transport 直接订阅公开 `/world/generated_world/pose/info`，从名称为 `a1_gazebo` 的模型位姿计算并发布：

```text
world -> hazardwalker_camera
```

它将模型世界位姿与相机安装外参 `(0.32, 0.0, 0.32)` 合成，并使用 PoseInfo 的**仿真时间戳**发布 TF。`hw_topic_relay` 将 RGB、深度、CameraInfo 的 `frame_id` 全部改为 `hazardwalker_camera`，避免与失真的 `odom -> base_link -> camera` 链并存。此过程只读取公开模型自身位姿，不读取危险源真值。

## 验收命令

```bash
ros2 topic info /hw/camera/image_raw
ros2 topic info /hw/camera/depth_image
ros2 topic info /hw/camera/camera_info
ros2 run tf2_ros tf2_echo world hazardwalker_camera
```

前三项均应显示 `Publisher count: 1`。受控红球复测显示：未修正 z 外参时，world XY 误差为 `0.0116 m`、Z 偏低约 `0.18 m`；安装本修复、重建并重启后，完整三维误差为 `0.0423 m`。直接 PoseInfo TF 的动态伪转向验证中，世界坐标漂移为 `0.0025 m` 且不会误增视角计数。

对应感知素材与测试表位于 `reports/perception/simulation/3d_native/official_simenv_20260710_rgbd_world_validation/`。

## 尚未解决的平台依赖

当前 ROS2 迁移日志已经标注 `unitree_guide / junior_ctrl` 未完成。实测表明 `/cmd_vel` 已到达 DiffDrive、轮速可达约 `2.125 rad/s`，但轮式占位模型的公开世界位姿存在迟滞/滑移，且不稳定地偏离 `Odometry_gazebo`。不要把该里程计变化作为真实四足视角或真实位姿；在真实 A1 控制迁移完成前，感知多视角策略应保持保守的未确认状态。
