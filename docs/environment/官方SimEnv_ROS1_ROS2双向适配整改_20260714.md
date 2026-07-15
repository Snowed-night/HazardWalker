# 官方 SimEnv ROS1 ↔ ROS2 双向适配整改

负责人：姜晨。

## 背景与边界

官方 SimEnv 为 ROS1 Noetic、Gazebo Classic 和 Unitree A1；HazardWalker 的导航、感知和决策保持
ROS2，仅使用稳定 `/hw/*`。本整改新增的不是第二套业务系统，而是位于官方容器内的 ROS1
适配节点：它运行在 ROS2 主机上，经官方容器已有的 `rosbridge_websocket` 订阅原始 ROS1 话题并发布为 `/hw/*`；
控制方向则由同一 WebSocket 把 `/hw/cmd_vel` 送回 ROS1 `/cmd_vel`。

`ros_gz_bridge.yaml` 和 Gazebo Harmonic 仅服务本地 profile，不能作为官方控制、RGB-D 或里程计可用的证据。
当前官方 `simenv_run` 容器未安装 ROS2 或 `dynamic_bridge`，不得假设容器内 ros1_bridge 可用。

## 映射

| 方向 | 官方 ROS1 | 稳定接口 | 类型 | 说明 |
|---|---|---|---|---|
| 输入 ROS2 | `/hazardwalker/odom` | `/hw/odom` | `nav_msgs/Odometry` | 由官方端最新值中继从 `/Odometry_gazebo` 生成，避免历史积压 |
| 输入 ROS2 | `/real_sense/rgb/image_raw` | `/hw/camera/image_raw` | `sensor_msgs/Image` | 可按官方实际传感器话题调整 |
| 输入 ROS2 | `/real_sense/depth/image_raw` | `/hw/camera/depth_image` | `sensor_msgs/Image` | RGB-D 定位输入 |
| 输入 ROS2 | RGB/深度 `camera_info` | `/hw/camera/*camera_info` | `sensor_msgs/CameraInfo` | 内参输入 |
| 输入 ROS2 | 深度点云、Livox 点云 | `/hw/camera/depth_points`、`/hw/lidar/points` | `sensor_msgs/PointCloud2` | 可选增强输入 |
| 输出 ROS1 | `/hw/cmd_vel` | `/cmd_vel` | `geometry_msgs/Twist` | 默认拒绝，需显式启用 |
| 输入 ROS2 | `/tf`、`/tf_static` | 同名 | `tf2_msgs/TFMessage` | 适配器转发静态外参，滤除陈旧的 `odom→base`，并以最新官方里程计重建该动态边 |

官方有些版本将前视 RGB 发布为 `/camera/image_raw`；运行前必须用 `rostopic list` 确认实际源话题，
然后用环境变量设置对应 RGB 与 CameraInfo 源。不能同时把两个物理相机混到一个
`/hw/camera/image_raw`：

~~~bash
export OFFICIAL_SIMENV_RGB_TOPIC=/camera/image_raw
export OFFICIAL_SIMENV_RGB_CAMERA_INFO_TOPIC=/camera/camera_info
~~~

深度源、深度内参、容器 ROS1 setup 和既有 dynamic bridge 启动入口也可分别通过
`OFFICIAL_SIMENV_DEPTH_TOPIC`、`OFFICIAL_SIMENV_DEPTH_CAMERA_INFO_TOPIC`、`SIMENV_ROS1_SETUP` 和
`SIMENV_ROS1_BRIDGE_COMMAND` 覆盖。未发现实际源话题时不得把验证失败归因于 ROS2 算法。

## 文件与运行

- `scripts/official_simenv_rosbridge_ros2_adapter_node.py`：ROS2 主机 WebSocket 适配、分片重组、安全速度门和状态审计。

## 2026-07-15 已验证 headless 接入

负责人姜晨已在独占 `simenv_run` 中验证：Xvfb 在 `gzserver` 前启动，`DISPLAY=:99` 与
`LIBGL_ALWAYS_SOFTWARE=1` 生效；ROS1 RGB、深度、内参、`/scan`、`/clock`、里程计均有真实消息。
原始高频 `/Odometry_gazebo` 不能直接交给 rosbridge：它会使新 WebSocket 客户端接收启动期旧位姿。
官方启动补丁新增 `/hazardwalker/odom` 最新值 20 Hz 中继，ROS2 适配器默认订阅该话题；若接入旧环境，
必须显式设置 `OFFICIAL_SIMENV_ODOM_TOPIC=/Odometry_gazebo`，且不得将其当作稳定导航验收。

本次证据显示 ROS2 `/hw/cmd_vel` 已实际驱动 A1：前进 0.872 m、转向 0.515 rad、停止漂移 0.0014 m；
RGB 为 640×480 `rgb8`，深度为 640×480 `32FC1`，相机内参已进入 `/hw/*`。详见
`reports/platform/official_simenv_ros1_ros2/20260715_headless_native_and_ros2_acceptance/`。
- `scripts/run_official_simenv_rosbridge_adapter.sh`：在 ROS2 主机启动 rosbridge 双向适配器。
- `scripts/run_official_simenv_ros1_adapter.sh`：兼容旧入口，转发到前一启动脚本，不再假设容器内存在 dynamic_bridge。
- `scripts/verify_official_simenv_ros1_adapter.sh`：逐段检查原始 ROS1 与 ROS2 `/hw/*`。
- `scripts/run_official_simenv_ros1_ros2_stack.sh`：官方容器已启动后的 ROS2 业务入口。
- `ros2_ws/src/hazardwalker_bringup/launch/official_simenv_business.launch.py`：只启动业务节点，不启动
  fake 平台或 Harmonic。

示例（必须由平台组先启动共享官方场景）：

~~~bash
export ROS_DOMAIN_ID=17
export SIMENV_CONTAINER=simenv_ros1_hazard_platform
bash ./scripts/run_official_simenv_rosbridge_adapter.sh
bash ./scripts/verify_official_simenv_ros1_adapter.sh
OFFICIAL_SIMENV_ENABLE_CONTROL=1 bash ./scripts/run_official_simenv_ros1_adapter.sh
bash ./scripts/verify_official_simenv_ros1_adapter.sh --control
bash ./scripts/run_official_simenv_ros1_ros2_stack.sh
~~~

控制默认关闭。只有确认 `/cmd_vel` 的真实订阅节点含 `junior_ctrl` 或
`unitree_gazebo_servo` 后才允许设置 `OFFICIAL_SIMENV_ENABLE_CONTROL=1`；中继以墙钟在断流后发送
零速度，避免 `/clock` 暂停导致看门狗失效。

ROS1 直连控制必须先于双向适配执行，并需预约独占场景：

~~~bash
OFFICIAL_SIMENV_EXCLUSIVE_SESSION=1 \
OFFICIAL_SIMENV_VIDEO_REFERENCE='共享盘/20260714_ros1_direct.mp4' \
./scripts/verify_official_simenv_ros1_direct_control.sh --run
~~~

脚本自动生成同一轮的控制器信息、前后里程计、`summary.json`、CSV 和 README；位移不足 1m、转角不足
0.2 rad 或未提供视频引用时，摘要结论仍为 `incomplete`，不能进入跨 ROS 验收。

## 验收顺序与证据

1. **ROS1 直连优先**：持续发布非零 Twist，记录 `/cmd_vel` 回显、控制器输入、前后
   `/Odometry_gazebo`、关节/控制器日志和视频。前进、转向、停止及前进至少 1 m 都必须通过。
2. **传感器单段**：逐项记录 ROS1 原话题与 ROS2 `/hw/*` 的 `echo --once`，并保存时间戳、编码、
   内参和深度有效率。
3. **控制单段**：ROS2 发布、ROS1 中继状态的 `forwarded_cmd_count`、ROS1 `/cmd_vel`、控制器最终输入、
   真实里程计变化必须同一时间窗记录。
4. **业务闭环**：仅在前三项通过后，运行导航、感知、决策；输出 `results/detected_danger.json` 与
   截图/视频、summary、测试 CSV 和 README。

`forwarded_cmd_count`、bridge 订阅数或模式切换日志仅能证明链路局部存在，不能证明实体运动。

## 当前结论与风险

截至 2026-07-15，ROS1 原生前进 1.1766 m、跨 ROS2 前进 0.8723 m、转向和停止，以及 RGB-D、内参、
里程计进入 `/hw/*` 均已完成真实验收；证据见
`reports/platform/official_simenv_ros1_ros2/20260715_headless_native_and_ros2_acceptance/`。同日一键业务栈
在独立 ROS 域验证了唯一命令发布者、唯一适配器订阅者和 SIGTERM 无残留回收，见
`reports/platform/official_simenv_ros1_ros2/20260715_oneclick_stack_lifecycle/`。

这不等于完整任务闭环：当前固定航点仅为接口诊断，尚未完成真实楼宇自主探索、红球搜索、多视角确认、
三维定位和结果文件验收。不得把节点数、短窗口命令或启动日志表述为上述任务已通过。

候选补丁位于 `patches/`：分别修复 headless 渲染、显式 headless RL 模式和 IOROS 回调执行器生命周期。
补丁必须在官方 SimEnv 独立副本审查、编译、备份后由平台组应用；失败可通过 `git apply -R` 回滚，
不得在共享运行目录直接覆盖源码。
