# 官方 SimEnv ROS1 ↔ ROS2 双向适配整改

负责人：姜晨。平台组协作。

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
| 输入 ROS2 | `/Odometry_gazebo` | `/hw/odom` | `nav_msgs/Odometry` | 必须以实际位姿变化验证 |
| 输入 ROS2 | `/real_sense/rgb/image_raw` | `/hw/camera/image_raw` | `sensor_msgs/Image` | 可按官方实际传感器话题调整 |
| 输入 ROS2 | `/real_sense/depth/image_raw` | `/hw/camera/depth_image` | `sensor_msgs/Image` | RGB-D 定位输入 |
| 输入 ROS2 | RGB/深度 `camera_info` | `/hw/camera/*camera_info` | `sensor_msgs/CameraInfo` | 内参输入 |
| 输入 ROS2 | 深度点云、Livox 点云 | `/hw/camera/depth_points`、`/hw/lidar/points` | `sensor_msgs/PointCloud2` | 可选增强输入 |
| 输出 ROS1 | `/hw/cmd_vel` | `/cmd_vel` | `geometry_msgs/Twist` | 默认拒绝，需显式启用 |
| 规划中 | `/tf`、`/tf_static` | 同名 | `tf2_msgs/TFMessage` | 当前 rosbridge 适配器未实现；不能当作可用输入 |

官方有些版本将前视 RGB 发布为 `/camera/image_raw`；运行前必须用 `rostopic list` 确认实际源话题，
然后用环境变量设置对应 RGB 与 CameraInfo 源。不能同时把两个物理相机混到一个
`/hw/camera/image_raw`：

~~~bash
export OFFICIAL_SIMENV_RGB_TOPIC=/camera/image_raw
export OFFICIAL_SIMENV_RGB_CAMERA_INFO_TOPIC=/camera/camera_info
~~~

深度源、深度内参和 ROS2 环境脚本可分别通过
`OFFICIAL_SIMENV_DEPTH_TOPIC`、`OFFICIAL_SIMENV_DEPTH_CAMERA_INFO_TOPIC` 和
`OFFICIAL_SIMENV_ROS2_SETUP` 覆盖。未发现实际源话题时不得把验证失败归因于 ROS2 算法。

若官方容器通过 Docker 端口映射暴露 rosbridge（例如宿主机 `9091` 映射到容器内 `9090`），部分
rosbridge 会校验 WebSocket `Host` 头。此时 URL 使用宿主机端口，额外设置容器监听端口对应的头：

~~~bash
export OFFICIAL_SIMENV_ROSBRIDGE_URL=ws://127.0.0.1:9091
export OFFICIAL_SIMENV_ROSBRIDGE_HOST_HEADER=127.0.0.1:9090
~~~

## 文件与运行

- `scripts/official_simenv_rosbridge_ros2_adapter_node.py`：ROS2 主机 WebSocket 适配、分片重组、安全速度门和状态审计。
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
./scripts/run_official_simenv_rosbridge_adapter.sh
./scripts/verify_official_simenv_ros1_adapter.sh
OFFICIAL_SIMENV_ENABLE_CONTROL=1 ./scripts/run_official_simenv_ros1_adapter.sh
./scripts/verify_official_simenv_ros1_adapter.sh --control
./scripts/run_official_simenv_ros1_ros2_stack.sh
~~~

控制默认关闭。只有确认 `/cmd_vel` 的真实订阅节点含 `junior_ctrl` 或
`unitree_gazebo_servo` 后才允许设置 `OFFICIAL_SIMENV_ENABLE_CONTROL=1`；中继以墙钟在断流后发送
零速度，避免 `/clock` 暂停导致看门狗失效。

### 官方 headless 控制器的必要启动约束

官方 `auto.sh` 不是“设为 `PAUSED=false` 就可控”的入口。实测中，未暂停启动会在 Gazebo 服务和
首个关节状态尚未就绪时直接退出控制器启动；手动补启又必须带上 libtorch 运行库和 headless 状态，
否则分别出现 `libcublas.so.11` 缺失或“收到 `/cmd_vel` 却不运动”。官方容器的可复现顺序为：

1. 使用 `PAUSED=true`、`START_CONTROLLER=1`、`CONTROLLER_FOREGROUND=0`、
   `SIMENV_AUTO_RL=1`、`AUTO_UNPAUSE_AFTER_CONTROLLER=0` 启动；同时令
   `LD_LIBRARY_PATH` 包含 `/opt/libtorch/lib`。
2. 等待 `/gazebo/unpause_physics`、`/unitree_gazebo_servo`、`/rosbridge_websocket` 都出现，检查
   `rostopic info /cmd_vel` 的真实订阅者是 `unitree_gazebo_servo`。
3. 再显式执行一次 `rosservice call /gazebo/unpause_physics`，随后才启动 ROS2 适配与业务层。

不要在共享 host network 中做控制验收；遗留容器会向同一 ROS master 注入速度。验收容器应采用独立
Docker 网络和端口映射，例如 `127.0.0.1:9091 -> 9090`，同时设置前述 Host 头。

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

截至 2026-07-14，已经在独立 Docker 网络的官方 ROS1 容器中完成无污染直连验收：

- `/Odometry_gazebo`、`/real_sense/rgb/image_raw`（640×480 `rgb8`）、
  `/real_sense/depth/image_raw`（640×480 `32FC1`）、RealSense 内参和 `/scan` 都有真实消息；
  本轮 `/scan` 为 360 个样本，其中 198 个有限量程值。
- 通过官方 `junior_ctrl` 的 `SIMENV_AUTO_RL=1` 入口直接发布 ROS1 `/cmd_vel`，前进
  **1.0014 m**、转向 **0.2540 rad**。连续零速度收敛后两个 3 s 观察窗口均仅约 0.00031 m 位移、
  0.00018 rad 偏航变化，满足原生停止保持的验收记录。
- 完整原始数据、RGB-D 截图和测试表见
  `reports/platform/official_simenv_ros1_ros2/20260714_ros1_clean_direct_acceptance/`。

此前“传感器话题存在但无数据”不能归结为 TCPROS 或 rosbridge：已知启动错误包括把不完整的项目目录
挂到官方 SimEnv 路径，以及 LiDAR 的标准 Gazebo `<ray>` 配置层级错误。另一个独立问题是共享 host 网络
中的遗留容器持续向同一 ROS master 发布 `angular.z=-0.8`；这会污染停止、变速和桥接试验。因此所有
控制验收必须独占 ROS master 或使用隔离 Docker 网络，并记录 `/cmd_vel` 发布者。

在上述受控启动顺序下，宿主 ROS2 Jazzy 已产生逐段运行证据：ROS1 `/Odometry_gazebo`、RGB、深度和
双 CameraInfo 均稳定进入 `/hw/*`；全量 RGB-D 同时转发时，ROS2 `/hw/cmd_vel` 经 rosbridge 和
ROS1 `/cmd_vel` 驱动官方 A1 连续移动 **0.5118 m**，随后零速度保持，适配器无重连、无坏帧。详细
JSON、测试表与 README 位于
`reports/platform/official_simenv_ros1_ros2/20260714_ros2_rosbridge_runtime_acceptance/`。

为防止原始大图像分片覆盖，适配器默认将 RGB、深度订阅节流为 500 ms，并在状态中记录
`image_throttle_rate_ms` 与 `dropped_invalid_image_frames`。该设置已在并发轮次得到验证；调高帧率必须
重做同结构回归，不能只凭订阅数宣称稳定。

这证明双向平台链路可用，但**不等于完整比赛任务通过**：ROS2 导航自主探索、感知多视角确认、红色
非球体排除、三维定位和决策结果文件尚未在官方复杂楼宇场景联调，仍为 `not_run`。

另外，隔离验收所用官方 SimEnv 副本在诊断阶段存在人工源码改动；候选补丁目录尚未覆盖其全部精确差异。
下一位维护者不能直接复制共享运行目录或盲目应用旧补丁，应先在干净官方副本导出、审查、编译这些差异，
再以同一套验收脚本回归。这是可复现性的待办项，不影响本目录中已经保存的 ROS1 运行时观测结论。

候选补丁位于 `patches/`：分别修复 headless 渲染、显式 headless RL 模式和 IOROS 回调执行器生命周期。
补丁必须在官方 SimEnv 独立副本审查、编译、备份后由平台组应用；失败可通过 `git apply -R` 回滚，
不得在共享运行目录直接覆盖源码。
