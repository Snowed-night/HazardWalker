# 官方 SimEnv ROS2 rosbridge 运行验收

负责人：姜晨。平台组协作。

本目录保存宿主机 ROS2 Jazzy 与隔离官方 ROS1 SimEnv 的实际互通记录。ROS2 适配器运行在宿主机，
通过 Docker 映射的 `9091 -> 9090` 连接容器内 rosbridge；因 rosbridge 校验 Host 头，使用
`rosbridge_host_header=127.0.0.1:9090`。

## 已验证

- ROS1 `/Odometry_gazebo` 到 ROS2 `/hw/odom`，以及 RGB、深度、两组 CameraInfo 到 `/hw/camera/*`
  均收到完整尺寸和字节数。
- 在禁用高带宽图像订阅的隔离控制轮次中，ROS2 `/hw/cmd_vel` 经适配器到 ROS1 `/cmd_vel`，以 ROS2
  `/hw/odom` 记录前进 **1.0051 m**；状态中 `forwarded_cmd_count=1120`，最后发送零速度。
- 另一控制轮记录到 ROS2 转向 **0.2553 rad**；ROS1 端监控到非零/零速度与关节命令。
- 在**全量 RGB-D、两组内参与里程计同时转发**时，ROS2 `/hw/cmd_vel` 使 A1 连续移动
  **0.5028 m**；其后零速度的 2 s 漂移为 **0.0955 m**。该轮收到 RGB **4** 帧、深度
  **9** 帧和动态 TF **3733** 条，适配器全程无重连；一个不完整深度帧被安全丢弃，未中断控制。
  原始 640×480 图像经过 rosbridge 以 **500 ms** 节流，避免下一帧覆盖未完成的分片。
- ROS1 `/tf`、`/tf_static` 已进入 ROS2；`odom→base` 与 `/Odometry_gazebo` 不一致的控制器
  估计变换被过滤，实际可查询 `world→map→odom→base→real_sense`，因此深度投影不再依赖虚构静态世界位姿。
- 当前分支的 HSV 节点已实际处理 640×480 `rgb8` 帧，并通过最新 TF 回退取得 `world` 位姿；
  本轮镜头中没有红球，结果为 0 检测，只证明输入管线，不证明识别闭环。

## 文件

- `hw_runtime_sensor_check.json`：完整 RGB-D、内参、里程计和适配器状态。
- `hw_forward_one_meter.json`：隔离 ROS2 `/hw/cmd_vel` 前进 1 m 与零速度尾迹。
- `hw_control_acceptance.json`：转向、零速度保持和控制转发计数。
- `hw_full_rgbd_soak_check.json`：修复分片边界后全量 RGB-D 空载连续 35 s 的实际 ROS2 接收与零错误日志检查。
- `hw_concurrent_rgbd_control_acceptance.json`：隔离容器内全量 RGB-D 与 ROS2 控制并发的里程计、
  帧数、适配状态和停止尾迹。
- `hw_tf_relay_check.json`：ROS2 实际接收的动态/静态 TF 样本和 `odom→base`、`base` 链检查。
- `hw_perception_pipeline_check.json`：业务 HSV 节点处理真实 RGB 与 TF 的部分验收记录。
- `shared_host_contention_diagnosis.json`：三套 Gazebo 并发时的 CPU、仿真时间与近入口控制诊断；
  只说明本机资源竞争，不作为导航性能结论。

## 限制

全量 RGB-D 已完成空载 35 s 和与控制并发的实际回归。官方 rosbridge 会把 RGB、深度的 fragment
都写为 `id=0`；适配器现为两路图像各使用独立 WebSocket，实测可同时收到完整 640×480 RGB 与深度帧，
且状态中的 `dropped_invalid_image_frames` 为 `{}`。接收线程只保留最新消息并由 ROS2 执行器发布，
避免跨线程 DDS 投递失效。

原始 RGB-D 的 JSON/base64 转发仍会显著占用一个 CPU 核，因此验证时可保守设为 1 Hz；正式高帧率链路
应改为容器内 ROS1 感知或二进制/压缩桥接，不能把 Python rosbridge 作为比赛高帧率方案。更高帧率、
ROS2 导航/感知/决策完整闭环仍未运行，不能据此宣称任务完成。
