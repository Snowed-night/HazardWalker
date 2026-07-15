# 官方 SimEnv ROS1 直连控制验收

负责人：姜晨。该目录由 `verify_official_simenv_ros1_direct_control.sh --run` 自动生成。

- `cmd_vel_info_*.txt`：控制话题订阅端快照。
- `unitree_gazebo_servo_before.txt`：A1 控制器节点快照。
- `odom_*.csv`：同轮控制前后里程计。
- `summary.json`：自动计算的位移、转角和证据完整性。
- `testing_record_platform.csv`：测试组记录。

没有视频引用、前后里程计或至少 1m 位移时，结论必须保持 `incomplete` 或 `invalid`。

## 2026-07-15 合并验收补充

负责人：姜晨。该轮在唯一的 `simenv_run` 容器内执行，先完成 ROS1 原生验收，再启用 ROS2 适配。

- ROS1 原生：`summary.json` 记录直接 `/cmd_vel` 前进 1.1766 m、转向 0.4096 rad；
  `ros1_two_speed_test.json` 记录第二个不同速度命令实际生效。
- ROS2 闭环：`ros2_full_control_acceptance.json` 记录 `/hw/cmd_vel → ROS1 /cmd_vel → A1`
  的真实前进 0.8723 m、转向 0.5149 rad、停止漂移 0.0014 m。
- ROS2 RGB-D：`ros2_hw_sensor_acceptance.json` 记录 640×480 `rgb8`、640×480 `32FC1`、
  两路内参和当前里程计均已进入 `/hw/*`。
- `camera_before.png`、`camera_after.png` 是原生相机截图；其余 CSV/TXT/JSON 为可复核原始记录。

本目录不宣称导航、感知、决策完整任务已经跑通；它证明官方平台已具备三模块联调所需的真实控制和
RGB-D 输入。图像在 rosbridge JSON 通道按 3 秒低频节流，以避免大帧传输拖慢 Gazebo；联调时可按
实际资源逐步提高速率并重新记录帧率。
