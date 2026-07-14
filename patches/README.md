# 外部平台候选补丁

本目录保存对官方 SimEnv 源码的候选补丁，**不会**由 HazardWalker 自动应用到共享平台。平台组应在官方
SimEnv 独立副本中执行 `git apply --check`、备份、编译和回归；验证失败可用 `git apply -R` 回滚。

- `official_simenv_cmd_vel_async_spinner.patch`：让 `IOROS` 的订阅执行器随对象存活，排查 `/cmd_vel`
  回调生命周期问题。未完成官方编译和运动实测。
- `official_simenv_headless_camera_and_cmd_vel.patch`：在 `GUI=false` 时显式启用 Gazebo Classic headless
  rendering，针对已记录的深度相机渲染关闭问题。未完成 RGB-D 发布实测。
- `official_simenv_headless_auto_rl.patch`：显式 `SIMENV_AUTO_RL=1` 时让 headless 控制器进入已有 RL
  模式；默认关闭，未完成控制回归。

旧版 JSON 管道不能恢复：它没有传输/重组 RGB 与深度 `Image.data`，且 `/hw/cmd_vel` 是空回调。新适配
使用 ROS 原生消息与 `ros1_bridge dynamic_bridge`；控制验证必须以实际里程计和视频为准。
