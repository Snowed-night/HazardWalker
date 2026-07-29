# 官方 SimEnv 远程键盘控制与备用中继验收

日期：2026-07-27。范围：远程 ROS2 键盘节点、ROS2 `/hw/cmd_vel`、rosbridge、ROS1 `/cmd_vel` 与 A1 仿真控制器。此记录不包含 GUI、SLAM、导航或感知结论。

## 背景与目的

本记录完成时，直接在 RDP/XWayland 运行 `gzclient` 会稳定渲染崩溃，因此本轮先验证远程终端键盘控制：`W/S/A/D/K` 经 ROS2 业务接口驱动官方 A1；同时记录完整适配器失活时的恢复边界。随后已在同日补充独立 Xvfb/noVNC GUI profile，见相邻的 `20260727_novnc_gui_profile` 记录；本记录的控制证据仍有效。

## 环境与方法

- 容器：`simenv_ros1_hazard_platform`，状态 `healthy`；本轮开始时 `/hw/cmd_vel` 无 ROS2 发布者、仅有正式适配器订阅者。
- ROS1：`/hazardwalker/odom` 和 `/Odometry_gazebo` 有新消息；`/cmd_vel` 的唯一控制订阅者是 `/unitree_gazebo_servo`。
- 正式 ROS2 适配器：仍出现在 ROS 图中，但 `/hw/odom`、`/hw/platform/official_simenv_adapter_status` 和参数服务在超时窗口内均没有响应；键盘节点发布后 ROS1 `/cmd_vel` 也没有消息。因此不能把“节点存在”当作适配器可用证据。
- 恢复方式：不重启容器、不终止正式适配器；临时启动仅订阅 `/hw/cmd_vel` 的 rosbridge 备用中继。备用中继连接 `ws://127.0.0.1:9090` 后，以伪终端注入 `W`，保持 0.5 秒，再注入 `K` 与 `Q`。
- 速度：键盘默认 `linear.x=0.30 m/s`；节点在 0.35 秒没有重复按键时自动发零速度，随后 `K` 再连续发三次零速度。

## 结果

- 键盘节点日志确认 `W` 被解释为 `linear.x=0.30`，随后出现“按键超时，已自动停止”和 `K 急停`。
- ROS1 `/cmd_vel` 实测收到连续 8 条 `linear.x=0.30`，之后收到多条全零 `Twist`；这证明备用中继和 rosbridge 控制方向闭环成功。
- 诊断里程计由 `(-0.2740633, -0.3393825)` 变化至 `(-0.2762448, -0.3385741)`，平面位移约 `0.00233 m`，方向与当前机身前向一致。短按主要覆盖起步阶段，不应将该位移误写成巡航速度性能。
- 结束后 `/hw/cmd_vel` 发布者数为 0；临时备用中继、隔离 venv 与远程临时目录均已清理。

## 结论与边界

远程键盘事件、`/hw/cmd_vel`、ROS1 `/cmd_vel`、控制器和零速度停止已经形成可复现实证链路。正式完整适配器的运行态失活仍是平台缺陷：恢复后应停止备用中继，避免并行速度桥接。GUI 已由后续的独立 Xvfb/noVNC profile 提供；本记录本身仍使用 headless 里程计作为控制验收依据。
