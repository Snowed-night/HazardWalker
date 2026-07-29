# 官方 SimEnv noVNC 图形界面与键盘控制验收

日期：2026-07-27。范围：在不重启、不停止共享官方仿真容器的前提下，为既有 Gazebo 服务端提供可观看的图形界面，并验证键盘控制可同时生效。

## 环境与目的

- 既有仿真容器：`simenv_ros1_hazard_platform`（全程保持运行）。
- 新增只读 GUI sidecar：`simenv_ros1_hazard_platform_gui`，镜像 `simenv_ros1-gui:noetic-focal`（`sha256:96db1a216f6abf8427d551456ae4024de2f2330ecda198b3f9a3594193dea034`）。
- Gazebo Master：`http://127.0.0.1:11345`；GUI 仅连接该现有 Master，不启动 `gzserver`、控制器或业务适配器。
- GUI 方案：sidecar 内独立 `Xvfb :100` + `llvmpipe` 软件渲染 + `gzclient` + `x11vnc` + noVNC/WebSockify。noVNC 与 VNC 只绑定 `127.0.0.1`，避免暴露公网端口。
- 目的：替代在 RDP/XWayland 上直接运行 `gzclient` 时的渲染段错误；操作者能够在远程浏览器看见仿真画面，同时在独占终端运行 ROS2 键盘节点控制机器狗。

## 方法

1. 复核原容器内 `DISPLAY=:99`、`LIBGL_ALWAYS_SOFTWARE=1` 下的 `gzclient` 可连接现有 Master 并持续运行；由此确认失败点是 RDP/XWayland 图形路径而非 Gazebo 服务端。
2. 启动独立 GUI sidecar，在 `:100` 上用软件渲染运行 `gzclient`，以 `http://127.0.0.1:6081/vnc.html` 提供 noVNC 页面。
3. 对该页面发起 HTTP 请求，检查 Xvfb、x11vnc、WebSockify 与 gzclient 存活；通过 VNC 原始帧保存 `gui_noVNC_final.png`。
4. GUI 持续运行时，启动 ROS2 键盘节点和临时备用 rosbridge 中继，伪终端输入 `W` 后输入 `K`、`Q`；记录 ROS1 `/cmd_vel` 与 `/Odometry_gazebo`。备用中继仅用于正式 ROS2 adapter 当前失活时的诊断，不替代正式 adapter。

## 结果

- `http://127.0.0.1:6081/vnc.html` 返回 HTTP 200；sidecar 内 `Xvfb`、`x11vnc`、`websockify` 与 `gzclient` 均存活。
- [gui_noVNC_final.png](gui_noVNC_final.png) 是 noVNC 的实际 Gazebo 画面：可见 Gazebo 菜单、场景面板、建筑物及激光可视化，证明不是空白桌面或静态占位页。
- GUI 运行期间，键盘节点将 `W` 解释为 `linear.x=0.30 m/s`，ROS1 `/cmd_vel` 收到 8 条非零速度，随后自动停止与 `K` 急停均发出零速度。
- 短按前后里程计从 `(-0.2772003, -0.3385831)` 变为 `(-0.2782511, -0.3381788)`，平面变化约 `0.00113 m`。该短窗口仅证明真实响应，不代表巡航速度性能。
- 验收结束后临时键盘进程和备用中继已清理；GUI sidecar 保持运行，供成员继续查看场景。正式共享仿真容器未被重启或停止。

## 使用与边界

在远程 RDP 桌面浏览器打开 `http://127.0.0.1:6081/vnc.html` 即可观看。键盘控制不应在 noVNC 页面内操作，而应在独占远程终端启动 `hazardwalker_platform.keyboard_control_node`，其只发布 `/hw/cmd_vel`；按键为 `W` 前进、`S` 后退、`A` 左转、`D` 右转、`K` 立即停止。

当前正式 ROS2 adapter 在本次验收窗口仍存在运行态失活：节点图可见，但其 `/hw/odom`、状态主题和参数服务未在超时窗口响应。因此本报告只确认 GUI 和备用控制诊断链路，不将其表述为 ROS2 业务适配器整体可用。恢复正式 adapter 后必须停止备用中继，避免两个速度桥接器并行。

启动、停止、端口与 SSH 隧道说明已写入 [官方 SimEnv 平台环境使用手册](../../../../docs/guidebook/官方SimEnv平台环境使用手册.md)。
