# HazardWalker 仿真平台 — 接口桥接测试报告

> 测试日期：2026-07-06  
> 测试环境：Docker `simenv_ros1_hazard_platform`（ROS1 Noetic + Gazebo Classic headless）  
> 桥接方案：`docker_pipe.py`（ROS1 JSON 管道）→ `hw_bridge.py`（宿主机 ROS2 Jazzy）

---

## 一、测试结果汇总

| /hw/* 话题 | 数据 | 频率 | 状态 |
|-----------|------|------|------|
| `/hw/Odometry_gazebo` | ✅ | ~85 Hz | 正常 |
| `/hw/trunk_imu` | ✅ | ~1000 Hz | 正常 |
| `/hw/livox/imu` | ✅ | ~1000 Hz | 正常 |
| `/hw/tf` | ✅ | — | 正常 |
| `/hw/livox/Pointcloud2` | ❌ | — | LiDAR 传感器无输出（headless） |
| `/hw/real_sense/rgb/image_raw` | ❌ | — | 相机无渲染（headless） |
| `/hw/real_sense/depth/image_raw` | ❌ | — | 相机无渲染（headless） |
| `/hw/real_sense/depth/points` | ❌ | — | 相机无渲染（headless） |
| `/hw/cmd_vel` | ✅ | — | 可订阅/写入 |

---

## 二、服务测试

| 服务 | 命令 | 结果 |
|------|------|------|
| 开关门 | `hw_service_call.sh door main_entrance true` | `accepted=True state=open` ✅ |
| 呼叫电梯 | `hw_service_call.sh elevator elevator_main 1 true` | `accepted=True floor=1 state=door_open` ✅ |

---

## 三、桥接架构

```
Docker (ROS1 Noetic)              宿主机 (ROS2 Jazzy)
─────────────────────              ─────────────────────
/Odometry_gazebo ──┐
/trunk_imu ────────┤  docker_pipe.py    hw_bridge.py
/livox/imu ────────┼──→ JSON stdout → 解析 → /hw/* (ROS2)
/tf ───────────────┤
/livox/Pointcloud2 ┤  (无源数据, LiDAR headless限制)
/real_sense/* ─────┘  (无源数据, 相机headless限制)
```

- `docker_pipe.py`：Docker 内 ROS1 订阅器，输出 JSON 到 stdout
- `hw_bridge.py`：宿主机 Python 进程，读 docker pipe、转 ROS2 `/hw/*`

---

## 四、已知问题

| # | 问题 | 根因 | 修复方向 |
|---|------|------|----------|
| 1 | LiDAR 点云无数据 | Gazebo headless 无 GPU 渲染 | 切换 xvfb 镜像 |
| 2 | RealSense 相机无数据 | 同上 | 同上 |
| 3 | `junior_ctrl` 崩溃 | 缺 RL 模型文件 | 获取 `.pt` 模型 |
| 4 | ros1_bridge DDS 不通 | Foxy/Jazzy 跨版本不兼容 | 改用 docker pipe 方案 |
