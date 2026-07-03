# Gazebo 远程 GUI 分离操作指南

更新时间：2026-07-03  
适用对象：**平台与仿真组、导航探索组、感知定位组、系统集成与测试组**  
主力机：`hxbl`（Tailscale `100.102.91.31`）

> 主力机 RDP 内无法稳定打开 Gazebo 原生 3D 窗口。本指南说明如何在 **hxbl 上 headless 跑仿真**、在 **本机用 `gz sim -g` 看原生画面并操作**。

---

## 1. 原理（30 秒读懂）

```text
┌──────────────────────── hxbl ────────────────────────┐
│  你的 hazard_* 账号                                   │
│  gz sim -s（headless server）+ ros2 算法节点          │
│  GZ_PARTITION=本组专用名  ROS_DOMAIN_ID=本组 ID        │
└──────────────────────────┬───────────────────────────┘
                           │  gz-transport（Tailscale）
┌──────────────────────────▼───────────────────────────┐
│  你的笔记本电脑 / 台式机                              │
│  gz sim -g（GUI client）→ 原生 Gazebo 3D 窗口在本机   │
└──────────────────────────────────────────────────────┘
```

- **仿真在 hxbl 算**，不占 RDP 的 OpenGL。  
- **画面在你本机 GPU 渲染**，可拖拽视角、暂停/播放、看场景树。  
- **四组可同时用**：各组 `GZ_PARTITION` 与 `ROS_DOMAIN_ID` 不同即可。

---

## 2. 四组参数对照表

| 组别 | Linux 账号 | `ROS_DOMAIN_ID` | 建议 `GZ_PARTITION` |
| --- | --- | --- | --- |
| 平台与仿真组 | `hazard_platform` | **11** | `hazardwalker_platform` |
| 导航探索组 | `hazard_nav` | **12** | `hazardwalker_nav` |
| 感知定位组 | `hazard_perception` | **13** | `hazardwalker_perception` |
| 系统集成与测试组 | `hazard_test` | **20** | `hazardwalker_test` |

**规则：**

- `ROS_DOMAIN_ID`：隔离 ROS 2 话题（`/hw/*` 等），**必须与同组 server 一致**。  
- `GZ_PARTITION`：隔离 Gazebo 传输，**server 与 client 必须完全一致**；**不同组必须用不同 partition 名**，否则会连错仿真。  
- 不要用默认值（默认 partition 含主机名，本机与 hxbl 不同会对不上）。

---

## 3. 本机前置条件

在 **你自己的电脑** 上准备：

1. **安装 Gazebo Harmonic 8.x**（与 hxbl 大版本一致，当前为 8.12.x）  
   - Ubuntu 24.04：随 ROS 2 Jazzy 的 `gz-sim` 包  
   - Windows / macOS：见 [Gazebo Harmonic 安装文档](https://gazebosim.org/docs/harmonic/install/)  
2. **仓库版本与 hxbl 一致**（至少同步 `worlds/`、`models/`）  
   ```bash
   git clone git@github.com:Snowed-night/HazardWalker.git
   git switch feature/platform   # 平台组；其他组按各自分支
   git pull
   ```  
3. **Tailscale 已连通 hxbl**（推荐；跨网时需 `GZ_RELAY`，见下文）  
4. 查本机 Tailscale IP（后面 `GZ_IP` 要用）：  
   ```bash
   # macOS / Linux
   tailscale ip -4
   ```

---

## 4. 主力机（hxbl）操作 — Server 端

SSH 登录 **本组** `hazard_*` 账号后执行。

### 4.1 平台组示例（其他组替换 partition 与 `ROS_DOMAIN_ID`）

```bash
source /opt/ros/jazzy/setup.bash
source ~/HazardWalker/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=11
export GZ_PARTITION=hazardwalker_platform

ros2 launch hazardwalker_platform gazebo_minimal.launch.py headless:=true
```

### 4.2 不用 launch、仅起空世界（连通性测试）

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=11
export GZ_PARTITION=hazardwalker_platform

gz sim -s -r /opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/empty.sdf -v 2
```

### 4.3 四组对照（复制即用）

**导航组：**

```bash
export ROS_DOMAIN_ID=12
export GZ_PARTITION=hazardwalker_nav
# 再启动本组仿真 launch 或 gz sim -s ...
```

**感知组：**

```bash
export ROS_DOMAIN_ID=13
export GZ_PARTITION=hazardwalker_perception
```

**集成与测试组：**

```bash
export ROS_DOMAIN_ID=20
export GZ_PARTITION=hazardwalker_test
```

### 4.4 注意

- **不要**在 RDP 终端里运行 `gz sim -r`（无 `-s`），会失败或拖垮远程桌面。  
- Server 保持运行；关掉终端前用 `Ctrl+C` 结束 launch。  
- 若用平台组世界文件，确保已 `colcon build` 且 `GZ_SIM_RESOURCE_PATH` 在 launch 里已配置（`gazebo_minimal.launch.py` 已处理）。

---

## 5. 本机操作 — GUI Client 端

在 **hxbl server 已启动** 后，在本机 **新开一个终端**（不要 SSH 进 hxbl）。

### 5.1 通用模板（把分区名换成上表）

```bash
# 与 hxbl server 相同的 partition
export GZ_PARTITION=hazardwalker_platform

# 本机 Tailscale IP（必填，用 tailscale ip -4 查看）
export GZ_IP=<你的本机 Tailscale IP>

# hxbl 的 Tailscale IP（跨网 / 经 Tailscale 时建议设置）
export GZ_RELAY=100.102.91.31

# 可选：加载本机 ROS（若需在本机同时开 rviz2 等）
# source /opt/ros/jazzy/setup.bash

gz sim -g -v 2
```

### 5.2 四组本机环境变量

| 组别 | 本机执行前设置 |
| --- | --- |
| 平台组 | `export GZ_PARTITION=hazardwalker_platform` |
| 导航组 | `export GZ_PARTITION=hazardwalker_nav` |
| 感知组 | `export GZ_PARTITION=hazardwalker_perception` |
| 集成测试组 | `export GZ_PARTITION=hazardwalker_test` |

`GZ_IP`、`GZ_RELAY` 四组相同（本机 IP + hxbl `100.102.91.31`）。

### 5.3 成功现象

- 本机弹出 **Gazebo Sim** 窗口，能看到 hxbl 上正在跑的世界。  
- 可旋转视角、播放/暂停仿真（与直连 GUI 类似）。  
- hxbl 上其他组的 RDP / headless 仿真 **不受影响**。

### 5.4 本机未装 ROS 时

仅看仿真、不跑本机 ROS 节点时，**只装 `gz sim` + 同步 world/models 即可**，不强制装完整 ROS 2 Jazzy。

---

## 6. 推荐日常流程（一组两人协作也适用）

```text
1. SSH 上 hxbl（本组 hazard_*）
2. 启动 headless launch（第 4 节）
3. 本机开 gz sim -g（第 5 节）
4. 需要 RViz / 算法联调：hxbl 上另开终端跑节点，或本机 ROS 用相同 ROS_DOMAIN_ID（仅当本机也装了 ROS 且网络可达时）
5. 结束：本机关 GUI 窗口 → hxbl Ctrl+C 停 launch
```

---

## 7. 常见问题

### 7.1 本机 GUI 空白、连不上 server

| 检查项 | 做法 |
| --- | --- |
| partition 不一致 | hxbl 与本机 `echo $GZ_PARTITION` 必须相同 |
| `GZ_IP` 未设或错误 | 设为本机 **Tailscale** IP，不是 `127.0.0.1` |
| 未设 `GZ_RELAY` | 跨网时加上 `export GZ_RELAY=100.102.91.31` |
| server 未启动 | hxbl 上确认 `gz sim -s` 或 launch 在跑 |
| 防火墙 | 确保 Tailscale 互通；不要只靠未放行的公网 IP |

### 7.2 连上但场景/模型缺失

- 本机 clone 的 `HazardWalker` 与 hxbl **commit 尽量一致**。  
- 平台组世界依赖 `models/`，本机需有同路径或通过 `GZ_SIM_RESOURCE_PATH` 指向本机 `models` 目录。

### 7.3 连到别人的仿真

- 说明 `GZ_PARTITION` 与别组重复或拼错；改回上表 **本组专用名**。

### 7.4 本机 `gz sim` 版本不对

```bash
gz sim --versions   # 应接近 8.12.x
```

与 hxbl 差太多时，升级本机 Gazebo Harmonic。

### 7.5 仍想在 hxbl 上看窗口

RDP 内 **不要** `gz sim -r`。可选：机房本机显示器、或 headless + 本指南（推荐）。

---

## 8. 环境变量持久化（可选）

经常使用时，可写入本机 `~/.bashrc` 或 `~/.zshrc`（**只写本组 partition，勿抄别组**）：

```bash
# 平台组示例
export GZ_PARTITION=hazardwalker_platform
export GZ_RELAY=100.102.91.31
# GZ_IP 因机器而异，可写脚本按 tailscale ip -4 自动设置
```

hxbl 上写入对应账号的 `~/.bashrc` / `~/.zshrc`：

```bash
export ROS_DOMAIN_ID=11          # 按组修改
export GZ_PARTITION=hazardwalker_platform
```

---

## 9. 相关文档

- [远程桌面使用指南.md](environment/远程桌面使用指南.md) — RDP 连接  
- [主力机环境搭建.md](主力机环境搭建.md) — `ROS_DOMAIN_ID` 与账号说明  
- [Gazebo Transport 环境变量](https://gazebosim.org/api/transport/14/envvars.html) — 官方 `GZ_IP` / `GZ_PARTITION` / `GZ_RELAY` 说明

---

*四组并发 headless 仿真 + 各组独立 partition 已在 hxbl 实测通过（2026-07-03）。*
