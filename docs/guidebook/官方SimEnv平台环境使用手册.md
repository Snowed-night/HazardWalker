# 官方 SimEnv 平台环境使用手册

- 维护：项目负责人、平台与仿真组
- 本轮控制链路与键盘规范修改人：负责人（姜晨）
- 适用：平台、导航、感知、决策与集成测试成员
- 环境：官方 ROS1 Noetic + Gazebo Classic + Unitree A1，HazardWalker ROS2 Jazzy 业务层

本文只说明日常操作。实现原理和历史整改记录分别见：

- [平台组历史整改记录](../groups/platform/history/官方SimEnv_ROS1_ROS2双向适配整改_20260714.md)
- [官方传感器与话题说明](../../ros2_ws/src/hazardwalker_platform/docs/sensors-and-topics.md)
- [平台补丁说明](../../patches/README.md)

## 1. 平台结构与使用边界

```text
官方 SimEnv Docker（ROS1、Gazebo、A1、官方底层运动策略）
                  │ rosbridge_websocket
                  ▼
HazardWalker ROS2 适配器
                  │ /hw/*
                  ▼
SLAM、导航、感知、决策与证据记录
```

- 业务节点统一使用 `/hw/*`，不要绑定临时容器内部实现。
- 官方底层策略只负责 A1 运动，不等于团队完成了 SLAM、探索或识别。
- `results/danger_truth.json` 是裁判真值，运行期算法严禁读取。
- 控制、导航和整场闭环必须独占一个 SimEnv 场景；只读调试也应避免占满 Gazebo 和网络资源。
- 容器 `Up`、节点存在或日志出现 `success` 均不等于平台验收通过。

| 角色 | 主要权限 |
|---|---|
| 普通成员 | 查看状态、启动自己的业务节点；不得重启共享容器或抢占控制 |
| 平台管理员 | 启停官方环境、验收传感器与控制器、安排独占时段 |
| 导航组 | 在独占时段使用控制、SLAM 和 Frontier |
| 感知组 | 使用 RGB-D、内参和合法 SLAM 位姿；不得在线读取真值 |
| 测试组 | 在同一时间窗保存视频、轨迹、图像、日志和结构化结果 |

## 2. 首次构建

在 HazardWalker 仓库根目录执行：

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  hazardwalker_platform hazardwalker_nav hazardwalker_perception \
  hazardwalker_decision hazardwalker_bringup
cd ..
```

构建失败时不得继续使用旧的 `ros2_ws/install/` 验证新代码。使用离线 Cartographer 时，按平台管理员
提供的实际路径设置：

```bash
export OFFICIAL_SIMENV_CARTOGRAPHER_PREFIX=/path/to/cartographer/prefix
```

该目录必须包含 `share/cartographer_ros`；只做传感器或感知检查时不需要设置。

## 3. 每轮实验的准备

### 3.1 设置本轮环境

容器名和 ROS 域由平台管理员分配，以下仅为示例：

```bash
export SIMENV_CONTAINER=simenv_ros1_hazard_platform
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

同一轮所有 ROS2 节点必须使用同一个 `ROS_DOMAIN_ID`。不同 ROS 域不能解决多个团队同时控制同一
Gazebo 场景的问题。

### 3.2 查看状态并确认独占

```bash
docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
docker inspect -f '{{.State.Running}}' "$SIMENV_CONTAINER"

bash scripts/check_official_simenv_exclusive_session.sh \
  --container "$SIMENV_CONTAINER" --require-exclusive
```

检查脚本只读，不会停止或删除容器。失败时由容器所有者清理残留实例；普通成员不要自行重置共享环境。

### 3.3 平台管理员重建并启动正式容器

确认独占且无人实验后，首次使用或修改镜像依赖时执行一次干净重建：

```bash
cd ros2_ws/src/hazardwalker_platform
./auto_docker.sh status
./auto_docker.sh down
./auto_docker.sh image --no-cache
./auto_docker.sh build force
./auto_docker.sh up
./auto_docker.sh logs
cd ../../..
```

`auto_docker.sh up` 现在唯一调用容器内的 `auto.sh`：它启动 Gazebo、`junior_ctrl`、
`/Odometry_gazebo -> /hazardwalker/odom` 最新值中继和 `rosbridge_websocket`。镜像已固定包含
`ros-noetic-rosbridge-server` 与 `expect`；不得再进入容器手工安装软件包、手工启动 rosbridge 或手工拉起控制器。
默认 `START_CONTROLLER=1`、`SIMENV_AUTO_RL=1`、`SIMENV_HEADLESS_MODE=move_base`、`START_ROSBRIDGE=1`、`START_ODOM_RELAY=1`，并在控制器日志确认
`[HEADLESS_FSM] mode=move_base auto_rl=1`、ROS 图确认
`/unitree_gazebo_servo` 已订阅 `/cmd_vel` 后解除物理暂停。随后必须等到
`Switched from fixed stand to RL`，并自动发送一段低速命令验证真实位移和机身高度；
只有日志出现 `Controller physical /cmd_vel probe passed` 才会宣布启动完成。默认控制周期为
`UNITREE_CTRL_DT=0.002`（500 Hz），不得仅为消除超时 warning 擅自放宽到 `0.004`。
只要修改或同步过 `src/unitree_guide/`，就必须先执行 `build force`；`up` 会拒绝复用时间戳早于控制源码的
`junior_ctrl`。ROS 图中的控制节点名是 `/unitree_gazebo_servo`，不能以未出现 `/junior_ctrl` 节点名判断订阅失败。
不要使用已弃用的
`ros2_ws/src/hazardwalker_platform/scripts/start_simenv.sh`，也不要在同一容器中重复运行启动脚本。

### 3.4 可视化 GUI（noVNC sidecar）

当前 RDP/XWayland 不能直接稳定运行 Gazebo Classic `gzclient`，但容器内 Xvfb 软件渲染已验收可用。
平台管理员应使用下列**独立 GUI sidecar**，它只连接现有 Gazebo Master，不会重启、停止或修改正式
仿真容器：

```bash
cd ros2_ws/src/hazardwalker_platform
export SIMENV_CONTAINER=simenv_ros1_hazard_platform
./auto_docker.sh gui build  # 首次或 GUI Dockerfile 更新后执行
./auto_docker.sh gui up
```

在远程 RDP 桌面的浏览器打开
`http://127.0.0.1:6081/vnc.html?autoconnect=1&resize=scale`，即可按浏览器视口自适应缩放显示 Gazebo。
需要真正全屏时再按浏览器 `F11`；若浏览器仍显示旧的 1440×900 会话，按 `Ctrl+F5` 后重新连接。
端口仅绑定远程主机 loopback；从本机访问时使用 SSH 隧道，不要暴露到公网：

```bash
ssh -L 6081:127.0.0.1:6081 hxbl-codex-main
```

浏览器窗口用于观察仿真；键盘控制节点运行在**远程独占终端**，两者并排使用。不要在 noVNC 窗口中
把 `w/s/a/d/k` 当作控制指令，它们属于 Gazebo GUI 快捷键。控制终端仍只向 `/hw/cmd_vel` 发布：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run hazardwalker_platform keyboard_control_node
```

按 `w` 前进、`s` 后退、`a` 左转、`d` 右转、`k` 立即停止，`q` 或 `Ctrl+C` 停止并退出。结束 GUI
观察后只停止 sidecar，不要执行正式容器的 `down`：

```bash
./auto_docker.sh gui down
```

## 4. 三种接入方式

### 4.0 每台 ROS2 主机的一次性依赖

适配器运行在 ROS2 主机，不运行在官方 ROS1 Docker。除 ROS2 Jazzy 外，还需要 Python 的
`websocket-client`。Ubuntu 24.04 受 PEP 668 保护时，不要全局执行 `pip install`；使用独立环境：

```bash
python3 -m venv --system-site-packages "$HOME/.local/share/hazardwalker-ros2-venv"
"$HOME/.local/share/hazardwalker-ros2-venv/bin/pip" install websocket-client
export OFFICIAL_SIMENV_PYTHON_BIN="$HOME/.local/share/hazardwalker-ros2-venv/bin/python"
```

每次启动适配器前保留最后一行环境变量。启动脚本会主动清理失效的 ROS2 工作区前缀，再加载
`/opt/ros/jazzy` 与当前仓库 `ros2_ws/install`；不要混用已删除工作区的 `setup.bash`。

### 4.1 只读验证 `/hw/*`

终端 A 启动唯一适配器：

```bash
export SIMENV_CONTAINER=simenv_ros1_hazard_platform
export ROS_DOMAIN_ID=42
bash scripts/run_official_simenv_rosbridge_adapter.sh
```

终端 B 验证 ROS1 输入和 ROS2 输出：

```bash
export SIMENV_CONTAINER=simenv_ros1_hazard_platform
export ROS_DOMAIN_ID=42
bash scripts/verify_official_simenv_ros1_adapter.sh
```

不带 `--control` 时不会发送速度命令。适配器默认转发 `/hazardwalker/odom` 到诊断 `/hw/odom`；这与是否转发
`/hw/cmd_vel`、是否转发点云完全独立，且 `/hw/odom` 不能作为正式 SLAM 位姿或比赛结果定位来源。至少应收到：

| ROS2 接口 | 内容 |
|---|---|
| `/clock` | 递增的仿真时间 |
| `/hw/camera/image_raw` | RGB |
| `/hw/camera/depth_image` | 深度 |
| `/hw/camera/camera_info` | RGB 内参 |
| `/hw/camera/depth_camera_info` | 深度内参 |
| `/hw/scan` | 激光 |
| `/hw/trunk_imu` | 机体 IMU |
| `/hw/odom` | 平台诊断里程计，不得作为正式评分定位真值 |
| `/tf`、`/tf_static` | 坐标变换 |
| `/hw/cmd_vel` | ROS2 控制输入；默认不向 ROS1 转发 |

若实际 RGB 来源是 `/camera/image_raw`，启动适配器和验证脚本前设置：

```bash
export OFFICIAL_SIMENV_RGB_TOPIC=/camera/image_raw
export OFFICIAL_SIMENV_RGB_CAMERA_INFO_TOPIC=/camera/camera_info
```

同一 ROS 域只能有一个 `/hazardwalker_official_rosbridge_adapter`。

### 4.2 无控制业务检查

```bash
export SIMENV_CONTAINER=simenv_ros1_hazard_platform
export ROS_DOMAIN_ID=42

bash scripts/run_official_simenv_ros1_ros2_stack.sh \
  start_navigation:=false \
  start_slam:=false
```

该模式适合检查感知和决策接口，不授权导航控制。按 `Ctrl+C` 后入口会统一回收本轮子进程。

### 4.3 正式 SLAM + Frontier + 感知闭环

仅在平台控制、传感器和独占检查全部通过后运行。下面是当前启动器要求的完整模板：

```bash
export SIMENV_CONTAINER=simenv_ros1_hazard_platform
export ROS_DOMAIN_ID=42
export OFFICIAL_SIMENV_ENABLE_CONTROL=1
export OFFICIAL_SIMENV_EXCLUSIVE_SESSION=1
export OFFICIAL_SIMENV_STACK_TIMEOUT_SEC=600

export SEED=2026071802
export RUN_ID="seed_${SEED}_$(date +%Y%m%d_%H%M%S)"
export CODE_VERSION="$(git rev-parse HEAD)"
export EVIDENCE_DIR="$PWD/reports/perception/official_random/${RUN_ID}"
export TEST_RECORD_DIR="$PWD/reports/perception/test_records/official_random/${RUN_ID}"
export RESULT_PATH="$PWD/results/detected_danger.json"

bash scripts/run_official_simenv_ros1_ros2_stack.sh \
  start_slam:=true \
  slam_backend:=cartographer \
  start_navigation:=true \
  nav_mode:=frontier \
  perception_output_frame:=world \
  localization_provenance:=lidar_imu_slam+public_floor_action \
  start_evidence_recorder:=true \
  scenario_seed:="$SEED" \
  code_version:="$CODE_VERSION" \
  evidence_output_dir:="$EVIDENCE_DIR" \
  test_record_dir:="$TEST_RECORD_DIR" \
  official_result_path:="$RESULT_PATH"
```

只有定位链路确实满足所填来源时，才能使用相应 `localization_provenance`。启动器会拒绝：

- 未显式开启控制的导航；
- 没有 SLAM 的 Frontier；
- 非 `frontier` 的正式导航；
- 非 `world` 的正式结果坐标；
- 未验证的定位来源；
- 缺少固定 SEED、代码版本或证据目录；
- 冻结的 `/clock`、重复适配器或非独占场景。

命令进入运行仅表示门禁通过；任务完成仍须同时满足 `FINISHED` 和本轮新生成的有效
`results/detected_danger.json`。

## 5. 平台就绪验收

平台管理员按以下顺序验收，任一项失败都应停止向业务组交付：

1. 容器唯一且运行稳定。
2. `junior_ctrl` 存活，日志先确认 `HEADLESS_FSM.*mode=move_base.*auto_rl=1`，再确认
   `Switched from fixed stand to RL`，且无模型加载失败和关节力矩 NaN。
3. `/clock` 连续递增；RGB-D、内参、激光、IMU 和里程计均有新消息。
4. `/cmd_vel` 有真实 A1 控制链订阅者，且本轮启动日志存在
   `Controller physical /cmd_vel probe passed`；只有订阅者不能证明回调和 RL 动作实际生效。
5. 在独占、安全条件下完成真实直行、转向和停止验收。

启动脚本会在第 4 项未满足时拒绝宣布就绪；启动探针只验证最小物理响应，不能替代第 5 项
完整控制验收。Docker 健康检查会继续监测控制器、订阅者和
rosbridge，但只标记 `unhealthy`，不会代替平台管理员重启进程。只读检查可使用：

```bash
docker inspect --format '{{.State.Status}} / {{if .State.Health}}{{.State.Health.Status}}{{else}}no-healthcheck{{end}}' \
  "$SIMENV_CONTAINER"
docker exec "$SIMENV_CONTAINER" pgrep -a -x junior_ctrl
docker exec "$SIMENV_CONTAINER" bash -lc '
  grep -E "HEADLESS_FSM.*auto_rl=1|Switched from fixed stand to RL|CMD_VEL_RX|RL_CMD_APPLIED|load model|setTau function meets Nan|Traceback" \
    logs/junior_ctrl.log | tail -30
  source /opt/ros/noetic/setup.bash
  rostopic info /cmd_vel
'
bash scripts/verify_official_simenv_ros1_adapter.sh
```

导航组执行直行、转向和停止测试时统一使用项目内
[官方 SimEnv 控制链路与键盘测试](../groups/nav/官方SimEnv控制链路与键盘测试.md)：
键盘节点只发布 `/hw/cmd_vel`，按键为 `w` 前进、`s` 后退、`a` 左转、`d` 右转、`k`
立即停止。不得同时运行键盘节点和 Nav2 速度发布者。

真实运动验收会控制机器人，只能由平台管理员执行：

```bash
STAMP="$(date +%Y%m%d_%H%M%S)"
export OFFICIAL_SIMENV_EXCLUSIVE_SESSION=1
export OFFICIAL_SIMENV_EVIDENCE_DIR="$PWD/reports/platform/official_simenv_ros1_ros2/${STAMP}_rl_acceptance"
export OFFICIAL_SIMENV_VIDEO_REFERENCE="${OFFICIAL_SIMENV_EVIDENCE_DIR}/rl_acceptance.mp4"

bash scripts/verify_official_simenv_ros1_direct_control.sh --run
```

最低通过标准：

- 直行位移不少于 1 m；
- 转向变化不少于 0.2 rad；
- 已发送零速度；
- 同一时间窗的视频和里程计均已保存；
- 控制器无 NaN、模型加载失败或异常退出。

脚本会输出 `summary.json`、测试 CSV、README 和前后里程计。重要联调必须使用本轮证据，不能用历史
成功记录代替当前验收。

## 6. 正确停止

1. 在业务栈终端按 `Ctrl+C`，等待进程组完成回收；不要直接关闭终端。
2. 检查无遗留控制发布者：

   ```bash
   ros2 node list
   ros2 topic info /hw/cmd_vel --verbose
   ```

3. 只有容器所有者确认无人使用时才停止官方环境：

   ```bash
   cd ros2_ws/src/hazardwalker_platform
   ./auto_docker.sh down
   cd ../../..
   ```

不要执行全局 `pkill`、批量 `docker rm` 或停止其他成员容器。控制中断时应优先发送零速度。

## 7. 故障速查

| 现象 | 依次检查 |
|---|---|
| 模型存在但机器人不动 | Docker 启动日志必须有物理探针通过 → `junior_ctrl` 已进入 `Switched from fixed stand to RL` → `CMD_VEL_RX` 与 `RL_CMD_APPLIED` 同时出现 → Gazebo 未暂停 → `/cmd_vel` 唯一订阅/发布链 → NaN 日志；仅有订阅者不算通过，共享容器只报告平台管理员 |
| `/hazardwalker/odom` 或 `/hw/odom` 缺失 | `auto_docker.sh image --no-cache` → `auto_docker.sh up` → 容器内 `rosnode list` 的 `hazardwalker_odom_relay` → 再启动唯一 ROS2 适配器；不要用点云或控制开关替代中继 |
| 没有 `/hw/*` | 容器名 → rosbridge → ROS1 原话题 → 唯一适配器 → 相同 `ROS_DOMAIN_ID` → 最新工作空间 |
| 有 `/clock` 但业务不运行 | 连续采样两帧确认时间递增；单帧旧消息无效 |
| `setTau ... Nan` | 立即停止控制，由平台管理员独占重启并检查关节状态 |
| `cuda::is_available():0` | 表示 CPU 回退；继续以周期、无 NaN 和真实运动验收 |
| 平台可用但 SLAM 失败 | 检查时间同步、TF、激光/IMU、合法位姿来源和 SLAM 参数；属于导航侧验收 |

## 8. 证据与结论

每次正式实验至少保存：

- 启动参数、固定 SEED、代码版本、容器名和 ROS 域；
- 展示视频或关键 RGB-D 帧；
- 轨迹、候选到确认记录和最终结果 JSON；
- `summary.json`、测试 CSV/JSON 和 README；
- 原始日志或可访问的日志位置；
- 失败原因和结论边界。

平台结果统一放在：

```text
reports/platform/official_simenv_ros1_ros2/<运行编号>/
```

大型 rosbag、原始 `.ppm`、模型权重和大视频不要提交 Git；放到共享大文件目录，并在 README 中记录
引用位置。

## 9. 一分钟检查清单

开始前：

- [ ] 已同步代码并重新构建。
- [ ] 已获得独占时段，容器名和 `ROS_DOMAIN_ID` 正确。
- [ ] 只有一个目标 SimEnv 和一个 ROS2 适配器。
- [ ] `/clock` 递增，RGB-D、内参、激光和 IMU 有新消息。
- [ ] 控制任务中 `junior_ctrl` 已实际进入 RL，启动物理探针通过，且无 NaN。
- [ ] 正式任务已填写 SEED、代码版本、合法定位来源和证据目录。

结束后：

- [ ] 已发送零速度并用 `Ctrl+C` 正常回收。
- [ ] 无遗留 `/hw/cmd_vel` 发布者。
- [ ] 视频、轨迹、结构化摘要和测试表来自同一轮。
- [ ] 结论未把平台、SLAM、探索、感知或整场任务的局部通过相互替代。

## 10. 本手册的验证范围

截至 2026-07-23，本文引用的仓库路径均存在；启动脚本中的关键参数、独占门禁、控制器 RL 状态、
真实运动探针、递增时钟、证据字段和退出清理流程已与当前代码核对，并在独立容器冷启动复验。
手册没有宣称任意历史容器自动可用，每轮仍须执行本轮验收。
