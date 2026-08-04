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
export DOCKER_SIMENV_USER=hazard_platform
export ROS_DOMAIN_ID=42
export OFFICIAL_SIMENV_ENABLE_CONTROL=1
./auto_docker.sh status
./auto_docker.sh down
./auto_docker.sh image --no-cache
./auto_docker.sh build force
./auto_docker.sh up
./auto_docker.sh logs
cd ../../..
```

主账号登录名不是正式容器名；必须保留 `DOCKER_SIMENV_USER=hazard_platform`，否则会创建错误的
`simenv_ros1_hxbl` 平行容器。`auto_docker.sh up` 先调用容器内的 `auto.sh`，启动 Gazebo、`junior_ctrl`、
`/Odometry_gazebo -> /hazardwalker/odom` 最新值中继和 `rosbridge_websocket`；容器健康后，再在宿主机自动
启动并去重 ROS2 适配器。`auto_docker.sh status` 同时报告两者状态，`down` 先停止适配器再停止容器。
再次执行 `up` 时，相同配置复用现有适配器；控制、话题或节流等数据流参数变化时只替换适配器实例，
不会为了应用参数而重建已经健康的容器。
镜像已固定包含
`ros-noetic-rosbridge-server` 与 `expect`；不得再进入容器手工安装软件包、手工启动 rosbridge 或手工拉起控制器。
`.ros1_catkin_ws` 是本地 ROS1 构建产物，不纳入 Git；日常不要删除。首次运行、误删或构建产物缺失时，
`./auto_docker.sh up` 会先自动重建该目录，再启动正式容器。构建结束会自动将目录所有权归还给当前宿主账号，
不得使用 `sudo` 或 `git clean` 清理它。
默认 `START_CONTROLLER=1`、`SIMENV_AUTO_RL=1`、`SIMENV_HEADLESS_MODE=move_base`、`START_ROSBRIDGE=1`、`START_ODOM_RELAY=1`。正式共享 profile 还在 `up` 前设置 `OFFICIAL_SIMENV_ENABLE_CONTROL=1`，使唯一适配器具备 `/hw/cmd_vel` 转发能力；这不会自行发送运动命令。启动后 A1 先保持固定站立，收到本轮授权发布者的合法非零速度才切换到 RL 行走。真实运动测试必须在独占时段执行。默认控制周期为 `UNITREE_CTRL_DT=0.004`（250 Hz），该值为当前平台稳定 profile，不要自行修改。
适配器默认以 200 ms 周期转发原始 RGB-D（上限 5 Hz），用于实时检测和辅助对准；第一人称页面使用独立 ROS1 压缩视频流。正式录包会拒绝超过 250 ms 的图像桥接周期，性能排障如需降频只能作为诊断运行。
只要修改或同步过 `src/unitree_guide/`，就必须先执行 `build force`；`up` 会拒绝复用时间戳早于控制源码的
`junior_ctrl`。ROS 图中的控制节点名是 `/unitree_gazebo_servo`，不能以未出现 `/junior_ctrl` 节点名判断订阅失败。
不要使用已弃用的
`ros2_ws/src/hazardwalker_platform/scripts/start_simenv.sh`，也不要在同一容器中重复运行启动脚本。

### 3.4 可视化 GUI 与第一人称画面

当前 RDP/XWayland 不直接运行 Gazebo Classic `gzclient`。默认 GUI 链路为宿主机 NVIDIA Xorg `:101`、
VirtualGL、TurboVNC `:110` 和 noVNC `:6081`；GPU 只负责图形渲染，不替代 Gazebo 物理性能、控制链路或
业务节点验收。默认分辨率为 1280×720。平台管理员使用独立 GUI sidecar，它只连接现有 Gazebo Master，不会重启、停止或修改正式仿真容器：

```bash
cd ros2_ws/src/hazardwalker_platform
export SIMENV_CONTAINER=simenv_ros1_hazard_platform
./auto_docker.sh gui up
```

首次 `gui up` 会在缺少镜像时自动构建 NVIDIA GUI 镜像。运行状态和日志：

```bash
./auto_docker.sh gui status
./auto_docker.sh gui logs
```

日志应包含 `vglrun(:101) → TurboVNC(:110) → noVNC(:6081)`，并在
`/tmp/hazardwalker-gui-glx.log` 显示 NVIDIA OpenGL renderer。若 GPU 链路排障期间不可用，可显式回退为 Xvfb
软件渲染；先停 GPU sidecar，再启动回退实例：

```bash
./auto_docker.sh gui down
SIMENV_GUI_XSERVER=xvfb SIMENV_GUI_DISPLAY=:100 ./auto_docker.sh gui up
```

恢复 GPU 图形渲染时，停止软件 sidecar 后直接执行 `./auto_docker.sh gui up`。在远程 RDP 桌面或经 SSH 隧道的
浏览器中打开 `http://127.0.0.1:6081/hazardwalker.html`；该专用页面会自动连接，且仅保留按浏览器视口铺满的 Gazebo
画面，页面内亦可使用“全屏”按钮或浏览器 `F11`。
该页面允许鼠标操作 Gazebo 视角，但浏览器键盘不会向 ROS `/hw/cmd_vel` 发布控制命令；机器狗仍由独占终端的 ROS2 键盘节点经统一控制仲裁器控制。
若仍显示旧页面，按 `Ctrl+F5` 后重新连接。
端口仅绑定远程主机 loopback；从本机访问时使用 SSH 隧道，不要暴露到公网：

```bash
ssh -N -L 6081:127.0.0.1:6081 -L 6082:127.0.0.1:6082 hxbl-codex-main
```

浏览器窗口用于观察仿真；键盘控制节点运行在**远程独占终端**，两者并排使用。不要在 noVNC 窗口中
把 `w/s/a/d/k` 当作控制指令，它们属于 Gazebo GUI 快捷键。先启动统一控制层（纯平台控制检查可关闭感知、定位和辅助节点）：

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/桌面/HazardWalker/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=42
ros2 launch hazardwalker_bringup official_simenv_control_interface.launch.py \
  control_mode:=keyboard \
  start_assist_alignment:=false \
  start_navigation:=false \
  start_slam:=false \
  start_perception:=false \
  start_legal_localization:=false \
  start_decision:=false
```

再在独立终端启动键盘节点。键盘节点只写 `/hw/control/keyboard_cmd_vel`，由 `command_mux_node` 唯一输出 `/hw/cmd_vel`：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export PYTHONPATH="$HOME/桌面/HazardWalker/ros2_ws/src/hazardwalker_platform:${PYTHONPATH}"
python3 -m hazardwalker_platform.keyboard_control_node
```

按 `w` 前进、`s` 后退、`a` 左转、`d` 右转、`k` 立即停止，`q` 或 `Ctrl+C` 停止并退出。默认线速度为 `0.45 m/s`、转向角速度为 `0.80 rad/s`、单次命令保持 `0.8 s`。运行中的节点支持以下受限热调参；不要超过提示范围：

```bash
ros2 param set /hazardwalker_keyboard_control linear_speed 0.45
ros2 param set /hazardwalker_keyboard_control angular_speed 0.80
ros2 param set /hazardwalker_keyboard_control command_hold_sec 0.8
```

第一人称服务直接订阅官方 `/real_sense/rgb/image_raw/compressed`，不发布速度命令。平台管理员可在同一目录启动：

```bash
./auto_docker.sh first_person up
```

普通成员完成 SSH 隧道后，在浏览器打开 `http://127.0.0.1:6082/first_person`。第一人称页面与上帝视角可同时打开；
页面按浏览器视口显示相机画面，并提供全屏按钮。感知业务栈与适配器 GUI 状态转发启用时，页面还会显示候选/确认框、复查建议，并提供需再次确认的“辅助对准”和“取消辅助”按钮。该按钮只调用 `/hw/control/assist_align/*` 服务，不能绕过控制仲裁器发布速度；辅助节点收到统一仲裁器的 `mode=assist` 确认后才允许转向，接管失败会超时停车并在页面显示原因。观察结束后仅停止 sidecar，不要执行正式容器的 `down`：

```bash
./auto_docker.sh gui down
./auto_docker.sh first_person down
```

`gui down` 会同时回收对应 TurboVNC/noVNC 图形进程；页面刷新后不应保留旧画面。`Xorg :101 already up` 表示宿主机共享 GPU 图形服务仍在运行，是再次 `gui up` 时的正常提示；不得手动停止 Xorg 或执行全局 VNC 清理。

## 4. 三种接入方式

### 4.0 每台 ROS2 主机的一次性依赖

适配器运行在 ROS2 主机，不运行在官方 ROS1 Docker。除 ROS2 Jazzy 外，还需要 Python 的
`websocket-client`。Ubuntu 24.04 受 PEP 668 保护时，不要全局执行 `pip install`；使用独立环境：

```bash
python3 -m venv --system-site-packages "$HOME/.local/share/hazardwalker-ros2-venv"
"$HOME/.local/share/hazardwalker-ros2-venv/bin/pip" install websocket-client
export OFFICIAL_SIMENV_PYTHON_BIN="$HOME/.local/share/hazardwalker-ros2-venv/bin/python"
```

该虚拟环境只需创建一次。`auto_docker.sh up` 会优先使用它，并主动清理失效的 ROS2 工作区前缀，再加载
`/opt/ros/jazzy` 与当前仓库源码；不要混用已删除工作区的 `setup.bash`。如虚拟环境使用了其他路径，需在
执行 `up` 前设置 `OFFICIAL_SIMENV_PYTHON_BIN`。

### 4.1 只读验证 `/hw/*`

平台终端确认 `auto_docker.sh up` 已自动启动唯一适配器：

```bash
cd ros2_ws/src/hazardwalker_platform
export DOCKER_SIMENV_USER=hazard_platform
export ROS_DOMAIN_ID=42
./auto_docker.sh status
cd ../../..
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
| `/hw/cmd_vel` | ROS2 控制输入；正式共享 profile 向 ROS1 转发，但只有本轮授权节点可发布 |

若实际 RGB 来源是 `/camera/image_raw`，启动适配器和验证脚本前设置：

```bash
export OFFICIAL_SIMENV_RGB_TOPIC=/camera/image_raw
export OFFICIAL_SIMENV_RGB_CAMERA_INFO_TOPIC=/camera/camera_info
```

同一 ROS 域只能有一个 `/hazardwalker_official_rosbridge_adapter`。不要再单独运行
`run_official_simenv_rosbridge_adapter.sh`；本账号的旧版手工实例会由下一次 `up` 接管并去重。若同一域实例属于
其他 Linux 账号，管理器会拒绝启动或停止容器，并提示先由该进程所有者收尾，避免制造重复发布者或孤儿进程。
适配器状态包含 `managed_lifecycle=true` 和所属容器；正式业务栈及录包预检会拒绝缺少该来源声明的手工实例。

### 4.2 无控制业务检查

```bash
export SIMENV_CONTAINER=simenv_ros1_hazard_platform
export ROS_DOMAIN_ID=42

bash scripts/run_official_simenv_ros1_ros2_stack.sh \
  start_navigation:=false \
  start_slam:=false
```

该模式适合检查感知和决策接口，不授权导航控制。按 `Ctrl+C` 后入口会统一回收本轮子进程。
该入口与人工巡检使用同一个 `command_mux_node`；即使后续切换为导航，感知、GUI、辅助复查和录包话题均不改变。
业务栈同时发布 `/hw/perception/patrol_coverage`，仅依据合法 SLAM 里程计累计本轮路程和跨度，不读取 Gazebo 真值，也不控制机器人。

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

启动器会自动把统一控制源设为 `navigation`。导航节点只发布
`/hw/control/navigation_cmd_vel`，最终 `/hw/cmd_vel` 仍由 `command_mux_node` 唯一发布；禁止在该命令中传入其他 `control_mode` 绕过控制合同。

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
2. `junior_ctrl` 存活，日志确认 `HEADLESS_FSM.*mode=move_base.*auto_rl=1` 与
   `fixed stand state is ready`，且无模型加载失败和关节力矩 NaN。
3. `/clock` 连续递增；RGB-D、内参、IMU 和里程计均有新消息。仅在启用
   `ENABLE_LIDAR=true` 的导航/SLAM profile 中检查激光。
4. `/cmd_vel` 有真实 A1 控制链订阅者；独占运动测试时还必须同时看到
   `CMD_VEL_RX` 与 `RL_CMD_APPLIED`，并由里程计或视频证明真实动作。只有订阅者不能证明回调和 RL 动作实际生效。
5. 在独占、安全条件下完成真实直行、转向和停止验收。

启动脚本会在控制链订阅者未就绪时拒绝宣布容器健康；它不自动发送速度命令，不能替代第 5 项完整控制验收。Docker 健康检查会继续监测控制器、订阅者和 rosbridge，但只标记 `unhealthy`，不会代替平台管理员重启进程。只读检查可使用：

```bash
docker inspect --format '{{.State.Status}} / {{if .State.Health}}{{.State.Health.Status}}{{else}}no-healthcheck{{end}}' \
  "$SIMENV_CONTAINER"
docker exec "$SIMENV_CONTAINER" pgrep -a -x junior_ctrl
docker exec "$SIMENV_CONTAINER" bash -lc '
  grep -E "HEADLESS_FSM.*auto_rl=1|fixed stand state is ready|Switched from fixed stand to RL|CMD_VEL_RX|RL_CMD_APPLIED|load model|setTau function meets Nan|Traceback" \
    logs/junior_ctrl.log | tail -30
  source /opt/ros/noetic/setup.bash
  rostopic info /cmd_vel
'
bash scripts/verify_official_simenv_ros1_adapter.sh
```

导航组执行直行、转向和停止测试时统一使用项目内
[官方 SimEnv 控制链路与键盘测试](../groups/nav/官方SimEnv控制链路与键盘测试.md)：
键盘和导航分别发布 `/hw/control/keyboard_cmd_vel` 与 `/hw/control/navigation_cmd_vel`，只有 `command_mux_node` 发布 `/hw/cmd_vel`。按键为 `w` 前进、`s` 后退、`a` 左转、`d` 右转、`k` 立即停止；切换控制模式前先停车，不得绕过仲裁器另启直接发布 `/hw/cmd_vel` 的节点。

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

1. 在键盘或业务栈终端先发送零速度（键盘节点按 `k`），再按 `Ctrl+C`，等待进程组完成回收；不要直接关闭终端。
2. 检查无遗留控制发布者：

   ```bash
   ros2 node list
   ros2 topic info /hw/cmd_vel --verbose
   ```

3. 只有容器所有者确认无人使用时，才在平台终端关闭观测服务、唯一适配器和官方环境：

   ```bash
   # 平台终端
   cd ~/桌面/HazardWalker/ros2_ws/src/hazardwalker_platform
   export DOCKER_SIMENV_USER=hazard_platform

   ./auto_docker.sh first_person down
   ./auto_docker.sh gui down
   ./auto_docker.sh down
   ```

不要执行全局 `pkill`、批量 `docker rm`、`pkill Xorg` 或停止其他成员容器。`Xorg :101 already up` 是 GPU GUI 共享服务正常复用的提示，不是故障。控制中断时应优先发送零速度。

## 7. 故障速查

| 现象 | 依次检查 |
|---|---|
| 模型存在但机器人不动 | 容器先处于 `fixed stand state is ready` → 确认唯一 ROS2 适配器已订阅 `/hw/cmd_vel` → 独占测试发送非零速度后同时检查 `CMD_VEL_RX`、`RL_CMD_APPLIED`、Gazebo 未暂停和里程计变化 → 检查 NaN 日志；仅有订阅者不算通过 |
| `/hazardwalker/odom` 或 `/hw/odom` 缺失 | `auto_docker.sh image --no-cache` → `auto_docker.sh up` → 容器内 `rosnode list` 的 `hazardwalker_odom_relay` → `auto_docker.sh status` 的适配器状态；不要手工补启适配器，也不要用点云或控制开关替代中继 |
| 没有 `/hw/*` | 容器名 → rosbridge → ROS1 原话题 → 唯一适配器 → 相同 `ROS_DOMAIN_ID` → 最新工作空间 |
| `同一 ROS 域仍有其他会话的适配器` 或 `ROS2 node: duplicate` | 不要强杀或继续 `up/down`；用 `ros2 node list --no-daemon` 和平台管理员的进程审计确认所有者，由原会话停止后重试；只有状态显示 `ROS2 node: ready (unique)` 才能进入正式业务 |
| 有 `/clock` 但业务不运行 | 连续采样两帧确认时间递增；单帧旧消息无效 |
| `setTau ... Nan` | 立即停止控制，由平台管理员独占重启并检查关节状态 |
| 机器人翻倒 | 立即发送零速度并结束键盘节点；不要在 Gazebo 拖动模型。由容器所有者停止 sidecar、适配器和主容器后重新启动本轮仿真 |
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
