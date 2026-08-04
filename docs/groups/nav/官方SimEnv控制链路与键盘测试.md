# 官方 SimEnv 控制链路与键盘测试

- 修改人：负责人（姜晨）
- 适用对象：导航组、平台组、集成测试组
- 结论：导航和键盘测试统一发布 `/hw/cmd_vel`，不得绕过适配器直接依赖容器内 `/cmd_vel`

## 验证状态

- 负责人已在独立环境验证底层 RL `/cmd_vel` 物理响应和停止链路。
- 本轮 W/S/A/D/K 映射、ROS2 安装入口和旧键盘兼容入口已通过离线契约测试。
- 为避免影响他人实验，本轮未重启或改写当前共享容器；导航组仍须在下一次独占时段完成本文第 3
  节的真实前进、后退、左右转和急停复验。

## 1. 控制链路

```text
导航 / 键盘控制节点
        │ ROS2 geometry_msgs/Twist
        ▼
    /hw/cmd_vel
        │ hazardwalker_official_rosbridge_adapter
        ▼
ROS1 /cmd_vel
        │ junior_ctrl（RL 状态）
        ▼
Unitree A1 仿真关节控制
        │
        ▼
公开传感器 → ROS2 /hw/* → SLAM、导航、感知
```

各层通过条件：

1. 平台启动日志出现 `Controller physical /cmd_vel probe passed`。
2. `junior_ctrl` 日志出现 `Switched from fixed stand to RL`，且无 NaN、模型加载失败或异常退出。
3. ROS2 适配器以 `enable_cmd_vel_relay=true` 启动。
4. `/hw/cmd_vel` 只有本轮授权控制节点发布，ROS1 `/cmd_vel` 有实际控制器订阅。
5. 机器人产生与命令一致的真实位姿变化；只有话题或订阅者不算运动通过。

`/hw/odom` 是平台诊断里程计，不能作为比赛定位或 SLAM 成果。导航必须使用合法 SLAM 位姿。

## 2. 键盘定义

| 按键 | 动作 | 默认速度 |
|---|---|---|
| `w` | 前进 | `linear.x = +0.30 m/s` |
| `s` | 后退 | `linear.x = -0.30 m/s` |
| `a` | 左转 | `angular.z = +0.60 rad/s` |
| `d` | 右转 | `angular.z = -0.60 rad/s` |
| `k` | 立即停止 | 线速度、角速度全部归零 |
| `q` / `Ctrl+C` | 停止并退出 | 退出前连续发布零速度 |

按住方向键才持续运动；停止按键约 `0.35 s` 后会自动发送零速度。`k` 是正式急停键。

## 3. 导航组测试步骤

### 3.1 前置条件

必须先由平台管理员确认独占时段和平台验收通过。普通成员不要重启共享容器或手工切换
`junior_ctrl`。

```bash
export SIMENV_CONTAINER=simenv_ros1_hazard_platform
export ROS_DOMAIN_ID=42
export OFFICIAL_SIMENV_ENABLE_CONTROL=1
export OFFICIAL_SIMENV_EXCLUSIVE_SESSION=1
```

平台管理员在启动正式容器前设置控制转发；`up` 会在容器健康后自动启动唯一适配器：

```bash
cd ~/桌面/HazardWalker/ros2_ws/src/hazardwalker_platform
export DOCKER_SIMENV_USER=hazard_platform
export ROS_DOMAIN_ID=42
export OFFICIAL_SIMENV_ENABLE_CONTROL=1
./auto_docker.sh up
./auto_docker.sh status
```

导航组只需确认适配器已启用控制，不得再手工启动第二份：

```bash
ros2 node list
ros2 topic info /hw/cmd_vel --verbose
```

### 3.2 启动键盘控制

首次使用或代码更新后构建：

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select hazardwalker_platform
source install/setup.bash
cd ..
```

在**远程开发机**的终端 C 运行（本机终端只可作为 SSH 键盘输入窗口，节点和仿真均运行在远程）：

```bash
ros2 run hazardwalker_platform keyboard_control_node
```

若远程工作区源代码已同步、但尚未执行 `colcon build`，可用同一份源码启动；这不是容器内手工安装，
也不需要重启任何平台进程：

```bash
cd ~/HazardWalker
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/ros2_ws/src/hazardwalker_platform:${PYTHONPATH}"
python3 -m hazardwalker_platform.keyboard_control_node
```

该入口与 `ros2 run` 使用同一节点、同一参数和同一停止保护。不要在本机运行 ROS2 节点，也不要
向容器内 `/cmd_vel` 直接发布。

### 3.3 完整适配器失活时的控制备用中继

如果 ROS1 原始相机/里程计正常，但完整适配器的 `/hw/odom`、状态话题和参数服务均长期无响应，
先保存诊断证据并通知平台组。**不得重启共享容器或杀死他人适配器进程。**仅在平台管理员确认
独占时段后，可在远程工作区另开终端临时启动下列备用中继：

```bash
cd ~/HazardWalker
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
"${OFFICIAL_SIMENV_PYTHON_BIN:-python3}" -c 'import rclpy, websocket'
"${OFFICIAL_SIMENV_PYTHON_BIN:-python3}" scripts/official_simenv_cmd_vel_relay_node.py
```

该节点仍只接收 `/hw/cmd_vel` 并通过 rosbridge 写入官方 ROS1 `/cmd_vel`，不转发传感器、
不读取真值。它是完整适配器失活时的受控恢复工具：完整适配器恢复后，先按 `Ctrl+C` 停止备用
中继，再恢复常规流程，禁止两个速度中继长期并行。若依赖检查失败，必须使用平台组已验收的
`OFFICIAL_SIMENV_PYTHON_BIN`（含 `websocket-client`）；不要向共享容器或系统 Python 手工安装包。

需要临时降低速度时使用 ROS 参数，不修改源码：

```bash
ros2 run hazardwalker_platform keyboard_control_node --ros-args \
  -p linear_speed:=0.20 \
  -p angular_speed:=0.45 \
  -p command_hold_sec:=0.35
```

依次短按或短按住 `w → k → s → k → a → k → d → k`。每一步均观察机器人方向、
SLAM 位姿和障碍物安全距离。遇到异常立即按 `k`，再按 `Ctrl+C` 退出。

### 3.4 链路复核

ROS2 侧：

```bash
ros2 topic info /hw/cmd_vel --verbose
ros2 topic echo /hw/cmd_vel
```

ROS1 侧只读检查：

```bash
docker exec "$SIMENV_CONTAINER" bash -lc '
  source /opt/ros/noetic/setup.bash
  rostopic info /cmd_vel
  grep -E "CMD_VEL_RX|RL_CMD_APPLIED|setTau function meets Nan|Traceback" \
    logs/junior_ctrl.log | tail -30
'
```

结束后必须确认没有遗留键盘或导航发布者：

```bash
ros2 topic info /hw/cmd_vel --verbose
```

## 4. 导航接入约束

- Nav2 或自研控制器只向 `/hw/cmd_vel` 发布 `geometry_msgs/Twist`。
- 同一轮只能有一个最终速度发布者；键盘与 Nav2 不得同时发布。
- 控制节点必须有命令超时、零速度退出和异常停止处理。
- 转向、避障和 Frontier 成功必须用本轮轨迹、地图、命令及视频共同证明。
- 感知复查请求只提供目标、建议方向和约束；最终运动由导航层仲裁。
- 不读取 `results/danger_truth.json`、Gazebo 真值位姿或内部场景文件。

## 5. 常见失败判定

| 现象 | 处理 |
|---|---|
| `/hw/cmd_vel` 有消息，机器人不动 | 检查适配器是否启用控制、ROS1 `/cmd_vel` 订阅者、RL 状态及控制器日志 |
| 机器人短暂运动后停下 | 键盘工具的安全超时生效；按住方向键即可持续发送 |
| 机器人持续运动 | 立即按 `k`；仍无效则停止本轮业务发布者并报告平台管理员 |
| 方向相反 | 停止测试，保存 `/hw/cmd_vel`、位姿和视频，禁止通过改地图坐标系掩盖问题 |
| SLAM 失败但控制正常 | 归入导航侧时间同步、TF、激光/IMU 或参数问题，不重启共享控制器 |

正式验收和容器操作仍以
[官方 SimEnv 平台环境使用手册](../../guidebook/官方SimEnv平台环境使用手册.md) 为准。
