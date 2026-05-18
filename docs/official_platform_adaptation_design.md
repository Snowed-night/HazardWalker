# Official Platform Adaptation and Algorithm Integration Design

本文档说明 HazardWalker 如何在官方仿真环境发布后快速接入，并让导航、感知、决策算法尽量只依赖稳定接口开发。目标是降低团队成员对官方平台、主力机环境和复杂 ROS 集成的依赖。

## 1. Design Goal

核心目标：

```text
官方平台复杂性由 platform / bringup / config 吸收；
算法组只面向 HazardWalker 内部接口开发；
最小流程优先跑通，再逐步替换和增强算法。
```

项目不应让每个算法成员都直接面对官方 SDK、仿真器插件、底层话题名和硬件细节。官方平台发布后，优先由核心集成成员完成平台适配，其他成员继续按固定接口实现算法。

## 2. Layered Architecture

```text
Official Simulator / Gazebo / Isaac / Real Robot
        |
        | platform-specific topics, SDK, control APIs
        v
hazardwalker_platform
        |
        | normalized HazardWalker internal interfaces
        v
SLAM / localization
        |
        v
hazardwalker_nav <----> hazardwalker_decision <----> hazardwalker_perception
        |
        v
hazardwalker_bringup
        |
        v
results / metrics / reports
```

各层职责：

| Layer | Responsibility |
|---|---|
| Official / Gazebo / Isaac / Real Robot | 提供仿真或真实机器人输入输出 |
| `hazardwalker_platform` | 话题转换、frame 规范、控制桥接、仿真时间处理 |
| SLAM / localization | 提供 `map`、`odom`、TF 和地图 |
| `hazardwalker_nav` | 固定航点、Nav2 封装、Frontier、返航、卡死恢复 |
| `hazardwalker_perception` | 红球检测、点云定位、多帧确认、去重 |
| `hazardwalker_decision` | 状态机、目标选择、重观察、返航约束 |
| `hazardwalker_bringup` | 一键启动、参数加载、系统组合 |
| tests / reports | 指标统计、结果输出、自测试报告 |

## 3. Development Principle

### 3.1 Keep Official Platform Out of Algorithm Code

算法代码中不应出现官方平台专有话题名，例如：

```text
/official/camera/front/image
/sim/robot/lidar_points
/unitree/go2/state
```

算法模块只订阅 HazardWalker 内部接口，例如：

```text
/hw/camera/image_raw
/hw/lidar/points
/hw/odom
/hw/map
/hw/perception/hazard_detections
```

如果官方平台话题名变化，只修改：

```text
hazardwalker_platform
config/topics.yaml
launch remap
```

### 3.2 Start with Minimal Demo

第一阶段先跑通最小流程，不直接追求完整自主探索：

```text
启动仿真
机器人初始化
读取相机和雷达
固定航点移动
检测红球
估计坐标
输出结果
返回起点
```

固定航点流程跑通后，再替换为 Frontier 探索。

### 3.3 Algorithm First as Pure Functions

算法组优先写函数和小样例测试，再封装 ROS 节点。

示例：

```text
detect_red_ball(image, camera_info, params) -> detections_2d
choose_next_frontier(map, robot_pose, params) -> goal_pose
update_mission_state(state, nav_state, hazards, time_budget, params) -> next_state
```

好处：

- 便于离线测试。
- 不依赖完整仿真。
- 大一队员也可以参与单函数、小脚本和样例数据测试。
- 后续嵌入 ROS 节点更直接。

## 4. Package Responsibilities

### 4.1 `hazardwalker_platform`

官方平台发布后，优先开发该包。

建议节点：

| Node | Responsibility |
|---|---|
| `official_sensor_adapter` | 将官方相机、雷达、里程计转换到内部话题 |
| `official_control_adapter` | 将 `/hw/cmd_vel` 或导航命令转换为官方控制接口 |
| `tf_normalizer` | 统一 `map/odom/base_link/camera_link/lidar_link` |
| `clock_adapter` | 处理 `/clock` 和 `use_sim_time` |
| `result_submitter` | 如果官方要求特殊结果格式，由该节点转换提交 |

输出目标：

```text
/hw/camera/image_raw
/hw/camera/camera_info
/hw/lidar/points
/hw/odom
/tf
/tf_static
```

输入目标：

```text
/hw/cmd_vel
```

### 4.2 `hazardwalker_nav`

第一阶段：

- 固定航点巡检。
- Nav2 goal wrapper。
- 记录起点。
- 返回起点。
- 导航失败简单重试。

第二阶段：

- SLAM Toolbox 接入。
- Frontier 提取。
- 探索目标评分。
- 卡死检测和重规划。

算法输入：

```text
/hw/map
/hw/odom
/tf
/hw/nav/goal
```

算法输出：

```text
/hw/cmd_vel
/hw/nav/state
/hw/nav/active_goal
```

### 4.3 `hazardwalker_perception`

第一阶段：

- HSV 红球检测。
- 输出 2D 检测框。
- 调试图像可视化。

第二阶段：

- 点云投影或 ROI 提取。
- 输出三维坐标。
- 多帧确认。
- 空间聚类去重。

算法输入：

```text
/hw/camera/image_raw
/hw/camera/camera_info
/hw/lidar/points
/tf
```

算法输出：

```text
/hw/perception/red_ball_candidates_2d
/hw/perception/hazard_detections
/hw/perception/tracked_hazards
/hw/perception/debug_image
```

### 4.4 `hazardwalker_decision`

第一阶段：

- 简单任务状态机。
- 固定航点任务调度。
- 发现危险源后记录。
- 航点完成后返航。

第二阶段：

- Frontier 目标选择。
- 疑似目标重观察。
- 返航约束。
- 任务超时处理。

算法输入：

```text
/hw/nav/state
/hw/nav/active_goal
/hw/perception/tracked_hazards
/hw/map
/hw/odom
```

算法输出：

```text
/hw/nav/goal
/hw/mission/state
/hw/mission/event
/hw/mission/result
```

### 4.5 `hazardwalker_bringup`

负责组合系统，不写复杂算法。

建议 launch：

| Launch | Purpose |
|---|---|
| `minimal_demo.launch.py` | 最小闭环，适合第一阶段验收 |
| `gazebo_minimal.launch.py` | 自建 Gazebo 仿真 |
| `official_minimal.launch.py` | 官方环境最小适配 |
| `full_system.launch.py` | 完整系统 |

## 5. Minimal Flow

第一阶段最小流程：

```text
1. Start simulator.
2. Start platform adapter.
3. Publish normalized camera, lidar, odom and TF.
4. Start perception detector.
5. Start waypoint navigation.
6. Move robot through predefined waypoints.
7. Detect red ball.
8. Estimate hazard position.
9. Save result JSON.
10. Return to start.
```

最小验收标准：

- 机器人能够移动到至少 2 个航点。
- 相机图像能被感知节点订阅。
- 红球检测能输出候选结果。
- 系统能输出一个危险源坐标 JSON。
- 机器人能回到起点附近。

允许简化：

- 第一版可不做完整多层楼。
- 第一版可不做 Frontier。
- 第一版可不做 YOLO。
- 第一版可不做实机。
- 第一版可只处理一个红球。

## 6. Official Platform Bring-up Procedure

官方平台发布后，建议按以下顺序执行。

### Step 1: Environment Capture

记录：

```text
OS version
ROS version
Simulator version
CUDA / driver version if needed
Python version
official SDK version
```

### Step 2: Launch Official Demo

目标：

- 官方 demo 能启动。
- 机器人模型能加载。
- 仿真时间正常。
- 无关键报错。

### Step 3: Inspect Topics and Frames

命令：

```bash
ros2 topic list
ros2 topic info <topic>
ros2 interface show <message_type>
ros2 run tf2_tools view_frames
ros2 topic echo /tf --once
```

记录到：

```text
docs/official_platform_notes.md
```

### Step 4: Verify Sensors

确认：

- RGB image 是否可用。
- CameraInfo 是否可用。
- PointCloud2 是否可用。
- Odom 是否可用。
- TF 是否完整。

### Step 5: Verify Control

确认官方控制方式：

```text
/cmd_vel
Nav2 action
service API
SDK function call
```

优先目标是适配成：

```text
/hw/cmd_vel
```

### Step 6: Build Platform Adapter

把官方话题转换到：

```text
/hw/camera/image_raw
/hw/camera/camera_info
/hw/lidar/points
/hw/odom
/hw/cmd_vel
```

### Step 7: Run Minimal Demo

启动：

```bash
ros2 launch hazardwalker_bringup official_minimal.launch.py
```

验收：

- 能读取传感器。
- 能控制机器人。
- 能检测红球。
- 能写出结果。

## 7. Algorithm Integration Workflow

算法成员提交代码前，按以下方式交付：

```text
1. 算法函数
2. 参数 YAML
3. 小样例数据
4. 离线测试脚本
5. ROS 节点封装
6. README 说明输入输出
```

集成人员嵌入时只需要确认：

- topic 是否符合 `docs/interface_spec.md`。
- frame 是否能通过 TF 转换。
- 参数是否能从 YAML 加载。
- 输出是否能被下游模块订阅。

## 8. Suggested First Implementation Order

推荐实现顺序：

```text
1. interface_spec.md
2. topics.yaml / frames.yaml
3. minimal_demo.launch.py skeleton
4. platform fake adapter or Gazebo adapter
5. HSV detector offline script
6. HSV detector ROS node
7. waypoint navigation node
8. mission state machine node
9. result writer
10. official adapter after official platform release
```

不要一开始做：

- 完整 NBV。
- 实机全流程。
- YOLO 训练。
- 多层楼复杂地图。
- 所有人远程主力机开发。

## 9. Files to Prepare

建议后续补充以下文件：

```text
config/topics.yaml
config/frames.yaml
config/perception.yaml
config/nav.yaml
config/decision.yaml
scripts/setup_env.sh
scripts/build.sh
scripts/run_minimal_demo.sh
docs/official_platform_notes.md
docs/minimal_demo_acceptance.md
```

## 10. Risk Control

| Risk | Control |
|---|---|
| 官方接口与预期不同 | 通过 `hazardwalker_platform` 转换，不改算法 |
| 团队环境不统一 | 主力机负责集成，个人环境只做模块开发 |
| 大一队员学习成本高 | 分配离线脚本、文档、样例数据和测试记录任务 |
| 仿真平台难装 | 先用自建 Gazebo 最小场景，官方环境发布后再适配 |
| 算法模块互相阻塞 | 先定义 topic、frame 和函数契约 |
| 高级算法拖慢闭环 | 固定航点和 HSV 先跑通，再替换 Frontier/NBV/YOLO |

## 11. Discussion Items for Technical Lead

需要技术负责人确认：

1. 是否接受 `hazardwalker_platform` 作为唯一平台适配入口。
2. 是否确认算法模块只面向 `/hw/*` 内部接口。
3. 是否同意第一阶段用固定航点替代自主探索。
4. 是否同意第一阶段用 HSV 替代 YOLO。
5. 是否同意官方平台发布后优先适配 topic、frame、control，而不是重写算法。
6. 是否同意主力机只由核心集成成员维护，降低团队工程门槛。

确认以上原则后，团队可以把精力集中在可运行的最小系统和可替换的算法模块上。
