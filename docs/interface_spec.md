# HazardWalker Interface Specification

本文档定义 HazardWalker 各模块之间的稳定接口。所有平台适配、仿真环境、官方环境和后续实机验证都应尽量转换到本文档定义的内部接口，算法模块不直接依赖某一个具体平台的话题名或 SDK。

接口变更原则：

- 修改话题名、坐标系、消息格式、参数文件时，必须同步更新本文档。
- 官方平台发布后，优先修改 `hazardwalker_platform` 和配置文件，不直接修改导航、感知、决策算法内部逻辑。
- 第一阶段允许使用标准 ROS 消息快速跑通；稳定后再补充 `hazardwalker_msgs` 自定义消息。

## 1. System Boundary

系统分为两层：

```text
Official / Gazebo / Isaac / Real Robot
        |
hazardwalker_platform
        |
HazardWalker internal interfaces
        |
nav / perception / decision / bringup / tests
```

算法组只面向内部接口开发。平台组负责把官方平台或自建仿真的原始输入输出转换为内部接口。

## 2. Internal Topics

### 2.1 Platform Sensor Topics

| Type | Topic | Message | Publisher | Subscriber | Description |
|---|---|---|---|---|---|
| RGB image | `/hw/camera/image_raw` | `sensor_msgs/msg/Image` | `hazardwalker_platform` | perception | RGB camera image |
| Camera info | `/hw/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `hazardwalker_platform` | perception | Camera intrinsic parameters |
| Point cloud | `/hw/lidar/points` | `sensor_msgs/msg/PointCloud2` | `hazardwalker_platform` | perception, nav | LiDAR point cloud |
| Odometry | `/hw/odom` | `nav_msgs/msg/Odometry` | `hazardwalker_platform` / localization | nav, decision | Robot odometry |
| TF | `/tf` | `tf2_msgs/msg/TFMessage` | platform, SLAM, robot_state_publisher | all modules | Transform tree |
| Static TF | `/tf_static` | `tf2_msgs/msg/TFMessage` | platform, robot_state_publisher | all modules | Static transforms |

说明：

- 如果官方平台已直接发布类似话题，可通过 launch remap 接入。
- 如果官方平台话题名、frame 或消息结构不同，由 `hazardwalker_platform` 转换。

### 2.2 Map and Navigation Topics

| Type | Topic | Message | Publisher | Subscriber | Description |
|---|---|---|---|---|---|
| Occupancy map | `/hw/map` | `nav_msgs/msg/OccupancyGrid` | SLAM | nav, decision | 2D occupancy grid |
| Map metadata | `/hw/map_metadata` | `nav_msgs/msg/MapMetaData` | SLAM | nav, tests | Map metadata if needed |
| Goal pose | `/hw/nav/goal` | `geometry_msgs/msg/PoseStamped` | decision | nav | Internal navigation goal |
| Active goal | `/hw/nav/active_goal` | `geometry_msgs/msg/PoseStamped` | nav | decision, tests | Current goal being executed |
| Navigation state | `/hw/nav/state` | `std_msgs/msg/String` | nav | decision, tests | Navigation state enum |
| Velocity command | `/hw/cmd_vel` | `geometry_msgs/msg/Twist` | nav / platform adapter | platform | Internal velocity command |

说明：

- `Nav2` 原生 action 接口由 `hazardwalker_nav` 内部封装。
- 决策模块不直接调用 Nav2，优先通过 `/hw/nav/goal` 或后续 action wrapper 下发任务。

### 2.3 Perception Topics

第一阶段使用 `vision_msgs` 或 JSON 字符串可快速验证；稳定后建议迁移到 `hazardwalker_msgs`。

| Type | Topic | Message | Publisher | Subscriber | Description |
|---|---|---|---|---|---|
| 2D candidates | `/hw/perception/red_ball_candidates_2d` | `vision_msgs/msg/Detection2DArray` or TBD | perception | perception, tests | Red ball image detections |
| 3D detections | `/hw/perception/hazard_detections` | `vision_msgs/msg/Detection3DArray` or TBD | perception | decision, tests | 3D hazard detections |
| Tracked hazards | `/hw/perception/tracked_hazards` | TBD custom msg | perception | decision, tests | Deduplicated hazard list |
| Debug image | `/hw/perception/debug_image` | `sensor_msgs/msg/Image` | perception | RViz, tests | Visualization image |

### 2.4 Decision and Mission Topics

| Type | Topic | Message | Publisher | Subscriber | Description |
|---|---|---|---|---|---|
| Mission state | `/hw/mission/state` | `std_msgs/msg/String` | decision | all modules, tests | Current mission state |
| Mission event | `/hw/mission/event` | `std_msgs/msg/String` | decision, nav, perception | tests | Important event log |
| Final result | `/hw/mission/result` | `std_msgs/msg/String` or TBD | decision | tests, result writer | Final mission result |

`/hw/mission/result` 第一阶段可以发布 JSON 字符串，后续再迁移为自定义消息或写入文件。

## 3. Frames

| Frame | Description | Owner |
|---|---|---|
| `map` | Global map frame. Origin should align with mission start after initialization. | SLAM / localization |
| `odom` | Continuous odometry frame. | platform / localization |
| `base_link` | Robot base frame. | platform |
| `camera_link` | RGB camera optical body frame. | platform |
| `camera_optical_frame` | Camera optical frame if available. | platform |
| `lidar_link` | LiDAR frame. | platform |
| `start` | Optional mission start frame. | decision / platform |

Recommended TF tree:

```text
map
└── odom
    └── base_link
        ├── camera_link
        │   └── camera_optical_frame
        └── lidar_link
```

坐标要求：

- 危险源最终坐标必须能转换到起点坐标系。
- 第一阶段可将任务开始时的 `map` 坐标原点视为起点；后续若官方评测要求严格相对起点，应发布 `start` frame 并统一转换。
- 所有 3D 检测结果必须记录 frame id 和 timestamp。

## 4. Hazard Detection Data Model

单个危险源建议包含：

```json
{
  "id": 1,
  "frame_id": "map",
  "position": [0.0, 0.0, 0.0],
  "confidence": 0.85,
  "status": "tentative",
  "first_seen_time": 0.0,
  "last_seen_time": 0.0,
  "observation_count": 3,
  "source": "hsv_lidar_fusion"
}
```

`status` 可取值：

- `tentative`：疑似危险源，需要继续确认。
- `confirmed`：已确认危险源，可进入最终结果。
- `rejected`：已剔除目标，不进入最终结果。

确认建议：

- 单帧视觉检测只产生 `tentative`。
- 多帧稳定观测且空间位置一致后升级为 `confirmed`。
- 长时间无法复现、几何尺寸异常或置信度持续降低时标记为 `rejected`。

## 5. Mission State

任务状态：

```text
IDLE
EXPLORING
NAVIGATING
REOBSERVING
REPLANNING
RETURNING
FINISHED
FAILED
```

第一阶段最小闭环可只实现：

```text
IDLE
NAVIGATING
RETURNING
FINISHED
FAILED
```

状态含义：

| State | Description |
|---|---|
| `IDLE` | System initialized but mission not started |
| `EXPLORING` | Selecting or executing exploration behavior |
| `NAVIGATING` | Moving toward a selected goal |
| `REOBSERVING` | Moving or rotating to verify tentative hazards |
| `REPLANNING` | Current goal failed, selecting a new goal |
| `RETURNING` | Returning to mission start |
| `FINISHED` | Mission completed successfully |
| `FAILED` | Mission failed or timeout occurred |

## 6. Algorithm Function Contracts

为降低平台耦合，算法组应优先按照以下函数级契约开发，ROS 节点只是外层封装。

### 6.1 Perception

```text
detect_red_ball(image, camera_info, params)
    -> list[RedBall2D]

localize_hazard(red_ball_2d, point_cloud, tf_buffer, params)
    -> Hazard3D

update_hazard_tracks(hazard_observations, existing_tracks, params)
    -> list[TrackedHazard]
```

### 6.2 Navigation

```text
choose_next_frontier(map, robot_pose, params)
    -> goal_pose

plan_return_home(current_pose, start_pose, map, params)
    -> goal_pose

detect_navigation_stuck(odom_history, cmd_history, params)
    -> stuck_state
```

### 6.3 Decision

```text
update_mission_state(current_state, nav_state, hazards, time_budget, params)
    -> next_state

select_next_goal(map, robot_pose, hazards, mission_state, params)
    -> goal_pose

should_return_home(current_pose, start_pose, time_budget, params)
    -> bool
```

### 6.4 Testing

```text
evaluate_run(result_file, ground_truth_file, params)
    -> metrics
```

## 7. Result Output

第一阶段建议输出 JSON 文件：

```json
{
  "mission_id": "minimal_demo_001",
  "start_time": 0.0,
  "end_time": 0.0,
  "status": "FINISHED",
  "hazards": [
    {
      "id": 1,
      "position": [1.2, -0.4, 0.8],
      "frame_id": "start",
      "confidence": 0.91
    }
  ],
  "metrics": {
    "duration_sec": 120.5,
    "return_success": true,
    "num_confirmed_hazards": 1,
    "num_false_positive_estimate": 0
  }
}
```

建议保存路径：

```text
reports/run_results/<timestamp>_result.json
```

## 8. Launch and Parameter Interfaces

建议统一使用 YAML 管理参数：

```text
config/
  topics.yaml
  frames.yaml
  perception.yaml
  nav.yaml
  decision.yaml
  mission.yaml
```

建议 launch 入口：

```text
hazardwalker_bringup/launch/minimal_demo.launch.py
hazardwalker_bringup/launch/gazebo_minimal.launch.py
hazardwalker_bringup/launch/official_minimal.launch.py
hazardwalker_bringup/launch/full_system.launch.py
```

第一阶段必须保证有一个最小入口：

```bash
ros2 launch hazardwalker_bringup minimal_demo.launch.py
```

## 9. Official Platform Adaptation Checklist

官方平台发布后，平台组按以下顺序确认：

1. 机器人控制接口：是否支持 `/cmd_vel`、action、service 或 SDK API。
2. RGB 相机话题、消息类型、频率、frame。
3. 雷达话题、消息类型、频率、frame。
4. 里程计和 TF 来源。
5. 地图、定位、SLAM 是否由官方提供。
6. 起点坐标定义和结果输出格式。
7. 评测脚本如何读取结果。
8. 是否允许额外 ROS 节点和自定义消息。
9. 是否允许修改 launch、参数、桥接节点。
10. 仿真时间 `/clock` 和真实时间使用方式。

确认后只修改：

```text
hazardwalker_platform
hazardwalker_bringup
config/topics.yaml
config/frames.yaml
```

算法模块原则上不改或少改。
