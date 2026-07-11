# 完整参考文档

本文保留拆分前的完整说明，便于交叉核对历史内容。比赛选手建议优先阅读根目录 [README](../README.md) 和本目录下的专题文档。

## 环境介绍

本目录为比赛仿真环境，面向 `ROS1 Noetic + Gazebo Classic + Unitree A1`。

当前版本已将随机楼栋生成、危险源/干扰源生成、真值记录和 Gazebo 启动流程合并为一个闭环。每次启动前先生成完整比赛场景，再启动 Gazebo、A1 机器人模型、传感器链路和控制器接口。

比赛侧默认语义如下：

- 楼栋为多楼层室内建筑，包含房间、走廊、楼梯、电梯和门。
- 危险源为红色球体。
- 干扰源为红色方块和绿色球体。
- 源只允许生成在房间内部。
- 源生成时避开墙体、家具、其他源以及房间门口保留区。
- 源高度贴合对应楼层地面，不悬空，不嵌入地板。
- 真值只写入 `results/danger_truth.json`，参赛算法应输出 `results/detected_danger.json`。

## 运行要求

- Ubuntu 20.04 或兼容环境
- ROS Noetic，建议安装 `ros-noetic-desktop-full`
- Gazebo Classic
- CUDA >= 11.7
- Python >= 3.8
- `python3-yaml`
- `scipy` 和 `numpy`，用于评估脚本
- libtorch C++ 版本，用于 Unitree A1 控制器

libtorch 和 CUDA 路径在 `src/unitree_guide/unitree_guide/unitree_guide/CMakeLists.txt` 中配置。当前工程默认指向 `/home/ros/Guoyulun/Download/libtorch` 和 `/usr/local/cuda/bin/nvcc`，如部署路径不同，需要按实际机器调整。

## 编译

```bash
cd /home/ros/Guoyulun/Competition/SimEnv
source /opt/ros/noetic/setup.bash
catkin_make -j
source ./devel/setup.bash
```

## 一键启动

```bash
cd /home/ros/Guoyulun/Competition/SimEnv
./auto.sh
```

`auto.sh` 会执行以下步骤：

1. 清理旧的 Gazebo、roslaunch、`junior_ctrl` 和可选虚拟手柄进程。
2. 生成随机楼栋、危险源、干扰源和真值文件。
3. 将完整场景写入 `generated_building/competition_scene.world`。
4. 设置 `BUILDING_WORLD_FILE`，启动 `unitree_guide multi_floor_gazeboSim.launch`。
5. 启动 Unitree A1 Gazebo 模型、传感器话题、状态话题和控制器接口。
6. 默认启动 `building_generator_classic` 门/电梯控制服务。
7. 默认启动 `devel/lib/unitree_guide/junior_ctrl`。

启动后关键文件如下：

| 文件 | 说明 |
|------|------|
| `generated_building/competition_scene.world` | Gazebo 使用的完整比赛世界，已包含楼栋和全部源模型 |
| `generated_building/layout_metadata.json` | 楼栋布局、房间、门、电梯、目标点等元数据 |
| `generated_building/door_config.yaml` | 动态门控制配置，由 `building_generator_classic` 读取 |
| `generated_building/elevator_config.yaml` | 简化电梯控制配置，由 `building_generator_classic` 读取 |
| `generated_building/scene_manifest.json` | 本次场景 manifest，记录 seed、文件路径、源数量、机器人出生点 |
| `generated_building/building_config.json` | 兼容脚本使用的建筑配置 |
| `results/danger_truth.json` | 裁判真值文件，包含危险源和干扰源列表 |
| `results/detected_danger.json` | 参赛算法应输出的检测结果文件 |
| `logs/competition_gazebo.log` | Gazebo/launch 日志 |
| `logs/building_control.log` | 楼栋门/电梯控制服务日志 |
| `logs/junior_ctrl.log` | 控制器日志 |

## 启动参数

`auto.sh` 通过环境变量控制随机场景和启动行为：

```bash
SEED=77 FLOOR_COUNT=3 ROOMS_PER_FLOOR=4 ./auto.sh
```

可用参数：

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `SEED` | 空 | 场景随机种子。为空时自动生成随机种子并写入 manifest |
| `FLOOR_COUNT` | `3` | 楼层数，支持单值 |
| `ROOMS_PER_FLOOR` | `4` | 每层房间数，支持单值 |
| `BUILDING_WIDTH` | `20.0` | 楼栋宽度，单位 m |
| `BUILDING_LENGTH` | `36.0` | 楼栋长度，单位 m |
| `DANGER_COUNT` | `3:6` | 危险源数量，支持 `min:max` |
| `DISTRACTOR_COUNT` | `4:8` | 干扰源数量，支持 `min:max` |
| `GUI` | `true` | 是否启动 Gazebo GUI |
| `PAUSED` | `true` | Gazebo 启动后是否暂停 |
| `START_CONTROLLER` | `1` | 是否启动 `junior_ctrl` |
| `CONTROLLER_FOREGROUND` | `1` | 是否在前台运行控制器。前台运行时可以在当前终端输入 `2`、`6` 切换状态 |
| `START_BUILDING_CONTROL` | `1` | 是否启动楼栋门/电梯控制服务 |
| `UNITREE_CTRL_DT` | `0.004` | `junior_ctrl` 控制周期，单位 s。默认 250 Hz，降低 Gazebo 大场景下控制循环超时 warning |
| `START_VIRTUAL_JOY` | `0` | 是否启动虚拟手柄。该功能通常需要 `uinput` 权限 |
| `ROBOT_X` | `0.0` | 机器人出生点 x |
| `ROBOT_Y` | `-2.2` | 机器人出生点 y |
| `ROBOT_Z` | `0.6` | 机器人出生点 z |
| `ROBOT_YAW` | `1.5708` | 机器人出生点 yaw |

示例：

```bash
SEED=20260507 FLOOR_COUNT=4 ROOMS_PER_FLOOR=5 DANGER_COUNT=5 DISTRACTOR_COUNT=8 GUI=false ./auto.sh
```

## 单独生成场景

如只需生成比赛场景，不启动 Gazebo：

```bash
source ./devel/setup.bash
rosrun building_obstacles generate_competition_scene.py \
  --seed 77 \
  --floor-count 3 \
  --rooms-per-floor 4 \
  --width 20 \
  --length 36 \
  --danger-count 4 \
  --distractor-count 6 \
  --output-dir ./generated_building \
  --results-dir ./results
```

兼容旧命令：

```bash
rosrun building_obstacles generate_multi_floor_building.py ./generated_building 3 4
```

该旧入口现在会转调新的比赛场景生成器。

默认楼栋尺寸按 Unitree A1 室内探索做了收敛：走廊约 2.2 m，单层默认 4 个房间，建筑占地约 20 m x 36 m。该尺寸保留进门、转向和传感器观测余量，同时避免场景过大导致探索时间主要消耗在长距离行走上。若需要提高比赛难度，可通过 `BUILDING_WIDTH`、`BUILDING_LENGTH` 和 `ROOMS_PER_FLOOR` 逐步增大场景。

## 源生成规则

生成器按楼栋布局元数据采样源位置，约束包括：

- 只在 `layout_metadata.json` 中登记的房间边界内部采样。
- 距房间墙体保持安全边距。
- 距家具保持安全边距，避免源模型与桌椅、柜体等重叠。
- 多个源之间保持最小间距，避免危险源和干扰源互相重叠。
- 房门附近设置保留区，不在门口及进入房间的短通道上放置源，避免阻挡机器狗进入房间。
- 球体中心高度为楼层高度加半径；方块中心高度为楼层高度加半高。
- 输出真值中的危险源为红色球体；干扰源为红色方块或绿色球体。

当前源特征：

| 类别 | 颜色 | 形状 | 尺寸 | 是否计为危险源 |
|------|------|------|------|----------------|
| 危险源 | 红色 | 球体 | 半径 0.15 m | 是 |
| 干扰源 | 红色 | 方块 | 0.30 m x 0.30 m x 0.30 m | 否 |
| 干扰源 | 绿色 | 球体 | 半径 0.15 m | 否 |

## 控制器与算法接入

`auto.sh` 默认以前台方式启动 `junior_ctrl`。该控制器仍遵循 Unitree 原有交互流程，进入 RL 模式后接收 `/cmd_vel`：

- 键盘输入 `2`：站立。
- 键盘输入 `6`：切换到 RL 模式。
- RL 模式下订阅 `/cmd_vel`，消息类型为 `geometry_msgs/Twist`。

`junior_ctrl` 原始控制周期为 `0.002 s`，即 500 Hz。在 Gazebo GUI、随机楼栋、传感器和 RL 推理同时运行时，部分机器会出现：

```text
[WARNING] The waitTime=2000 of function absoluteWait is not enough!
The program has already cost 2435us.
```

该提示表示单次控制循环耗时超过了 2 ms 目标周期，不代表场景生成失败。当前 `auto.sh` 默认设置 `UNITREE_CTRL_DT=0.004`，即 250 Hz，通常能减少该 warning，并保持仿真控制稳定。如机器性能充足或需要沿用 500 Hz，可显式启动：

```bash
UNITREE_CTRL_DT=0.002 ./auto.sh
```

如仍持续刷屏，建议同时降低运行负载：

```bash
GUI=false UNITREE_CTRL_DT=0.006 ./auto.sh
```

算法接入建议：

| 接口 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 机器人速度指令输入 |
| `/Odometry_gazebo` | `nav_msgs/Odometry` | 仿真里程计输出 |
| `/scan` | `sensor_msgs/PointCloud2` | Livox Mid-360 点云数据 |
| `/livox/imu` | `sensor_msgs/Imu` | Livox 内置 IMU |
| `/trunk_imu` | `sensor_msgs/Imu` | 机体 IMU |
| `/camera/image_raw` | `sensor_msgs/Image` | 前视 RGB 图像 |
| `/real_sense/depth/points` | `sensor_msgs/PointCloud2` | 深度相机点云 |

参赛算法不应读取 `results/danger_truth.json`。该文件用于裁判评估和环境自检。

## 门与电梯控制

随机楼栋会同步生成动态门和简化电梯配置：

- `generated_building/door_config.yaml`
- `generated_building/elevator_config.yaml`

控制服务由 `building_generator_classic` 提供，当前已合并到 `SimEnv/src`。正常使用 `auto.sh` 时，脚本会在 Gazebo 启动后自动启动该服务：

```bash
START_BUILDING_CONTROL=1 ./auto.sh
```

如需手动启动或重启门/电梯控制服务，可在 Gazebo 场景启动后运行：

```bash
cd /home/ros/Guoyulun/Competition/SimEnv
source ./devel/setup.bash
rosrun building_generator_classic building_generator_classic_control \
  --door-config ./generated_building/door_config.yaml \
  --elevator-config ./generated_building/elevator_config.yaml
```

服务日志默认写入 `logs/building_control.log`。

服务启动后可使用以下接口：

| 服务 | 类型 | 说明 |
|------|------|------|
| `/set_door_state` | `building_generator_interfaces/SetDoorState` | 打开或关闭指定动态门 |
| `/call_elevator` | `building_generator_interfaces/CallElevator` | 将电梯轿厢移动到目标楼层 |

楼层索引从 `0` 开始：

- `0`：1 楼
- `1`：2 楼
- `2`：3 楼

常用开关门命令：

```bash
# 打开主入口门
rosservice call /set_door_state "{door_id: 'main_entrance', open: true}"

# 关闭主入口门
rosservice call /set_door_state "{door_id: 'main_entrance', open: false}"

# 打开 1 楼电梯厅门
rosservice call /set_door_state "{door_id: 'elevator_floor_0', open: true}"

# 关闭 1 楼电梯厅门
rosservice call /set_door_state "{door_id: 'elevator_floor_0', open: false}"
```

常用上下电梯命令：

```bash
# 呼叫电梯到 1 楼，保持门关闭
rosservice call /call_elevator "{elevator_id: 'elevator_main', target_floor: 0, open_doors: false}"

# 打开 1 楼电梯厅门，机器人进入轿厢
rosservice call /set_door_state "{door_id: 'elevator_floor_0', open: true}"

# 关闭 1 楼电梯厅门
rosservice call /set_door_state "{door_id: 'elevator_floor_0', open: false}"

# 电梯上行到 2 楼
rosservice call /call_elevator "{elevator_id: 'elevator_main', target_floor: 1, open_doors: false}"

# 打开 2 楼电梯厅门，机器人离开轿厢
rosservice call /set_door_state "{door_id: 'elevator_floor_1', open: true}"

# 电梯下行回 1 楼
rosservice call /call_elevator "{elevator_id: 'elevator_main', target_floor: 0, open_doors: false}"
```

说明：

- `main_entrance` 为首层主入口门。
- `elevator_floor_0`、`elevator_floor_1` 等为各楼层电梯厅门。
- `elevator_main` 为当前楼栋默认电梯 ID。
- 电梯厅门默认采用 60 s 缓慢开门或关门过程，控制服务会持续插值更新左右门板位置，`rosservice call /set_door_state` 通常在动作完成后返回。
- 当前电梯为简化仿真模型，`/call_elevator` 负责移动轿厢到目标楼层；机器人进出轿厢仍由参赛算法通过 `/cmd_vel` 控制。
- `open_doors` 字段记录电梯状态，但楼层电梯厅门建议仍通过 `/set_door_state` 明确开关，便于比赛流程可复现。

## 结果文件格式

参赛算法完成探索和识别后，应生成：

```text
results/detected_danger.json
```

格式如下：

```json
{
  "exploration_time": 98.76,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]},
    {"position": [-3.21, 4.78, 1.75]}
  ]
}
```

字段说明：

- `exploration_time`：探索耗时，单位秒。
- `detected_danger_sources`：识别出的危险源列表。
- `position`：危险源在 Gazebo `world` 坐标系下的位置，三维坐标 `[x, y, z]`，单位 m。

## 评估

完成检测结果输出后运行：

```bash
python3 ./src/building_obstacles/scripts/evaluate_danger.py \
  --truth-file ./results/danger_truth.json \
  --detected-file ./results/detected_danger.json \
  --output-file ./results/evaluation_result.json
```

旧拼写脚本仍保留兼容：

```bash
python3 ./src/building_obstacles/scripts/evaulate_danger.py
```

主要客观指标：

- 探索时间。
- 危险源识别概率。
- 危险源虚警率。

评估脚本会在阈值内进行一对一匹配，并输出 `results/evaluation_result.json`。

## 传感器位姿配置

本仿真环境中的 Unitree A1 机器人配备了以下传感器，其安装位姿定义如下（所有坐标系遵循ROS标准：X-前，Y-左，Z-上）：

### 1. 机载IMU (`imu_link`)

| 属性 | 数值 |
|------|------|
| **父坐标系** | `trunk` (机器人躯干) |
| **安装位置** | 躯干中心 |
| **位姿 (xyz, rpy)** | `(0.0, 0.0, 0.0)`, `(0.0, 0.0, 0.0)` |
| **质量** | 0.001 kg |

&gt; 注：IMU安装于机器人质心位置，与躯干坐标系重合。

---

### 2. Livox Mid-360 激光雷达 (`laser_livox`)

| 属性 | 数值 |
|------|------|
| **父坐标系** | `base` |
| **安装位置** | 躯干前上方 |
| **位姿 (xyz, rpy)** | `(0.2, 0.0, 0.08)`, `(0.0, 0.785, 0.0)` |
| **备注** | 绕Y轴倾斜45°（0.785 rad），优化前方及地面点云覆盖 |

**Livox内置IMU** (`livox_imu_link`):
- **父坐标系**: `laser_livox`
- **相对位姿**: `(-0.011, -0.02329, 0.04412)`, `(0.0, 0.0, 0.0)`
- 该IMU固连于激光雷达本体，提供高频姿态数据

---

### 3. RealSense D415 深度相机 (`real_sense`)

| 属性 | 数值 |
|------|------|
| **父坐标系** | `base` |
| **安装位置** | 躯干最前端 |
| **位姿 (xyz, rpy)** | `(0.28, 0.0, 0.043)`, `(0.0, 0.0, 0.0)` |
| **视觉朝向** | 绕Z轴旋转90°（1.5708 rad），确保图像坐标系对齐 |
| **质量** | 0.103 kg |

---

## ROS话题列表

以下为Gazebo仿真环境中所有可用的ROS话题，按功能分类：

### 1. 状态估计与真值

| 话题名称 | 消息类型 | 发布频率 | 说明 |
|---------|---------|---------|------|
| `/trunk_imu` | `sensor_msgs/Imu` | 1000 Hz | 躯干IMU数据（加速度、角速度、姿态） |
| `/livox/imu` | `sensor_msgs/Imu` | 1000 Hz | Livox雷达内置IMU数据 |
| `/ground_truth/base_trunk` | `nav_msgs/Odometry` | 100 Hz | base相对于trunk的真值位姿（用于调试） |
| `/ground_truth/base_w` | `nav_msgs/Odometry` | 100 Hz | base相对于world的真值位姿 |

### 2. 足端状态真值

| 话题名称 | 消息类型 | 发布频率 | 说明 |
|---------|---------|---------|------|
| `/ground_truth/FL_foot` | `nav_msgs/Odometry` | 100 Hz | 左前足(FL)相对于base的位姿与速度 |
| `/ground_truth/FR_foot` | `nav_msgs/Odometry` | 100 Hz | 右前足(FR)相对于base的位姿与速度 |
| `/ground_truth/RL_foot` | `nav_msgs/Odometry` | 100 Hz | 左后足(RL)相对于base的位姿与速度 |
| `/ground_truth/RR_foot` | `nav_msgs/Odometry` | 100 Hz | 右后足(RR)相对于base的位姿与速度 |

### 3. 足端接触力

| 话题名称 | 消息类型 | 发布频率 | 说明 |
|---------|---------|---------|------|
| `/FR_foot_contact` | `gazebo_msgs/ContactsState` | 100 Hz | 右前足接触力（含可视化） |
| `/FL_foot_contact` | `gazebo_msgs/ContactsState` | 100 Hz | 左前足接触力（含可视化） |
| `/RR_foot_contact` | `gazebo_msgs/ContactsState` | 100 Hz | 右后足接触力（含可视化） |
| `/RL_foot_contact` | `gazebo_msgs/ContactsState` | 100 Hz | 左后足接触力（含可视化） |

### 4. 激光雷达

| 话题名称 | 消息类型 | 发布频率 | 说明 |
|---------|---------|---------|------|
| `/scan` | `sensor_msgs/PointCloud2` | 10 Hz | Livox Mid-360点云数据 |
| `/livox/imu` | `sensor_msgs/Imu` | 1000 Hz | 雷达内置IMU（同状态估计） |

**雷达参数**：
- 水平FOV: 360° (`0 ~ 2π`)
- 垂直FOV: -5.22° ~ 57.22°
- 测距范围: 0.1 ~ 40 m
- 分辨率: 0.01 m
- 噪声: 高斯噪声 σ=0.005

### 5. 视觉传感器

#### 5.1 前视单目相机 (`front_camera`)

| 话题名称 | 消息类型 | 发布频率 | 说明 |
|---------|---------|---------|------|
| `/camera/image_raw` | `sensor_msgs/Image` | 30 Hz | RGB图像 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | 相机标定参数 |

**相机参数**：
- 分辨率: 800x800
- 水平FOV: 1.396 rad (80°)
- 近/远裁剪面: 0.02 / 300 m
- 噪声: 高斯噪声 σ=0.007

#### 5.2 RealSense D415深度相机 (`real_sense`)

| 话题名称 | 消息类型 | 发布频率 | 说明 |
|---------|---------|---------|------|
| `/real_sense/rgb/image_raw` | `sensor_msgs/Image` | 10 Hz | RGB图像 |
| `/real_sense/rgb/camera_info` | `sensor_msgs/CameraInfo` | 10 Hz | RGB相机标定 |
| `/real_sense/depth/image_raw` | `sensor_msgs/Image` | 10 Hz | 深度图像 |
| `/real_sense/depth/camera_info` | `sensor_msgs/CameraInfo` | 10 Hz | 深度相机标定 |
| `/real_sense/depth/points` | `sensor_msgs/PointCloud2` | 10 Hz | 点云数据 |

**相机参数**：
- 分辨率: 640x480
- 水平FOV: 60°
- 近/远裁剪面: 0.05 / 8.0 m
- 点云最小距离: 0.4 m

### 6. 外部控制

| 话题名称 | 消息类型 | 功能 |
|---------|---------|------|
| `/apply_force/trunk` | `geometry_msgs/Wrench` | 向躯干施加外力/力矩（用于扰动测试） |

---

### 坐标系定义

| 坐标系ID | 父坐标系 | 说明 |
|---------|---------|------|
| `world` | - | Gazebo世界坐标系 |
| `base` | `world` | 机器人基坐标系（几何中心） |
| `trunk` | `base` | 浮动基座（质心） |
| `imu_link` | `trunk` | 机载IMU |
| `laser_livox` | `base` | Livox雷达 |
| `livox_imu_link` | `laser_livox` | 雷达内置IMU |
| `real_sense` | `base` | RealSense深度相机 |
| `front_camera` | - | 前视单目相机（需在URDF中定义joint） |
| `FR_foot` / `FL_foot` / `RR_foot` / `RL_foot` | 各小腿 | 足端接触点 |

### 使用示例

```bash
# 查看IMU数据
rostopic echo /trunk_imu

# 可视化点云
rosrun rviz rviz
# 添加 PointCloud2 显示，订阅 /scan

# 查看深度图像
rosrun image_view image_view image:=/real_sense/depth/image_raw
