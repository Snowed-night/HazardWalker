# 快速启动

本文面向第一次运行比赛仿真环境的选手。默认工作目录为：

```bash
cd /home/ros/Guoyulun/Competition/SimEnv
```

## 运行要求

- Ubuntu 20.04 或兼容环境
- ROS Noetic，建议安装 `ros-noetic-desktop-full`
- Gazebo Classic
- Python >= 3.8
- `python3-yaml`
- `numpy` 和 `scipy`，用于评估脚本
- CUDA >= 11.7
- libtorch C++ 版本，用于 Unitree A1 控制器

libtorch 和 CUDA 路径在 `src/unitree_guide/unitree_guide/unitree_guide/CMakeLists.txt` 中配置。当前工程默认指向 `/home/ros/Guoyulun/Download/libtorch` 和 `/usr/local/cuda/bin/nvcc`。如部署路径不同，需要按实际机器调整。

## 编译

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j
source ./devel/setup.bash
```

## 一键启动

```bash
./auto.sh
```

`auto.sh` 会执行以下流程：

1. 清理旧的 Gazebo、roslaunch、`junior_ctrl`、门/电梯控制服务和可选虚拟手柄进程。
2. 生成随机楼栋、危险源、干扰源和真值文件。
3. 写入 `generated_building/competition_scene.world`。
4. 启动 Gazebo、Unitree A1 模型、传感器、状态话题和控制器接口。
5. 启动 `building_generator_classic` 门/电梯控制服务。
6. 启动 `devel/lib/unitree_guide/junior_ctrl`。

## 常用启动方式

固定随机种子，便于复现实验：

```bash
SEED=77 ./auto.sh
```

无 GUI 启动，适合远程服务器或性能较弱机器：

```bash
GUI=false ./auto.sh
```

调大场景规模：

```bash
SEED=20260507 FLOOR_COUNT=4 ROOMS_PER_FLOOR=5 DANGER_COUNT=5 DISTRACTOR_COUNT=8 ./auto.sh
```

不启动控制器，只启动环境：

```bash
START_CONTROLLER=0 ./auto.sh
```

## 启动参数

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
| `CONTROLLER_FOREGROUND` | `1` | 是否在前台运行控制器 |
| `START_BUILDING_CONTROL` | `1` | 是否启动楼栋门/电梯控制服务 |
| `UNITREE_CTRL_DT` | `0.004` | `junior_ctrl` 控制周期，单位 s。默认 250 Hz |
| `START_VIRTUAL_JOY` | `0` | 是否启动虚拟手柄，通常需要 `uinput` 权限 |
| `ROBOT_X` | `0.0` | 机器人出生点 x |
| `ROBOT_Y` | `-2.2` | 机器人出生点 y |
| `ROBOT_Z` | `0.6` | 机器人出生点 z |
| `ROBOT_YAW` | `1.5708` | 机器人出生点 yaw |

## 单独生成场景

只生成比赛场景，不启动 Gazebo：

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

旧入口会转调新的比赛场景生成器。

## 默认场景规模

默认楼栋尺寸按 Unitree A1 室内探索做了收敛：走廊约 2.2 m，单层默认 4 个房间，建筑占地约 20 m x 36 m。该尺寸保留进门、转向和传感器观测余量，同时避免探索时间主要消耗在长距离行走上。需要提高难度时，可逐步增大 `BUILDING_WIDTH`、`BUILDING_LENGTH` 和 `ROOMS_PER_FLOOR`。
