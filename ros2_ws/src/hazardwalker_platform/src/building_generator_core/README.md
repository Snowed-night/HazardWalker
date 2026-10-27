# building_generator

面向 `ROS1 Noetic + Gazebo Classic` 的随机楼栋生成器，用于为机器狗训练生成可复现的多楼层室内场景。

当前版本提供：

- 固定首层主入口中心为世界原点
- 支持楼层数、每层房间数的固定值和范围采样
- 强制包含楼梯核心区和电梯核心区
- 输出 `world.sdf`、`model.sdf`、布局元数据、门配置、电梯配置、生成检查报告
- ROS1 服务接口控制关键门和简化电梯状态机

当前默认几何效果：

- 楼梯为室内连续之字形楼梯
- 电梯轿厢开口与楼层电梯门同侧
- 电梯轿厢地板与楼层板齐平
- 电梯门到轿厢前缘只保留小安全缝
- 电梯门下方、门内侧、楼梯边缘已补板，降低踩空风险
- 大厅默认不生成桌椅长凳障碍物

## 包结构

- `building_generator_core`
  - 约束解析
  - 拓扑生成
  - SDF 导出
  - CLI
  - Python API
- `building_generator_interfaces`
  - `CallElevator.srv`
  - `SetDoorState.srv`
- `building_generator_classic`
  - Gazebo Classic 场景导出清单
  - `rospy` 控制服务节点
  - 门 / 电梯内存状态机

## 工作区要求

这三个包现在是 `catkin` 包，不应继续放在当前这个混合 ROS2 工作区里直接构建。

推荐把下面三个目录单独放进一个 Noetic catkin 工作区的 `src/` 下：

- `building_generator_core`
- `building_generator_interfaces`
- `building_generator_classic`

## 快速构建

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/building_generator_ws/src
cd ~/building_generator_ws/src

# 把这三个包拷贝或软链接到这里
# building_generator_core
# building_generator_interfaces
# building_generator_classic

cd ~/building_generator_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_VERSION=3.8 -j2
source devel/setup.bash
```

## CLI 用法

### 生成单个场景

```bash
rosrun building_generator_core building_generator_cli generate \
  --seed 41 \
  --floor-count 2:4 \
  --rooms-per-floor 4:8 \
  --width 30 \
  --length 58 \
  --target gazebo_classic \
  --output-dir /tmp/building_gen_case
```

输出目录包含：

- `world.sdf`
- `model.sdf`
- `layout_metadata.json`
- `elevator_config.yaml`
- `door_config.yaml`
- `generation_checks.json`

`generation_checks.json` 会在导出时自动检查：

- 电梯轿厢姿态、轿厢地板标高、门前间隙、门下补板
- 楼梯平台和楼梯段衔接、楼梯边补板
- 动态门门板是否存在、开关位姿是否可用
- 房间边界、房间重叠、房门位姿、目标点和家具边界是否合法

任一检查失败，导出会直接报错。

### 批量生成

```bash
rosrun building_generator_core building_generator_cli batch \
  --seed-list 11,12,13 \
  --floor-count 2:5 \
  --rooms-per-floor 4:10 \
  --width 36 \
  --length 72 \
  --target gazebo_classic \
  --output-dir /tmp/building_gen_batch
```

## Python API

```python
from building_generator_core import BuildingConstraints, export_sdf, generate_layout
from building_generator_classic import export_classic_bundle

constraints = BuildingConstraints.from_dict(
    {
        "seed": 52,
        "floor_count": {"min": 2, "max": 4},
        "rooms_per_floor": {"min": 4, "max": 8},
        "room_type_mix": {"office": 3, "meeting": 1, "storage": 1},
        "building_footprint_limit": {"width": 32.0, "length": 64.0},
    }
)

layout = generate_layout(constraints)
artifacts = export_sdf(layout, target="gazebo_classic", output_dir="/tmp/building_python_case")
manifest_path = export_classic_bundle(layout, "/tmp/building_python_case")
```

主要入口：

- `BuildingConstraints.from_dict(...)`
- `generate_layout(constraints)`
- `export_sdf(layout, target, output_dir)`
- `export_classic_bundle(layout, output_dir)`

## 约束格式

`floor_count` 和 `rooms_per_floor` 支持三种写法：

```python
2
{"exact": 4}
{"min": 2, "max": 6}
```

推荐完整输入：

```python
{
    "seed": 7,
    "floor_count": {"min": 2, "max": 4},
    "rooms_per_floor": {"min": 4, "max": 8},
    "room_type_mix": {"office": 3, "storage": 1, "meeting": 1, "lounge": 1},
    "building_footprint_limit": {"width": 36.0, "length": 72.0},
    "stair_required": True,
    "elevator_required": True,
    "origin_anchor": "main_entrance_center",
    "dynamic_doors": ["main_entrance", "elevator"],
    "dynamic_elevator": True,
}
```

## 生成语义

当前版本保证：

- 入口位姿固定为 `(0, 0, 0, 0, 0, 0)`
- 每层均可到达楼梯和电梯
- 每层所有房间均从主走廊可达
- 一栋楼默认只有一个楼梯核心和一个电梯核心
- 普通房门保持静态开口
- 动态门覆盖主入口门和电梯门
- 电梯采用简单召唤模型，不做群控和乘客逻辑

## Gazebo Classic 接入

Classic 侧走离线导出路径：

```python
from building_generator_classic import export_classic_bundle

manifest_path = export_classic_bundle(layout, "/tmp/building_classic_case")
```

`classic_bundle.yaml` 会引用：

- `world.sdf`
- `model.sdf`
- `door_config.yaml`
- `elevator_config.yaml`
- `layout_metadata.json`
- `validation_report`

可以直接用 Gazebo Classic 加载导出的 `world.sdf`。

## ROS1 控制接口

### 服务定义

- `building_generator_interfaces/CallElevator`
- `building_generator_interfaces/SetDoorState`

`CallElevator.srv`

```text
string elevator_id
int32 target_floor
bool open_doors
---
bool accepted
int32 current_floor
string state
string message
```

`SetDoorState.srv`

```text
string door_id
bool open
---
bool accepted
string state
string message
```

### 启动控制服务

```bash
rosrun building_generator_classic building_generator_classic_control \
  --door-config /tmp/building_gen_case/door_config.yaml \
  --elevator-config /tmp/building_gen_case/elevator_config.yaml
```

### 调用示例

```bash
rosservice call /call_elevator "{elevator_id: 'elevator_main', target_floor: 2, open_doors: true}"
rosservice call /set_door_state "{door_id: 'main_entrance', open: false}"
```

楼层索引从 `0` 开始：

- `0` = 1 楼
- `1` = 2 楼
- `2` = 3 楼

说明：

- 当前服务节点负责控制面和状态机
- 它会维护门和电梯的内存状态
- 真实 Gazebo 插件联动仍可在此基础上继续扩展

## 测试

在 Noetic catkin 工作区中可以运行：

```bash
source /opt/ros/noetic/setup.bash
cd ~/building_generator_ws
catkin_make run_tests
catkin_test_results build
```

## 当前边界

当前版本还没有实现：

- 多电梯群控
- 多核心筒
- 复杂动态房门
- Gazebo world 内真实门体 / 电梯插件动作联动
- 大规模高密度室内家具布置
