# 门与电梯控制

随机楼栋会同步生成动态门和简化电梯配置：

- `generated_building/door_config.yaml`
- `generated_building/elevator_config.yaml`

控制服务由 `building_generator_classic` 提供，并已合并到 `SimEnv/src`。正常使用 `auto.sh` 时会自动启动。

## 服务接口

| 服务 | 类型 | 说明 |
|------|------|------|
| `/set_door_state` | `building_generator_interfaces/SetDoorState` | 打开或关闭指定动态门 |
| `/call_elevator` | `building_generator_interfaces/CallElevator` | 将电梯轿厢移动到目标楼层 |

楼层索引从 `0` 开始：

- `0`：1 楼
- `1`：2 楼
- `2`：3 楼

## 手动启动控制服务

如需手动启动或重启门/电梯控制服务，可在 Gazebo 场景启动后运行：

```bash
cd /home/ros/Guoyulun/Competition/SimEnv
source ./devel/setup.bash
rosrun building_generator_classic building_generator_classic_control \
  --door-config ./generated_building/door_config.yaml \
  --elevator-config ./generated_building/elevator_config.yaml
```

服务日志默认写入：

```text
logs/building_control.log
```

## 开关门命令

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

## 上下电梯流程

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

## 说明

- `main_entrance` 为首层主入口门。
- `elevator_floor_0`、`elevator_floor_1` 等为各楼层电梯厅门。
- `elevator_main` 为当前楼栋默认电梯 ID。
- 电梯厅门默认采用 60 s 缓慢开门或关门过程，控制服务会持续插值更新左右门板位置。
- `rosservice call /set_door_state` 通常在门动作完成后返回。
- `/call_elevator` 负责移动轿厢到目标楼层。
- 机器人进出轿厢仍由参赛算法通过 `/cmd_vel` 控制。
- `open_doors` 字段记录电梯状态，但楼层电梯厅门建议通过 `/set_door_state` 明确开关，便于比赛流程复现。
