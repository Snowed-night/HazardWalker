# 常见问题

本文记录比赛环境启动和运行中常见问题。更多日志可查看 `logs/` 目录。

## `absoluteWait is not enough`

现象：

```text
[WARNING] The waitTime=2000 of function absoluteWait is not enough!
The program has already cost 2435us.
```

含义：`junior_ctrl` 单次控制循环耗时超过目标周期。`waitTime=2000` 表示目标周期为 2000 us，即 500 Hz。该提示不代表场景生成失败。

当前 `auto.sh` 默认设置：

```bash
UNITREE_CTRL_DT=0.004
```

即 250 Hz，通常能减少该 warning。仍持续刷屏时可降低 Gazebo 负载：

```bash
GUI=false UNITREE_CTRL_DT=0.006 ./auto.sh
```

如机器性能充足并需要沿用 500 Hz：

```bash
UNITREE_CTRL_DT=0.002 ./auto.sh
```

## Gazebo 没有正常退出或端口被占用

`auto.sh` 启动前会尝试清理旧进程，包括：

- `roslaunch unitree_guide multi_floor_gazeboSim.launch`
- `building_generator_classic_control`
- `gzserver`
- `gzclient`
- `gazebo`
- `junior_ctrl`
- `virtual_joy.py`

如仍异常，可手动检查：

```bash
ps aux | rg "gazebo|gzserver|gzclient|roslaunch|junior_ctrl|building_generator_classic_control"
```

## 门或电梯服务不可用

先确认服务是否存在：

```bash
rosservice list | rg "set_door_state|call_elevator"
```

如服务不存在，可手动启动：

```bash
source ./devel/setup.bash
rosrun building_generator_classic building_generator_classic_control \
  --door-config ./generated_building/door_config.yaml \
  --elevator-config ./generated_building/elevator_config.yaml
```

检查日志：

```bash
tail -n 80 logs/building_control.log
```

## 找不到 ROS 包

如果 `rosrun` 或 `rospack find` 找不到包，通常是没有 source 工作空间：

```bash
source /opt/ros/noetic/setup.bash
source ./devel/setup.bash
```

## 评估脚本缺少依赖

评估脚本依赖 `numpy` 和 `scipy`。如报导入错误，请安装：

```bash
sudo apt install python3-numpy python3-scipy
```

## 虚拟手柄权限问题

`START_VIRTUAL_JOY=1` 会尝试启动虚拟手柄，通常需要 `uinput` 权限。比赛算法一般可以直接发布 `/cmd_vel`，不必开启虚拟手柄。
