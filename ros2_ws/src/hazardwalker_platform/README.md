# 比赛仿真环境

本目录为比赛仿真环境，面向 `ROS1 Noetic + Gazebo Classic + Unitree A1`。环境启动时会随机生成多楼层室内楼栋，并同步生成危险源、干扰源、门、电梯、传感器链路和机器人控制接口。

比赛目标是控制机器狗完成未知室内环境探索，识别并输出危险源位置。危险源真值文件仅供裁判评估使用，参赛算法不应读取。

## HazardWalker 官方适配覆盖层

本目录还包含 `hazardwalker_platform/official_simenv_mapping.py`，用于约束 HazardWalker 对官方 ROS1
Noetic + Gazebo Classic 环境的稳定 `/hw/*` 接口。官方场景不复用本地 `ros_gz_bridge.yaml`：当前接入通过容器
`rosbridge_websocket` 与 ROS2 适配器完成，不能假设容器内存在 `ros1_bridge dynamic_bridge`。启动、传感器与控制
验收应按 [`docs/guidebook/官方SimEnv平台环境使用手册.md`](../../../docs/guidebook/官方SimEnv平台环境使用手册.md)
逐段完成后，才运行业务闭环。

负责人统一的导航控制、W/S/A/D/K 键盘与安全停止流程见
[`docs/groups/nav/官方SimEnv控制链路与键盘测试.md`](../../../docs/groups/nav/官方SimEnv控制链路与键盘测试.md)。

## 选手快速入口

| 你要做什么 | 推荐阅读 |
|------------|----------|
| 第一次启动环境 | [快速启动](docs/quick-start.md) |
| 接入导航、感知或控制算法 | [算法接入接口](docs/algorithm-interfaces.md) |
| 理解楼栋、危险源和干扰源 | [比赛场景规则](docs/competition-rules.md) |
| 控制门和电梯 | [门与电梯控制](docs/doors-and-elevator.md) |
| 输出结果并计算分数 | [结果格式与评估方法](docs/evaluation.md) |
| 查看传感器安装、话题和坐标系 | [传感器与 ROS 话题](docs/sensors-and-topics.md) |
| 处理启动 warning 或服务异常 | [常见问题](docs/troubleshooting.md) |
| 查看旧版完整长文档 | [完整参考文档](docs/reference.md) |

## 任务描述

- 楼栋为多楼层室内建筑，包含房间、走廊、楼梯、电梯和动态门。
- 危险源为红色球体。
- 干扰源为红色方块和绿色球体。
- 源只生成在房间内部，并避开墙体、家具、其他源和房间门口保留区。
- 真值写入 `results/danger_truth.json`。
- 参赛算法应输出 `results/detected_danger.json`。

## 启动流程

```bash
cd /home/ros/Guoyulun/Competition/SimEnv
source /opt/ros/noetic/setup.bash
catkin_make -j
source ./devel/setup.bash
./auto.sh
```

`auto.sh` 会自动完成随机场景生成、Gazebo 启动、A1 模型与传感器启动、门/电梯控制服务启动和 `junior_ctrl` 控制器启动。更多启动方式见 [快速启动](docs/quick-start.md)。

## 算法接口

| 接口 | 类型 | 用途 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 机器人速度指令输入 |
| `/Odometry_gazebo` | `nav_msgs/Odometry` | 仿真里程计 |
| `/scan` | `sensor_msgs/LaserScan` | 官方 Gazebo 激光扫描 |
| `/real_sense/rgb/image_raw` | `sensor_msgs/Image` | RealSense RGB 图像 |
| `/real_sense/depth/image_raw` | `sensor_msgs/Image` | RealSense 深度图（`32FC1`） |
| `/camera/image_raw` | `sensor_msgs/Image` | 前视 RGB 图像 |
| `/real_sense/depth/points` | `sensor_msgs/PointCloud2` | 深度相机点云 |

正式 Docker 链路由 `auto_docker.sh up` 调用 `auto.sh`：它固定启动 `junior_ctrl`、确认官方已编译控制器的
headless-RL 配置后解除物理暂停，等待状态机实际进入 RL，并以低速 `/cmd_vel` 探针验证 A1
产生有限真实位移且没有倒地，再启动
`/hazardwalker/odom` 中继和 rosbridge。手工启动 `junior_ctrl` 只允许诊断，不能作为控制就绪证据。每轮控制
验收必须独占 ROS master，避免遗留 `/cmd_vel` 发布者污染结果。
容器健康后，`auto_docker.sh up` 还会在 ROS2 主机自动启动并去重官方适配器；`status` 同时显示容器和
适配器状态，`down` 先停止适配器再停止容器。业务栈只能复用该实例，不再自行维护第二份适配器。
修改 `src/unitree_guide/` 后必须先执行 `./auto_docker.sh build force`；若源码比
`devel/lib/unitree_guide/junior_ctrl` 新，`auto_docker.sh up` 会拒绝复用旧二进制并给出重编提示。
完整接口见 [算法接入接口](docs/algorithm-interfaces.md)。

## 结果文件

参赛算法完成探索后应生成：

```text
results/detected_danger.json
```

格式：

```json
{
  "exploration_time": 98.76,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]}
  ]
}
```

评估命令：

```bash
python3 ./src/building_obstacles/scripts/evaluate_danger.py \
  --truth-file ./results/danger_truth.json \
  --detected-file ./results/detected_danger.json \
  --output-file ./results/evaluation_result.json
```

评分细则和匹配规则见 [结果格式与评估方法](docs/evaluation.md)。

## 关键文件

| 文件 | 说明 |
|------|------|
| `generated_building/competition_scene.world` | Gazebo 使用的完整比赛世界 |
| `generated_building/layout_metadata.json` | 楼栋布局、房间、门、电梯和目标点元数据 |
| `generated_building/door_config.yaml` | 动态门控制配置 |
| `generated_building/elevator_config.yaml` | 电梯控制配置 |
| `generated_building/scene_manifest.json` | 本次随机场景 manifest |
| `results/danger_truth.json` | 裁判真值文件 |
| `results/detected_danger.json` | 参赛算法输出文件 |
| `logs/competition_gazebo.log` | Gazebo/launch 日志 |
| `logs/building_control.log` | 门/电梯控制服务日志 |
| `logs/junior_ctrl.log` | 控制器日志 |

## 文档维护说明

此文档用于西南技术物理研究所揭榜挂帅赛题，仅用作比赛用途。
