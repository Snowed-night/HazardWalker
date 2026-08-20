# SLAM 跳变/漂移/地图质量实时监测节点 — 设计文档

日期：2026-08-20
分支：feature/nav
状态：已批准（方案 B）

## 背景与目标

Cartographer 在对称/重复走廊存在 scan matching 多解，会导致 map→base 位姿瞬移（跳变，历史实测 7.5~31m），并在地图上表现为墙壁多重漂移/重影。此前通过 git 提交调了多轮 lua 参数（相关性匹配、回环约束 2.0、子图 90），但发现实际运行的 Cartographer 读的是 install 目录里的旧 lua（8/3 版本），改动从未生效。

在重新部署 lua 之后，需要一个**便捷、快速**的实时监测工具，让用户在建图过程中立刻看到：

1. **跳变**是否发生、幅度多大、累计多少次。
2. **漂移**（map→odom 累积偏移）的当前值与变化趋势。
3. **地图质量**（occupied/free/unknown 比例）的演化，辅助判断墙重影/擦墙。

工具定位是**独立观测器**，只读 TF 与 `/map`，不拦截、不改写导航控制输出，与 frontier_explorer_node 完全解耦。

## 组件划分

新增 2 个文件 + 1 个测试 + 1 行注册：

| 文件 | 职责 | 依赖 |
|---|---|---|
| `hazardwalker_nav/slam_metrics.py`（新） | 纯函数库：跳变判定、漂移量、地图指标统计 | 仅 `numpy`，无 ROS |
| `hazardwalker_nav/slam_monitor_node.py`（新） | ROS2 节点：采集 TF + `/map`，调纯函数，打印 + 落盘 | rclpy、tf2、slam_metrics |
| `tests/offline/test_slam_metrics.py`（新） | 纯函数离线测试 | run_offline_tests.py |
| `setup.py`（改 1 行） | 注册 `slam_monitor` entry_point | — |

`detect_pose_jump` 复用历史提交 982434c 中被 revert 的 `is_pose_jump` 判定逻辑，恢复为纯函数，不重新发明。

## 检测指标定义

### ① 跳变（重点）

- 每 tick 从 `map→base` TF 取位姿 `(x, y)`。
- 相邻两次取样的位移 `d = √(Δx² + Δy²)`，时间间隔用 `time.monotonic()`（不用 ROS 时间，避免 sim_time 时钟跳）。
- 判定：`d > max_speed_m_s × elapsed_s + min_distance_m`。
- 默认阈值沿用历史值 `max_speed_m_s = 1.0`、`min_distance_m = 0.5`（机器人实际 0.35~0.45 m/s，10Hz 下正常 0.1s 位移约 0.035m，远低于 0.6m 门槛；历史跳变 7.5~31m 可稳定捕获）。
- 命中即打印醒目告警 + 落盘一条 jump 事件（时间戳、位移、位置、累计次数）。
- 仅检测平移瞬移，不检测旋转。

### ② 漂移

- 每 tick 从 `map→odom` TF 取平移量 `√(x² + y²)`。
- 实时报告当前漂移值 + 落盘历史曲线。
- 超阈值 `drift_warn_m = 2.0` 告警。
- 语义：这是 SLAM 对里程计的累积校正量，回环闭合后会自然重置，因此曲线呈「缓涨 → 回环后归零」锯齿是正常的；重点观察是否「越涨越高不回落」。

### ③ 地图质量

- 订阅 `/map`（OccupancyGrid），降采样（约 1Hz）统计：
  - `occupied_ratio` = 占用 / (占用+自由)，即已知区域里墙的占比。
  - `free_ratio`、`unknown_ratio`（未知占比 = 探索剩余量）。
  - 地图尺寸 `width × height`。
- 落盘历史曲线；墙重影会让 `occupied_ratio` 异常偏高，漂移擦墙会异常偏低，对应历史实测「1.22% ↔ 0.11%」两个极端。

## 输出与使用

### 落盘

仿 `nav_recorder.py` 的 JSONL 风格（每行一条 + 立即 flush）：

```
reports/slam_monitor/run_<时间戳>/
├── jumps.jsonl          # 跳变事件：time, displacement_m, x, y, cumulative
├── drift.jsonl          # 漂移历史：time, drift_m, x, y
├── map_metrics.jsonl    # 地图指标：time, occupied_ratio, free_ratio, unknown_ratio, w, h
└── run_meta.json        # 参数快照 + 汇总（总跳变次数、最大跳变、结束状态）
```

### 终端实时

- 跳变告警用 `[JUMP]` 前缀 + 累计计数。
- 漂移/地图指标周期打印一行摘要。

### 启动方式

```bash
ros2 run hazardwalker_nav slam_monitor          # 或 python -m hazardwalker_nav.slam_monitor_node
```

参数可 `ros2 param set` 热调：`max_speed_m_s`、`min_distance_m`、`drift_warn_m`、`monitor_rate_hz`、`output_dir`。

## 测试

`test_slam_metrics.py` 覆盖纯函数：

- `detect_pose_jump` 边界：正常 0.035m 放行、7.5m/31m 命中、NaN/负 elapsed/非法输入防御。
- `drift_magnitude` 计算。
- `occupied_ratio` 统计正确性 + `total==0` 除零防御。

## 验证方式

1. 离线测试全绿：`python scripts/run_offline_tests.py` 新增 `test_slam_metrics.py` 通过，现有测试不回归。
2. 部署后（重新 colcon build）在 hxbl 上 `ros2 run hazardwalker_nav slam_monitor`，能正常订阅 TF 与 `/map` 并打印。
3. 手动键盘控制走一段（含长廊停留），观察终端跳变/漂移告警与落盘 JSONL。
