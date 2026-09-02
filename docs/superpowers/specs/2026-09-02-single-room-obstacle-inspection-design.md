# 单房间探索：逐障碍巡检（找藏在障碍物后面的红球）

日期：2026-09-02
所属组：导航组
分支：feature/nav

## 背景与问题

官方 SimEnv 默认 3 层 × 每层 4 房，占地约 20 m × 36 m，走廊约 2.2 m；每房由
生成器摆 4-6 件高矮悬殊的家具/障碍（托盘 0.24 m 到柜架 1.7-1.8 m）。红色球体
危险源（半径 0.15 m）在房间内采样，**避开墙/家具/门区，但可被家具遮挡**——
即"红球藏在障碍物后面/侧面"。

现行 `frontier_explorer_node.py` 已有一整套"房间扫描"机制：
- 触发：`_follow_path` 走到一个 `frontier_type='room'`（bbox 纵横比 < 2.5）或
  corridor 型窄边 ≥1 m 且 aspect ≤4 的门口前沿时，`_start_room_sweep` 自动进入
  `enter → explore → exit`（`FEN:3177`、`_should_enter_room`）。
- explore：反复刷新"房内前沿"（自由↔未知边界），A* 走近每个内部前沿、到点
  原地环视，直到入口 4 m 方框内的 `CoverageGrid` 覆盖率 ≥85% 或 60 s 超时。

**缺陷**：explore 目标是"前沿覆盖"，而红球在障碍物背后。障碍背后的区域对
2D 前沿图常不可见/不可达，覆盖率达标也可能从未把相机对到障碍正后方，
红球漏检。

## 需求（已与用户确认）

1. 定位：**增强现有 room_sweep**，不新建并行模块/不重写状态机。
2. 找球策略：**从 OccupancyGrid 提取房内障碍簇 + 逐障碍巡检**，保证把每个障碍
   四周都看一遍，而不是依赖感知先给出球坐标。
3. 布局先验：**通用运行时提取为主 + 固定场景可注入**（YAML 注入房间矩形）。
4. 完成判据：**每个障碍四周都看过**（可配时间兜底）。

## 方案

### 节 1：新增纯函数模块 `room_obstacle_profiler.py`

放 `ros2_ws/src/hazardwalker_nav/hazardwalker_nav/`，仿 `coverage_tracker.py` /
`frontier_detector.py`：ROS 无关、numpy/纯 python、可被离线测试直接 import。

**① `extract_room_mask(grid, entry_wx, entry_wy, entry_yaw, door_width_m, ...)`**
→ `np.ndarray[bool]`（房间内部可达 free 连通域掩码）
- 入口沿进入朝向推进 `seed_offset_m`（默认 0.5）得种子 free 格；
- 在入口 pose 画一条垂直进入朝向、宽 ≈ `door_width_m + 2*wall_margin` 的
  "虚拟门板"窄带，从 free 掩码里扣掉，阻止 flood 经门洞流回走廊；
- 从种子 flood free（四邻域，只在 `0 ≤ grid ≤ FREE_MAX` 上走）得到房间连通域；
- 失败/过小兜底：返回 None，由调用方回退现有"入口 + room_half_extent_m 方框"。

**② `extract_room_obstacles(grid, room_mask, resolution_m, min_area_m2)`**
→ `List[ObstacleCluster]`
- `occupied = grid >= OCCUPIED_THRESHOLD(65)`；
- 只取 room_mask 附近（或直接房内 bbox）的 occupied，做连通域 BFS 得簇；
- 排除墙：room_mask 做一次腐蚀（radius≈1 格）后，簇质心若落在腐蚀掩码
  之外（贴房边界墙的长簇）则丢弃；
- 面积 `cells * res² ≥ min_area_m2` 才保留；
- `ObstacleCluster`：质心(world)、grid_centroid、bbox、半径、points、area_m2。

**③ `plan_obstacle_viewpoints(grid, grid_msg, cluster, room_mask, count, standoff_m)`**
→ `List[ViewPoint]`
- 以质心为圆心 `count` 等分方位角；半径 = `cluster_radius_m + standoff_m`；
- 每候选点取整到 grid，校验：在 `room_mask` 内、free/可通行、不压障碍簇；
- 视角若贴墙/出界则跳过该方向；
- `ViewPoint`：wx、wy、`face_yaw`（朝向障碍质心）。

### 节 2：explore 阶段改造（`frontier_explorer_node.py`）

`_handle_room_sweep` 的 `explore` 分支改为**障碍巡检驱动**：

```
explore tick:
  1. 首次/地图变化：调 profiler 得 room_mask + 障碍簇 + 每个障碍的 viewpoints
  2. 若感知已在本房确认红球 → 提前 exit（_room_begin_exit）
  3. 选"最欠看"的障碍（未看方向最多者/距机器人最近）：
     - 没有 → 全部障碍看全 → _room_begin_exit()
     - 有  → A* 到该障碍一个未看观察点（复用 a_star_path + _room_drive_to）
  4. 到点：机头 face_yaw → 短环视（复用 _room_spin_in_place，转到 _room_spin_accum
     ≥ 目标 rad，默认约 2π/3）→ 记该障碍该方向已看 → 回 2
  5. 房间级预算 obstacle_inspection_timeout_s 到 → 强制 exit
```

看全判据：某障碍已看方向数 ≥ `obstacle_covered_viewpoints`（默认 3，要求分布
多象限）；房间完成 = 房内所有可达障碍看全，或感知确认，或预算到。

保留：enter/exit 逻辑、`_finish_room_sweep`、`room_*` 状态字段、感知 REOBSERVING
抢占（`_transition` 不清 `_room_*`，已核对）、卡死/门禁/net-progress 看门狗。

### 节 3：新参数（全部 declare + launch 可覆盖，向后兼容）

| 参数 | 默认 | 说明 |
|---|---|---|
| `obstacle_inspection_enabled` | True | 总开关；False 退回覆盖率式 explore |
| `obstacle_min_area_m2` | 0.05 | 障碍簇面积下限 |
| `obstacle_viewpoint_count` | 6 | 每障碍环绕观察点数 |
| `obstacle_standoff_m` | 0.50 | 观察点离障碍表面安全距离 |
| `obstacle_covered_viewpoints` | 3 | 看全一个障碍需到访方向数 |
| `obstacle_seed_offset_m` | 0.5 | room_mask 种子推进距离 |
| `room_layout_yaml` | '' | 固定场景注入（矩形列表） |

### 节 4：固定场景注入

`room_layout_yaml`：字符串 YAML，仿 `elevator_positions` 传法。
```yaml
rooms:
  - x_min: 2.0  x_max: 10.0  y_min: 3.0  y_max: 11.0
```
`_start_room_sweep` 命中注入矩形（入口点在矩形内）时用它当 room_mask 范围，
跳过运行时提取；空 = 纯运行时。注入仅作可选调参，不影响默认路径。

### 节 5：测试与验收

新增 `tests/offline/test_room_obstacle_profiler.py`：
- 合成 occupancy（墙 + 房内 4/5 个独立方块障碍 + 门洞），验 extract_room_mask
  不回流走廊、extract_room_obstacles 精确数出 4/5 个且不含墙、plan_obstacle_
  viewpoints 全部落 free 且朝向质心；
- 房级预算强制 exit 的纯逻辑分支。

验收：仿真实测单房 4-5 障碍 + 贴障红球，巡检后每个障碍四周视野覆盖；logs
输出每个障碍 seen 方向数。

## 关键风险与兜底

- 障碍检测出错导致 explore 永不 exit：房间级预算兜底 + exploration_timeout 全局兜底。
- room_mask flood 经门洞回流：虚拟门板法（默认门宽 ±裕量），失败回退固定方框。
- SLAM 地图逐步更新：障碍簇/掩码在地图刷新时可重提取（每次进房重新 profiler）。

## 涉及文件

- 新增 `ros2_ws/src/hazardwalker_nav/hazardwalker_nav/room_obstacle_profiler.py`
- 新增 `tests/offline/test_room_obstacle_profiler.py`
- 修改 `ros2_ws/src/hazardwalker_nav/hazardwalker_nav/frontier_explorer_node.py`
  （参数声明 + explore 分支 + room sweep 触发/结束衔接）
- 可选 `ros2_ws/src/hazardwalker_bringup/launch/official_simenv_business.launch.py`
  （新参数默认值注入）

复用：`frontier_detector.py`（`a_star_path`/`_build_traversable_mask`/
`grid_to_world`/`world_to_grid`/FREE 常量）、`coverage_tracker.py`、
节点内 `_room_drive_to`/`_room_spin_in_place`/`_room_refresh_frontiers`。
