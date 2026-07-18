# 感知组进度记录：球面门控与官方定位入口修复

日期：2026-07-18
状态：算法改进、PR #33 审查、合法 Cartographer 建图入口和固定 SEED 官方随机场景烟测完成；完整探索未完成，不构成正式成绩。

## 本轮基线与范围

- 两轮烟测基线：`origin/dev@a62fe7e`，已包含 PR #32 恢复的五类历史内部回归记录。
- 当前开发基线已同步到 `origin/dev@555a325`（PR #33）；同步后重新通过全量回归。
- 开发分支：`agent/perception-depth-isotropy`。
- 唯一正式目标仍是半径 `0.15 m` 的红色球体。
- 红色圆柱、圆锥、平板、弧形板和其他非球体只能作为复查候选，不得写入
  `results/detected_danger.json`。

## 问题与改进

旧深度形状判断只比较目标中心和外环的深度差。它能过滤平面端面，却可能把圆柱侧面、
弧形板等“只有一个方向弯曲”的曲面误记为球面；仅比较水平和竖直方向还会漏掉斜放约
45° 的圆柱。

本轮将单视角 RGB-D 几何证据改为四方向一致性：

1. 在候选框内计算中心区和外环深度中位数。
2. 分别沿水平、竖直、正对角线和负对角线采样外环深度。
3. 四个方向都有足够有效深度、曲率均超过阈值，且最小/最大曲率比例不低于
   `min_sphere_axis_curvature_ratio=0.35` 时，才输出 `spherical`。
4. 中心与外环近似等深时输出 `flat`。
5. 总体有凸曲率但四方向明显不一致时输出 `anisotropic`。
6. 任一方向采样不足时输出 `unknown`，只保留候选并请求换视角，不把不确定性当成球面正证据。

`flat` 和 `anisotropic` 均不能增加确认观测数；两个独立非球面视角会使轨迹进入
`rejected_non_spherical`。ROS1、ROS2 检测载荷同时输出四方向曲率、采样点数和曲率各向同性比例，
便于后续实测调参和失败复盘。

## 正式入口断点修复

对官方 ROS1 运行链在线审计后修复了三个会直接污染或清空最终结果的问题：

1. 深度反投影输出的是光学坐标（Z 向前），官方 `real_sense` TF 是机体链路坐标（X 向前）。
   旧节点未转换坐标轴；现已强制使用 `gazebo_link_x_forward` 后再变换到 `world`。
2. 仓库文档把 `/scan` 写为 `PointCloud2`，本机当前官方环境实际发布 `LaserScan`。
   定位节点改为启动时读取 ROS master 的真实类型，同时支持两种公开接口。
3. 累计占据图相关在长墙场景会沿墙滑移。现改为“IMU 锁定旋转 + 相邻帧鲁棒 ICP 估计平移”，
   同分时采用最小运动先验，历史占据图仅在 ICP 证据不足时回退。
4. 多层 z 不再固定为 0。定位节点新增
   `/hazardwalker/navigation/floor_index` 合法动作状态入口，按官方生成器公开固定层高
   `2.6 m` 计算相对高度；换层时清空旧楼层扫描地图，防止相似楼层误配。该接口尚需与
   导航的电梯成功响应/楼梯到层确认联调，不能仅凭离线测试宣称多层定位完成。
5. PR #33 曾把官方业务栈的导航默认值改为开启，并把未显式确认的定位来源默认写为
   `lidar_imu_slam`。本分支恢复 fail-closed：导航必须在独占会话显式开启，定位来源默认
   `unverified`，防止普通联调抢占控制或把缺失/错误 TF 下的候选伪装成可提交世界坐标。

正式感知载荷同时补充 `stamp_sec`、`localization_ready` 和
`localization_provenance`。证据记录器在无候选时也按低频保存复杂楼宇上下文 RGB-D，
上限 80 帧，避免只留下检出画面而无法证明真实运行环境。

## 离线验证

命令：

~~~text
python scripts/run_offline_tests.py
git diff --check
~~~

结果：

- `225 passed, 0 failed`。
- 新增完整球面、0°–165° 每 15° 的旋转圆柱、方向深度缺失、主动侧视建议和跨视角拒绝回归。
- 新增官方扫描类型自适应、静止不漂移、纯旋转 ICP、光学/链路坐标转换和自主环视安全门禁。
- ROS1 Python 3.8 语法兼容、合法 SLAM 来源、最终结果严格导出等既有门禁保持通过。

这些结果只证明纯函数和接口逻辑，没有证明官方随机楼宇中的召回率、虚警率或定位精度。

## 效果素材归档约束

本轮把五类实验的归档规则固化到 `scripts/run_official_simenv_classic_evidence.py`：

- 真实运行使用 `--run-id YYYYMMDD_<seed-or-batch>` 和 `--code-version` 后，只能写入既有五类目录的
  `reruns/<run-id>/`，不再新增含义重复的效果目录。
- 每轮同步生成原始 RGB、标注图、深度可视化、状态快照、`cases.csv`、`summary.json` 和 README；
  对应测试组 CSV/JSON 自动写入 `reports/perception/test_records/` 的同名实验与运行目录。
- 受控物品回归会明确标记 `evidence_class=internal_regression`、
  `official_score_eligible=false`，不得冒充官方随机楼宇成绩。
- 没有真实 RGB-D 输入或无法追溯代码版本时，不生成、补画或归档仿真效果图。

## 官方随机场景真实烟测

初始远端被导航组容器占用且仅剩约 `4.9 GiB` 可用内存。经负责人允许后停止该容器，
内存恢复到约 `88 GiB`，随后新建独占容器，以官方 `auto_headless.sh`、
固定 `SEED=2026071801`、三层楼栋启动两轮同种子烟测。运行期未读取布局、manifest 或真值。

### smoke_01：旧平移相关

- 记录 1890 帧、178 个合法定位轨迹样本和 12 组真实复杂环境 RGB-D。
- 自主环视目标 360°，45 秒只实际完成 `65.518°`，按超时停车。
- 视野内无红色候选或确认目标，最终 JSON 为空。
- 赛后独立定位诊断平面误差约 `0.728 m`。
- 证据目录：`reports/perception/official_random/seed_2026071801/smoke_01/`。

### smoke_02_icp：相邻帧 ICP

- 同一 SEED 重启，记录 800 帧、77 个合法定位轨迹样本和 8 组真实复杂环境 RGB-D。
- 第一段 `65.045°` 环视在 `16.99 s` 内完成；平面误差约 `0.118 m`。
- 第二段因入口墙角约束，55 秒只完成 `26.81°`，按超时停车。
- 两段累计约 `91.855°` 后平面误差约 `0.135 m`；视野内仍无红色候选。
- 与旧算法可比阶段相比，误差从约 `0.728 m` 降至约 `0.118 m`。
- 证据目录：`reports/perception/official_random/seed_2026071801/smoke_02_icp/`。

上述两轮都是官方复杂随机楼宇真实运行，但只覆盖入口环视，不是完整自主探索，
因此 `official_score_eligible=false`，不得据此计算召回率或正式得分。
当前版本独立验收器也已对两轮给出不通过结论：旧记录缺少任务完成契约、未收到
`FINISHED`、无候选且无确认红球。验收报告分别写入各目录的
`independent_post_evaluation.json`；该门槛用于防止把短时冒烟记录误报成完整任务证据。

## 导航 PR #33 覆盖审查

审查对象为已合入 `dev` 的 `555a325`。该 PR 同时包含有用的 frontier/scan/IMU 框架和会污染
官方业务链的默认行为，因此没有整包回滚，而是按风险逐项修复：

- 保留 frontier 探索、水平扫描配置、IMU 接线和导航诊断材料。
- 撤销官方业务栈“无参数即启动导航/SLAM”，两者恢复为显式开启，避免共享环境抢占
  `/hw/cmd_vel`。
- 禁止旧 JSON relay/bridge 转发 Gazebo 真值 `/Odometry_gazebo` 和动态 `/tf`；正式定位只允许
  `/hw/scan`、公开 IMU、公开控制和楼层动作状态。
- 修复旧 compose 工作区挂载不完整、启动脚本未 fail-fast 和 lifecycle SLAM 只停在
  `unconfigured` 的问题。
- 保留 `slam_toolbox` 作为显式诊断回退；官方首选改为 Cartographer 的
  scan+RGB-D scan+IMU+合法里程计融合。

因此不建议回滚整个 merge commit；当前分支已经覆盖其危险默认值，同时保留可复用实现。

## Smoke 13～17：从“无法 SLAM”到正式入口可用

- `smoke_13_cartographer_scan_imu`：发现墙钟时间戳导致巨大假位姿，改为全链仿真时间后，
  scan+IMU 可持续建图，但走廊平移仍退化。
- `smoke_14_cartographer_legal_odom_fusion`：加入不读取真值的控制先验里程计。约 90° 原地旋转中，
  估计约 88.5°、事后真值约 88.0°，原地平移漂移约 5 cm。
- `smoke_15_deadzone_odom_fusion`：确认 0.20 m/s 位于 A1 低速死区，0.35 m/s 有效段事后真值
  位移约 1.0065 m、估计约 1.0078 m；frontier 已实际选点、规划并驱动机器人移动，但尚未完成楼宇覆盖。
- `smoke_16_rgbd_lidar_cartographer`：新增 320 线 RGB-D 中带扫描。Cartographer 同时接收两路扫描、
  IMU 和合法里程计并生成三阶段地图；地图边界仍有射线扇形和稀疏障碍，质量只能记阶段通过。
- `smoke_17_business_launch_cartographer`：修复正式 launch 对未注册 `cartographer` ament 包的错误依赖，
  改从 `cartographer_ros` 前缀定位上游 Lua。在独立 `ROS_DOMAIN_ID=43` 下，五个必需节点存在、
  `/map` 发布者唯一，611×714、0.05 m/像素地图成功保存。

当前“机器狗可控但无法 SLAM”的准确结论已更新为：旧 `slam_toolbox`/旧入口不可用；当前正式
Cartographer 入口能使用官方公开输入生成并保存地图。多层全覆盖、回环精度和红球任务闭环仍未完成。
完整优化路径及各轮限制见 `reports/perception/official_random/seed_2026071801/README.md`。

## 下一轮真实验收

按以下顺序继续：

1. 联调多楼层 z 定位。代码已支持公开动作确认的楼层编号和 `2.6 m` 固定层高，但尚未完成
   电梯/楼梯真实到层测试，不能证明换层后的高度满足 1 m 阈值。
2. 与导航组合法探索输出联调候选触发、侧向复查和完整返航，至少跑多个固定 SEED；
   入口墙角不再用持续原地强转代替探索。
3. 在独立官方 ROS1/Gazebo Classic 容器重跑五类内部回归中的 `red_objects`，重点保留标准红球、
   水平/竖直/斜放圆柱、圆锥和弧形物体的真实 RGB-D；要求 24+ 非球体确认数为 0。
4. 复跑 `active_multiview`，要求 `anisotropic` 候选只产生侧向复查请求，真实红球仍能在三个稳定、
   视线跨度不少于 25° 的视角确认。
5. 只有上述指标不退化后，才在固定 SEED 的官方复杂随机楼宇运行完整探索闭环，并将本轮代码版本、
   RGB-D、候选到确认记录、合法 SLAM 轨迹、最终 JSON 和赛后独立评测归档。

本轮新增的截图只位于上述 `official_random/seed_2026071801/` 真实烟测目录；
没有把入口无候选画面塞入五类专项实验，也没有补画或伪造检出结果。
