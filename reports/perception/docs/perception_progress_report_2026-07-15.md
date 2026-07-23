# 感知组进度记录：官方规则复核与部分可见红球实测

日期：2026-07-15
状态：进行中；不得据此宣称完整闭环或比赛 SOTA。

## 官方规则复核

以官方 SimEnv 仓库 `docs/algorithm-interfaces.md`、`docs/competition-rules.md` 和
`docs/evaluation.md` 为准：

- 唯一危险源是半径 0.15 m 的红色球体；红色方块、绿色球体均是干扰物，不能作为提交目标。
- 正式算法可使用 RGB-D、相机内参、深度点云、雷达、IMU 和 `/cmd_vel`；
  `/Odometry_gazebo`、`/ground_truth/*` 只用于裁判/调试，禁止作为比赛算法输入。
- 最终文件为 `results/detected_danger.json`，位置必须在 `world` 坐标系；评估会把
  红色干扰物误报计入虚警。

因此正式感知链路改为：未验证定位只输出候选/复查请求，只有导航组提供的合法 SLAM
定位（例如 `lidar_imu_slam` 或 `visual_inertial_slam`）才能导出最终 `world` 坐标。
不能再把 Gazebo 真值里程计或静态别名包装成比赛定位成功。

## 本轮算法修复

- 部分可见候选不再因同帧已存在完整球而被整体抑制；仅在与严格框实质重叠时去重。
  这样“一个完整球 + 一个被遮挡球”能同时进入候选与重观察流程，不会产生同一球的重复框。
- 保持“单帧近圆红色物体仅为候选”的原则。深度平面、非圆侧面、多视角不一致或无合法
  侧向视差的轨迹不能确认成红球。
- ROS1 官方 RGB-D 入口与 ROS2 入口统一：最终导出同时要求显式合法 SLAM 来源、至少两个
  独立视角的 RGB-D 球面正证据，以及既有的三视角/25° 侧向视差门槛。默认
  `localization_provenance=unverified`，因此调试 TF 或 Gazebo 里程计不能意外写入结果文件。
- 部分可见弧段即使局部深度看似球面，也不能补足球体正证据；它只会维持
  `requires_reobservation`，避免“少于 45° 展露”或遮挡噪声借由多帧累计误确认。
- 利用题目给出的唯一物理先验：除多视角尺寸稳定外，RGB-D 反投影的中位表观直径还须接近
  标准直径 `0.30 m`（35% 裕量）。尺寸明显不符的红色圆物会进入复查，不会写入官方结果。
- 动态证据记录器改为默认 `internal_regression`、显式 `official_random_scene` 才可通过的
  fail-closed 契约：正式归档必须填写固定 SEED、代码版本、合法 SLAM 位姿话题和来源。每次候选/
  确认记录可保存时间配对 RGB、米制深度、候选到确认 JSONL 与合法轨迹采样；`/hw/odom`、
  `/Odometry_gazebo`、`/ground_truth/*` 会使正式证据资格失败。
- 若调用参数误把 `/hw/odom`、`/Odometry_gazebo` 或 `/ground_truth/*` 作为 `legal_pose_topic`，
  记录器会直接拒绝订阅，避免禁用位姿被写入任何感知证据文件。
- 新增独立的正式随机场景结构校验器。它只读取运行后归档的 manifest、RGB-D、轨迹、候选到确认
  记录与 `detected_danger.json`，检查 600 秒上限和材料完整性；它明确不读取真值，也不替代主办方
  后评的召回率、虚警率和定位误差。最终输出 JSON 必须复制进同一场证据目录，避免只保留临时路径。
- 新增专项**内部回归**执行器的隔离容器保护、每案例独立检测器、清理失败即终止套件和运行后才
  使用真值评估的约束，避免残留模型、轨迹串扰或真值泄露污染结果。该执行器可以验证算法退化，
  但不属于官方随机场景正式成绩。

## 已完成内部回归：部分可见

在隔离的官方 ROS1/Gazebo Classic 场景，通过 `/hw/camera/image_raw` 和
`/hw/perception/hazard_detections` 采集了 21 个案例：完整基线，以及左右两侧各
10 个可见比例（5%--85%）。SDF 设计值只用于生成遮挡物；实际像素比例由保存后的 RGB
快照独立测量。由于该套件人为放置了测试物和遮挡物，它仅是内部回归，不得用于官方随机场景得分宣称。

| 指标 | 结果 |
|---|---:|
| 案例数 | 21 |
| 通过/失败 | 21/0 |
| 最低实测可见比例 | 左 4.17%，右 4.81% |
| 5%--35% 行为 | 仅输出 `requires_reobservation` 候选，不确认 |
| 45% 以上行为 | 在严格轮廓门通过时可输出严格候选；仍须合法多视角确认 |

原始 RGB、标注图、单帧 JSON、`cases.csv/json` 与 `summary.json` 已保留在本地工作区
等待与其余四类实测一起按最终五目录规范归档；未将旧的简化 7 月 10 日结果混入该统计。
本次恢复到 Git、后按统一类别重命名的 `official_simenv_20260710_*` 目录仍是旧历史基线，不是本节所述
ROS1/Gazebo Classic 新一轮统计；两者不得合并计算通过率。

## 尚未完成与当前阻塞

内部回归仍按五类目录保留：多球粘连、部分可见、20+ 红色物品鲁棒性、20+ 真实多视角、复杂
环境三维定位。它们用于发现回归与展示算法边界；**正式成绩**必须另行在官方随机 SEED 场景完成，
不允许读取场景布局、危险源真值或 Gazebo 真值里程计。

远端当前可通过只读挂载启动独立 ROS1/Gazebo Classic、RGB-D 与 rosbridge，不会写入共享
`simenv_run`。但独立实例曾被宿主环境提前清理，且官方控制器在命令间会重置姿态；因此红色
物品、多视角、粘连和三维定位尚未完成可复核的整批重跑，不能以离线合成图或近墙单帧代替。
红色物品中的圆柱、圆锥等只作为扩展压力干扰物，正式危险源仍只有红球。

### 2026-07-15 复跑附记

已将执行器改为在测试夹具生成阶段读取 `map -> real_sense` TF，在相机前方临时放置模型；该
TF 只决定 SDF 生成位置，绝不输入检测、控制或定位算法。单红球预验收已产生 1 个严格候选，
但两次画面的去红球背景边缘比例仅为 `0.0041` 与 `0.0050`，低于正式复杂环境门槛 `0.0060`，
故均明确标记为失败并只保留在临时工作区。该门槛会排除近墙/空白画面，最终五个目录不会包含
这些预验收素材。控制链路已能在隔离实例改变位姿，但尚未形成可复核的连续横向多视角，不能
宣称主动复查成功。

## 本地验证

`python scripts/run_offline_tests.py`：166 passed，0 failed。

### 2026-07-15 Linux 暂存自检

为避免覆盖远端感知账号旧工作树，在 `/home/hazard_perception/perception_stage_20260715` 创建了独立
暂存目录，仅同步当前感知代码进行 Python/纯函数检查；未启动、停止或写入共享 `simenv_run`，也未修改
官方场景。该目录的 ROS1 兼容解析和感知纯函数回归为 **41 passed**。这只证明 Linux 解释器兼容性，
不构成官方随机场景运行或比赛成绩。

### 2026-07-15 官方运行期输入只读核查

对正在运行的共享 `simenv_run` 仅执行了一次 `rostopic list`（未发布消息、未读场景文件、未修改
容器）。确认 RGB、深度、点云、雷达、IMU、`/tf` 和 `/hazardwalker/odom` 均存在；但当前话题表中
**没有团队自建的 SLAM 位姿/`world -> camera` TF 发布入口**，同时存在规则禁止的
`/Odometry_gazebo` 与 `/ground_truth/*`。因此感知节点只能在该实例上输出二维候选/复查建议，不能
合法导出 `detected_danger.json` 的 world 坐标，也不能将本轮称为正式随机场景闭环。最小复现命令为：

~~~text
docker exec simenv_run bash -lc 'source /opt/ros/noetic/setup.bash && rostopic list'
~~~

等待导航定位组发布明确声明为 `lidar_imu_slam` 或 `visual_inertial_slam` 的位姿和 `world -> camera`
TF 后，才可按本文档的正式证据契约启动无人工干预的感知记录。

### 2026-07-15 激光—IMU 定位与 RGB-D 被动联调记录

为避免控制、场景和结果文件污染，只在共享容器中使用独立 `/tmp` 文件、`trial_start` 坐标系和
`/hazardwalker/slam/odometry_trial` 试验话题短时运行；未发布 `/cmd_vel`，结束后已杀死两个试验节点
并删除临时文件。

1. 首次运行暴露官方 Noetic 容器未向 Python 3 导出 `rospy` 路径，已在两个 ROS1 感知启动器中补齐
   `/opt/ros/noetic/lib/python3/dist-packages`；修复后节点能够实际订阅公开扫描/IMU 并发布试验 Odometry。
2. 首次 TF 方案尝试发布 `trial_start -> base`，与官方已有 `odom -> base` 同 child frame 冲突，已撤回；
   代码改为独立 `start -> slam_base -> real_sense`，不复用官方 `base`。
3. 改造后的第二轮短时联调中，RGB-D 输出仍为无候选且 `camera_stable=false`，`trial_start` 未在 TF 查询中建立。
   容器同时持续报告 `odom -> base` 重复 TF 数据；因此本轮只能证明“节点进程可启动”，**未证明**合法
   定位 TF 与 RGB-D 稳定联通，更不能证明定位精度、红球检测或正式闭环。

后续正式验证必须在进程唯一、可记录固定 SEED 的独占官方启动会话中进行，并以自身扫描—IMU输出与
赛后真值评测计算漂移；不得使用共享容器中的 `/Odometry_gazebo`、`map -> odom` 或 `odom -> base`。

### 旧五类素材保留与重跑约定

对旧 `official_simenv_20260710_*` 五目录运行原有归档校验器时，`active_multiview` 因
`strict_view_semantics_audited != true` 失败：未证明水平基线/朝向造成的真实多视角，也未达到
25° 侧向视差要求。为保留算法优化路径，这五套旧素材和对应测试表恢复到 `reports/perception/`，
但统一标记为 `historical_internal_regression`、`official_score_eligible=false`，不得作为内部回归
成功率或官方成绩。后续重做版本放在同一目录的 `reruns/YYYYMMDD_<seed>/`，必须明确写入
`internal_regression` 或 `official_random_scene`，并通过独立视角语义、真值隔离和证据契约审计。

### 2026-07-15 平台接口复验与合规启动冒烟

在平台组更新后的 `simenv_run` 中，仅执行了只读话题检查与一次**不发布 `/cmd_vel`、不移动
机器人**的短时感知节点启动。结论如下：

- 当前仅有一个 `simenv_run` 容器；未见残留感知或导航业务节点。RGB、对齐深度约 20 Hz，激光
  10 Hz，IMU 1000 Hz；`/cmd_vel` 只有 rosbridge 发布、`/unitree_gazebo_servo` 订阅。
- 新版 ROS1 RGB-D 感知节点能实际启动并发布 `/hazardwalker/perception/hazard_detections`；启动时
  显式关闭自动控制，检查期间 `/cmd_vel` 发布者未增加。随后节点被停止，容器 `/tmp` 下的临时目录
  和空 `detected_danger.json` 已删除，未归档任何图像或结果。
- 官方 `/tf` 仍持续报告 `TF_REPEATED_DATA`：`/unitree_gazebo_servo` 与
  `/state_from_gazebo` 都在广播基础坐标边。感知正式链路不使用该 `odom/map` 链，而使用独立的
  `start -> slam_base -> real_sense` 激光—IMU坐标链；在平台侧降低重复广播前，这一告警仍需持续
  监控其对实时性的影响。
- 官方文档说明 `scene_manifest.json` 同时包含布局元数据、危险源数量和裁判真值路径。审计发现旧
  感知启动器曾期待不存在的 `team_scene_info.json`，既无法在当前官方容器启动，也存在误把场景
  资料引入算法输入的风险。现已改为**不读取任何生成场景文件**：world 对齐只采用启动方显式记录的
  公开出生点参数（默认值与官方 `ROBOT_X/Y/Z/YAW` 默认参数一致）。若平台改变出生点，正式运行
  必须把四个出生点参数写入启动命令和证据 manifest。

上述联调只证明公开传感器接口与感知进程可接入，**不是**红球检测、多视角确认、三维定位、探索返航
或官方随机场景成绩。正式记录仍须由固定 `SEED` 的独占 `auto.sh` 会话发起，并同时启动自建合法
激光—IMU定位、导航自主探索和动态证据记录器。

### 2026-07-15 合法激光—IMU到相机坐标链被动联调

在同一官方容器中再次以 `/tmp` 临时副本启动定位节点，显式关闭感知自动控制；定位节点只订阅
`/scan`、`/trunk_imu`，并显式把输出写到临时话题。实测 `/hazardwalker/slam/odometry_tf_smoke`
约为 10 Hz；从原始 `/tf` 流直接筛选到 `child_frame_id: "slam_base"` 和
`child_frame_id: "real_sense"`，后者外参为 `(0.28, 0, 0.043)`。这证明独立
`start -> slam_base -> real_sense` 链在当前官方传感器流中能够建立，且没有复用
`/Odometry_gazebo`、`/hazardwalker/odom` 或官方 `base` child frame。

随后 RGB-D 感知节点可与该定位节点并存启动，检测话题正常发布，`/cmd_vel` 仍只有平台 bridge
发布者。两节点均已停止，临时里程计、日志和空结果已删除。该轮机器人未移动、未检验定位误差，
也未完成红球候选/多视角确认，故仅为**合法定位接入通过**，不是正式定位得分或完整比赛证据。

### 2026-07-15 ROS1 正式证据记录器被动联调

新增 `scripts/official_simenv_ros1_evidence_recorder.py`，补齐官方 ROS1 感知链与既有 ROS2
`/hw/*` 记录器之间的证据归档缺口。它只订阅公开 RGB-D、`/hazardwalker/perception/hazard_detections`
和调用方声明的自建 SLAM 位姿；终止时写入与独立校验器相同的 manifest、frames、trajectory、
summary、failure reasons、测试表，并在结果节点先结束后复制最终 `detected_danger.json` 到证据目录。

在当前未固定 SEED 的共享场景进行了一次不移动机器人的三节点被动联调（定位、感知、记录器）。
记录器实测落盘 35 条帧记录、8 条合法 SLAM 轨迹和结果副本，且 `/cmd_vel` 发布者仍只有平台 bridge。
镜头内没有红球，摘要如实为 0 候选、0 确认；证据契约自动标记
`internal_regression`、`missing_fixed_scenario_seed`，因此不能被正式校验器误判为成绩。所有容器
`/tmp` 临时目录、日志和空结果均已删除；这一轮只证明**正式证据机制可运行**，不保留为五类专项
素材或官方随机场景结果。

本次代码修改后的离线回归：`167 passed, 0 failed`。

### 2026-07-15 固定 SEED 正式感知编排入口

新增 `scripts/run_official_simenv_perception_evidence.sh`，把已实测的合法定位、RGB-D 感知和
ROS1 证据记录器封装为一条感知侧运行入口。脚本在启动前强制要求平台写入
`OFFICIAL_SIMENV_EXCLUSIVE_SESSION=1`、固定场景 `SEED`、代码版本、证据目录和测试表目录；
若发现同名残留节点或时限不在 1--600 秒内则直接退出。它不启动导航、不发布 `/cmd_vel`、不切换
控制器，仅等待导航在任务状态话题发布 `FINISHED`，随后严格按“感知先产出结果 → 记录器复制结果
并落盘 → 定位退出”的顺序收尾。未收到 `FINISHED` 时会保留失败原因，不能作为任务完成。

该入口只是可复现运行机制，尚未在固定 SEED 的独占楼宇中执行；不产生比赛成绩或替代导航探索。
脚本 shell 语法检查和本次全量离线回归为：`168 passed, 0 failed`。
