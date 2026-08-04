# Scripts

本目录存放项目运行、构建和结果检查脚本。

## 当前文件

- `build.sh`：进入 `ros2_ws` 后执行 `colcon build --symlink-install`，用于构建 ROS 2 工作空间。
- `setup_env.sh`：主力机环境检查脚本，只检查系统、NVIDIA、基础工具、ROS 2 和 Gazebo，不自动安装大型依赖。
- `run_minimal_demo.sh`：启动最小 demo，自动设置 `HAZARDWALKER_ROOT`，加载 ROS 环境和工作空间后运行 `minimal_demo.launch.py`。
- `run_offline_tests.py`：不依赖 pytest 的离线测试入口，扫描 `tests/offline/` 并执行测试函数。
- `evaluate_result.py`：检查 `reports/run_results/<timestamp>_result.json` 的结构和统计字段。
- `generate_perception_cases.py`：生成红球检测可视化案例、标注图、summary 表、precision/recall/AP50 指标和汇报拼图。
- `evaluate_real_red_ball_images.py`：读取本地实物红球图片，统一编号并生成多目标检测标注图和参数图。
- `run_official_simenv_rosbridge_adapter.sh`：唯一官方 ROS1↔ROS2 适配入口；由 ROS2 主机通过
  容器 `rosbridge_websocket` 双向传输 `/hw/*` 与 `/cmd_vel`，官方 Docker 不需要也不具备 `dynamic_bridge`。
- `manage_official_simenv_rosbridge_adapter.sh`：适配器宿主侧生命周期管理器；由
  `auto_docker.sh up/down/status` 调用，按容器名和 ROS 域维护 PID、日志、去重及安全停止，普通成员不应手工运行。
- `tests/runtime/verify_official_simenv_adapter_lifecycle.sh`：不连接共享容器的 Linux 进程级自检，验证适配器管理器
  重复启动不增殖、DDS 图中同名节点唯一、状态可见和停止无残留；正式验收仍需在独占官方容器中执行。
- `verify_official_simenv_ros1_adapter.sh`：检查 ROS1 原话题、ROS2 `/hw/*` 与控制器订阅；仅显式
  `--control` 才发送低速速度命令，仍需以视频和里程计证明真实运动。
- `verify_official_simenv_ros1_direct_control.sh`：绕过适配层直接验收官方 ROS1 `/cmd_vel`，要求平台组
  设置 `OFFICIAL_SIMENV_EXCLUSIVE_SESSION=1` 才会运行，并自动保存直行≥1m、转向、停止的里程计与测试表。
- `run_official_simenv_ros1_ros2_stack.sh`：复用 `auto_docker.sh up` 已启动的唯一适配器，再通过
  `official_simenv_control_interface.launch.py` 启动统一控制与 ROS2 业务层；正式导航自动选择
  `navigation` 输入，仍由唯一仲裁器发布 `/hw/cmd_vel`，不启动 fake 平台、第二份适配器或 Gazebo Harmonic。
- `official_simenv_cmd_vel_relay_node.py`：正式完整适配器已失活时的**临时控制备用中继**；只将
  `/hw/cmd_vel` 发送到官方 ROS1 `/cmd_vel`，带 WebSocket 重连与超时零速度。仅限独占时段使用，
  正式适配器恢复后必须停止，禁止长期并行桥接。
- `ros2_ws/src/hazardwalker_platform/docker/gui_client.sh`：官方 Gazebo Classic 的 noVNC GUI sidecar；
  连接已运行的 Master，不重启正式容器。通过 Xvfb 软件渲染避开 RDP/XWayland 的 Qt/OpenGL 崩溃。
- `official_simenv_ros1_evidence_recorder.py`：官方 ROS1 感知证据记录入口。仅订阅 RealSense
  RGB-D、感知候选 JSON 和调用方声明的自建 SLAM 位姿；停止时输出 `run_manifest.json`、逐帧
  RGB-D/轨迹、`summary.json`、测试表，并将最终 `detected_danger.json` 复制到同一证据目录。
  正式模式必须显式提供 `run_mode=official_random_scene`、固定 SEED、代码版本、
  `legal_pose_topic=/hazardwalker/slam/odometry` 和
  `localization_provenance=lidar_imu_slam+public_floor_action`；
  `/Odometry_gazebo`、`/ground_truth/*`、`/hw/odom` 会被拒绝。
- `validate_official_random_perception_evidence.py`：运行结束后的独立结构校验器。默认检查正式
  证据合同、原始/标注 RGB-D、合法 SLAM 轨迹、多视角球面证据、world 坐标结果与去重；B 阶段
  加 `--require-active-reobservation` 后，还要求同一局部候选经过可测位姿变化、完整观测和确认，
  防止把“仅发布动作建议”误记为主动复查成功。
- `summarize_official_random_perception_campaign.py`：B5 多随机场景活动门禁。运行前必须在
  `campaign_manifest.json` 中预注册至少 3 个 SEED、同一干净 Git 提交和参数快照 SHA-256；
  运行后使用当前独立校验器重新检查主动复查证据，再检查非空正式结果、测试表和赛后
  `evaluation_result.json`，再输出活动 JSON/CSV 及召回率、虚警率、耗时的最差值。漏跑 SEED、
  使用未提交代码、参数快照变化、过期校验报告或只挑成功结果都会使活动失败。
- `official_perception_bag.py`：录制固定 SEED 人工巡检 rosbag；正式录制绑定五分钟内通过且来自同一干净 Git 提交的实时预检报告，开包前清零合法 SLAM 覆盖计数，并校验 RGB-D、雷达、IMU、定位来源、地图、控制、检测、至少 60 秒时间跨度、8 m 路程和 3 m 平面跨度；也可在独立 ROS 域安全回放公开传感器输入。
- `verify_perception_live_chain.py`：正式人工巡检录包前的只读门禁；短时检查时钟、RGB-D、内参、激光、TF、地图、合法 SLAM 及来源发布者、感知与控制状态、第一人称实时画面和叠加状态，并要求平台输入、地图及 `/hw/cmd_vel` 均满足唯一发布者合同，同时绑定干净 Git 提交。脚本不发送速度命令。
- `build_perception_bag_regression_catalog.py`：重新核验并汇总不同 SEED 的唯一有效人工巡检，正式索引默认至少覆盖 3 个不同 SEED，输出含时长和关键话题计数的动态回归集 JSON/CSV 并拒绝重复 SEED。
- `run_perception_replay_experiment.py`：在独立 ROS 域加载真实感知参数、回放一份有效 rosbag、安全收尾证据记录器，并可接续人工标注评估；正式输出只允许写入 `reports/perception/` 子目录，并拒绝未提交代码。脚本固化实际参数和标注快照并验证哈希。缺代表图、配对深度、合法轨迹、地图或测试表时失败，无标注只记为 `captured_unlabeled`。
- `compare_perception_replay_runs.py`：横向汇总 rosbag 内容指纹、SEED、人工标注和 Git 来源完全受控的多轮回放；校验实际启动参数哈希后输出 JSON/CSV，拒绝脏代码、路径相同但内容已变、跨数据集或重复参数组合。
- `compare_perception_replay_campaign.py`：将至少 3 个 SEED 下完全相同的算法/参数/Git 版本组成完整回放矩阵，输出跨场景微平均、最差场景、定位、时延及频率指标；漏跑任一算法或 SEED、版本漂移和事后挑选结果都会失败。
- `run_official_simenv_perception_evidence.sh`：感知侧正式随机场景编排器。要求显式独占标志、
  固定 SEED、代码版本、证据目录和测试表目录；启动自建激光—IMU定位、RGB-D感知及 ROS1
  记录器，默认**不**切换控制器或发布 `/cmd_vel`。它只等待导航在任务状态话题发布 `FINISHED`，
  最多 600 秒，并接收导航依据公开电梯/楼梯动作确认后的
  `/hazardwalker/navigation/floor_index`，按“感知结果→记录器→定位”顺序停止和归档。
- `official_simenv_classic_evidence_cases.py`：生成**官方 ROS1 + Gazebo Classic**可加载的五类受控
  SDF 清单：10 个非规则多球、21 个部分可见、24 种红色物品、20 个真实多视角对象、8 个复杂定位布局。
  这是案例定义，不连接 ROS、不驱动机器人，也不读取 `danger_truth.json`；执行器必须在隔离容器中逐例
  生成/删除模型、从 `/hw/*` 采集真实结果后才可计算真值指标。
- `run_official_simenv_classic_evidence.py`：上述五类案例的实测执行器。它硬拒绝共享
  `simenv_run`，每案例重启检测器、保存真实 RGB/标注/JSON，并在模型清理失败时中止套件，
  防止残留物污染后续案例。有效复跑应显式传入 `--run-id YYYYMMDD_<seed或批次>`、
  `--code-version` 和 `--test-record-root`；执行器会自动把图片及结构化结果放入五类既有目录的
  `reruns/<run-id>/`，并把同批 CSV/JSON 写入对应测试记录目录。正式比赛模式不得把
  `/Odometry_gazebo` 或 `ground_truth` 作为运行期输入；真值只在采集完成后用于离线评分。

官方 SimEnv 的实际 RGB 源可能是 `/camera/image_raw` 而非默认 RealSense 路径。运行前用 `rostopic list`
确认，并通过 `OFFICIAL_SIMENV_RGB_TOPIC`、`OFFICIAL_SIMENV_RGB_CAMERA_INFO_TOPIC` 覆盖；完整验收顺序见
[`docs/guidebook/官方SimEnv平台环境使用手册.md`](../docs/guidebook/官方SimEnv平台环境使用手册.md)。

生成案例清单（只做离线检查，不产生实验结论）：

```powershell
python scripts/official_simenv_classic_evidence_cases.py red_objects
python scripts/official_simenv_classic_evidence_cases.py active_multiview --center 1.5 -0.3 0.15
```

## 约定

- 新增脚本先放这里，再根据用途拆分子目录。
- 脚本名称应直接表达用途，不保留已经删除或不存在的入口名。
- 如果新增仿真启动、录包或批处理脚本，README 需要同步更新。
