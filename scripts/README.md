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
- `run_official_simenv_ros1_adapter.sh`：将官方容器内 ROS1 原始话题中继为 `/hw/*`，并通过
  `ros1_bridge dynamic_bridge` 送入 ROS2；速度中继默认关闭。
- `verify_official_simenv_ros1_adapter.sh`：检查 ROS1 原话题、ROS2 `/hw/*` 与控制器订阅；仅显式
  `--control` 才发送低速速度命令，仍需以视频和里程计证明真实运动。
- `verify_official_simenv_ros1_direct_control.sh`：绕过适配层直接验收官方 ROS1 `/cmd_vel`，要求平台组
  设置 `OFFICIAL_SIMENV_EXCLUSIVE_SESSION=1` 才会运行，并自动保存直行≥1m、转向、停止的里程计与测试表。
- `run_official_simenv_ros1_ros2_stack.sh`：在官方容器已启动后启动 ROS2 业务层，不启动 fake 平台或
  Gazebo Harmonic。

官方 SimEnv 的实际 RGB 源可能是 `/camera/image_raw` 而非默认 RealSense 路径。运行前用 `rostopic list`
确认，并通过 `OFFICIAL_SIMENV_RGB_TOPIC`、`OFFICIAL_SIMENV_RGB_CAMERA_INFO_TOPIC` 覆盖；完整验收顺序见
[`docs/environment/官方SimEnv_ROS1_ROS2双向适配整改_20260714.md`](../docs/environment/官方SimEnv_ROS1_ROS2双向适配整改_20260714.md)。

## 约定

- 新增脚本先放这里，再根据用途拆分子目录。
- 脚本名称应直接表达用途，不保留已经删除或不存在的入口名。
- 如果新增仿真启动、录包或批处理脚本，README 需要同步更新。
