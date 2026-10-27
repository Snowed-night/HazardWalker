# 复用与开发

建议先跑离线检查，再在固定种子下建立自己的完整任务对照，随后一次替换一个模块。评价修改时同时检查匹配、漏报、虚警、房间覆盖、返航、SLAM和耗时。

换检测器时保留候选、时间戳、坐标、跟踪和主动复查接口。学习型检测器可先替换二维候选生成，再决定是否沿用RGB-D几何与跨帧确认。运动模糊、深度配对和尺寸变化需单独验证。

换定位来源时，重新核对尺度、轴向、时间同步、map/odom/base_link/camera变换和楼层切换。原位姿依赖仿真平台，不能将原成绩作为实机定位精度。

换建筑结构时需要调整逐房组织或接入在线门位与区域发现。换机器人时需要修改footprint、运动约束、局部规划器与控制接口，并逐步完成静态传感、低速导航和完整任务验证。

`reproduction/frozen-files.json`校验当前保留的运行文件并记录整理差异；原冻结版本在服务器归档。有意修改算法后，在自己的分支保留新版本和参数，不要更新旧成绩来代表新代码。

```bash
bash check.sh --source-only
python3 scripts/run_offline_tests.py
python3 -m unittest discover -s tests/reproduction -v
bash build.sh
```

原文件有意变化后，基线哈希检查失败是预期行为。新分支应建立自己的版本记录和验证结论。

## 运行自己的开发变体

根目录入口会核对运行文件清单。需要修改算法时，先建立功能分支，使用底层构建与实验脚本，保留修改前的清单用于对照：

```bash
git switch -c feature/my-variant
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths ros2_ws/src --packages-select \
  hazardwalker_bringup hazardwalker_decision hazardwalker_msgs hazardwalker_nav hazardwalker_perception hazardwalker_platform
python3 scripts/run_offline_tests.py
```

如果改动ROS1控制器，另执行`python3 scripts/reproduction/build_noetic.py`。提交自己修改的文件，确认工作树满足原运行器要求，并协调服务器独占后运行：

```bash
export HAZARDWALKER_SEED=202609102
export DOCKER_SIMENV_USER=station_cluster
export ROS_DOMAIN_ID=43
export USE_GPU=0
bash scripts/run_stable_three_floor_experiment.sh reports/nav/my_variant_seed202609102
python3 scripts/reproduction_check_result.py reports/nav/my_variant_seed202609102
```

为变体记录新的Git提交、参数和本次结果，不将其标记为旧冻结版成绩。
