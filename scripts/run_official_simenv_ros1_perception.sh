#!/usr/bin/env bash
# 官方 ROS1 Noetic SimEnv 感知启动器：不启动或修改场景，只接入已运行的公开传感器接口。
# 正式模式会按官方 /joy 协议自动完成“站立 -> 稳定 -> RL /cmd_vel”切换，
# 因而无需人工键盘介入。自主导航节点随后负责持续发布 /cmd_vel 与 FINISHED。
# 本启动器故意不声明合法 SLAM 来源；未由联调调用方显式传入
# _localization_provenance:=lidar_imu_slam 或 visual_inertial_slam 时，感知仅保留候选，
# 不会写出任何官方危险源坐标。
# 禁止读取 generated_building/scene_manifest.json：它含布局和真值路径。world 输出所需
# 的公开出生点由下列显式参数给出；参数默认值与官方 docs/reference.md 一致。若平台以
# ROBOT_X/Y/Z/YAW 修改出生点，调用方必须同步传入四个 _public_start_world_* 参数并记录。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIMENV_ROOT="${SIMENV_ROOT:?请设置官方 SimEnv 根目录，例如 /home/user/SimEnv}"

source /opt/ros/noetic/setup.bash
source "$SIMENV_ROOT/devel/setup.bash"
# 官方容器可能未把 Noetic Python 3 包目录注入 PYTHONPATH；显式补齐保证 rospy/cv_bridge
# 与感知纯函数来自同一 Python 解释器，且不改变场景或控制接口。
ROS_PYTHON_DIST_PACKAGES="/opt/ros/noetic/lib/python3/dist-packages"
export PYTHONPATH="$ROS_PYTHON_DIST_PACKAGES:$REPO_ROOT/ros2_ws/src/hazardwalker_perception:$REPO_ROOT/ros2_ws/src/hazardwalker_decision:${PYTHONPATH:-}"
cd "$SIMENV_ROOT"

exec python3 "$REPO_ROOT/scripts/official_simenv_ros1_perception_node.py" \
  _output_path:="$SIMENV_ROOT/results/detected_danger.json" \
  _localization_frame:=start \
  _world_frame:=world \
  _public_start_world_x:=0.0 \
  _public_start_world_y:=-2.2 \
  _public_start_world_z:=0.6 \
  _public_start_world_yaw:=1.5708 \
  _auto_activate_cmd_vel:=true \
  "$@"
