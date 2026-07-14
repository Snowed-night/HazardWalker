#!/usr/bin/env bash
# 官方 ROS1 Noetic SimEnv 感知启动器：不启动或修改场景，只接入已运行的公开传感器接口。
# 正式模式会按官方 /joy 协议自动完成“站立 -> 稳定 -> RL /cmd_vel”切换，
# 因而无需人工键盘介入。自主导航节点随后负责持续发布 /cmd_vel 与 FINISHED。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIMENV_ROOT="${SIMENV_ROOT:?请设置官方 SimEnv 根目录，例如 /home/user/SimEnv}"

source /opt/ros/noetic/setup.bash
source "$SIMENV_ROOT/devel/setup.bash"
export PYTHONPATH="$REPO_ROOT/ros2_ws/src/hazardwalker_perception:$REPO_ROOT/ros2_ws/src/hazardwalker_decision:${PYTHONPATH:-}"
cd "$SIMENV_ROOT"

exec python3 "$REPO_ROOT/scripts/official_simenv_ros1_perception_node.py" \
  _output_path:="$SIMENV_ROOT/results/detected_danger.json" \
  _team_scene_info_path:="$SIMENV_ROOT/generated_building/team_scene_info.json" \
  _localization_frame:=start \
  _world_frame:=world \
  _auto_activate_cmd_vel:=true
