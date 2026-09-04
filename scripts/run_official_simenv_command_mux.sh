#!/usr/bin/env bash
# 在 ROS2 主机运行唯一速度命令仲裁器；生命周期由 manage 脚本负责。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${OFFICIAL_SIMENV_ROS_DOMAIN_ID:-42}}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX \
  AMENT_CURRENT_PREFIX CMAKE_PREFIX_PATH PYTHONPATH \
  ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH
set +u
source "${OFFICIAL_SIMENV_ROS2_SETUP:-/opt/ros/jazzy/setup.bash}"
source "$ROOT/ros2_ws/install/setup.bash"
set -u

exec python3 -m hazardwalker_platform.command_mux_node --ros-args \
  -p default_mode:=keyboard \
  -p use_sim_time:=false
