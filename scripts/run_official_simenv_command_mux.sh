#!/usr/bin/env bash
# 负责人维护：启动官方 SimEnv 宿主侧唯一 ROS2 速度仲裁器。
set -euo pipefail

ROOT="${HAZARDWALKER_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
PLATFORM_SOURCE="$ROOT/ros2_ws/src/hazardwalker_platform"

# 远端登录终端可能先加载过其他 ROS/工作区。启动公共基础节点前清除这些
# 覆盖项，再只加载 ROS2 Jazzy；源码路径保证未执行 colcon build 时也能运行。
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX \
  AMENT_CURRENT_PREFIX CMAKE_PREFIX_PATH ROS_DISTRO ROS_VERSION \
  ROS_PYTHON_VERSION ROS_PACKAGE_PATH
set +u
source "${OFFICIAL_SIMENV_ROS2_SETUP:-/opt/ros/jazzy/setup.bash}"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${OFFICIAL_SIMENV_ROS_DOMAIN_ID:-42}}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export PYTHONPATH="$PLATFORM_SOURCE${PYTHONPATH:+:$PYTHONPATH}"

exec python3 -m hazardwalker_platform.command_mux_node --ros-args \
  -p default_mode:="${OFFICIAL_SIMENV_CONTROL_MODE:-keyboard}" \
  -p use_sim_time:=false
