#!/usr/bin/env bash
# 官方 ROS1 SimEnv 的合法激光—IMU 定位启动器。
# 只消费 /scan 与 /trunk_imu；不读取 Gazebo 真值里程计、不写场景、不发送控制命令。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIMENV_ROOT="${SIMENV_ROOT:?请设置官方 SimEnv 根目录，例如 /home/user/SimEnv}"

source /opt/ros/noetic/setup.bash
source "$SIMENV_ROOT/devel/setup.bash"
# 部分官方容器的 setup.bash 未向 Python 3 进程导出 rospy 路径；显式补齐后仍只使用
# Noetic 自带包和本仓库纯函数，不安装额外依赖。
ROS_PYTHON_DIST_PACKAGES="/opt/ros/noetic/lib/python3/dist-packages"
export PYTHONPATH="$ROS_PYTHON_DIST_PACKAGES:$REPO_ROOT/ros2_ws/src/hazardwalker_perception:${PYTHONPATH:-}"
cd "$SIMENV_ROOT"

exec python3 "$REPO_ROOT/scripts/official_simenv_lidar_imu_slam_node.py" "$@"
