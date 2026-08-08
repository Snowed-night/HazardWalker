#!/usr/bin/env bash
# First-time catkin build for SimEnv ROS1 inside Ubuntu 20.04 / Noetic container.
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 本目录同时是 ROS2 包，不能直接作为 catkin_make 的工作区根目录。
# ROS1 产物固定放进独立子工作区，避免根目录 package.xml 触发 Catkin 报错。
CATKIN_WORKSPACE_DIR="${SIMENV_CATKIN_WORKSPACE_DIR:-$WORKSPACE_DIR/.ros1_catkin_ws}"

source /opt/ros/noetic/setup.bash

if [[ ! -d "$WORKSPACE_DIR/src" ]]; then
  echo "ERROR: src/ missing. Run sync_ros1_from_platform.py first." >&2
  exit 1
fi

LIBTORCH="${LIBTORCH_ROOT:-/home/ros/Guoyulun/Download/libtorch}"
if [[ ! -f "$LIBTORCH/share/cmake/Torch/TorchConfig.cmake" ]]; then
  echo "ERROR: LibTorch not found at $LIBTORCH" >&2
  echo "Rebuild image: ./docker/auto_noetic.sh image" >&2
  exit 1
fi
export LD_LIBRARY_PATH="${LIBTORCH}/lib:${LD_LIBRARY_PATH:-}"
export CMAKE_PREFIX_PATH="${LIBTORCH}:${CMAKE_PREFIX_PATH:-}"

mkdir -p "$CATKIN_WORKSPACE_DIR"
if [[ -e "$CATKIN_WORKSPACE_DIR/src" && ! -L "$CATKIN_WORKSPACE_DIR/src" ]]; then
  echo "ERROR: $CATKIN_WORKSPACE_DIR/src must be a symbolic link to $WORKSPACE_DIR/src" >&2
  exit 1
fi
ln -sfn "$WORKSPACE_DIR/src" "$CATKIN_WORKSPACE_DIR/src"

cd "$CATKIN_WORKSPACE_DIR"
# 官方源码同时含实体机 SDK 示例；这些程序需要真实 A1 的专有动态库，仿真不使用，
# 不能让它们阻断 Gazebo、楼宇控制和 junior_ctrl 的构建。
# 如需扩展仿真依赖，可通过 SIMENV_CATKIN_PACKAGES 覆盖此列表。
if [[ -n "${SIMENV_CATKIN_PACKAGES:-}" ]]; then
  read -r -a CATKIN_PACKAGES <<< "$SIMENV_CATKIN_PACKAGES"
else
  CATKIN_PACKAGES=(
    building_generator_core
    building_generator_interfaces
    building_obstacles
    building_generator_classic
    unitree_legged_msgs
    # Gazebo 的 UnitreeJointController 插件；未构建时 A1 只能趴伏，无法输出关节力矩。
    unitree_legged_control
    unitree_gazebo
    unitree_guide
  )
fi
catkin_make -DCMAKE_BUILD_TYPE=Release -j"$(nproc)" --pkg "${CATKIN_PACKAGES[@]}"

if [[ -n "${HOST_UID:-}" && -n "${HOST_GID:-}" ]]; then
  # 容器内 catkin_make 默认以 root 运行。把整个独立工作区交还给宿主账号，
  # 包括 `.catkin_workspace` 与 `src` 符号链接，避免 Git/VS Code 无法清理未跟踪产物。
  # GNU chown 默认不跟随 src 符号链接，因此不会改写真实 ROS1 源码目录的所有权。
  chown -R "${HOST_UID}:${HOST_GID}" "$CATKIN_WORKSPACE_DIR" 2>/dev/null || true
fi

echo "catkin build finished: $CATKIN_WORKSPACE_DIR/devel/setup.bash"
