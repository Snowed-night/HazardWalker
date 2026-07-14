#!/usr/bin/env bash
# First-time catkin build for SimEnv ROS1 inside Ubuntu 20.04 / Noetic container.
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"

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

if [[ ! -f "$WORKSPACE_DIR/src/CMakeLists.txt" ]]; then
  catkin_init_workspace "$WORKSPACE_DIR/src"
fi

cd "$WORKSPACE_DIR"
catkin_make -DCMAKE_BUILD_TYPE=Release -j"$(nproc)"

if [[ -n "${HOST_UID:-}" && -n "${HOST_GID:-}" ]]; then
  chown -R "${HOST_UID}:${HOST_GID}" "$WORKSPACE_DIR/build" "$WORKSPACE_DIR/devel" "$WORKSPACE_DIR/install" 2>/dev/null || true
fi

echo "catkin build finished: $WORKSPACE_DIR/devel/setup.bash"
