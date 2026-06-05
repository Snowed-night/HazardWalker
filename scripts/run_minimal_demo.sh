#!/usr/bin/env bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export HAZARDWALKER_ROOT="$REPO_ROOT"

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

if [ -f "$REPO_ROOT/ros2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$REPO_ROOT/ros2_ws/install/setup.bash"
else
  echo "Workspace is not built. Run ./scripts/build.sh first." >&2
  exit 1
fi

ros2 launch hazardwalker_bringup minimal_demo.launch.py
