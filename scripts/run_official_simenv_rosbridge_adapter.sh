#!/usr/bin/env bash
# 在 ROS2 主机启动官方 ROS1 rosbridge 双向适配器；官方 Docker 只需运行 rosbridge_websocket。
set -euo pipefail

# 负责人：姜晨。默认适配实际官方容器 simenv_run；可用环境变量覆盖地址和容器名。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${SIMENV_CONTAINER:-simenv_run}"
ROSBRIDGE_URL="${OFFICIAL_SIMENV_ROSBRIDGE_URL:-ws://127.0.0.1:9090}"
ENABLE_CONTROL="${OFFICIAL_SIMENV_ENABLE_CONTROL:-0}"

source /opt/ros/jazzy/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
  echo "[rosbridge-adapter] 官方容器未运行：$CONTAINER" >&2; exit 1
fi
if ! docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode list | grep -qx /rosbridge_websocket'; then
  echo '[rosbridge-adapter] 容器内没有 /rosbridge_websocket；请先由官方 auto.sh 启动 ROS1 rosbridge。' >&2; exit 1
fi
exec python3 "$ROOT/scripts/official_simenv_rosbridge_ros2_adapter_node.py" --ros-args \
  -p rosbridge_url:="$ROSBRIDGE_URL" -p enable_cmd_vel_relay:="$ENABLE_CONTROL"
