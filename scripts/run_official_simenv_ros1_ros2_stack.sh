#!/usr/bin/env bash
# 官方 SimEnv + ROS2 业务层统一入口。默认不越权启动或重置共享仿真。
set -euo pipefail

# 负责人：姜晨。先确认官方容器，再启动独立适配，再启动不含 fake/Harmonic 的 ROS2 业务层。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${SIMENV_CONTAINER:-simenv_ros1_hazard_platform}"

source /opt/ros/jazzy/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:?请先设置与官方 dynamic_bridge 相同的 ROS_DOMAIN_ID}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
  if [[ -z "${SIMENV_START_COMMAND:-}" ]]; then
    echo "[stack] 官方容器未运行：$CONTAINER" >&2
    echo "[stack] 为避免错误重置共享场景，请由平台组启动；或显式提供 SIMENV_START_COMMAND。" >&2
    exit 1
  fi
  echo "[stack] 执行调用方明确提供的官方启动命令。"
  bash -lc "$SIMENV_START_COMMAND"
fi

"$ROOT/scripts/run_official_simenv_ros1_adapter.sh"
echo '[stack] 启动 ROS2 业务层（不含 fake 平台；固定航点导航默认关闭）。'
exec ros2 launch hazardwalker_bringup official_simenv_business.launch.py "$@"
