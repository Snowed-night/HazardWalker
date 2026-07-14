#!/usr/bin/env bash
# 官方 SimEnv + ROS2 业务层统一入口。默认不越权启动或重置共享仿真。
set -euo pipefail

# 负责人：姜晨。先确认官方容器，再启动独立适配，再启动不含 fake/Harmonic 的 ROS2 业务层。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${SIMENV_CONTAINER:-simenv_run}"
ROS2_SETUP="${OFFICIAL_SIMENV_ROS2_SETUP:-/opt/ros/jazzy/setup.bash}"

if [[ ! -f "$ROS2_SETUP" ]]; then
  echo "[stack] 找不到 ROS2 环境：$ROS2_SETUP；请设置 OFFICIAL_SIMENV_ROS2_SETUP。" >&2; exit 1
fi
source "$ROS2_SETUP"
source "$ROOT/ros2_ws/install/setup.bash"
# rosbridge 只跨 WebSocket，ROS_DOMAIN_ID 仅要求本机 ROS2 业务节点一致。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
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

"$ROOT/scripts/run_official_simenv_rosbridge_adapter.sh" &
ADAPTER_PID=$!
cleanup_adapter() {
  # 业务层退出时同步停止适配器，避免遗留进程在下一轮继续向 ROS1 发布速度。
  kill "$ADAPTER_PID" 2>/dev/null || true
  wait "$ADAPTER_PID" 2>/dev/null || true
}
trap cleanup_adapter EXIT INT TERM
sleep 2
if ! kill -0 "$ADAPTER_PID" 2>/dev/null; then
  echo '[stack] rosbridge 适配器启动失败，未启动业务层。' >&2
  exit 1
fi
echo '[stack] 启动 ROS2 业务层（不含 fake 平台；固定航点导航默认关闭）。'
ros2 launch hazardwalker_bringup official_simenv_business.launch.py "$@"
