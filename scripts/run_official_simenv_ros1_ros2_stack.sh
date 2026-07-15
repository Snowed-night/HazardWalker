#!/usr/bin/env bash
# 官方 SimEnv + ROS2 业务层统一入口。默认不越权启动或重置共享仿真。
set -euo pipefail

# 负责人：姜晨。先确认官方容器，再启动独立适配，再启动不含 fake/Harmonic 的 ROS2 业务层。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${SIMENV_CONTAINER:-simenv_run}"

# Jazzy 的 setup.bash 会读取若干可选变量；在 set -u 下未定义时会提前退出。
# 与独立适配器入口保持同一加载方式，避免一键业务栈还未启动就中断。
set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
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

"$ROOT/scripts/check_official_simenv_exclusive_session.sh" --container "$CONTAINER" --require-exclusive
"$ROOT/scripts/run_official_simenv_rosbridge_adapter.sh" &
ADAPTER_PID=$!
cleanup_adapter() {
  # ros2 launch 会派生导航、感知和决策子进程；只杀 launch 父进程会留下多个
  # /hw/cmd_vel 发布者。setsid 为业务栈创建独立进程组后统一回收。
  if [[ -n "${BUSINESS_PID:-}" ]]; then
    kill -- "-$BUSINESS_PID" 2>/dev/null || true
  fi
  kill "$ADAPTER_PID" 2>/dev/null || true
}
trap cleanup_adapter EXIT INT TERM
echo '[stack] 启动 ROS2 业务层（不含 fake 平台；固定航点导航默认关闭）。'
setsid ros2 launch hazardwalker_bringup official_simenv_business.launch.py "$@" &
BUSINESS_PID=$!
wait "$BUSINESS_PID"
