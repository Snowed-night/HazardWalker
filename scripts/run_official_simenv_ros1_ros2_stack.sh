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

NAVIGATION_REQUESTED=0
for ARG in "$@"; do
  case "$ARG" in
    start_navigation:=true|start_navigation:=True|start_navigation:=1)
      NAVIGATION_REQUESTED=1
      ;;
  esac
done
case "${OFFICIAL_SIMENV_ENABLE_CONTROL:-0}" in
  1|true|True|TRUE|yes|YES|on|ON) CONTROL_REQUESTED=1 ;;
  *) CONTROL_REQUESTED=0 ;;
esac
if [[ "$NAVIGATION_REQUESTED" == 1 && "$CONTROL_REQUESTED" != 1 ]]; then
  echo '[stack] start_navigation=true 但控制适配未显式开启；拒绝启动“看似运行、实际不可控”的导航。' >&2
  echo '[stack] 独占验收时请设置 OFFICIAL_SIMENV_ENABLE_CONTROL=1。' >&2
  exit 1
fi

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
  if [[ -z "${SIMENV_START_COMMAND:-}" ]]; then
    echo "[stack] 官方容器未运行：$CONTAINER" >&2
    echo "[stack] 为避免错误重置共享场景，请由平台组启动；或显式提供 SIMENV_START_COMMAND。" >&2
    exit 1
  fi
  echo "[stack] 执行调用方明确提供的官方启动命令。"
  bash -lc "$SIMENV_START_COMMAND"
fi

bash "$ROOT/scripts/check_official_simenv_exclusive_session.sh" \
  --container "$CONTAINER" --require-exclusive
if ros2 node list 2>/dev/null | grep -qx '/hazardwalker_official_rosbridge_adapter'; then
  echo '[stack] ROS_DOMAIN_ID 内已存在官方适配器；请由该进程所有者收尾后重试，避免重复 /clock、/scan 或控制转发。' >&2
  exit 1
fi
# 显式经 bash 调用，不依赖 Git checkout、共享目录或 Windows 挂载是否保留可执行位。
bash "$ROOT/scripts/run_official_simenv_rosbridge_adapter.sh" &
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
# 业务节点统一使用仿真时间；必须等到适配器转发出两帧递增的 /clock。
# 单个非零值可能只是冻结的旧消息，不能证明仿真仍在推进。
CLOCK_READY=0
LAST_CLOCK_NS=
for _ in {1..20}; do
  CLOCK_SAMPLE="$(timeout 2 ros2 topic echo /clock --once 2>/dev/null || true)"
  CLOCK_SEC="$(printf '%s\n' "$CLOCK_SAMPLE" | awk '/sec:/{print $2; exit}')"
  CLOCK_NSEC="$(printf '%s\n' "$CLOCK_SAMPLE" | awk '/nanosec:/{print $2; exit}')"
  if [[ "$CLOCK_SEC" =~ ^[0-9]+$ ]] && [[ "$CLOCK_NSEC" =~ ^[0-9]+$ ]]; then
    CLOCK_NS=$((10#$CLOCK_SEC * 1000000000 + 10#$CLOCK_NSEC))
    if [[ -n "$LAST_CLOCK_NS" ]] && (( CLOCK_NS > LAST_CLOCK_NS )); then
      CLOCK_READY=1
      break
    fi
    if (( CLOCK_NS > 0 )); then
      LAST_CLOCK_NS="$CLOCK_NS"
    fi
  fi
done
if [[ "$CLOCK_READY" != "1" ]]; then
  echo '[stack] 20 次采样仍未收到连续递增的 /clock；拒绝启动混合时间域业务栈。' >&2
  exit 1
fi
if [[ "$NAVIGATION_REQUESTED" == 1 ]]; then
  ADAPTER_STATUS="$(
    timeout 3 ros2 topic echo /hw/platform/official_simenv_adapter_status \
      --field data --once 2>/dev/null || true
  )"
  if [[ "$ADAPTER_STATUS" != *'"enable_cmd_vel_relay": true'* ]]; then
    echo '[stack] 适配器状态未确认控制转发已开启；拒绝启动导航。' >&2
    exit 1
  fi
fi
echo '[stack] 启动 ROS2 业务层（不含 fake 平台；固定航点导航默认关闭）。'
setsid ros2 launch hazardwalker_bringup official_simenv_business.launch.py "$@" &
BUSINESS_PID=$!
wait "$BUSINESS_PID"
