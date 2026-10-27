#!/usr/bin/env bash
# 负责人维护：管理共享官方环境唯一 ROS2 控制仲裁器及其控制模式。
set -euo pipefail

ROOT="${HAZARDWALKER_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
RUNNER="$ROOT/scripts/run_official_simenv_command_mux.sh"
PROCESS_TOKEN='hazardwalker_platform.command_mux_node'
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${OFFICIAL_SIMENV_ROS_DOMAIN_ID:-42}}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

STATE_ROOT="${OFFICIAL_SIMENV_CONTROL_STATE_DIR:-$HOME/.local/state/hazardwalker}"
SAFE_KEY="$(printf 'domain-%s' "$ROS_DOMAIN_ID" | tr -c '[:alnum:]_.-' '_')"
PID_FILE="$STATE_ROOT/${SAFE_KEY}.command-mux.pid"
LOG_FILE="$STATE_ROOT/${SAFE_KEY}.command-mux.log"
LOCK_FILE="$STATE_ROOT/${SAFE_KEY}.command-mux.lock"
START_TIMEOUT_SEC="${OFFICIAL_SIMENV_CONTROL_START_TIMEOUT_SEC:-20}"
STOP_TIMEOUT_SEC="${OFFICIAL_SIMENV_CONTROL_STOP_TIMEOUT_SEC:-8}"
DISCOVERY_SETTLE_SEC="${OFFICIAL_SIMENV_CONTROL_DISCOVERY_SETTLE_SEC:-4}"
mkdir -p "$STATE_ROOT"

ros2_cli() {
  (
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX \
      AMENT_CURRENT_PREFIX CMAKE_PREFIX_PATH PYTHONPATH \
      ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH
    set +u
    source "${OFFICIAL_SIMENV_ROS2_SETUP:-/opt/ros/jazzy/setup.bash}"
    set -u
    "$@"
  )
}

pid_matches_mux() {
  local pid="$1"
  [[ -r "/proc/$pid/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/$pid/cmdline" | grep -Fq "$PROCESS_TOKEN"
}

pid_matches_domain() {
  local pid="$1" process_domain=0
  [[ -r "/proc/$pid/environ" ]] || return 1
  process_domain="$(tr '\0' '\n' < "/proc/$pid/environ" |
    sed -n 's/^ROS_DOMAIN_ID=//p' | tail -n 1)"
  [[ "${process_domain:-0}" == "$ROS_DOMAIN_ID" ]]
}

matching_pids() {
  local pid
  while read -r pid; do
    [[ -n "$pid" ]] || continue
    if pid_matches_mux "$pid" && pid_matches_domain "$pid"; then
      printf '%s\n' "$pid"
    fi
  done < <(pgrep -u "$(id -u)" -f "$PROCESS_TOKEN" 2>/dev/null || true)
}

visible_node_count() {
  ros2_cli timeout 3 ros2 node list --no-daemon 2>/dev/null |
    grep -cx '/hazardwalker_command_mux' || true
}

node_visible() {
  (( $(visible_node_count) > 0 ))
}

stable_single_node_visible() {
  local deadline=$((SECONDS + DISCOVERY_SETTLE_SEC)) count stable=0
  while (( SECONDS < deadline )); do
    count="$(visible_node_count)"
    if (( count > 1 )); then
      return 1
    elif (( count == 1 )); then
      stable=$((stable + 1))
      (( stable >= 2 )) && return 0
    else
      stable=0
    fi
    sleep 0.2
  done
  return 1
}

stop_pid() {
  local pid="$1" deadline
  pid_matches_mux "$pid" || return 0
  # command_mux_node 收到 TERM 后会先发布零速度，再退出。
  kill -TERM "$pid" 2>/dev/null || true
  deadline=$((SECONDS + STOP_TIMEOUT_SEC))
  while kill -0 "$pid" 2>/dev/null && (( SECONDS < deadline )); do
    sleep 0.1
  done
  kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
}

stop_mux() {
  local pid deadline pids=()
  [[ -f "$PID_FILE" ]] && {
    pid="$(tr -cd '0-9' < "$PID_FILE")"
    [[ -n "$pid" ]] && pids+=("$pid")
  }
  while read -r pid; do
    [[ -n "$pid" ]] && pids+=("$pid")
  done < <(matching_pids)

  if (( ${#pids[@]} == 0 )); then
    rm -f "$PID_FILE"
    if node_visible; then
      echo "[control-manager] 同一 ROS 域存在其他账号的仲裁器，拒绝越权停止：domain=$ROS_DOMAIN_ID" >&2
      return 1
    fi
    echo "[control-manager] 仲裁器未运行：domain=$ROS_DOMAIN_ID"
    return 0
  fi
  for pid in "${pids[@]}"; do
    stop_pid "$pid"
  done
  rm -f "$PID_FILE"
  deadline=$((SECONDS + DISCOVERY_SETTLE_SEC))
  while node_visible && (( SECONDS < deadline )); do sleep 0.2; done
  if node_visible; then
    echo "[control-manager] 本账号实例已停止，但同域仍有其他仲裁器：domain=$ROS_DOMAIN_ID" >&2
    return 1
  fi
  echo "[control-manager] 仲裁器已停止：domain=$ROS_DOMAIN_ID"
}

start_mux() {
  local pid deadline count
  if [[ -f "$PID_FILE" ]]; then
    pid="$(tr -cd '0-9' < "$PID_FILE")"
    if [[ -n "$pid" ]] && pid_matches_mux "$pid" && stable_single_node_visible; then
      echo "[control-manager] 仲裁器已运行：pid=$pid domain=$ROS_DOMAIN_ID"
      return 0
    fi
  fi
  if node_visible && [[ -z "$(matching_pids)" ]]; then
    echo "[control-manager] 同一 ROS 域已有其他账号仲裁器，拒绝重复启动：domain=$ROS_DOMAIN_ID" >&2
    return 1
  fi
  stop_mux >/dev/null || true
  : > "$LOG_FILE"
  nohup setsid bash "$RUNNER" >"$LOG_FILE" 2>&1 < /dev/null 9>&- &
  pid=$!
  printf '%s\n' "$pid" > "$PID_FILE"
  deadline=$((SECONDS + START_TIMEOUT_SEC))
  while (( SECONDS < deadline )); do
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "[control-manager] 仲裁器启动失败；日志：$LOG_FILE" >&2
      tail -n 40 "$LOG_FILE" >&2 || true
      rm -f "$PID_FILE"
      return 1
    fi
    count="$(visible_node_count)"
    if (( count > 1 )); then
      echo "[control-manager] 发现重复仲裁器：count=$count domain=$ROS_DOMAIN_ID" >&2
      stop_pid "$pid"
      rm -f "$PID_FILE"
      return 1
    fi
    if (( count == 1 )) && stable_single_node_visible; then
      echo "[control-manager] 仲裁器已启动：pid=$pid domain=$ROS_DOMAIN_ID log=$LOG_FILE"
      return 0
    fi
    sleep 0.5
  done
  echo "[control-manager] ${START_TIMEOUT_SEC}s 内未发现仲裁器；日志：$LOG_FILE" >&2
  stop_pid "$pid"
  rm -f "$PID_FILE"
  tail -n 40 "$LOG_FILE" >&2 || true
  return 1
}

set_mode() {
  local mode="${1:-}"
  case "$mode" in keyboard|navigation|assist|stopped) ;; *)
    echo 'Usage: manage_official_simenv_command_mux.sh mode {keyboard|navigation|assist|stopped}' >&2
    return 2 ;;
  esac
  stable_single_node_visible || {
    echo "[control-manager] 仲裁器未唯一就绪，不能切换模式：domain=$ROS_DOMAIN_ID" >&2
    return 1
  }
  ros2_cli timeout 5 ros2 topic pub --once /hw/control/mode_request \
    std_msgs/msg/String "{data: '$mode'}" >/dev/null
  echo "[control-manager] 已请求控制模式：$mode"
}

status_mux() {
  local pid=''
  [[ -f "$PID_FILE" ]] && pid="$(tr -cd '0-9' < "$PID_FILE")"
  if [[ -n "$pid" ]] && pid_matches_mux "$pid"; then
    echo "[control-manager] running pid=$pid domain=$ROS_DOMAIN_ID"
    stable_single_node_visible
    return
  fi
  if node_visible; then
    echo "[control-manager] external-node domain=$ROS_DOMAIN_ID" >&2
    return 1
  fi
  echo "[control-manager] stopped domain=$ROS_DOMAIN_ID"
}

exec 9>"$LOCK_FILE"
flock -x 9
case "${1:-status}" in
  start) start_mux ;;
  stop) stop_mux ;;
  restart) stop_mux; start_mux ;;
  status) status_mux ;;
  mode) set_mode "${2:-}" ;;
  *) echo "Usage: $0 {start|stop|restart|status|mode MODE}" >&2; exit 2 ;;
esac
