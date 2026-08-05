#!/usr/bin/env bash
# 管理官方 SimEnv ROS2 适配器的宿主侧生命周期，保证同一 ROS 域只有一个受管实例。
set -euo pipefail

ROOT="${HAZARDWALKER_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
RUNNER="$ROOT/scripts/run_official_simenv_rosbridge_adapter.sh"
NODE_SCRIPT="$ROOT/scripts/official_simenv_rosbridge_ros2_adapter_node.py"
NODE_SCRIPT_NAME="$(basename "$NODE_SCRIPT")"
CONTAINER="${SIMENV_CONTAINER:-simenv_ros1_${DOCKER_SIMENV_USER:-${USER:-default}}}"
export SIMENV_CONTAINER="$CONTAINER"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${OFFICIAL_SIMENV_ROS_DOMAIN_ID:-42}}"
export OFFICIAL_SIMENV_MANAGED_LIFECYCLE=1
export OFFICIAL_SIMENV_LIFECYCLE_CONTAINER="$CONTAINER"

STATE_ROOT="${OFFICIAL_SIMENV_ADAPTER_STATE_DIR:-$HOME/.local/state/hazardwalker}"
SAFE_KEY="$(printf '%s-domain-%s' "$CONTAINER" "$ROS_DOMAIN_ID" | tr -c '[:alnum:]_.-' '_')"
PID_FILE="$STATE_ROOT/${SAFE_KEY}.adapter.pid"
SIGNATURE_FILE="$STATE_ROOT/${SAFE_KEY}.adapter.signature"
LOG_FILE="$STATE_ROOT/${SAFE_KEY}.adapter.log"
LOCK_FILE="$STATE_ROOT/${SAFE_KEY}.adapter.lock"
START_TIMEOUT_SEC="${OFFICIAL_SIMENV_ADAPTER_START_TIMEOUT_SEC:-30}"
STOP_TIMEOUT_SEC="${OFFICIAL_SIMENV_ADAPTER_STOP_TIMEOUT_SEC:-10}"
DISCOVERY_SETTLE_SEC="${OFFICIAL_SIMENV_ADAPTER_DISCOVERY_SETTLE_SEC:-5}"

mkdir -p "$STATE_ROOT"

adapter_signature() {
  {
    printf 'container=%s\ndomain=%s\n' "$CONTAINER" "$ROS_DOMAIN_ID"
    # 只纳入会改变适配器数据流的 OFFICIAL_SIMENV_* 参数；生命周期超时不应触发重启。
    env | LC_ALL=C sort |
      grep '^OFFICIAL_SIMENV_' |
      grep -Ev '^OFFICIAL_SIMENV_(ADAPTER_|AUTO_ADAPTER|CONTAINER_READY|ROS_DOMAIN_ID)' || true
  } | sha256sum | awk '{print $1}'
}

pid_matches_adapter() {
  local pid="$1"
  [[ -r "/proc/$pid/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/$pid/cmdline" | grep -Fq "$NODE_SCRIPT_NAME"
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
    if pid_matches_adapter "$pid" && pid_matches_domain "$pid"; then
      printf '%s\n' "$pid"
    fi
  done < <(pgrep -u "$(id -u)" -f "$NODE_SCRIPT_NAME" 2>/dev/null || true)
}

visible_node_count() {
  (
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX \
      AMENT_CURRENT_PREFIX CMAKE_PREFIX_PATH PYTHONPATH \
      ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH
    set +u
    source "${OFFICIAL_SIMENV_ROS2_SETUP:-/opt/ros/jazzy/setup.bash}"
    set -u
    timeout 3 ros2 node list --no-daemon 2>/dev/null |
      grep -cx '/hazardwalker_official_rosbridge_adapter' || true
  )
}

node_visible() {
  (( $(visible_node_count) > 0 ))
}

single_node_visible() {
  (( $(visible_node_count) == 1 ))
}

stable_single_node_visible() {
  local deadline=$((SECONDS + DISCOVERY_SETTLE_SEC))
  while (( SECONDS < deadline )); do
    (( $(visible_node_count) == 1 )) || return 1
    sleep 0.2
  done
}

stop_pid() {
  local pid="$1" deadline
  pid_matches_adapter "$pid" || return 0
  kill -TERM "$pid" 2>/dev/null || true
  deadline=$((SECONDS + STOP_TIMEOUT_SEC))
  while kill -0 "$pid" 2>/dev/null && (( SECONDS < deadline )); do
    sleep 0.1
  done
  if kill -0 "$pid" 2>/dev/null; then
    kill -KILL "$pid" 2>/dev/null || true
  fi
}

stop_adapter() {
  local pid deadline pids=()
  if [[ -f "$PID_FILE" ]]; then
    pid="$(tr -cd '0-9' < "$PID_FILE")"
    [[ -n "$pid" ]] && pids+=("$pid")
  fi
  while read -r pid; do
    [[ -n "$pid" ]] && pids+=("$pid")
  done < <(matching_pids)

  if (( ${#pids[@]} == 0 )); then
    rm -f "$PID_FILE" "$SIGNATURE_FILE"
    # 每次 ros2 CLI 都创建新的 DDS 参与者，首个快照可能尚未发现其他账号的
    # 同名节点。完整等待一个发现窗口，防止把“暂未发现”误判为环境空闲。
    deadline=$((SECONDS + DISCOVERY_SETTLE_SEC))
    while (( SECONDS < deadline )); do
      if node_visible; then
        echo "[adapter-manager] 同一 ROS 域仍有其他会话的适配器；请由进程所有者停止：domain=$ROS_DOMAIN_ID" >&2
        return 1
      fi
      sleep 0.2
    done
    echo "[adapter-manager] 适配器未运行：domain=$ROS_DOMAIN_ID"
    return 0
  fi
  for pid in "${pids[@]}"; do
    stop_pid "$pid"
  done
  rm -f "$PID_FILE" "$SIGNATURE_FILE"
  if [[ -n "$(matching_pids)" ]]; then
    echo "[adapter-manager] 适配器停止失败：domain=$ROS_DOMAIN_ID" >&2
    return 1
  fi
  # DDS 图发现具有短暂缓存；先等待本实例从图中消失，再判断是否存在其他账号实例。
  deadline=$((SECONDS + DISCOVERY_SETTLE_SEC))
  while node_visible && (( SECONDS < deadline )); do
    sleep 0.2
  done
  if node_visible; then
    echo "[adapter-manager] 本实例已停止，但同一 ROS 域仍有其他会话适配器：domain=$ROS_DOMAIN_ID" >&2
    return 1
  fi
  echo "[adapter-manager] 适配器已停止：domain=$ROS_DOMAIN_ID"
}

start_adapter() {
  local pid deadline node_count desired_signature current_signature=''
  if [[ ! -x "$RUNNER" && ! -f "$RUNNER" ]]; then
    echo "[adapter-manager] 找不到启动脚本：$RUNNER" >&2
    return 1
  fi

  desired_signature="$(adapter_signature)"
  [[ -f "$SIGNATURE_FILE" ]] && current_signature="$(tr -cd '0-9a-f' < "$SIGNATURE_FILE")"
  if [[ -f "$PID_FILE" ]]; then
    pid="$(tr -cd '0-9' < "$PID_FILE")"
    if [[ -n "$pid" ]] && pid_matches_adapter "$pid" && stable_single_node_visible &&
       [[ "$current_signature" == "$desired_signature" ]]; then
      echo "[adapter-manager] 适配器已运行：pid=$pid domain=$ROS_DOMAIN_ID"
      return 0
    fi
  fi

  # 接管旧版手工启动或异常退出留下的同域实例，避免重复发布 /clock、/scan 和控制。
  stop_adapter >/dev/null
  : > "$LOG_FILE"
  nohup setsid bash "$RUNNER" >"$LOG_FILE" 2>&1 < /dev/null 9>&- &
  pid=$!
  printf '%s\n' "$pid" > "$PID_FILE"
  printf '%s\n' "$desired_signature" > "$SIGNATURE_FILE"

  deadline=$((SECONDS + START_TIMEOUT_SEC))
  while (( SECONDS < deadline )); do
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "[adapter-manager] 适配器启动失败；日志：$LOG_FILE" >&2
      tail -n 40 "$LOG_FILE" >&2 || true
      rm -f "$PID_FILE" "$SIGNATURE_FILE"
      return 1
    fi
    node_count="$(visible_node_count)"
    if (( node_count > 1 )); then
      echo "[adapter-manager] 发现重复适配器：count=$node_count domain=$ROS_DOMAIN_ID" >&2
      stop_pid "$pid"
      rm -f "$PID_FILE" "$SIGNATURE_FILE"
      return 1
    fi
    if (( node_count == 1 )) && stable_single_node_visible; then
      echo "[adapter-manager] 适配器已启动：pid=$pid domain=$ROS_DOMAIN_ID log=$LOG_FILE"
      return 0
    fi
    sleep 0.5
  done

  echo "[adapter-manager] ${START_TIMEOUT_SEC}s 内未发现适配器节点；日志：$LOG_FILE" >&2
  stop_pid "$pid"
  rm -f "$PID_FILE" "$SIGNATURE_FILE"
  tail -n 40 "$LOG_FILE" >&2 || true
  return 1
}

status_adapter() {
  local pid managed_pid=''
  [[ -f "$PID_FILE" ]] && managed_pid="$(tr -cd '0-9' < "$PID_FILE")"
  if [[ -n "$managed_pid" ]] && pid_matches_adapter "$managed_pid"; then
    echo "[adapter-manager] running pid=$managed_pid domain=$ROS_DOMAIN_ID container=$CONTAINER"
    node_count="$(visible_node_count)"
    if [[ "$node_count" == 1 ]]; then
      echo '[adapter-manager] ROS2 node: ready (unique)'
      return 0
    fi
    if (( node_count > 1 )); then
      echo "[adapter-manager] ROS2 node: duplicate count=$node_count" >&2
      return 1
    fi
    echo '[adapter-manager] ROS2 node: not-visible' >&2
    return 1
  fi
  pid="$(matching_pids | head -n 1)"
  if [[ -n "$pid" ]]; then
    echo "[adapter-manager] unmanaged pid=$pid domain=$ROS_DOMAIN_ID container=$CONTAINER"
    return 1
  fi
  if node_visible; then
    echo "[adapter-manager] external-node domain=$ROS_DOMAIN_ID container=$CONTAINER"
    return 1
  fi
  echo "[adapter-manager] stopped domain=$ROS_DOMAIN_ID container=$CONTAINER"
}

exec 9>"$LOCK_FILE"
flock -x 9
case "${1:-status}" in
  start) start_adapter ;;
  stop) stop_adapter ;;
  restart) stop_adapter; start_adapter ;;
  status) status_adapter ;;
  *)
    echo "Usage: $0 {start|stop|restart|status}" >&2
    exit 2
    ;;
esac
