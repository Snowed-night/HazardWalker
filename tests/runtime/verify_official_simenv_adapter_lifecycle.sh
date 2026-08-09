#!/usr/bin/env bash
# 在不连接 Docker/ROS 的隔离环境中验收适配器管理器的 PID、去重、状态和停止语义。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MANAGER="$REPO_ROOT/scripts/manage_official_simenv_rosbridge_adapter.sh"
TMP_ROOT="$(mktemp -d)"
FAKE_ROOT="$TMP_ROOT/repo"
FAKE_BIN="$TMP_ROOT/bin"
STATE_DIR="$TMP_ROOT/state"
DOMAIN_ID="${LIFECYCLE_TEST_DOMAIN_ID:-942}"

cleanup() {
  HAZARDWALKER_ROOT="$FAKE_ROOT" \
  OFFICIAL_SIMENV_ADAPTER_STATE_DIR="$STATE_DIR" \
  OFFICIAL_SIMENV_ROS2_SETUP="$TMP_ROOT/setup.bash" \
  ROS_DOMAIN_ID="$DOMAIN_ID" \
  PATH="$FAKE_BIN:$PATH" \
    bash "$MANAGER" stop >/dev/null 2>&1 || true
  rm -rf "$TMP_ROOT"
}
trap cleanup EXIT INT TERM

mkdir -p "$FAKE_ROOT/scripts" "$FAKE_BIN" "$STATE_DIR"

cat > "$FAKE_ROOT/scripts/run_official_simenv_rosbridge_adapter.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
# argv[0] 模拟真实 Python 节点脚本名，使管理器能通过 /proc 精确识别。
exec -a official_simenv_rosbridge_ros2_adapter_node.py sleep 300
SH
chmod +x "$FAKE_ROOT/scripts/run_official_simenv_rosbridge_adapter.sh"

cat > "$FAKE_BIN/ros2" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
same_domain_adapter_running() {
  local pid process_domain
  while read -r pid; do
    [[ -r "/proc/$pid/environ" ]] || continue
    process_domain="$(tr '\0' '\n' < "/proc/$pid/environ" |
      sed -n 's/^ROS_DOMAIN_ID=//p' | tail -n 1)"
    if [[ "${process_domain:-0}" == "${ROS_DOMAIN_ID:-0}" ]]; then
      return 0
    fi
  done < <(pgrep -u "$(id -u)" -f official_simenv_rosbridge_ros2_adapter_node.py || true)
  return 1
}
if [[ "${1:-}" == node && "${2:-}" == list ]]; then
  if [[ -n "${FAKE_NODE_MISS_FILE:-}" && -f "$FAKE_NODE_MISS_FILE" ]]; then
    miss_count="$(cat "$FAKE_NODE_MISS_FILE")"
    if (( miss_count > 0 )); then
      printf '%s\n' "$((miss_count - 1))" > "$FAKE_NODE_MISS_FILE"
      exit 0
    fi
  fi
  if [[ "${FAKE_EXTERNAL_NODE:-0}" == 1 ]] ||
     same_domain_adapter_running; then
    echo /hazardwalker_official_rosbridge_adapter
  fi
  if [[ "${FAKE_DUPLICATE_NODE:-0}" == 1 ]]; then
    echo /hazardwalker_official_rosbridge_adapter
  fi
  exit 0
fi
exit 2
SH
chmod +x "$FAKE_BIN/ros2"

cat > "$TMP_ROOT/setup.bash" <<SH
export PATH="$FAKE_BIN:\${PATH}"
SH

manager() {
  HAZARDWALKER_ROOT="$FAKE_ROOT" \
  OFFICIAL_SIMENV_ADAPTER_STATE_DIR="$STATE_DIR" \
  OFFICIAL_SIMENV_ROS2_SETUP="$TMP_ROOT/setup.bash" \
  OFFICIAL_SIMENV_ADAPTER_START_TIMEOUT_SEC=5 \
  OFFICIAL_SIMENV_ADAPTER_STOP_TIMEOUT_SEC=2 \
  FAKE_NODE_MISS_FILE="${FAKE_NODE_MISS_FILE:-}" \
  ROS_DOMAIN_ID="$DOMAIN_ID" \
  DOCKER_SIMENV_USER=lifecycle_test \
  PATH="$FAKE_BIN:$PATH" \
    bash "$MANAGER" "$@"
}

test_domain_pids() {
  local pid process_domain
  while read -r pid; do
    [[ -r "/proc/$pid/environ" ]] || continue
    process_domain="$(tr '\0' '\n' < "/proc/$pid/environ" |
      sed -n 's/^ROS_DOMAIN_ID=//p' | tail -n 1)"
    if [[ "${process_domain:-0}" == "$DOMAIN_ID" ]]; then
      printf '%s\n' "$pid"
    fi
  done < <(pgrep -u "$(id -u)" -f official_simenv_rosbridge_ros2_adapter_node.py || true)
}

manager start
FIRST_PID="$(find "$STATE_DIR" -name '*.adapter.pid' -type f -exec cat {} \;)"
[[ "$FIRST_PID" =~ ^[0-9]+$ ]] && kill -0 "$FIRST_PID"

# 第二次 start 必须复用同一个受管实例，而不是多启动一个发布者。
NODE_MISS_FILE="$TMP_ROOT/node-miss-count"
printf '1\n' > "$NODE_MISS_FILE"
FAKE_NODE_MISS_FILE="$NODE_MISS_FILE" manager start
SECOND_PID="$(find "$STATE_DIR" -name '*.adapter.pid' -type f -exec cat {} \;)"
[[ "$SECOND_PID" == "$FIRST_PID" ]]
[[ "$(test_domain_pids | wc -l)" -eq 1 ]]

# `status` 也必须容忍首轮 DDS 发现暂时为空，不能误报健康实例不可见。
printf '1\n' > "$NODE_MISS_FILE"
FAKE_NODE_MISS_FILE="$NODE_MISS_FILE" manager status |
  grep -q 'ROS2 node: ready (unique)'

# 即使 PID 文件和当前进程都正常，只要 DDS 图中还有第二个同名节点，就不能
# 把当前实例误报为唯一；start 也必须先回收本账号实例再因外部节点失败关闭。
if FAKE_DUPLICATE_NODE=1 manager status; then
  echo 'FAIL: duplicate ROS2 node was reported as a unique managed adapter' >&2
  exit 1
fi
if FAKE_DUPLICATE_NODE=1 manager start; then
  echo 'FAIL: duplicate ROS2 node did not block adapter startup' >&2
  exit 1
fi
if [[ -n "$(test_domain_pids)" ]]; then
  echo 'FAIL: local adapter remained after duplicate-node rejection' >&2
  exit 1
fi

# 前一案例会清理受管实例；重新启动以继续验证配置签名热替换。
manager start
SECOND_PID="$(find "$STATE_DIR" -name '*.adapter.pid' -type f -exec cat {} \;)"

# 数据流参数变化时必须替换旧实例，不能沿用错误的控制/传感器转发配置。
OFFICIAL_SIMENV_ENABLE_CONTROL=1 manager start
RECONFIGURED_PID="$(find "$STATE_DIR" -name '*.adapter.pid' -type f -exec cat {} \;)"
[[ "$RECONFIGURED_PID" =~ ^[0-9]+$ ]]
[[ "$RECONFIGURED_PID" != "$SECOND_PID" ]]
[[ "$(test_domain_pids | wc -l)" -eq 1 ]]

manager stop
if [[ -n "$(test_domain_pids)" ]]; then
  echo 'FAIL: adapter process remained after stop' >&2
  exit 1
fi
if find "$STATE_DIR" -name '*.adapter.pid' -type f | grep -q .; then
  echo 'FAIL: pid file remained after stop' >&2
  exit 1
fi

echo 'PASS: adapter lifecycle start/idempotent/status/stop'

# 同一 ROS 域若已有其他会话节点，当前账号无法安全回收，必须拒绝制造第二份实例。
if FAKE_EXTERNAL_NODE=1 manager start; then
  echo 'FAIL: external adapter node did not block duplicate startup' >&2
  exit 1
fi
if FAKE_EXTERNAL_NODE=1 manager stop; then
  echo 'FAIL: external adapter node did not block container-side shutdown sequence' >&2
  exit 1
fi
if find "$STATE_DIR" -name '*.adapter.pid' -type f | grep -q .; then
  echo 'FAIL: external-node rejection created a managed pid file' >&2
  exit 1
fi
echo 'PASS: external same-domain adapter fails closed'

# 复制正式 auto_docker.sh，使用假 Docker 与假 manager 验证容器—适配器编排顺序。
ORCH_ROOT="$TMP_ROOT/orchestration-repo"
PLATFORM_ROOT="$ORCH_ROOT/ros2_ws/src/hazardwalker_platform"
EVENT_LOG="$TMP_ROOT/orchestration-events.log"
CONTAINER_STATE="$TMP_ROOT/container-state"
mkdir -p "$PLATFORM_ROOT/docker" "$PLATFORM_ROOT/.ros1_catkin_ws/devel/lib/unitree_guide" \
  "$PLATFORM_ROOT/src/unitree_guide" "$ORCH_ROOT/scripts"
cp "$REPO_ROOT/ros2_ws/src/hazardwalker_platform/auto_docker.sh" "$PLATFORM_ROOT/auto_docker.sh"
touch "$PLATFORM_ROOT/.ros1_catkin_ws/devel/setup.bash"
touch "$PLATFORM_ROOT/.ros1_catkin_ws/devel/lib/unitree_guide/junior_ctrl"
chmod +x "$PLATFORM_ROOT/.ros1_catkin_ws/devel/lib/unitree_guide/junior_ctrl"

cat > "$PLATFORM_ROOT/docker/auto_noetic.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
case "${1:-}" in
  up)
    echo container-up >> "$TEST_EVENT_LOG"
    echo running > "$TEST_CONTAINER_STATE"
    ;;
  down)
    echo container-down >> "$TEST_EVENT_LOG"
    echo stopped > "$TEST_CONTAINER_STATE"
    ;;
  status) echo fake-container-status ;;
  *) exit 0 ;;
esac
SH
chmod +x "$PLATFORM_ROOT/docker/auto_noetic.sh"

cat > "$PLATFORM_ROOT/docker/bootstrap_runtime_assets.sh" <<'SH'
#!/usr/bin/env bash
# 编排测试不需要真实控制器资产；保留空实现以覆盖正式启动调用顺序。
exit 0
SH
chmod +x "$PLATFORM_ROOT/docker/bootstrap_runtime_assets.sh"

cat > "$ORCH_ROOT/scripts/manage_official_simenv_rosbridge_adapter.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
echo "adapter-${1:-}" >> "$TEST_EVENT_LOG"
if [[ "${1:-}" == start && "${TEST_ADAPTER_START_FAIL:-0}" == 1 ]]; then
  exit 1
fi
SH

cat > "$FAKE_BIN/docker" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
case "${1:-}" in
  info) exit 0 ;;
  inspect)
    state="$(cat "$TEST_CONTAINER_STATE" 2>/dev/null || true)"
    format="${3:-}"
    if [[ "$format" == *State.Running* ]]; then
      [[ "$state" == running ]] && echo true || echo false
    elif [[ "$format" == *State.Health* ]]; then
      [[ "$state" == running ]] && echo healthy || echo none
    elif [[ "$format" == *Config.Healthcheck* ]]; then
      echo true
    fi
    ;;
  *) exit 0 ;;
esac
SH
chmod +x "$FAKE_BIN/docker"

orchestrate() {
  HAZARDWALKER_ROOT="$ORCH_ROOT" \
  DOCKER_SIMENV_USER=lifecycle_test \
  TEST_EVENT_LOG="$EVENT_LOG" \
  TEST_CONTAINER_STATE="$CONTAINER_STATE" \
  OFFICIAL_SIMENV_CONTAINER_READY_TIMEOUT_SEC=2 \
  PATH="$FAKE_BIN:$PATH" \
    bash "$PLATFORM_ROOT/auto_docker.sh" "$@"
}

: > "$EVENT_LOG"
rm -f "$CONTAINER_STATE"
orchestrate up
[[ "$(paste -sd, "$EVENT_LOG")" == 'container-up,adapter-start' ]]
orchestrate down
[[ "$(paste -sd, "$EVENT_LOG")" == \
  'container-up,adapter-start,adapter-stop,container-down' ]]

# 本轮新建容器遇到适配器启动失败时必须回滚。
: > "$EVENT_LOG"
rm -f "$CONTAINER_STATE"
if TEST_ADAPTER_START_FAIL=1 orchestrate up; then
  echo 'FAIL: adapter startup failure was accepted' >&2
  exit 1
fi
[[ "$(paste -sd, "$EVENT_LOG")" == 'container-up,adapter-start,container-down' ]]

# 已经运行的容器不属于本轮创建；适配器失败不能擅自停止它。
: > "$EVENT_LOG"
echo running > "$CONTAINER_STATE"
if TEST_ADAPTER_START_FAIL=1 orchestrate up; then
  echo 'FAIL: adapter startup failure on existing container was accepted' >&2
  exit 1
fi
[[ "$(paste -sd, "$EVENT_LOG")" == 'container-up,adapter-start' ]]

echo 'PASS: auto_docker up/down ordering and startup rollback'
