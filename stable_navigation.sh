#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLATFORM="$ROOT/ros2_ws/src/hazardwalker_platform"
STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/hazardwalker-stable"
PID_FILE="$STATE_DIR/navigation.pid"
LOG_FILE="$STATE_DIR/navigation.log"
RESULT_ROOT="$HOME/HazardWalker_results"

export DOCKER_SIMENV_USER=station_cluster
export ROS_DOMAIN_ID=43
export ROSBRIDGE_PORT=9091
export OFFICIAL_SIMENV_ROSBRIDGE_URL=ws://127.0.0.1:9091
export OFFICIAL_SIMENV_ENABLE_CONTROL=1
export OFFICIAL_SIMENV_ENABLE_ODOM_RELAY=1
export OFFICIAL_SIMENV_ENABLE_ODOM_TF_RELAY=0
export OFFICIAL_SIMENV_ENABLE_UNITREE_MOVE_BASE_BRIDGE=1
export OFFICIAL_SIMENV_ENABLE_POINTCLOUD_RELAY=0
export OFFICIAL_SIMENV_ENABLE_TRUNK_IMU_RELAY=1

mkdir -p "$STATE_DIR" "$RESULT_ROOT"

platform_up() {
  cd "$PLATFORM"
  SEED="${SEED:-20260728}" \
  ENABLE_LIDAR=true \
  ENABLE_LIVOX_3D=false \
  START_UNITREE_MOVE_BASE=1 \
  START_CONTROLLER=1 \
  SIMENV_AUTO_RL=1 \
  SIMENV_HEADLESS_MODE=move_base \
  START_ROSBRIDGE=1 \
  START_ODOM_RELAY=1 \
  START_BUILDING_CONTROL=1 \
  PAUSED=true \
  ./auto_docker.sh up
}

platform_down() {
  cd "$PLATFORM"
  ./auto_docker.sh first_person down || true
  ./auto_docker.sh gui down || true
  ./auto_docker.sh down
}

build_if_needed() {
  if [[ -f "$ROOT/install/setup.bash" ]]; then
    return
  fi
  env -u COLCON_CURRENT_PREFIX bash -lc \
    "cd '$ROOT' && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"
}

run_foreground() {
  build_if_needed
  local run_id="stable_three_floor_$(date +%Y%m%d_%H%M%S)"
  local work_output="$ROOT/reports/nav/$run_id"
  local final_output="$RESULT_ROOT/$run_id"
  local runner_pid=''
  forward_stop() {
    [[ -n "$runner_pid" ]] && kill -TERM "$runner_pid" 2>/dev/null || true
  }
  trap forward_stop TERM INT
  set +e
  env -u COLCON_CURRENT_PREFIX bash -lc \
    "cd '$ROOT' && export ROS_DOMAIN_ID=43 ROSBRIDGE_PORT=9091 DOCKER_SIMENV_USER=station_cluster OFFICIAL_SIMENV_ROSBRIDGE_URL=ws://127.0.0.1:9091 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && exec python3 scripts/run_official_slam_exploration.py --seed '${SEED:-20260728}' --output-dir '$work_output' --wall-timeout-sec 18000 --exploration-timeout-sec 1500 --mission-time-budget-sec 1500 --target-floors 0,1,2 --per-floor-exploration-sec 480 --truth-file '$PLATFORM/results/danger_truth.json'" &
  runner_pid="$!"
  wait "$runner_pid"
  local rc=$?
  if kill -0 "$runner_pid" 2>/dev/null; then
    wait "$runner_pid"
    rc=$?
  fi
  set -e
  trap - TERM INT
  if [[ -d "$work_output" ]]; then
    mv "$work_output" "$final_output"
    printf '结果目录：%s\n' "$final_output"
  fi
  return "$rc"
}

start_background() {
  if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    printf '稳定导航已经运行，PID=%s\n' "$(cat "$PID_FILE")"
    return
  fi
  platform_up
  nohup "$0" run >"$LOG_FILE" 2>&1 < /dev/null &
  printf '%s\n' "$!" > "$PID_FILE"
  printf '稳定三层导航已启动，PID=%s，日志=%s\n' "$!" "$LOG_FILE"
}

stop_all() {
  if [[ -f "$PID_FILE" ]]; then
    local pid
    pid="$(cat "$PID_FILE")"
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid"
      for _ in $(seq 1 60); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 1
      done
      if kill -0 "$pid" 2>/dev/null; then
        printf '导航监督进程仍在清理，请稍后再次执行 stop。\n' >&2
        return 1
      fi
    fi
    rm -f "$PID_FILE"
  fi
  platform_down
}

show_status() {
  if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    printf '导航：运行中 PID=%s\n' "$(cat "$PID_FILE")"
  else
    printf '导航：未运行\n'
  fi
  cd "$PLATFORM"
  ./auto_docker.sh status || true
  [[ -f "$LOG_FILE" ]] && tail -n 20 "$LOG_FILE" || true
}

case "${1:-}" in
  start) start_background ;;
  run) run_foreground ;;
  status) show_status ;;
  stop) stop_all ;;
  gui) cd "$PLATFORM" && ./auto_docker.sh gui up ;;
  first-person) cd "$PLATFORM" && ./auto_docker.sh first_person up ;;
  *)
    printf '用法：%s {start|status|stop|gui|first-person}\n' "$0"
    exit 2
    ;;
esac
