#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_PATH="$ROOT/stable_navigation.sh"
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

restore_runtime_git_state() {
  git -C "$ROOT" restore --worktree -- \
    ros2_ws/src/hazardwalker_platform/generated_building \
    ros2_ws/src/hazardwalker_platform/results/danger_truth.json \
    2>/dev/null || true
}

platform_up() {
  restore_runtime_git_state
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
  if [[ -x "$PLATFORM/docker/first_person_client.sh" ]]; then
    ./auto_docker.sh first_person down || true
  fi
  ./auto_docker.sh gui down || true
  ./auto_docker.sh down
  restore_runtime_git_state
}

build_if_needed() {
  if [[ ! -f "$ROOT/install/setup.bash" ]]; then
    env -u COLCON_CURRENT_PREFIX bash -lc \
      "cd '$ROOT' && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"
  fi
  if [[ ! -e "$ROOT/ros2_ws/install" ]]; then
    ln -s ../install "$ROOT/ros2_ws/install"
  fi
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
  restore_runtime_git_state
  return "$rc"
}

start_background() {
  if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    printf '稳定导航已经运行，PID=%s\n' "$(cat "$PID_FILE")"
    return
  fi
  build_if_needed
  platform_up
  nohup "$SCRIPT_PATH" run >"$LOG_FILE" 2>&1 < /dev/null &
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
