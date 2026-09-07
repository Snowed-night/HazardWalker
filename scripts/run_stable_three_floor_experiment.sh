#!/usr/bin/env bash
# 负责人固定的三层正式实验入口：复用已验收单层算法，统一管理平台、录像和收尾。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLATFORM="$ROOT/ros2_ws/src/hazardwalker_platform"
OUTPUT="${1:-$ROOT/reports/nav/three_floor_stable_current}"
SEED="${HAZARDWALKER_SEED:-20260728}"
export DOCKER_SIMENV_USER="${DOCKER_SIMENV_USER:-station_cluster}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"

if pgrep -f 'python3 scripts/run_official_slam_exploration.py' >/dev/null; then
  echo 'ERROR: 已有正式探索运行器，拒绝重复启动。' >&2
  exit 2
fi
if [[ -e "$OUTPUT" ]]; then
  echo "ERROR: 输出目录已存在，拒绝覆盖：$OUTPUT" >&2
  exit 3
fi

# 同机多套 Gazebo 会显著降低实时倍率，并曾使 A1 在启动阶段无法站立。
# 正式成绩只允许独占运行；本检查只报告并退出，绝不停止别人的容器。
mapfile -t active_simenv < <(
  docker ps --format '{{.Names}}' | grep -E '^(simenv|hazardwalker)' || true
)
if (( ${#active_simenv[@]} > 0 )); then
  printf 'ERROR: 服务器已有仿真容器，正式实验拒绝并行：%s\n' \
    "${active_simenv[*]}" >&2
  exit 4
fi

platform_started=0
cleanup() {
  if (( platform_started == 1 )); then
    cd "$PLATFORM"
    ./auto_docker.sh down || true
  fi
}
trap cleanup EXIT INT TERM

cd "$PLATFORM"
SEED="$SEED" \
FLOOR_COUNT=3 \
ROOMS_PER_FLOOR=4 \
DANGER_COUNT=3:6 \
DISTRACTOR_COUNT=4:8 \
ENABLE_LIDAR=true \
ENABLE_LIVOX_3D=false \
START_CONTROLLER=1 \
SIMENV_AUTO_RL=1 \
SIMENV_HEADLESS_MODE=move_base \
UNITREE_CTRL_DT=0.004 \
VERIFY_CONTROLLER_MOTION=0 \
START_ROSBRIDGE=1 \
START_ODOM_RELAY=1 \
START_BUILDING_CONTROL=1 \
START_UNITREE_MOVE_BASE=1 \
PAUSED=true \
./auto_docker.sh up
platform_started=1

set +u
unset COLCON_CURRENT_PREFIX
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
cd "$ROOT"

python3 scripts/run_official_slam_exploration.py \
  --seed "$SEED" \
  --output-dir "$OUTPUT" \
  --wall-timeout-sec 7200 \
  --exploration-timeout-sec 1200 \
  --mission-time-budget-sec 1200 \
  --target-floors 0,1,2 \
  --per-floor-exploration-sec 480 \
  --enable-perception \
  --truth-file "$PLATFORM/results/danger_truth.json"
