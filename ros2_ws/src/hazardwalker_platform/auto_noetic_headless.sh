#!/usr/bin/env bash
# ROS 1 Noetic entry for SimEnv inside Docker (Gazebo Classic + optional junior_ctrl).
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

SEED="${SEED:-}"
FLOOR_COUNT="${FLOOR_COUNT:-3}"
ROOMS_PER_FLOOR="${ROOMS_PER_FLOOR:-4}"
BUILDING_WIDTH="${BUILDING_WIDTH:-20.0}"
BUILDING_LENGTH="${BUILDING_LENGTH:-36.0}"
DANGER_COUNT="${DANGER_COUNT:-3:6}"
DISTRACTOR_COUNT="${DISTRACTOR_COUNT:-4:8}"
GUI="${GUI:-false}"
PAUSED="${PAUSED:-false}"
START_CONTROLLER="${START_CONTROLLER:-0}"
START_VIRTUAL_JOY="${START_VIRTUAL_JOY:-0}"
CONTROLLER_FOREGROUND="${CONTROLLER_FOREGROUND:-0}"
START_BUILDING_CONTROL="${START_BUILDING_CONTROL:-1}"
UNITREE_CTRL_DT="${UNITREE_CTRL_DT:-0.004}"
ROBOT_X="${ROBOT_X:-0.0}"
ROBOT_Y="${ROBOT_Y:--2.2}"
ROBOT_Z="${ROBOT_Z:-0.6}"
ROBOT_YAW="${ROBOT_YAW:-1.5708}"

echo "Terminating previous Gazebo / controller old processes..."
pkill -f "roslaunch unitree_guide multi_floor_gazeboSim.launch" 2>/dev/null || true
pkill -f "building_generator_classic_control" 2>/dev/null || true
pkill -f "gzserver|gzclient|gazebo" 2>/dev/null || true
pkill -f "junior_ctrl" 2>/dev/null || true
pkill -f "virtual_joy.py" 2>/dev/null || true
# 仅启动前清理旧relay，不再设置退出自动杀死
pkill -f "topic_tools relay" 2>/dev/null || true

# Platform devel soft link
LEGACY_WS="/home/ros/Guoyulun/Competition/SimEnv"
mkdir -p "$(dirname "$LEGACY_WS")"
if [[ "$(readlink -f "$WORKSPACE_DIR")" != "$(readlink -f "$LEGACY_WS" 2>/dev/null || echo __missing__)" ]]; then
  ln -sfn "$WORKSPACE_DIR" "$LEGACY_WS"
fi

if [[ ! -f "$WORKSPACE_DIR/devel/setup.bash" ]]; then
  echo "ERROR: devel/setup.bash not found."
  echo "Run on host first: ./auto_docker.sh build"
  exit 1
fi

source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "$WORKSPACE_DIR/devel/setup.bash"
export GAZEBO_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${LD_LIBRARY_PATH:-}"

BUILDING_OBSTACLES_DIR="$(rospack find building_obstacles)"
UNITREE_GAZEBO_MODELS="$(rospack find unitree_gazebo)/models"
SCENE_OUTPUT_DIR="$WORKSPACE_DIR/generated_building"
RESULTS_DIR="$WORKSPACE_DIR/results"
mkdir -p "$SCENE_OUTPUT_DIR" "$RESULTS_DIR" "$WORKSPACE_DIR/logs"

echo "Generating competition scene..."
GENERATOR_ARGS=(
  --output-dir "$SCENE_OUTPUT_DIR"
  --results-dir "$RESULTS_DIR"
  --floor-count "$FLOOR_COUNT"
  --rooms-per-floor "$ROOMS_PER_FLOOR"
  --width "$BUILDING_WIDTH"
  --length "$BUILDING_LENGTH"
  --danger-count "$DANGER_COUNT"
  --distractor-count "$DISTRACTOR_COUNT"
  --robot-x "$ROBOT_X"
  --robot-y "$ROBOT_Y"
  --robot-z "$ROBOT_Z"
  --robot-yaw "$ROBOT_YAW"
)
if [[ -n "$SEED" ]]; then
  GENERATOR_ARGS+=(--seed "$SEED")
fi
python3 "$BUILDING_OBSTACLES_DIR/scripts/generate_competition_scene.py" "${GENERATOR_ARGS[@]}" \
  > "$SCENE_OUTPUT_DIR/scene_manifest.stdout.json"

export BUILDING_WORLD_FILE="$SCENE_OUTPUT_DIR/competition_scene.world"
export COMPETITION_ROBOT_X="$ROBOT_X"
export COMPETITION_ROBOT_Y="$ROBOT_Y"
export COMPETITION_ROBOT_Z="$ROBOT_Z"
export COMPETITION_ROBOT_YAW="$ROBOT_YAW"
export UNITREE_CTRL_DT
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:$SCENE_OUTPUT_DIR:$UNITREE_GAZEBO_MODELS"

echo "=========================================="
echo "Competition scene ready (Noetic / Classic)"
echo "  World:   $BUILDING_WORLD_FILE"
echo "  Truth:   $RESULTS_DIR/danger_truth.json"
echo "  Result:  $RESULTS_DIR/detected_danger.json"
echo "=========================================="

if [[ "$START_VIRTUAL_JOY" == "1" ]]; then
  rosrun unitree_guide virtual_joy.py > "$WORKSPACE_DIR/logs/virtual_joy.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/virtual_joy.pid"
fi

# ---- headless OpenGL: Xvfb虚拟显示（相机/LiDAR渲染必需） ----
# docker restart 会保留 /tmp；失效的 X 锁若不清理，Gazebo 虽能启动却不会加载
# 相机插件。与正式 auto.sh 保持同一 fail-fast 合同。
DISPLAY_VALUE="${SIMENV_HEADLESS_DISPLAY:-${DISPLAY:-:99}}"
DISPLAY_NUMBER="${DISPLAY_VALUE#:}"
DISPLAY_LOCK="/tmp/.X${DISPLAY_NUMBER}-lock"
DISPLAY_SOCKET="/tmp/.X11-unix/X${DISPLAY_NUMBER}"
XVFB_LOG="$WORKSPACE_DIR/logs/xvfb.log"

display_is_ready() {
  if command -v xdpyinfo >/dev/null 2>&1; then
    timeout 1 xdpyinfo -display "$DISPLAY_VALUE" >/dev/null 2>&1
  else
    [ -S "$DISPLAY_SOCKET" ] && pgrep -f "Xvfb ${DISPLAY_VALUE}" >/dev/null 2>&1
  fi
}

if ! display_is_ready; then
  if [ -f "$DISPLAY_LOCK" ]; then
    LOCK_PID="$(tr -dc '0-9' < "$DISPLAY_LOCK")"
    if [ -n "$LOCK_PID" ] && kill -0 "$LOCK_PID" 2>/dev/null; then
      echo "DISPLAY=$DISPLAY_VALUE lock owner PID $LOCK_PID is alive but unreachable." >&2
      exit 1
    fi
    rm -f "$DISPLAY_LOCK" "$DISPLAY_SOCKET"
  elif [ -e "$DISPLAY_SOCKET" ]; then
    rm -f "$DISPLAY_SOCKET"
  fi
  Xvfb "$DISPLAY_VALUE" -screen 0 1280x1024x24 +extension GLX +render \
    > "$XVFB_LOG" 2>&1 &
  XVFB_PID=$!
  DISPLAY_READY=0
  for _ in $(seq 1 50); do
    if ! kill -0 "$XVFB_PID" 2>/dev/null; then
      break
    fi
    if display_is_ready; then
      DISPLAY_READY=1
      break
    fi
    sleep 0.1
  done
  if [ "$DISPLAY_READY" != "1" ]; then
    echo "Xvfb failed on DISPLAY=$DISPLAY_VALUE; see $XVFB_LOG." >&2
    tail -20 "$XVFB_LOG" >&2 || true
    exit 1
  fi
fi
export DISPLAY="$DISPLAY_VALUE"
export LIBGL_ALWAYS_SOFTWARE=1
echo "Xvfb ready on DISPLAY=$DISPLAY"

echo "Launching Gazebo Classic + Unitree A1 (gui=$GUI, paused=$PAUSED)..."
roslaunch unitree_guide multi_floor_gazeboSim.launch \
  gui:="$GUI" \
  paused:="$PAUSED" \
  user_debug:=False \
  rname:=a1 \
  robot_x:="$ROBOT_X" \
  robot_y:="$ROBOT_Y" \
  robot_z:="$ROBOT_Z" \
  robot_yaw:="$ROBOT_YAW" \
  > "$WORKSPACE_DIR/logs/competition_gazebo.log" 2>&1 &
echo $! > "$WORKSPACE_DIR/logs/competition_gazebo.pid"

# 等待30秒，保证Gazebo、雷达、里程话题全部加载完成
echo "Wait 30 seconds for all gazebo sensor topics to spawn..."
sleep 30


# 建筑门梯控制节点
if [[ "$START_BUILDING_CONTROL" == "1" ]]; then
  rosrun building_generator_classic building_generator_classic_control \
    --door-config "$SCENE_OUTPUT_DIR/door_config.yaml" \
    --elevator-config "$SCENE_OUTPUT_DIR/elevator_config.yaml" \
    > "$WORKSPACE_DIR/logs/building_control.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/building_control.pid"
fi
# 机器人控制器 (headless 自动站立 + RL模式)
if [[ "$START_CONTROLLER" == "1" ]]; then
  echo "Starting junior_ctrl with auto-stand and RL mode (headless expect)..."
  MODEL_DIR="$WORKSPACE_DIR/src/unitree_guide/logs"
  mkdir -p "$MODEL_DIR"
  if [[ ! -f "$MODEL_DIR/policy_act_inference_stair.pt" ]]; then
    SIMENV_MODEL="/home/ros/Guoyulun/Competition/SimEnv/src/unitree_guide/logs/policy_act_inference_stair.pt"
    if [[ -f "$SIMENV_MODEL" ]]; then
      cp "$SIMENV_MODEL" "$MODEL_DIR/"
      echo "RL model copied from SimEnv"
    else
      echo "WARNING: RL model not found"
    fi
  fi
  apt-get update -qq 2>/dev/null
  apt-get install -y -qq expect 2>/dev/null
  expect -c "
    set timeout -1
    spawn $WORKSPACE_DIR/devel/lib/unitree_guide/junior_ctrl
    sleep 5
    send "2\r"
    sleep 2
    send "6\r"
    sleep 2
    expect eof
  " > "$WORKSPACE_DIR/logs/junior_ctrl.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/junior_ctrl.pid"
  echo "junior_ctrl started with auto RL mode"
fi

echo "Noetic simulation startup completed, script keep running to hold gazebo and control services."
# 关键：无限阻塞循环，脚本不会退出，relay进程不会被销毁
while true; do
  sleep 3600
done
