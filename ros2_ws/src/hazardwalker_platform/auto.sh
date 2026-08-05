#!/usr/bin/env bash
# 官方 SimEnv Docker 的唯一正式 ROS1 启动链路。
# 依次启动 Gazebo、控制器、/hazardwalker/odom 最新值中继和 rosbridge；本脚本不把
# Gazebo 里程计包装成 SLAM 真值，/hazardwalker/odom 仅供平台诊断适配。
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CATKIN_WORKSPACE_DIR="${CATKIN_WORKSPACE_DIR:-$WORKSPACE_DIR/.ros1_catkin_ws}"
CATKIN_DEVEL_DIR="$CATKIN_WORKSPACE_DIR/devel"
cd "$WORKSPACE_DIR"
RUNTIME_READY_FILE="/tmp/hazardwalker-simenv-runtime-ready"
# 健康检查只能在本轮完整启动结束后通过；先清除同容器上轮残留标记。
rm -f "$RUNTIME_READY_FILE"

SEED="${SEED:-}"
FLOOR_COUNT="${FLOOR_COUNT:-3}"
ROOMS_PER_FLOOR="${ROOMS_PER_FLOOR:-4}"
BUILDING_WIDTH="${BUILDING_WIDTH:-20.0}"
BUILDING_LENGTH="${BUILDING_LENGTH:-36.0}"
DANGER_COUNT="${DANGER_COUNT:-3:6}"
DISTRACTOR_COUNT="${DISTRACTOR_COUNT:-4:8}"
GUI="${GUI:-true}"
PAUSED="${PAUSED:-true}"
START_CONTROLLER="${START_CONTROLLER:-1}"
START_VIRTUAL_JOY="${START_VIRTUAL_JOY:-0}"
START_ROSBRIDGE="${START_ROSBRIDGE:-1}"
START_ODOM_RELAY="${START_ODOM_RELAY:-1}"
CONTROLLER_FOREGROUND="${CONTROLLER_FOREGROUND:-0}"
SIMENV_AUTO_RL="${SIMENV_AUTO_RL:-1}"
# 已编译的官方控制器仅在 move_base headless 模式下把 /cmd_vel 接入 RL 行走状态。
# 不能只设置 SIMENV_AUTO_RL，否则节点会存在却不对速度命令产生有效运动。
SIMENV_HEADLESS_MODE="${SIMENV_HEADLESS_MODE:-move_base}"
START_BUILDING_CONTROL="${START_BUILDING_CONTROL:-1}"
UNITREE_CTRL_DT="${UNITREE_CTRL_DT:-0.004}"
AUTO_UNPAUSE_AFTER_CONTROLLER="${AUTO_UNPAUSE_AFTER_CONTROLLER:-1}"
CONTROLLER_SENSOR_READY_TIMEOUT_SEC="${CONTROLLER_SENSOR_READY_TIMEOUT_SEC:-5}"
# GUI=false 只禁用 gzclient；相机与激光的 headless 渲染仍由 Xvfb 提供软件 GL。
GAZEBO_HEADLESS="${GAZEBO_HEADLESS:-false}"
# 默认不加载激光雷达：感知组的 RGB-D 与平台控制不依赖 /scan，关闭可显著降低
# Gazebo 图形与物理负载。导航/SLAM 测试显式设置 ENABLE_LIDAR=true。
ENABLE_LIDAR="${ENABLE_LIDAR:-false}"
# 第一人称和感知均复用 RealSense RGB。官方插件默认仅 2 Hz，浏览器画面会明显跳帧；
# 设为 10 Hz 以匹配当前低负载 profile。JPEG 质量仅在压缩话题有订阅者时生效。
CAMERA_IMAGER_RATE_HZ="${CAMERA_IMAGER_RATE_HZ:-10}"
CAMERA_JPEG_QUALITY="${CAMERA_JPEG_QUALITY:-92}"
# 后台控制器必须完成 FSM 初始化后再解除暂停，避免 A1 在接管前跌倒。
CONTROLLER_READY_TIMEOUT_SEC="${CONTROLLER_READY_TIMEOUT_SEC:-60}"
# 自动控制器使用当前已编译 Unitree 控制器的 SIMENV_AUTO_RL headless 模式。
# 伪终端只为兼容其键盘接口；不能在物理暂停前等待“状态切换”日志，否则状态机没有
# 仿真步进机会而会形成启动死锁。
CONTROLLER_AUTO_STAND_DELAY_SEC="${CONTROLLER_AUTO_STAND_DELAY_SEC:-5}"
CONTROLLER_AUTO_RL_DELAY_SEC="${CONTROLLER_AUTO_RL_DELAY_SEC:-2}"
VERIFY_CONTROLLER_MOTION="${VERIFY_CONTROLLER_MOTION:-0}"
controller_motion_verified=0
CONTROLLER_RL_SETTLE_SEC="${CONTROLLER_RL_SETTLE_SEC:-3.0}"
CONTROLLER_PROBE_SPEED_MPS="${CONTROLLER_PROBE_SPEED_MPS:-0.30}"
CONTROLLER_PROBE_DURATION_SEC="${CONTROLLER_PROBE_DURATION_SEC:-3.0}"
CONTROLLER_PROBE_MIN_DISPLACEMENT_M="${CONTROLLER_PROBE_MIN_DISPLACEMENT_M:-0.05}"
CONTROLLER_PROBE_MAX_DISPLACEMENT_M="${CONTROLLER_PROBE_MAX_DISPLACEMENT_M:-1.00}"
CONTROLLER_PROBE_MIN_BASE_HEIGHT_M="${CONTROLLER_PROBE_MIN_BASE_HEIGHT_M:-0.30}"
# 等仿真时钟稳定后再启动 rosbridge，避免新客户端收到启动期陈旧队列。
ROSBRIDGE_START_AFTER_SIM_TIME_SEC="${ROSBRIDGE_START_AFTER_SIM_TIME_SEC:-1}"
ROBOT_X="${ROBOT_X:-0.0}"
ROBOT_Y="${ROBOT_Y:--2.2}"
ROBOT_Z="${ROBOT_Z:-0.6}"
ROBOT_YAW="${ROBOT_YAW:-1.5708}"

echo "Terminating previous Gazebo, launch, controller, and optional joystick processes..."
pkill -f "roslaunch unitree_guide multi_floor_gazeboSim.launch" 2>/dev/null || true
pkill -f "building_generator_classic_control" 2>/dev/null || true
pkill -f "gzserver|gzclient|gazebo" 2>/dev/null || true
pkill -f "junior_ctrl" 2>/dev/null || true
pkill -f "virtual_joy.py" 2>/dev/null || true
pkill -f "rosbridge_odom_relay.py" 2>/dev/null || true

echo "Sourcing ROS environment..."
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
source /opt/ros/noetic/setup.bash
if [[ ! -f "$CATKIN_DEVEL_DIR/setup.bash" ]]; then
  echo "ERROR: missing $CATKIN_DEVEL_DIR/setup.bash." >&2
  echo "Run './auto_docker.sh build' from the host before starting this container." >&2
  exit 66
fi
source "$CATKIN_DEVEL_DIR/setup.bash"

# ROS 的 OpenNI Kinect 包依赖 Gazebo Classic 自带的 DepthCameraPlugin。
# 该目录默认在 GAZEBO_PLUGIN_PATH，却不一定进入动态链接器搜索路径；缺失时
# Gazebo 仍会生成内部 image topic，但 ROS 的 RGB/深度/点云话题完全不存在。
GAZEBO_CLASSIC_PLUGIN_DIR="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
if [ -d "$GAZEBO_CLASSIC_PLUGIN_DIR" ]; then
  export GAZEBO_PLUGIN_PATH="$CATKIN_DEVEL_DIR/lib:/opt/ros/noetic/lib:$GAZEBO_CLASSIC_PLUGIN_DIR:${GAZEBO_PLUGIN_PATH:-}"
  export LD_LIBRARY_PATH="$GAZEBO_CLASSIC_PLUGIN_DIR:${LD_LIBRARY_PATH:-}"
fi

# pip/nvidia 拆分安装把 CUDA 运行库放在各自包目录，而不一定在
# /usr/local/cuda/lib64。junior_ctrl 依赖这些动态库；逐个加入实际存在的目录，
# 避免控制器在物理解除暂停前因 libcublas/libcudart 缺失退出。
for CUDA_RUNTIME_DIR in \
  /usr/local/lib/python3.8/dist-packages/nvidia/cublas/lib \
  /usr/local/lib/python3.8/dist-packages/nvidia/cuda_runtime/lib \
  /usr/local/lib/python3.8/dist-packages/nvidia/nvtx/lib; do
  if [ -d "$CUDA_RUNTIME_DIR" ]; then
    export LD_LIBRARY_PATH="$CUDA_RUNTIME_DIR:${LD_LIBRARY_PATH:-}"
  fi
done

BUILDING_OBSTACLES_DIR="$(rospack find building_obstacles)"
UNITREE_GAZEBO_MODELS="$(rospack find unitree_gazebo)/models"
SCENE_OUTPUT_DIR="$WORKSPACE_DIR/generated_building"
RESULTS_DIR="${RESULTS_DIR:-$WORKSPACE_DIR/results}"
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
if [ -n "$SEED" ]; then
  GENERATOR_ARGS+=(--seed "$SEED")
fi
if [ "${SIMENV_PRACTICE:-0}" = "1" ]; then
  "$CATKIN_DEVEL_DIR/lib/building_obstacles/generate_competition_scene.py" "${GENERATOR_ARGS[@]}" \
    > "$SCENE_OUTPUT_DIR/scene_manifest.stdout.json"
else
  python3 "$BUILDING_OBSTACLES_DIR/scripts/generate_competition_scene.py" "${GENERATOR_ARGS[@]}" \
    > "$SCENE_OUTPUT_DIR/scene_manifest.stdout.json"
fi

export BUILDING_WORLD_FILE="$SCENE_OUTPUT_DIR/competition_scene.world"
export COMPETITION_ROBOT_X="$ROBOT_X"
export COMPETITION_ROBOT_Y="$ROBOT_Y"
export COMPETITION_ROBOT_Z="$ROBOT_Z"
export COMPETITION_ROBOT_YAW="$ROBOT_YAW"
export UNITREE_CTRL_DT
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:$SCENE_OUTPUT_DIR:$UNITREE_GAZEBO_MODELS"

echo "=========================================="
echo "Competition scene is ready"
echo "  World:   $BUILDING_WORLD_FILE"
echo "  Truth:   $RESULTS_DIR/danger_truth.json"
echo "  Manifest:$SCENE_OUTPUT_DIR/scene_manifest.json"
echo "  Result:  $RESULTS_DIR/detected_danger.json"
echo "=========================================="

if [ "$START_VIRTUAL_JOY" = "1" ]; then
  echo "Starting virtual joystick. This may require uinput permissions."
  rosrun unitree_guide virtual_joy.py > "$WORKSPACE_DIR/logs/virtual_joy.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/virtual_joy.pid"
fi

# ---- headless OpenGL：相机/LiDAR 渲染仍需要虚拟显示 ----
# auto_headless.sh 与本脚本必须共享同一个显示所有权；docker restart 会保留
# writable-layer /tmp，若旧 Xvfb 已死而 .X*-lock/socket 仍在，Gazebo 会静默
# 跳过 RGB-D 插件。这里只清理“锁内 PID 已不存在”的本显示残留，绝不误杀
# 其他显示；启动错误写入日志并 fail-fast。
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

if display_is_ready; then
  echo "Reusing live Xvfb on DISPLAY=$DISPLAY_VALUE"
else
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

echo "Launching Gazebo, Unitree A1 model, sensors, and ROS interfaces..."
roslaunch unitree_guide multi_floor_gazeboSim.launch \
  gui:="$GUI" \
  headless:="$GAZEBO_HEADLESS" \
  paused:="$PAUSED" \
  enable_lidar:="$ENABLE_LIDAR" \
  user_debug:=False \
  rname:=a1 \
  robot_x:="$ROBOT_X" \
  robot_y:="$ROBOT_Y" \
  robot_z:="$ROBOT_Z" \
  robot_yaw:="$ROBOT_YAW" \
  > "$WORKSPACE_DIR/logs/competition_gazebo.log" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$WORKSPACE_DIR/logs/competition_gazebo.pid"

# paused 模式同样必须等 Gazebo 服务真正注册；固定 sleep 会在冷启动或重启后偶发
# 早于 gzserver 就绪，继而使解除暂停失败并退出整个容器。
gazebo_ready_deadline=$((SECONDS + CONTROLLER_READY_TIMEOUT_SEC))
until rosservice info /gazebo/unpause_physics >/dev/null 2>&1; do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "Gazebo launch exited before /gazebo/unpause_physics became available." >&2
    exit 1
  fi
  if [ "$SECONDS" -ge "$gazebo_ready_deadline" ]; then
    echo "Timed out waiting for /gazebo/unpause_physics." >&2
    exit 1
  fi
  sleep 0.2
done

# OpenNI/RealSense 的 imager_rate 属于动态参数，xacro 内 updateRate 不能覆盖其
# 默认 2 Hz。等待插件服务注册后显式设置，避免第一人称画面因低帧率产生闪烁感。
camera_config_deadline=$((SECONDS + CONTROLLER_READY_TIMEOUT_SEC))
until rosservice info /real_sense/set_parameters >/dev/null 2>&1; do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "Gazebo launch exited before RealSense dynamic parameters became available." >&2
    exit 1
  fi
  if [ "$SECONDS" -ge "$camera_config_deadline" ]; then
    echo "Timed out waiting for RealSense dynamic parameters." >&2
    exit 1
  fi
  sleep 0.2
done
rosrun dynamic_reconfigure dynparam set /real_sense imager_rate "$CAMERA_IMAGER_RATE_HZ" >/dev/null
rosparam set /real_sense/rgb/image_raw/compressed/jpeg_quality "$CAMERA_JPEG_QUALITY"
rosparam set /real_sense/rgb/image_raw/compressed/jpeg_progressive false
rosparam set /real_sense/rgb/image_raw/compressed/jpeg_optimize true
echo "RealSense RGB configured: imager_rate=${CAMERA_IMAGER_RATE_HZ} Hz, jpeg_quality=${CAMERA_JPEG_QUALITY}."

# 非暂停启动时先取得真实关节状态；暂停 profile 只需让服务完成注册。
if [ "$START_CONTROLLER" = "1" ] && [ "$PAUSED" != "true" ]; then
  echo "Waiting for the first A1 joint-state sample before starting controller..."
  timeout "$CONTROLLER_SENSOR_READY_TIMEOUT_SEC" \
    rostopic echo -n 1 "/a1_gazebo/FL_hip_controller/state" >/dev/null || {
      echo "A1 joint state was not ready; refusing uninitialized controller startup." >&2
      exit 1
    }
else
  sleep 1
fi

if [ "$START_BUILDING_CONTROL" = "1" ]; then
  echo "Starting building door/elevator control service..."
  rosrun building_generator_classic building_generator_classic_control \
    --door-config "$SCENE_OUTPUT_DIR/door_config.yaml" \
    --elevator-config "$SCENE_OUTPUT_DIR/elevator_config.yaml" \
    > "$WORKSPACE_DIR/logs/building_control.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/building_control.pid"
fi

if [ "$START_CONTROLLER" = "1" ]; then
  if [ "$CONTROLLER_FOREGROUND" = "1" ]; then
    echo "Starting junior_ctrl controller in the foreground."
    echo "UNITREE_CTRL_DT=$UNITREE_CTRL_DT seconds."
    echo "Use keyboard input in this terminal: 2 = stand, 6 = RL mode."
    "$CATKIN_DEVEL_DIR/lib/unitree_guide/junior_ctrl"
  elif [ "$SIMENV_HEADLESS_MODE" = "move_base" ]; then
    if ! command -v expect >/dev/null 2>&1; then
      echo "expect is missing; the image is incomplete and cannot start the headless controller." >&2
      exit 1
    fi
    export CONTROLLER_BINARY="$CATKIN_DEVEL_DIR/lib/unitree_guide/junior_ctrl"
    export SIMENV_HEADLESS_MODE
    export CONTROLLER_AUTO_STAND_DELAY_SEC CONTROLLER_AUTO_RL_DELAY_SEC
    echo "Starting junior_ctrl with formal headless controller mode..."
    expect -c '
      set timeout -1
      log_user 1
      # Debian/Noetic 自带 expect 不接受 spawn 的 GNU 风格 "--"；直接传绝对路径。
      spawn -noecho $env(CONTROLLER_BINARY)
      after [expr {$env(CONTROLLER_AUTO_STAND_DELAY_SEC) * 1000}]
      send -- "2\r"
      after [expr {$env(CONTROLLER_AUTO_RL_DELAY_SEC) * 1000}]
      send -- "6\r"
      expect eof
    ' > "$WORKSPACE_DIR/logs/junior_ctrl.log" 2>&1 &
    echo $! > "$WORKSPACE_DIR/logs/junior_ctrl.pid"

    controller_deadline=$((SECONDS + CONTROLLER_READY_TIMEOUT_SEC))
    until grep -Eq '\[HEADLESS_FSM\].*mode=move_base.*auto_rl=[01]' \
        "$WORKSPACE_DIR/logs/junior_ctrl.log" 2>/dev/null; do
      if ! kill -0 "$(cat "$WORKSPACE_DIR/logs/junior_ctrl.pid")" 2>/dev/null; then
        echo "junior_ctrl exited before entering headless controller mode; see logs/junior_ctrl.log." >&2
        exit 1
      fi
      if [ "$SECONDS" -ge "$controller_deadline" ]; then
        echo "Timed out waiting for junior_ctrl headless controller mode; refusing control-ready status." >&2
        exit 1
      fi
      sleep 0.2
    done
    # 日志只证明控制器解析了 headless 参数；若节点随后退出、重名节点尚未注册，
    # 或 move_base 订阅没有建立，/cmd_vel 仍会显示 Subscribers: None。正式入口
    # 必须同时验证 ROS 图中的真实订阅者，不能把一行旧日志当作控制就绪证据。
    until rostopic info /cmd_vel 2>/dev/null \
        | grep -Eq '^[[:space:]]*\*[[:space:]]+/unitree_gazebo_servo[[:space:]]'; do
      if ! kill -0 "$(cat "$WORKSPACE_DIR/logs/junior_ctrl.pid")" 2>/dev/null; then
        echo "junior_ctrl exited before /cmd_vel subscription became ready." >&2
        exit 1
      fi
      if [ "$SECONDS" -ge "$controller_deadline" ]; then
        echo "Timed out waiting for /unitree_gazebo_servo to subscribe /cmd_vel." >&2
        exit 1
      fi
      sleep 0.2
    done
    echo "junior_ctrl headless controller and /cmd_vel subscription are ready; physics can now unpause."
  else
    echo "Background junior_ctrl requires SIMENV_HEADLESS_MODE=move_base for formal startup." >&2
    echo "Use CONTROLLER_FOREGROUND=1 for manual diagnosis, or set SIMENV_HEADLESS_MODE=move_base." >&2
    exit 1
  fi
fi

# 先控制、后解除物理暂停，防止无控制状态下从出生高度跌落。
if [ "$PAUSED" = "true" ] && [ "$START_CONTROLLER" = "1" ] \
    && [ "$AUTO_UNPAUSE_AFTER_CONTROLLER" = "1" ]; then
  echo "Controller launched while physics is paused; unpausing after controller warm-up..."
  sleep 1
  rosservice call /gazebo/unpause_physics >/dev/null || {
    echo "Failed to unpause Gazebo after controller startup." >&2
    exit 1
  }
fi

if [ "$START_CONTROLLER" = "1" ] && [ "$SIMENV_HEADLESS_MODE" = "move_base" ]; then
  # 正式静止验收：无命令时必须停在固定站立，不能把“进程存在”误报为已站稳。
  controller_state_deadline=$((SECONDS + CONTROLLER_READY_TIMEOUT_SEC))
  until grep -q 'Switched from passive to fixed stand' \
      "$WORKSPACE_DIR/logs/junior_ctrl.log" 2>/dev/null; do
    if ! kill -0 "$(cat "$WORKSPACE_DIR/logs/junior_ctrl.pid")" 2>/dev/null; then
      echo "junior_ctrl exited before entering fixed stand." >&2
      exit 1
    fi
    if [ "$SECONDS" -ge "$controller_state_deadline" ]; then
      echo "Timed out waiting for junior_ctrl fixed stand state." >&2
      exit 1
    fi
    sleep 0.2
  done
  echo "junior_ctrl fixed stand state is ready; non-zero /cmd_vel will switch to RL walking."
fi

# 仅保留给专门的动态控制诊断，不在正常启动时自动进入 RL。
if [ "$START_CONTROLLER" = "1" ] && [ "${CONTROLLER_DYNAMIC_RL_STARTUP:-0}" = "1" ] \
    && [ "$SIMENV_AUTO_RL" = "1" ]; then
  # 初始化日志只表示 headless 参数已生效；必须等状态机实际进入 RL，订阅者存在本身
  # 不能证明回调被处理。
  controller_state_deadline=$((SECONDS + CONTROLLER_READY_TIMEOUT_SEC))
  until grep -q 'Switched from fixed stand to RL' \
      "$WORKSPACE_DIR/logs/junior_ctrl.log" 2>/dev/null; do
    if ! kill -0 "$(cat "$WORKSPACE_DIR/logs/junior_ctrl.pid")" 2>/dev/null; then
      echo "junior_ctrl exited before entering the active RL state." >&2
      exit 1
    fi
    if [ "$SECONDS" -ge "$controller_state_deadline" ]; then
      echo "Timed out waiting for junior_ctrl to enter the active RL state." >&2
      exit 1
    fi
    sleep 0.2
  done

elif [ "$START_CONTROLLER" = "1" ] && [ "${CONTROLLER_DYNAMIC_RL_STARTUP:-0}" = "1" ] \
    && [ "$SIMENV_HEADLESS_MODE" = "move_base" ]; then
  # 标准步态模式：先完成固定站立，再切到可接收 /cmd_vel 的 move_base 状态。
  controller_state_deadline=$((SECONDS + CONTROLLER_READY_TIMEOUT_SEC))
  until grep -q 'Switched from fixed stand to move_base' \
      "$WORKSPACE_DIR/logs/junior_ctrl.log" 2>/dev/null; do
    if ! kill -0 "$(cat "$WORKSPACE_DIR/logs/junior_ctrl.pid")" 2>/dev/null; then
      echo "junior_ctrl exited before entering move_base state." >&2
      exit 1
    fi
    if [ "$SECONDS" -ge "$controller_state_deadline" ]; then
      echo "Timed out waiting for junior_ctrl to enter move_base state." >&2
      exit 1
    fi
    sleep 0.2
  done
  echo "junior_ctrl move_base state is ready; the robot is standing and /cmd_vel is active."

  if [ "$VERIFY_CONTROLLER_MOTION" = "1" ]; then
    # RL 推理线程需完成历史观测初始化；刚切换状态就发速度会被初始化阶段吞掉。
    sleep "$CONTROLLER_RL_SETTLE_SEC"
    read_odom_pose() {
      timeout 10 rostopic echo -n 1 /Odometry_gazebo |
        awk '
          /^pose:/{in_pose=1; next}
          in_pose && /^  pose:/{in_pose_pose=1; next}
          in_pose_pose && /^    position:/{in_position=1; next}
          in_position && /^      x:/{x=$2; next}
          in_position && /^      y:/{y=$2; next}
          in_position && /^      z:/{print x, y, $2; exit}
        '
    }
    read -r probe_x_before probe_y_before probe_z_before < <(read_odom_pose)
    python3 "$WORKSPACE_DIR/scripts/controller_motion_probe.py" \
      --speed-mps "$CONTROLLER_PROBE_SPEED_MPS" \
      --duration-sec "$CONTROLLER_PROBE_DURATION_SEC"
    sleep 1
    read -r probe_x_after probe_y_after probe_z_after < <(read_odom_pose)
    probe_displacement="$(
      python3 -c 'import math,sys; print(math.hypot(float(sys.argv[3])-float(sys.argv[1]), float(sys.argv[4])-float(sys.argv[2])))' \
        "$probe_x_before" "$probe_y_before" "$probe_x_after" "$probe_y_after"
    )"
    if ! python3 -c '
import sys
distance, minimum, maximum, height, min_height = map(float, sys.argv[1:])
raise SystemExit(0 if minimum <= distance <= maximum and height >= min_height else 1)
' "$probe_displacement" "$CONTROLLER_PROBE_MIN_DISPLACEMENT_M" \
        "$CONTROLLER_PROBE_MAX_DISPLACEMENT_M" "$probe_z_after" \
        "$CONTROLLER_PROBE_MIN_BASE_HEIGHT_M"; then
      echo "Controller probe failed: displacement=${probe_displacement}m, base_z=${probe_z_after}m." >&2
      exit 1
    fi
    echo "Controller physical /cmd_vel probe passed: ${probe_displacement}m, base_z=${probe_z_after}m."
    controller_motion_verified=1
  fi
fi

# 正式 rosbridge 必须在控制器、当前里程计和 /clock 稳定后启动。
if [ "$START_ROSBRIDGE" = "1" ]; then
  echo "Waiting for current odometry and stable simulation time before starting rosbridge..."
  timeout "$CONTROLLER_READY_TIMEOUT_SEC" rostopic echo -n 1 /Odometry_gazebo >/dev/null || {
    echo "Odometry did not become available; refusing stale rosbridge startup." >&2
    exit 1
  }
  rosbridge_deadline=$((SECONDS + CONTROLLER_READY_TIMEOUT_SEC))
  while true; do
    current_sim_sec="$(rostopic echo -n 1 /clock 2>/dev/null | awk '/secs:/{print $2; exit}')"
    if [ -n "$current_sim_sec" ] \
        && [ "$current_sim_sec" -ge "$ROSBRIDGE_START_AFTER_SIM_TIME_SEC" ]; then
      break
    fi
    if [ "$SECONDS" -ge "$rosbridge_deadline" ]; then
      echo "Simulation clock did not become stable; refusing rosbridge startup." >&2
      exit 1
    fi
    sleep 0.5
  done
  if [ "$START_ODOM_RELAY" = "1" ]; then
    python3 "$WORKSPACE_DIR/scripts/rosbridge_odom_relay.py" \
      --source /Odometry_gazebo --output /hazardwalker/odom --rate-hz 20 \
      > "$WORKSPACE_DIR/logs/rosbridge_odom_relay.log" 2>&1 &
    echo $! > "$WORKSPACE_DIR/logs/rosbridge_odom_relay.pid"
    timeout "$CONTROLLER_READY_TIMEOUT_SEC" \
      rostopic echo -n 1 /hazardwalker/odom >/dev/null || {
        echo "Latest-value odometry relay did not become available." >&2
        exit 1
      }
  fi
  roslaunch rosbridge_server rosbridge_websocket.launch \
    > "$WORKSPACE_DIR/logs/rosbridge.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/rosbridge.pid"
  timeout "$CONTROLLER_READY_TIMEOUT_SEC" bash -lc \
    'until rosnode ping -c 1 /rosbridge_websocket >/dev/null 2>&1; do sleep 0.2; done' || {
      echo "rosbridge node did not become ready." >&2
      exit 1
    }
fi

touch "$RUNTIME_READY_FILE"
echo "Simulation startup completed; keeping Docker main process attached to Gazebo."
if [ "$controller_motion_verified" = "1" ]; then
  echo "Headless walking state and physical /cmd_vel response were verified."
elif [ "$START_CONTROLLER" = "1" ]; then
  echo "Controller process, fixed stand, and /cmd_vel subscription are ready; physical motion probe was not requested."
fi
# Docker 以本脚本为 PID 1。等待 roslaunch 退出可避免“脚本结束但 Gazebo/中继被
# Docker 回收”的脱节；容器停止时 Docker 会向同一进程组发送终止信号。
wait "$LAUNCH_PID"
