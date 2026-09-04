#!/usr/bin/env bash
# 在 ROS2 主机启动官方 ROS1 rosbridge 双向适配器；官方 Docker 只需运行 rosbridge_websocket。
set -euo pipefail

# 负责人：姜晨。容器命名与 auto_docker.sh 保持一致；共享账号或自定义容器
# 可继续用 SIMENV_CONTAINER 显式覆盖。控制转发仍默认关闭。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${SIMENV_CONTAINER:-simenv_ros1_${DOCKER_SIMENV_USER:-${USER:-default}}}"
# 多组账号共享同一 ROS_DOMAIN_ID 时禁用按用户隔离的 Fast DDS SHM，保证平台
# 账号发布的 /hw/* 能被导航、感知、决策和测试账号实际接收。
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
ROSBRIDGE_URL="${OFFICIAL_SIMENV_ROSBRIDGE_URL:-ws://127.0.0.1:9090}"
ROSBRIDGE_HOST_HEADER="${OFFICIAL_SIMENV_ROSBRIDGE_HOST_HEADER:-}"
ENABLE_CONTROL="${OFFICIAL_SIMENV_ENABLE_CONTROL:-0}"
ENABLE_UNITREE_MOVE_BASE_BRIDGE="${OFFICIAL_SIMENV_ENABLE_UNITREE_MOVE_BASE_BRIDGE:-0}"
MANAGED_LIFECYCLE="${OFFICIAL_SIMENV_MANAGED_LIFECYCLE:-0}"
LIFECYCLE_CONTAINER="${OFFICIAL_SIMENV_LIFECYCLE_CONTAINER:-$CONTAINER}"
SCENARIO_SEED="${OFFICIAL_SIMENV_SCENARIO_SEED:-}"
RGB_TOPIC="${OFFICIAL_SIMENV_RGB_TOPIC:-/real_sense/rgb/image_raw}"
DEPTH_TOPIC="${OFFICIAL_SIMENV_DEPTH_TOPIC:-/real_sense/depth/image_raw}"
RGB_INFO_TOPIC="${OFFICIAL_SIMENV_RGB_CAMERA_INFO_TOPIC:-/real_sense/rgb/camera_info}"
DEPTH_INFO_TOPIC="${OFFICIAL_SIMENV_DEPTH_CAMERA_INFO_TOPIC:-/real_sense/depth/camera_info}"
ENABLE_IMAGE_RELAY="${OFFICIAL_SIMENV_ENABLE_IMAGE_RELAY:-1}"
# 实时辅助对准至少需要稳定的多帧检测。200 ms 对应桥接上限 5 Hz；相比
# 第一人称直接 ROS1 视频仍显著降采样，避免 rosbridge 原始 RGB-D JSON 压垮主机。
IMAGE_THROTTLE_RATE_MS="${OFFICIAL_SIMENV_IMAGE_THROTTLE_RATE_MS:-200}"
CLOCK_THROTTLE_RATE_MS="${OFFICIAL_SIMENV_CLOCK_THROTTLE_RATE_MS:-20}"
TF_THROTTLE_RATE_MS="${OFFICIAL_SIMENV_TF_THROTTLE_RATE_MS:-20}"
ODOM_THROTTLE_RATE_MS="${OFFICIAL_SIMENV_ODOM_THROTTLE_RATE_MS:-20}"
SCAN_THROTTLE_RATE_MS="${OFFICIAL_SIMENV_SCAN_THROTTLE_RATE_MS:-50}"
SCAN_SELF_FILTER_RANGE_M="${OFFICIAL_SIMENV_SCAN_SELF_FILTER_RANGE_M:-0.40}"
POINTCLOUD_THROTTLE_RATE_MS="${OFFICIAL_SIMENV_POINTCLOUD_THROTTLE_RATE_MS:-200}"
IMU_THROTTLE_RATE_MS="${OFFICIAL_SIMENV_IMU_THROTTLE_RATE_MS:-20}"
ENABLE_POINTCLOUD_RELAY="${OFFICIAL_SIMENV_ENABLE_POINTCLOUD_RELAY:-0}"
ENABLE_LIVOX_IMU_RELAY="${OFFICIAL_SIMENV_ENABLE_LIVOX_IMU_RELAY:-0}"
ENABLE_TRUNK_IMU_RELAY="${OFFICIAL_SIMENV_ENABLE_TRUNK_IMU_RELAY:-1}"
ODOM_TOPIC="${OFFICIAL_SIMENV_ODOM_TOPIC:-/Odometry_gazebo}"
# 平台诊断默认转发容器内最新值中继；它与控制、点云互相独立，且不构成合法 SLAM 位姿。
ENABLE_ODOM_RELAY="${OFFICIAL_SIMENV_ENABLE_ODOM_RELAY:-1}"
ENABLE_ODOM_TF_RELAY="${OFFICIAL_SIMENV_ENABLE_ODOM_TF_RELAY:-0}"
PUBLIC_START_X="${OFFICIAL_PUBLIC_START_X:-0.0}"
PUBLIC_START_Y="${OFFICIAL_PUBLIC_START_Y:--2.2}"
PUBLIC_START_Z="${OFFICIAL_PUBLIC_START_Z:-0.6}"
PUBLIC_START_YAW="${OFFICIAL_PUBLIC_START_YAW:-1.5708}"
ROS2_SETUP="${OFFICIAL_SIMENV_ROS2_SETUP:-/opt/ros/jazzy/setup.bash}"
PYTHON_BIN="${OFFICIAL_SIMENV_PYTHON_BIN:-python3}"
RUNTIME_VENV="${OFFICIAL_SIMENV_RUNTIME_VENV:-$HOME/.local/share/hazardwalker-ros2-venv}"
LEGACY_RUNTIME_VENV="$HOME/.local/share/hazardwalker/ros2_bridge_venv"

as_ros_bool() {
  case "${1,,}" in
    1|true|yes|on) echo true ;;
    *) echo false ;;
  esac
}
ENABLE_CONTROL="$(as_ros_bool "$ENABLE_CONTROL")"
ENABLE_UNITREE_MOVE_BASE_BRIDGE="$(as_ros_bool "$ENABLE_UNITREE_MOVE_BASE_BRIDGE")"
MANAGED_LIFECYCLE="$(as_ros_bool "$MANAGED_LIFECYCLE")"
ENABLE_IMAGE_RELAY="$(as_ros_bool "$ENABLE_IMAGE_RELAY")"
ENABLE_ODOM_RELAY="$(as_ros_bool "$ENABLE_ODOM_RELAY")"
ENABLE_ODOM_TF_RELAY="$(as_ros_bool "$ENABLE_ODOM_TF_RELAY")"
ENABLE_POINTCLOUD_RELAY="$(as_ros_bool "$ENABLE_POINTCLOUD_RELAY")"
ENABLE_LIVOX_IMU_RELAY="$(as_ros_bool "$ENABLE_LIVOX_IMU_RELAY")"
ENABLE_TRUNK_IMU_RELAY="$(as_ros_bool "$ENABLE_TRUNK_IMU_RELAY")"

if [[ ! -f "$ROS2_SETUP" ]]; then
  echo "[rosbridge-adapter] 找不到 ROS2 环境：$ROS2_SETUP；请设置 OFFICIAL_SIMENV_ROS2_SETUP。" >&2; exit 1
fi
# 共享主机常遗留已删除工作区的 AMENT/COLCON 前缀；Jazzy 会沿这些前缀继续 source
# setup.bash，进而把一次性启动误报成“缺少 rclpy”。正式适配器从干净 ROS2 基础环境
# 加载；适配器以源码方式运行，只补充当前包路径，不能再 source 可能过期的 install。
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX AMENT_CURRENT_PREFIX \
  CMAKE_PREFIX_PATH PYTHONPATH \
  ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH
# ROS 2 Jazzy 的 setup.bash 在未定义 AMENT_TRACE_SETUP_FILES 时会被本脚本的
# set -u 中断；加载环境期间暂时关闭 nounset，随后立即恢复严格模式。
set +u
source "$ROS2_SETUP"
set -u
# 脚本直接运行适配器源码，需要显式加入当前包；保留 ROS2 setup 提供的 PYTHONPATH。
export PYTHONPATH="$ROOT/ros2_ws/src/hazardwalker_platform:${PYTHONPATH:-}"
# 系统 Python 未安装 websocket-client 时，优先使用平台运行时 venv。该 venv 使用
# --system-site-packages 创建，仍可访问 ROS2 的 rclpy，避免改动系统 Python。
if ! "$PYTHON_BIN" -c 'import rclpy, websocket' 2>/dev/null \
  && [[ "$PYTHON_BIN" == "python3" ]]; then
  if [[ -x "$RUNTIME_VENV/bin/python" ]]; then
    PYTHON_BIN="$RUNTIME_VENV/bin/python"
  elif [[ -x "$LEGACY_RUNTIME_VENV/bin/python" ]]; then
    PYTHON_BIN="$LEGACY_RUNTIME_VENV/bin/python"
  fi
fi
if ! command -v ros2 >/dev/null || ! "$PYTHON_BIN" -c 'import rclpy, websocket' 2>/dev/null; then
  echo '[rosbridge-adapter] ROS2 主机缺少 ros2/rclpy 或 websocket-client；未启动适配器。' >&2
  echo "请先创建 $RUNTIME_VENV 并安装 websocket-client，或设置 OFFICIAL_SIMENV_PYTHON_BIN；不能在仅 ROS1 的官方容器内运行。" >&2
  exit 1
fi
if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
  echo "[rosbridge-adapter] 官方容器未运行：$CONTAINER" >&2; exit 1
fi
# 以容器实际环境为准；宿主 shell 中后来修改的 SEED 不能冒充本轮场景。
CONTAINER_SCENARIO_SEED="$(
  docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$CONTAINER" |
    sed -n 's/^SEED=//p' | tail -n 1
)"
if [[ -n "$SCENARIO_SEED" && "$SCENARIO_SEED" != "$CONTAINER_SCENARIO_SEED" ]]; then
  echo "[rosbridge-adapter] 受管 SEED 与容器实际 SEED 不一致：$SCENARIO_SEED != $CONTAINER_SCENARIO_SEED" >&2
  exit 1
fi
SCENARIO_SEED="$CONTAINER_SCENARIO_SEED"
if ! docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode list | grep -qx /rosbridge_websocket'; then
  echo '[rosbridge-adapter] 容器内没有 /rosbridge_websocket；请先由官方 auto.sh 启动 ROS1 rosbridge。' >&2; exit 1
fi
if [[ "$ENABLE_ODOM_RELAY" == "true" ]] && ! docker exec "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; timeout 5 rostopic echo -n 1 '$ODOM_TOPIC' >/dev/null"; then
  echo "[rosbridge-adapter] 官方最新值里程计未就绪：$ODOM_TOPIC；请用修复后的 auto_docker.sh 重建并启动容器。" >&2
  exit 1
fi
ARGS=(--ros-args -p rosbridge_url:="$ROSBRIDGE_URL")
if [[ -n "$ROSBRIDGE_HOST_HEADER" ]]; then
  ARGS+=(-p rosbridge_host_header:="$ROSBRIDGE_HOST_HEADER")
fi
# SEED 只作为实验标识字符串；额外 YAML 引号避免纯数字被 ROS2 CLI 推断为整数。
exec "$PYTHON_BIN" "$ROOT/scripts/official_simenv_rosbridge_ros2_adapter_node.py" "${ARGS[@]}" \
  -p use_sim_time:=false \
  -p managed_lifecycle:="$MANAGED_LIFECYCLE" \
  -p lifecycle_container:="$LIFECYCLE_CONTAINER" \
  -p "scenario_seed:='$SCENARIO_SEED'" \
  -p enable_clock_relay:=true \
  -p clock_throttle_rate_ms:="$CLOCK_THROTTLE_RATE_MS" \
  -p enable_cmd_vel_relay:="$ENABLE_CONTROL" \
  -p enable_unitree_move_base_bridge:="$ENABLE_UNITREE_MOVE_BASE_BRIDGE" \
  -p rgb_topic:="$RGB_TOPIC" -p depth_topic:="$DEPTH_TOPIC" \
  -p rgb_camera_info_topic:="$RGB_INFO_TOPIC" \
  -p depth_camera_info_topic:="$DEPTH_INFO_TOPIC" \
  -p enable_image_relay:="$ENABLE_IMAGE_RELAY" \
  -p image_throttle_rate_ms:="$IMAGE_THROTTLE_RATE_MS" \
  -p enable_odom_relay:="$ENABLE_ODOM_RELAY" \
  -p enable_odom_tf_relay:="$ENABLE_ODOM_TF_RELAY" \
  -p ros1_odom_topic:="$ODOM_TOPIC" \
  -p odom_throttle_rate_ms:="$ODOM_THROTTLE_RATE_MS" \
  -p scan_throttle_rate_ms:="$SCAN_THROTTLE_RATE_MS" \
  -p scan_self_filter_range_m:="$SCAN_SELF_FILTER_RANGE_M" \
  -p pointcloud_throttle_rate_ms:="$POINTCLOUD_THROTTLE_RATE_MS" \
  -p imu_throttle_rate_ms:="$IMU_THROTTLE_RATE_MS" \
  -p enable_pointcloud_relay:="$ENABLE_POINTCLOUD_RELAY" \
  -p enable_livox_imu_relay:="$ENABLE_LIVOX_IMU_RELAY" \
  -p enable_trunk_imu_relay:="$ENABLE_TRUNK_IMU_RELAY" \
  -p tf_throttle_rate_ms:="$TF_THROTTLE_RATE_MS" \
  -p public_start_world_x:="$PUBLIC_START_X" \
  -p public_start_world_y:="$PUBLIC_START_Y" \
  -p public_start_world_z:="$PUBLIC_START_Z" \
  -p public_start_world_yaw:="$PUBLIC_START_YAW"
