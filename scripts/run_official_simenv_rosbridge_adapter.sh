#!/usr/bin/env bash
# 在 ROS2 主机启动官方 ROS1 rosbridge 双向适配器；官方 Docker 只需运行 rosbridge_websocket。
set -euo pipefail

# 负责人：姜晨。默认适配实际官方容器 simenv_run；可用环境变量覆盖地址和容器名。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${SIMENV_CONTAINER:-simenv_run}"
ROSBRIDGE_URL="${OFFICIAL_SIMENV_ROSBRIDGE_URL:-ws://127.0.0.1:9090}"
ROSBRIDGE_HOST_HEADER="${OFFICIAL_SIMENV_ROSBRIDGE_HOST_HEADER:-}"
ENABLE_CONTROL="${OFFICIAL_SIMENV_ENABLE_CONTROL:-0}"
RGB_TOPIC="${OFFICIAL_SIMENV_RGB_TOPIC:-/real_sense/rgb/image_raw}"
DEPTH_TOPIC="${OFFICIAL_SIMENV_DEPTH_TOPIC:-/real_sense/depth/image_raw}"
RGB_INFO_TOPIC="${OFFICIAL_SIMENV_RGB_CAMERA_INFO_TOPIC:-/real_sense/rgb/camera_info}"
DEPTH_INFO_TOPIC="${OFFICIAL_SIMENV_DEPTH_CAMERA_INFO_TOPIC:-/real_sense/depth/camera_info}"
ENABLE_IMAGE_RELAY="${OFFICIAL_SIMENV_ENABLE_IMAGE_RELAY:-1}"
IMAGE_THROTTLE_RATE_MS="${OFFICIAL_SIMENV_IMAGE_THROTTLE_RATE_MS:-500}"
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
ODOM_TOPIC="${OFFICIAL_SIMENV_ODOM_TOPIC:-/hazardwalker/odom}"
# 平台诊断默认转发容器内最新值中继；它与控制、点云互相独立，且不构成合法 SLAM 位姿。
ENABLE_ODOM_RELAY="${OFFICIAL_SIMENV_ENABLE_ODOM_RELAY:-1}"
PUBLIC_START_X="${OFFICIAL_PUBLIC_START_X:-0.0}"
PUBLIC_START_Y="${OFFICIAL_PUBLIC_START_Y:--2.2}"
PUBLIC_START_Z="${OFFICIAL_PUBLIC_START_Z:-0.6}"
PUBLIC_START_YAW="${OFFICIAL_PUBLIC_START_YAW:-1.5708}"
ROS2_SETUP="${OFFICIAL_SIMENV_ROS2_SETUP:-/opt/ros/jazzy/setup.bash}"
PYTHON_BIN="${OFFICIAL_SIMENV_PYTHON_BIN:-python3}"

as_ros_bool() {
  case "${1,,}" in
    1|true|yes|on) echo true ;;
    *) echo false ;;
  esac
}
ENABLE_CONTROL="$(as_ros_bool "$ENABLE_CONTROL")"
ENABLE_IMAGE_RELAY="$(as_ros_bool "$ENABLE_IMAGE_RELAY")"
ENABLE_ODOM_RELAY="$(as_ros_bool "$ENABLE_ODOM_RELAY")"
ENABLE_POINTCLOUD_RELAY="$(as_ros_bool "$ENABLE_POINTCLOUD_RELAY")"
ENABLE_LIVOX_IMU_RELAY="$(as_ros_bool "$ENABLE_LIVOX_IMU_RELAY")"
ENABLE_TRUNK_IMU_RELAY="$(as_ros_bool "$ENABLE_TRUNK_IMU_RELAY")"

if [[ ! -f "$ROS2_SETUP" ]]; then
  echo "[rosbridge-adapter] 找不到 ROS2 环境：$ROS2_SETUP；请设置 OFFICIAL_SIMENV_ROS2_SETUP。" >&2; exit 1
fi
# 共享主机常遗留已删除工作区的 AMENT/COLCON 前缀；Jazzy 会沿这些前缀继续 source
# setup.bash，进而把一次性启动误报成“缺少 rclpy”。正式适配器从干净 ROS2 基础环境
# 加载，再只叠加当前仓库 install，避免把其他成员的终端状态带入平台链路。
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX AMENT_CURRENT_PREFIX \
  CMAKE_PREFIX_PATH PYTHONPATH \
  ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH
# ROS 2 Jazzy 的 setup.bash 在未定义 AMENT_TRACE_SETUP_FILES 时会被本脚本的
# set -u 中断；加载环境期间暂时关闭 nounset，随后立即恢复严格模式。
set +u
source "$ROS2_SETUP"
source "$ROOT/ros2_ws/install/setup.bash"
set -u
if ! command -v ros2 >/dev/null || ! "$PYTHON_BIN" -c 'import rclpy, websocket' 2>/dev/null; then
  echo '[rosbridge-adapter] ROS2 主机缺少 ros2/rclpy 或 websocket-client；未启动适配器。' >&2
  echo '请安装完整 ROS2 运行时和 websocket-client 后重试；Ubuntu 受 PEP 668 保护时请按手册使用独立 venv，不能在仅 ROS1 的官方容器内运行。' >&2
  exit 1
fi
if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
  echo "[rosbridge-adapter] 官方容器未运行：$CONTAINER" >&2; exit 1
fi
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
exec "$PYTHON_BIN" "$ROOT/scripts/official_simenv_rosbridge_ros2_adapter_node.py" "${ARGS[@]}" \
  -p use_sim_time:=false \
  -p enable_clock_relay:=true \
  -p clock_throttle_rate_ms:="$CLOCK_THROTTLE_RATE_MS" \
  -p enable_cmd_vel_relay:="$ENABLE_CONTROL" \
  -p rgb_topic:="$RGB_TOPIC" -p depth_topic:="$DEPTH_TOPIC" \
  -p rgb_camera_info_topic:="$RGB_INFO_TOPIC" \
  -p depth_camera_info_topic:="$DEPTH_INFO_TOPIC" \
  -p enable_image_relay:="$ENABLE_IMAGE_RELAY" \
  -p image_throttle_rate_ms:="$IMAGE_THROTTLE_RATE_MS" \
  -p enable_odom_relay:="$ENABLE_ODOM_RELAY" \
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
