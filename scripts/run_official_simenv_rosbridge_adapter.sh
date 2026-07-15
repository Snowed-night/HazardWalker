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
TF_THROTTLE_RATE_MS="${OFFICIAL_SIMENV_TF_THROTTLE_RATE_MS:-20}"
ODOM_THROTTLE_RATE_MS="${OFFICIAL_SIMENV_ODOM_THROTTLE_RATE_MS:-20}"
ODOM_TOPIC="${OFFICIAL_SIMENV_ODOM_TOPIC:-/hazardwalker/odom}"
ROS2_SETUP="${OFFICIAL_SIMENV_ROS2_SETUP:-/opt/ros/jazzy/setup.bash}"

as_ros_bool() {
  case "${1,,}" in
    1|true|yes|on) echo true ;;
    *) echo false ;;
  esac
}
ENABLE_CONTROL="$(as_ros_bool "$ENABLE_CONTROL")"
ENABLE_IMAGE_RELAY="$(as_ros_bool "$ENABLE_IMAGE_RELAY")"

if [[ ! -f "$ROS2_SETUP" ]]; then
  echo "[rosbridge-adapter] 找不到 ROS2 环境：$ROS2_SETUP；请设置 OFFICIAL_SIMENV_ROS2_SETUP。" >&2; exit 1
fi
# ROS 2 Jazzy 的 setup.bash 在未定义 AMENT_TRACE_SETUP_FILES 时会被本脚本的
# set -u 中断；加载环境期间暂时关闭 nounset，随后立即恢复严格模式。
set +u
source "$ROS2_SETUP"
source "$ROOT/ros2_ws/install/setup.bash"
set -u
if ! command -v ros2 >/dev/null || ! python3 -c 'import rclpy, websocket' 2>/dev/null; then
  echo '[rosbridge-adapter] ROS2 主机缺少 ros2/rclpy 或 websocket-client；未启动适配器。' >&2
  echo '请安装完整 ROS2 运行时和 websocket-client 后重试，不能在仅 ROS1 的官方容器内运行。' >&2
  exit 1
fi
if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
  echo "[rosbridge-adapter] 官方容器未运行：$CONTAINER" >&2; exit 1
fi
if ! docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode list | grep -qx /rosbridge_websocket'; then
  echo '[rosbridge-adapter] 容器内没有 /rosbridge_websocket；请先由官方 auto.sh 启动 ROS1 rosbridge。' >&2; exit 1
fi
if ! docker exec "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; timeout 5 rostopic echo -n 1 '$ODOM_TOPIC' >/dev/null"; then
  echo "[rosbridge-adapter] 官方最新值里程计未就绪：$ODOM_TOPIC；请应用官方 headless 补丁，或显式设置 OFFICIAL_SIMENV_ODOM_TOPIC=/Odometry_gazebo（旧环境回退）。" >&2
  exit 1
fi
ARGS=(--ros-args -p rosbridge_url:="$ROSBRIDGE_URL")
if [[ -n "$ROSBRIDGE_HOST_HEADER" ]]; then
  ARGS+=(-p rosbridge_host_header:="$ROSBRIDGE_HOST_HEADER")
fi
exec python3 "$ROOT/scripts/official_simenv_rosbridge_ros2_adapter_node.py" "${ARGS[@]}" \
  -p enable_cmd_vel_relay:="$ENABLE_CONTROL" \
  -p rgb_topic:="$RGB_TOPIC" -p depth_topic:="$DEPTH_TOPIC" \
  -p rgb_camera_info_topic:="$RGB_INFO_TOPIC" \
  -p depth_camera_info_topic:="$DEPTH_INFO_TOPIC" \
  -p enable_image_relay:="$ENABLE_IMAGE_RELAY" \
  -p image_throttle_rate_ms:="$IMAGE_THROTTLE_RATE_MS" \
  -p ros1_odom_topic:="$ODOM_TOPIC" \
  -p odom_throttle_rate_ms:="$ODOM_THROTTLE_RATE_MS" \
  -p tf_throttle_rate_ms:="$TF_THROTTLE_RATE_MS"
