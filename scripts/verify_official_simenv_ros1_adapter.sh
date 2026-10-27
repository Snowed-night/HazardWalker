#!/usr/bin/env bash
# 官方 SimEnv ROS1 ↔ ROS2 适配验收脚本。默认只读传感器；显式传 --control 才发布速度命令。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${SIMENV_CONTAINER:-simenv_run}"
RUN_CONTROL=0
[[ "${1:-}" == "--control" ]] && RUN_CONTROL=1
RGB_TOPIC="${OFFICIAL_SIMENV_RGB_TOPIC:-/real_sense/rgb/image_raw}"
DEPTH_TOPIC="${OFFICIAL_SIMENV_DEPTH_TOPIC:-/real_sense/depth/image_raw}"
RGB_INFO_TOPIC="${OFFICIAL_SIMENV_RGB_CAMERA_INFO_TOPIC:-/real_sense/rgb/camera_info}"

source /opt/ros/jazzy/setup.bash
source "$ROOT/ros2_ws/install/setup.bash" 2>/dev/null || true

check_ros1() {
  local topic="$1"
  docker exec "$CONTAINER" bash -lc \
    "source /opt/ros/noetic/setup.bash; timeout 8 rostopic echo -n 1 '$topic' >/dev/null"
}
check_ros2() {
  local topic="$1"
  timeout 10 ros2 topic echo --once "$topic" >/dev/null
}

echo '[verify] ROS1 原始输入与 ROS2 /hw 输出：'
declare -a TOPICS=(
  '/Odometry_gazebo:/hw/odom'
  "$RGB_TOPIC:/hw/camera/image_raw"
  "$DEPTH_TOPIC:/hw/camera/depth_image"
  "$RGB_INFO_TOPIC:/hw/camera/camera_info"
)
for pair in "${TOPICS[@]}"; do
  source_topic="${pair%%:*}"
  hw_topic="${pair#*:}"
  check_ros1 "$source_topic"
  check_ros2 "$hw_topic"
  echo "  PASS $source_topic -> $hw_topic"
done

echo '[verify] ROS1 控制器订阅审计：'
docker exec "$CONTAINER" bash -lc \
  "source /opt/ros/noetic/setup.bash; rostopic info /cmd_vel"
echo '[verify] ROS2 rosbridge 适配器状态：'
timeout 5 ros2 topic echo --once /hw/platform/official_simenv_adapter_status

if [[ "$RUN_CONTROL" -eq 1 ]]; then
  echo '[verify] 发布 20 Hz 低速前进 4 秒，再发送零速度。请确保场景无障碍且机器人已站稳。'
  timeout 4 ros2 topic pub -r 20 /hw/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.25}, angular: {z: 0.0}}' || true
  ros2 topic pub --once /hw/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}'
  echo '[verify] 此命令只完成速度链路；必须另存前后 /Odometry_gazebo 与视频，确认平面位移 >= 1m 后才可写为真实运动通过。'
fi
