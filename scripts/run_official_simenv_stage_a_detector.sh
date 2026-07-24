#!/usr/bin/env bash
# A阶段静态感知检测器：只订阅合法RGB-D/TF并发布候选和局部三维定位，不控制机器人。
set -euo pipefail

ROOT="${HAZARDWALKER_RUNTIME_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ROS_SETUP="${HAZARDWALKER_ROS2_SETUP:-/opt/ros/jazzy/setup.bash}"
OUTPUT_FRAME="${HAZARDWALKER_STAGE_A_OUTPUT_FRAME:-base}"
LOCALIZATION_PROVENANCE="${HAZARDWALKER_STAGE_A_LOCALIZATION_PROVENANCE:-camera_rgbd_static_extrinsic}"
RGBD_MAX_DELTA_SEC="${HAZARDWALKER_STAGE_A_RGBD_MAX_DELTA_SEC:-0.06}"

if [[ "$OUTPUT_FRAME" != "real_sense" && "$OUTPUT_FRAME" != "base" ]]; then
  echo "A阶段只允许 real_sense 或 base 局部定位，拒绝伪造 map/world：$OUTPUT_FRAME" >&2
  exit 2
fi
if [[ ! -f "$ROS_SETUP" || ! -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  echo "缺少ROS2或当前工作区安装环境：$ROOT" >&2
  exit 2
fi

set +u
source "$ROS_SETUP"
source "$ROOT/ros2_ws/install/setup.bash"
set -u

# 跨用户FastDDS必须固定UDP，否则可能只能发现端点却收不到图像。
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

exec ros2 run hazardwalker_perception hsv_detector_node --ros-args \
  -p use_sim_time:=true \
  -p output_frame:="$OUTPUT_FRAME" \
  -p camera_axis_convention:=gazebo_link_x_forward \
  -p localization_provenance:="$LOCALIZATION_PROVENANCE" \
  -p max_rgb_depth_sync_delta_sec:="$RGBD_MAX_DELTA_SEC"
