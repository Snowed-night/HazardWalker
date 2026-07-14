#!/usr/bin/env bash
# 官方 SimEnv ROS1 适配启动器。运行在可执行 docker 的 ROS2 主机，不修改官方源码。
set -euo pipefail

# 负责人：姜晨。该脚本只部署本仓库的独立中继到容器 /tmp，不复制或修改 SimEnv catkin 源码。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${SIMENV_CONTAINER:-simenv_ros1_hazard_platform}"
NODE_FILE="/tmp/hazardwalker_official_simenv_ros1_adapter.py"
ENABLE_CONTROL="${OFFICIAL_SIMENV_ENABLE_CONTROL:-0}"
ALLOW_UNKNOWN="${OFFICIAL_SIMENV_ALLOW_UNKNOWN_CONTROLLER:-0}"
ROS1_SETUP="${SIMENV_ROS1_SETUP:-/home/ros/simenv_ws/devel/setup.bash}"
BRIDGE_COMMAND="${SIMENV_ROS1_BRIDGE_COMMAND:-/home/ros/simenv_ws/docker/ros1_bridge.sh}"
RGB_TOPIC="${OFFICIAL_SIMENV_RGB_TOPIC:-/real_sense/rgb/image_raw}"
DEPTH_TOPIC="${OFFICIAL_SIMENV_DEPTH_TOPIC:-/real_sense/depth/image_raw}"
RGB_INFO_TOPIC="${OFFICIAL_SIMENV_RGB_CAMERA_INFO_TOPIC:-/real_sense/rgb/camera_info}"
DEPTH_INFO_TOPIC="${OFFICIAL_SIMENV_DEPTH_CAMERA_INFO_TOPIC:-/real_sense/depth/camera_info}"

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
  echo "[adapter] 官方 ROS1 容器未运行：$CONTAINER" >&2
  exit 1
fi

# 动态桥必须在同一个网络命名空间中同时看到 ROS1 master 和 ROS2 DDS。
if ! docker exec "$CONTAINER" pgrep -f 'ros1_bridge/dynamic_bridge' >/dev/null; then
  echo "[adapter] 未发现 ros1_bridge dynamic_bridge，启动官方容器内既有启动脚本。"
  docker exec -d "$CONTAINER" bash -lc \
    "nohup '$BRIDGE_COMMAND' >/tmp/hazardwalker_ros1_bridge.log 2>&1 &"
  sleep 2
fi

docker cp "$ROOT/scripts/official_simenv_ros1_adapter_node.py" "$CONTAINER:$NODE_FILE"
docker exec "$CONTAINER" bash -lc \
  "source /opt/ros/noetic/setup.bash; source '$ROS1_SETUP' 2>/dev/null || true; \
   rosnode kill /hazardwalker_official_simenv_adapter >/dev/null 2>&1 || true; \
   nohup python3 $NODE_FILE _enable_cmd_vel_relay:=$ENABLE_CONTROL _allow_unknown_controller:=$ALLOW_UNKNOWN \
   _rgb_topic:='$RGB_TOPIC' _depth_topic:='$DEPTH_TOPIC' \
   _rgb_camera_info_topic:='$RGB_INFO_TOPIC' _depth_camera_info_topic:='$DEPTH_INFO_TOPIC' \
   >/tmp/hazardwalker_official_simenv_adapter.log 2>&1 &"

echo "[adapter] 已启动 ROS1 中继：$CONTAINER。"
echo "[adapter] 控制转发 enable=$ENABLE_CONTROL；未知控制器放行=$ALLOW_UNKNOWN。"
echo "[adapter] RGB=$RGB_TOPIC；Depth=$DEPTH_TOPIC。"
echo "[adapter] 仅在 ROS2 与容器设置同一 ROS_DOMAIN_ID、且 dynamic_bridge 与四足控制器都已验证后才允许控制。"
