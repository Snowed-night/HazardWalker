#!/usr/bin/env bash
# 第一人称视频 sidecar 管理脚本：只订阅官方 RGB 压缩话题，不改动主仿真。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAIN_CONTAINER="${SIMENV_CONTAINER:-simenv_ros1_${DOCKER_SIMENV_USER:-${USER:-default}}}"
FPV_CONTAINER="${SIMENV_FIRST_PERSON_CONTAINER:-${MAIN_CONTAINER}_first_person}"
FPV_IMAGE="${SIMENV_FIRST_PERSON_IMAGE:-simenv_ros1-gui:noetic-focal}"
FPV_PORT="${SIMENV_FIRST_PERSON_PORT:-6082}"
FPV_TOPIC="${SIMENV_FIRST_PERSON_TOPIC:-/real_sense/rgb/image_raw/compressed}"
FPV_MAX_FPS="${SIMENV_FIRST_PERSON_MAX_FPS:-15}"
FPV_JPEG_QUALITY="${SIMENV_FIRST_PERSON_JPEG_QUALITY:-92}"
ACTION="${1:-up}"

case "$ACTION" in
  up|start)
    if ! docker inspect -f '{{.State.Running}}' "$MAIN_CONTAINER" 2>/dev/null | grep -qx true; then
      echo "正式 SimEnv 容器未运行：$MAIN_CONTAINER" >&2
      exit 1
    fi
    if docker inspect -f '{{.State.Running}}' "$FPV_CONTAINER" 2>/dev/null | grep -qx true; then
      echo "第一人称服务已在运行：http://127.0.0.1:${FPV_PORT}/first_person"
      exit 0
    fi
    docker rm -f "$FPV_CONTAINER" >/dev/null 2>&1 || true
    # 主容器和 sidecar 均使用 host 网络，ROS_MASTER_URI 指向主机即可连接 ROS1 Master。
    docker run -d --name "$FPV_CONTAINER" --network host \
      --entrypoint bash \
      --mount "type=bind,src=$SCRIPT_DIR/first_person_server.py,dst=/usr/local/bin/hazardwalker_first_person_server.py,readonly" \
      -e ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}" \
      -e ROS_IP=127.0.0.1 \
      -e FIRST_PERSON_TOPIC="$FPV_TOPIC" \
      -e FIRST_PERSON_PORT="$FPV_PORT" \
      -e FIRST_PERSON_MAX_FPS="$FPV_MAX_FPS" \
      -e FIRST_PERSON_JPEG_QUALITY="$FPV_JPEG_QUALITY" \
      "$FPV_IMAGE" -lc 'source /opt/ros/noetic/setup.bash && exec python3 /usr/local/bin/hazardwalker_first_person_server.py --topic "$FIRST_PERSON_TOPIC" --port "$FIRST_PERSON_PORT" --max-fps "$FIRST_PERSON_MAX_FPS" --jpeg-quality "$FIRST_PERSON_JPEG_QUALITY"' >/dev/null
    echo "第一人称服务已启动：http://127.0.0.1:${FPV_PORT}/first_person"
    ;;
  down|stop)
    docker rm -f "$FPV_CONTAINER" >/dev/null 2>&1 || true
    ;;
  status)
    docker ps -a --filter "name=^/${FPV_CONTAINER}$" \
      --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
    ;;
  logs)
    docker logs -f "$FPV_CONTAINER"
    ;;
  *)
    echo "Usage: $0 {up|down|status|logs}" >&2
    exit 1
    ;;
esac
