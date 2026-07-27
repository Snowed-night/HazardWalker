#!/usr/bin/env bash
# 平台管理员 GUI sidecar 管理脚本；绝不重启或停止正式 SimEnv 容器。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIMENV_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MAIN_CONTAINER="${SIMENV_CONTAINER:-simenv_ros1_${DOCKER_SIMENV_USER:-${USER:-default}}}"
GUI_CONTAINER="${SIMENV_GUI_CONTAINER:-${MAIN_CONTAINER}_gui}"
GUI_IMAGE="${SIMENV_GUI_IMAGE:-simenv_ros1-gui:noetic-focal}"
NOVNC_PORT="${SIMENV_GUI_NOVNC_PORT:-6081}"
VNC_PORT="${SIMENV_GUI_VNC_PORT:-5901}"
ACTION="${1:-up}"

build_image() {
  docker build \
    -f "$SCRIPT_DIR/Dockerfile.gui" \
    -t "$GUI_IMAGE" \
    "$SCRIPT_DIR"
}

case "$ACTION" in
  build)
    build_image
    ;;
  up|start)
    if ! docker inspect -f '{{.State.Running}}' "$MAIN_CONTAINER" 2>/dev/null | grep -qx true; then
      echo "正式 SimEnv 容器未运行：$MAIN_CONTAINER" >&2
      exit 1
    fi
    if docker inspect -f '{{.State.Running}}' "$GUI_CONTAINER" 2>/dev/null | grep -qx true; then
      echo "GUI sidecar 已在运行：http://127.0.0.1:${NOVNC_PORT}/vnc.html"
      exit 0
    fi
    if ! docker image inspect "$GUI_IMAGE" >/dev/null 2>&1; then
      build_image
    fi
    docker rm -f "$GUI_CONTAINER" >/dev/null 2>&1 || true
    # 默认 1080p；noVNC 的 resize=scale 可在不同 RDP/本地窗口中继续铺满视口。
    docker run -d --name "$GUI_CONTAINER" --network host \
      --entrypoint bash \
      --mount "type=bind,src=$SIMENV_ROOT,dst=/home/ros/simenv_ws,readonly" \
      -e GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}" \
      -e SIMENV_GUI_NOVNC_PORT="$NOVNC_PORT" \
      -e SIMENV_GUI_VNC_PORT="$VNC_PORT" \
      -e SIMENV_GUI_RESOLUTION="${SIMENV_GUI_RESOLUTION:-1920x1080x24}" \
      "$GUI_IMAGE" /usr/local/bin/hazardwalker_gui_entrypoint.sh >/dev/null
    echo "GUI sidecar 已启动：http://127.0.0.1:${NOVNC_PORT}/vnc.html"
    echo "RDP 中用浏览器打开上述地址；键盘控制仍在独占终端向 /hw/cmd_vel 发布。"
    ;;
  down|stop)
    docker rm -f "$GUI_CONTAINER" >/dev/null 2>&1 || true
    ;;
  logs)
    docker logs -f "$GUI_CONTAINER"
    ;;
  status)
    docker ps -a --filter "name=^/${GUI_CONTAINER}$" \
      --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
    ;;
  *)
    echo "Usage: $0 {build|up|down|logs|status}" >&2
    exit 1
    ;;
esac
