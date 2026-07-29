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
GUI_USE_GPU="${SIMENV_GUI_USE_GPU:-0}"
GUI_DISPLAY="${SIMENV_GUI_DISPLAY:-:100}"
GUI_DISPLAY_BACKEND="${SIMENV_GUI_DISPLAY_BACKEND:-xvfb}"
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
    # gzserver 会把 A1 的 package:// 网格展开为主容器 catkin 工作区中的绝对路径。
    # 因此 sidecar 直接只读挂载主容器的完整工作区；仅挂载当前目录会缺失
    # `.ros1_catkin_ws`，GUI 就只能显示足端碰撞几何而没有机器狗本体。
    MAIN_WORKSPACE_HOST="$(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/home/ros/simenv_ws"}}{{.Source}}{{end}}{{end}}' "$MAIN_CONTAINER")"
    # 宿主机工作区通常归平台账号所有，当前成员账号未必有目录遍历权限；
    # Docker 守护进程仍可只读挂载，因此这里只验证路径是否从正式容器解析成功。
    if [[ -z "$MAIN_WORKSPACE_HOST" ]]; then
      echo "未解析到正式 SimEnv 的工作区挂载，无法启动完整模型 GUI。" >&2
      exit 1
    fi
    GPU_ARGS=()
    if [[ "$GUI_USE_GPU" == "1" ]]; then
      GPU_ARGS=(--gpus all)
    fi
    DISPLAY_MOUNT_ARGS=()
    if [[ "$GUI_DISPLAY_BACKEND" == "host-xorg" ]]; then
      # 宿主机 GPU Xorg 的 Unix socket；无 TCP 监听，不暴露额外网络端口。
      DISPLAY_MOUNT_ARGS=(--mount "type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix")
    fi
    docker rm -f "$GUI_CONTAINER" >/dev/null 2>&1 || true
    # 默认 540p，降低 llvmpipe 软件渲染占用；noVNC 会继续缩放到浏览器视口。
    # 全屏页面作为只读配置挂载，旧 Docker 构建器异常时也不会回退到旧页面。
    docker run -d --name "$GUI_CONTAINER" --network host "${GPU_ARGS[@]}" \
      --entrypoint bash \
      --mount "type=bind,src=$MAIN_WORKSPACE_HOST,dst=/home/ros/simenv_ws,readonly" \
      --mount "type=bind,src=$SCRIPT_DIR/gui_entrypoint.sh,dst=/usr/local/bin/hazardwalker_gui_entrypoint.sh,readonly" \
      --mount "type=bind,src=$SCRIPT_DIR/gui_fullscreen.html,dst=/usr/share/novnc/hazardwalker.html,readonly" \
      "${DISPLAY_MOUNT_ARGS[@]}" \
      -e GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}" \
      -e SIMENV_GUI_NOVNC_PORT="$NOVNC_PORT" \
      -e SIMENV_GUI_VNC_PORT="$VNC_PORT" \
      -e SIMENV_GUI_RESOLUTION="${SIMENV_GUI_RESOLUTION:-1280x720x24}" \
      -e SIMENV_GUI_USE_GPU="$GUI_USE_GPU" \
      -e SIMENV_GUI_DISPLAY="$GUI_DISPLAY" \
      -e SIMENV_GUI_DISPLAY_BACKEND="$GUI_DISPLAY_BACKEND" \
      -e NVIDIA_DRIVER_CAPABILITIES=all \
      "$GUI_IMAGE" /usr/local/bin/hazardwalker_gui_entrypoint.sh >/dev/null
    echo "GUI sidecar 已启动：http://127.0.0.1:${NOVNC_PORT}/hazardwalker.html"
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
