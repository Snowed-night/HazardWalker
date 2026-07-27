#!/usr/bin/env bash
# 独立 GUI sidecar 入口：以软件渲染显示现有 Gazebo，不启动或控制仿真容器。
set -euo pipefail

DISPLAY_VALUE="${SIMENV_GUI_DISPLAY:-:100}"
VNC_PORT="${SIMENV_GUI_VNC_PORT:-5901}"
NOVNC_PORT="${SIMENV_GUI_NOVNC_PORT:-6081}"
WORKSPACE_DIR="${SIMENV_WORKSPACE_DIR:-/home/ros/simenv_ws}"
HOME_DIR="/tmp/hazardwalker-gui-home"

export HOME="$HOME_DIR"
export DISPLAY="$DISPLAY_VALUE"
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
export QT_X11_NO_MITSHM=1
export XDG_RUNTIME_DIR="/tmp/runtime-root"
mkdir -p "$HOME_DIR" "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"

# Noetic 的 profile 脚本会直接展开 ROS_MASTER_URI；严格模式下必须先给出默认值。
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
source /opt/ros/noetic/setup.bash
if [[ -f "$WORKSPACE_DIR/devel/setup.bash" ]]; then
  # 只读取模型、插件和消息定义；sidecar 的工作区卷为只读。
  source "$WORKSPACE_DIR/devel/setup.bash"
fi

GAZEBO_PLUGIN_DIR="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
export GAZEBO_PLUGIN_PATH="$WORKSPACE_DIR/devel/lib:/opt/ros/noetic/lib:$GAZEBO_PLUGIN_DIR:${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="$GAZEBO_PLUGIN_DIR:${LD_LIBRARY_PATH:-}"

cleanup() {
  # GUI 客户端退出不能影响共享 gzserver；只收尾本 sidecar 的子进程。
  kill "${GZCLIENT_PID:-}" "${NOVNC_PID:-}" "${VNC_PID:-}" "${XVFB_PID:-}" 2>/dev/null || true
  wait "${GZCLIENT_PID:-}" "${NOVNC_PID:-}" "${VNC_PID:-}" "${XVFB_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

Xvfb "$DISPLAY_VALUE" -screen 0 "${SIMENV_GUI_RESOLUTION:-1440x900x24}" +extension GLX +render \
  > /tmp/hazardwalker-gui-xvfb.log 2>&1 &
XVFB_PID=$!

DISPLAY_NUMBER="${DISPLAY_VALUE#:}"
DISPLAY_SOCKET="/tmp/.X11-unix/X${DISPLAY_NUMBER}"
display_ready() {
  if command -v xdpyinfo >/dev/null 2>&1; then
    xdpyinfo -display "$DISPLAY_VALUE" >/dev/null 2>&1
  else
    [[ -S "$DISPLAY_SOCKET" ]] && kill -0 "$XVFB_PID" 2>/dev/null
  fi
}
for _ in $(seq 1 50); do
  if display_ready; then
    break
  fi
  sleep 0.1
done
if ! display_ready; then
  echo "GUI Xvfb 启动失败：$(tail -20 /tmp/hazardwalker-gui-xvfb.log)" >&2
  exit 1
fi

# 两个端口均只绑定 loopback；RDP 中访问本机浏览器或通过 SSH 隧道访问。
x11vnc -display "$DISPLAY_VALUE" -localhost -forever -shared -nopw \
  -rfbport "$VNC_PORT" > /tmp/hazardwalker-gui-vnc.log 2>&1 &
VNC_PID=$!
websockify --web /usr/share/novnc "127.0.0.1:${NOVNC_PORT}" "127.0.0.1:${VNC_PORT}" \
  > /tmp/hazardwalker-gui-novnc.log 2>&1 &
NOVNC_PID=$!

echo "HazardWalker GUI 已启动：noVNC http://127.0.0.1:${NOVNC_PORT}/vnc.html"
echo "Gazebo Master：${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"

gzclient --verbose > /tmp/hazardwalker-gui-gzclient.log 2>&1 &
GZCLIENT_PID=$!
sleep 3
if ! kill -0 "$GZCLIENT_PID" 2>/dev/null; then
  tail -80 /tmp/hazardwalker-gui-gzclient.log >&2 || true
  exit 1
fi

wait "$GZCLIENT_PID"
