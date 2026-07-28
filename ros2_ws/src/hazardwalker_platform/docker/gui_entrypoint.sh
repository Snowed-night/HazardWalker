#!/usr/bin/env bash
# 独立 GUI sidecar 入口：以软件渲染显示现有 Gazebo，不启动或控制仿真容器。
set -euo pipefail

DISPLAY_VALUE="${SIMENV_GUI_DISPLAY:-:100}"
DISPLAY_BACKEND="${SIMENV_GUI_DISPLAY_BACKEND:-xvfb}"
VNC_PORT="${SIMENV_GUI_VNC_PORT:-5901}"
NOVNC_PORT="${SIMENV_GUI_NOVNC_PORT:-6081}"
GUI_RESOLUTION="${SIMENV_GUI_RESOLUTION:-1280x720x24}"
# Qt 接受的 geometry 不含色深，例如 1920x1080+0+0。
GUI_GEOMETRY="${GUI_RESOLUTION%x*}+0+0"
WORKSPACE_DIR="${SIMENV_WORKSPACE_DIR:-/home/ros/simenv_ws}"
HOME_DIR="/tmp/hazardwalker-gui-home"

export HOME="$HOME_DIR"
export DISPLAY="$DISPLAY_VALUE"
# 当前 NVIDIA GLX 与 Xvfb/Gazebo Classic 会黑屏，默认使用已验证的软件渲染。
# 仅在平台完成独立 GPU 兼容性验收后，才可显式设置 SIMENV_GUI_USE_GPU=1。
if [[ "${SIMENV_GUI_USE_GPU:-0}" == "1" ]]; then
  unset LIBGL_ALWAYS_SOFTWARE
  unset GALLIUM_DRIVER
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
else
  export LIBGL_ALWAYS_SOFTWARE=1
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
fi
export QT_X11_NO_MITSHM=1
export XDG_RUNTIME_DIR="/tmp/runtime-root"
mkdir -p "$HOME_DIR" "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"

DISPLAY_NUMBER="${DISPLAY_VALUE#:}"
DISPLAY_SOCKET="/tmp/.X11-unix/X${DISPLAY_NUMBER}"
DISPLAY_LOCK="/tmp/.X${DISPLAY_NUMBER}-lock"
# 旧 Docker 构建器曾把运行中 sidecar 的 X lock 写进镜像；仅删除本容器内无对应进程的陈旧锁。
if [[ -f "$DISPLAY_LOCK" ]]; then
  LOCK_PID="$(cat "$DISPLAY_LOCK" 2>/dev/null || true)"
  if [[ -z "$LOCK_PID" ]] || ! kill -0 "$LOCK_PID" 2>/dev/null; then
    rm -f "$DISPLAY_LOCK" "$DISPLAY_SOCKET"
  fi
fi

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
  kill "${GZCLIENT_PID:-}" "${OPENBOX_PID:-}" "${NOVNC_PID:-}" "${VNC_PID:-}" "${XVFB_PID:-}" 2>/dev/null || true
  wait "${GZCLIENT_PID:-}" "${OPENBOX_PID:-}" "${NOVNC_PID:-}" "${VNC_PID:-}" "${XVFB_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

XVFB_PID=""
case "$DISPLAY_BACKEND" in
  xvfb)
    Xvfb "$DISPLAY_VALUE" -screen 0 "$GUI_RESOLUTION" +extension GLX +render \
      > /tmp/hazardwalker-gui-xvfb.log 2>&1 &
    XVFB_PID=$!
    ;;
  host-xorg)
    # GPU Xorg 由宿主机 systemd 服务负责；sidecar 仅连接既有显示，不得终止它。
    if [[ ! -S "$DISPLAY_SOCKET" ]]; then
      echo "GPU Xorg 显示 $DISPLAY_VALUE 不可用，请先启动 hazardwalker-gpu-xorg 服务。" >&2
      exit 1
    fi
    ;;
  *)
    echo "不支持的 SIMENV_GUI_DISPLAY_BACKEND：$DISPLAY_BACKEND" >&2
    exit 1
    ;;
esac

display_ready() {
  if command -v xdpyinfo >/dev/null 2>&1; then
    xdpyinfo -display "$DISPLAY_VALUE" >/dev/null 2>&1
  else
    [[ -S "$DISPLAY_SOCKET" ]] && [[ "$DISPLAY_BACKEND" = "host-xorg" || -n "$XVFB_PID" ]]
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

# Xvfb 本身没有窗口管理器。Openbox 接管后可将 gzclient 真正最大化，消除 VNC 桌面内的黑边。
DISPLAY="$DISPLAY_VALUE" openbox --sm-disable > /tmp/hazardwalker-gui-openbox.log 2>&1 &
OPENBOX_PID=$!

# 两个端口均只绑定 loopback；RDP 中访问本机浏览器或通过 SSH 隧道访问。
x11vnc -display "$DISPLAY_VALUE" -localhost -forever -shared -nopw \
  -rfbport "$VNC_PORT" > /tmp/hazardwalker-gui-vnc.log 2>&1 &
VNC_PID=$!
websockify --web /usr/share/novnc "127.0.0.1:${NOVNC_PORT}" "127.0.0.1:${VNC_PORT}" \
  > /tmp/hazardwalker-gui-novnc.log 2>&1 &
NOVNC_PID=$!

echo "HazardWalker GUI 已启动：noVNC http://127.0.0.1:${NOVNC_PORT}/vnc.html"
echo "Gazebo Master：${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"

# Xvfb 没有窗口管理器；不指定 Qt geometry 时 gzclient 只会显示默认小窗口，造成四周黑边。
gzclient -geometry "$GUI_GEOMETRY" --verbose > /tmp/hazardwalker-gui-gzclient.log 2>&1 &
GZCLIENT_PID=$!
for _ in $(seq 1 50); do
  if DISPLAY="$DISPLAY_VALUE" wmctrl -l 2>/dev/null | grep -q ' Gazebo$'; then
    DISPLAY="$DISPLAY_VALUE" wmctrl -r Gazebo -b add,maximized_vert,maximized_horz || true
    break
  fi
  sleep 0.1
done
sleep 1
if ! kill -0 "$GZCLIENT_PID" 2>/dev/null; then
  tail -80 /tmp/hazardwalker-gui-gzclient.log >&2 || true
  exit 1
fi

wait "$GZCLIENT_PID"
