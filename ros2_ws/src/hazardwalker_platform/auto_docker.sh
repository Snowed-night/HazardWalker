#!/usr/bin/env bash
# SimEnv ROS 1 host entry (hxbl Ubuntu 24.04) — runs Classic Gazebo inside Docker 20.04.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
CATKIN_WORKSPACE="$ROOT/.ros1_catkin_ws"

if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: docker not found. Ask hazard_admin to run setup_hxbl_docker_group.sh" >&2
  exit 1
fi

if ! docker info >/dev/null 2>&1; then
  echo "ERROR: cannot access Docker daemon. Ensure your user is in group 'docker' and re-login." >&2
  echo "  groups | grep docker" >&2
  exit 1
fi

if [[ ! -d "$ROOT/src" ]]; then
  echo "ERROR: $ROOT/src missing. Sync from platform first:" >&2
  echo "  python3 scripts/simenv_ros2/sync_ros1_from_platform.py" >&2
  exit 1
fi

chmod +x "$ROOT/auto.sh" "$ROOT/scripts/rosbridge_odom_relay.py" \
  "$ROOT/docker/auto_noetic.sh" "$ROOT/docker/build_catkin.sh" \
  "$ROOT/docker/gui_client.sh" 2>/dev/null || true

# `.ros1_catkin_ws` 是 Docker 内 Catkin 的独立构建工作区，而不是源码目录。
# 若它被清理、首次克隆尚未构建，`up` 必须先恢复产物；否则容器入口加载
# `devel/setup.bash` 会立即退出，留下一个反复重启的无效容器。
runtime_ready() {
  [[ -f "$CATKIN_WORKSPACE/devel/setup.bash" && \
     -x "$CATKIN_WORKSPACE/devel/lib/unitree_guide/junior_ctrl" ]]
}

ensure_runtime() {
  if runtime_ready; then
    return 0
  fi
  echo "Catkin runtime missing; rebuilding .ros1_catkin_ws before startup..."
  "$ROOT/docker/auto_noetic.sh" build
  if ! runtime_ready; then
    echo "ERROR: Catkin rebuild did not produce devel/setup.bash and junior_ctrl." >&2
    echo "       Run './auto_docker.sh build' and inspect its complete output." >&2
    exit 1
  fi
}

case "${1:-up}" in
  build)
    if [[ -f "$CATKIN_WORKSPACE/devel/lib/unitree_guide/junior_ctrl" && "${2:-}" != "force" ]]; then
      echo ".ros1_catkin_ws/devel/ already contains junior_ctrl."
      echo "Use './auto_docker.sh build force' to rebuild inside container (LibTorch is in the image)."
      echo "Or './auto_docker.sh up' to run with the existing binary."
      exit 0
    fi
    if [[ "${2:-}" == "force" ]]; then
      exec "$ROOT/docker/auto_noetic.sh" build force
    else
      exec "$ROOT/docker/auto_noetic.sh" build
    fi
    ;;
  up|start)
    ensure_runtime
    controller_binary="$CATKIN_WORKSPACE/devel/lib/unitree_guide/junior_ctrl"
    controller_source_root="$ROOT/src/unitree_guide"
    if [[ -x "$controller_binary" && -d "$controller_source_root" ]] &&
       find "$controller_source_root" -type f \
         \( -name '*.cpp' -o -name '*.h' -o -name 'CMakeLists.txt' -o -name 'package.xml' \) \
         -newer "$controller_binary" -print -quit | grep -q .; then
      echo "ERROR: unitree_guide source is newer than .ros1_catkin_ws/devel/lib/unitree_guide/junior_ctrl." >&2
      echo "Run './auto_docker.sh build force' before './auto_docker.sh up'." >&2
      exit 1
    fi
    exec "$ROOT/docker/auto_noetic.sh" up
    ;;
  down|stop)
    exec "$ROOT/docker/auto_noetic.sh" down
    ;;
  logs)
    exec "$ROOT/docker/auto_noetic.sh" logs
    ;;
  shell)
    exec "$ROOT/docker/auto_noetic.sh" shell
    ;;
  status)
    exec "$ROOT/docker/auto_noetic.sh" status
    ;;
  gui)
    # GUI sidecar 只连接现有 Master，不会重启或重建正式仿真容器。
    exec "$ROOT/docker/gui_client.sh" "${@:2}"
    ;;
  image)
    # 仅重建镜像；`--no-cache` 等参数原样转交，供平台管理员执行干净验收。
    exec "$ROOT/docker/auto_noetic.sh" image "${@:2}"
    ;;
  *)
    echo "Usage: $0 {build|image|up|down|logs|shell|status|gui}" >&2
    exit 1
    ;;
esac
