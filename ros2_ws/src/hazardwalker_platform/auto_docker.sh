#!/usr/bin/env bash
# SimEnv ROS 1 host entry (hxbl Ubuntu 24.04) — runs Classic Gazebo inside Docker 20.04.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
CATKIN_WORKSPACE="$ROOT/.ros1_catkin_ws"
CONTAINER_NAME="simenv_ros1_${DOCKER_SIMENV_USER:-${USER:-default}}"

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
  "$ROOT/docker/gui_client.sh" "$ROOT/docker/first_person_client.sh" 2>/dev/null || true

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
      echo "ERROR: unitree_guide source is newer than devel/lib/unitree_guide/junior_ctrl." >&2
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
  first_person|fpv)
    # 第一人称 MJPEG：订阅 /real_sense/rgb/image_raw/compressed，端口 6082。
    exec "$ROOT/docker/first_person_client.sh" "${@:2}"
    ;;
  recover)
    # 倒地恢复只重置当前回合的物理状态；不重建镜像、不删除结果目录，也不启动第二套容器。
    if ! docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
      echo "ERROR: $CONTAINER_NAME is not running. Start it with './auto_docker.sh up' first." >&2
      exit 1
    fi
    exec docker exec "$CONTAINER_NAME" bash \
      /home/ros/simenv_ws/scripts/recover_a1_standing.sh
    ;;
  verify_stand)
    # 供平台、导航、感知和测试组在开始独占试验前只读核验当前站立状态。
    if ! docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
      echo "ERROR: $CONTAINER_NAME is not running." >&2
      exit 1
    fi
    exec docker exec "$CONTAINER_NAME" bash -lc \
      'set +u; export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"; source /opt/ros/noetic/setup.bash && source /home/ros/simenv_ws/.ros1_catkin_ws/devel/setup.bash && python3 /home/ros/simenv_ws/scripts/controller_stand_probe.py'
    ;;
  image)
    # 仅重建镜像；`--no-cache` 等参数原样转交，供平台管理员执行干净验收。
    exec "$ROOT/docker/auto_noetic.sh" image "${@:2}"
    ;;
  *)
    echo "Usage: $0 {build|image|up|down|logs|shell|status|gui|first_person|recover|verify_stand}" >&2
    exit 1
    ;;
esac
