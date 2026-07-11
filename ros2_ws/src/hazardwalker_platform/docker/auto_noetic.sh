#!/usr/bin/env bash
# Host-side wrapper for SimEnv ROS1 Docker (Ubuntu 20.04 + Noetic).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIMENV_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
COMPOSE_FILE="$SCRIPT_DIR/docker-compose.yml"

export SIMENV_HOST_PATH="$SIMENV_ROOT"
export DOCKER_SIMENV_USER="${USER:-default}"
export COMPOSE_PROJECT_NAME="simenv_ros1_${DOCKER_SIMENV_USER}"

CONTAINER_NAME="simenv_ros1_${DOCKER_SIMENV_USER}"

COMPOSE_FILES=(-f "$COMPOSE_FILE")
if [[ "${USE_GPU:-1}" != "0" ]] && docker info 2>/dev/null | grep -qi nvidia; then
  COMPOSE_FILES+=(-f "$SCRIPT_DIR/docker-compose.gpu.yml")
elif [[ "${USE_GPU:-1}" != "0" ]]; then
  echo "NOTE: NVIDIA Container Toolkit not detected — container starts without GPU passthrough." >&2
  echo "      LibTorch CUDA is still in the image; run setup_hxbl_nvidia_docker.sh on hxbl for GPU runtime." >&2
fi

compose() {
  docker compose "${COMPOSE_FILES[@]}" "$@"
}

usage() {
  cat <<EOF
Usage: $0 {build|up|down|logs|shell|status|image}

  build   Build/pull Ubuntu 20.04 + Noetic image and catkin_make inside container
  up      Start Classic Gazebo simulation container
  down    Stop and remove container for current user
  logs    Follow container stdout
  shell   Interactive bash inside running container (with devel sourced)
  status  Show container and key process state
  image   Build Docker image only (no catkin)

Environment (passed into container):
  GUI=false  PAUSED=false  START_CONTROLLER=0|1  SEED=...
  Container name: ${CONTAINER_NAME}

Note: Docker image includes LibTorch CUDA (cu118) + CUDA 11.8 toolkit (for cmake compile).
      GPU passthrough at runtime: USE_GPU=1 (default) when nvidia-container-toolkit is installed.
      First 'build' may take 10–30 min. Use 'build force' to rebuild when devel/ already exists.
EOF
}

cmd="${1:-up}"
shift || true

case "$cmd" in
  image)
    compose build "$@"
    ;;
  build)
    force=""
    if [[ "${1:-}" == "force" ]]; then
      force=1
      shift
    fi
    compose build "$@"
    clean=""
    if [[ -n "$force" ]]; then
      clean="rm -rf build devel install && "
      echo "Force rebuild: cleaning build/ devel/ install/"
    fi
    compose run --rm --entrypoint /ros_entrypoint.sh \
      -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
      -e CUDA_HOME="${CUDA_HOME:-/usr/local/cuda}" \
      simenv_noetic bash -lc "${clean}chmod +x docker/build_catkin.sh && ./docker/build_catkin.sh"
    ;;
  up)
    chmod +x "$SIMENV_ROOT/auto_noetic_headless.sh" 2>/dev/null || true
    compose up -d "$@"
    echo "Container ${CONTAINER_NAME} started. Logs: $0 logs"
    ;;
  down)
    compose down "$@"
    ;;
  logs)
    compose logs -f "$@"
    ;;
  shell)
    docker exec -it "$CONTAINER_NAME" bash -lc \
      'source /opt/ros/noetic/setup.bash && source devel/setup.bash && bash'
    ;;
  status)
    docker ps -a --filter "name=${CONTAINER_NAME}" --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
    if docker ps --filter "name=${CONTAINER_NAME}" --format '{{.Names}}' | grep -q "$CONTAINER_NAME"; then
      docker exec "$CONTAINER_NAME" bash -lc \
        'pgrep -af "gazebo|roslaunch|junior_ctrl" | head -8 || true'
    fi
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "Unknown command: $cmd" >&2
    usage >&2
    exit 1
    ;;
esac
