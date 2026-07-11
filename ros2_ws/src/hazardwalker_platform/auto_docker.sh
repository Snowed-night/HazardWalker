#!/usr/bin/env bash
# SimEnv ROS 1 host entry (hxbl Ubuntu 24.04) — runs Classic Gazebo inside Docker 20.04.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

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

chmod +x "$ROOT/docker/auto_noetic.sh" "$ROOT/docker/build_catkin.sh" 2>/dev/null || true

case "${1:-up}" in
  build)
    if [[ -f "$ROOT/devel/lib/unitree_guide/junior_ctrl" && "${2:-}" != "force" ]]; then
      echo "devel/ already contains junior_ctrl."
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
    if [[ ! -f "$ROOT/devel/setup.bash" ]]; then
      echo "WARN: devel/setup.bash not found — run './auto_docker.sh build' first (or rsync devel/ from platform)."
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
  *)
    echo "Usage: $0 {build|up|down|logs|shell|status}" >&2
    exit 1
    ;;
esac
