#!/usr/bin/env bash
# ====================================================================
# SimEnv + HazardWalker 一键启动脚本
#
# 用法: ./scripts/start_simenv.sh         # 启动
#       ./scripts/start_simenv.sh stop    # 停止
# ====================================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLATFORM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$PLATFORM_DIR/../../.." && pwd)"
SIMENV_DIR="$PLATFORM_DIR"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log()  { echo -e "${GREEN}[SIMENV]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
err()  { echo -e "${RED}[ERROR]${NC} $*" >&2; }

do_stop() {
    log "Stopping HazardWalker nodes..."
    pkill -f "hw_topic_relay" 2>/dev/null || true
    pkill -f "hsv_detector_node" 2>/dev/null || true
    pkill -f "waypoint_patrol_node" 2>/dev/null || true
    pkill -f "mission_state_machine_node" 2>/dev/null || true
    log "Stopping Docker container..."
    cd "$SIMENV_DIR" && sg docker -c "./auto_docker.sh down" 2>/dev/null || true
    log "All stopped."
    exit 0
}

if [ "${1:-}" = "stop" ]; then
    do_stop
fi

if [ ! -f "$SIMENV_DIR/auto_docker.sh" ]; then
    err "找不到 SimEnv: $SIMENV_DIR"
    exit 1
fi

# 1. 启动 Docker ROS1
CURRENT_USER=$(whoami)
export DOCKER_SIMENV_USER="$CURRENT_USER"
CONTAINER_NAME="simenv_ros1_${CURRENT_USER}"

cd "$SIMENV_DIR"
if sg docker -c "docker ps --format '{{.Names}}' | grep -q $CONTAINER_NAME" 2>/dev/null; then
    log "Docker 容器已在运行: $CONTAINER_NAME"
else
    log "启动 Docker ROS1..."
    sg docker -c "./auto_docker.sh up"
    log "等待 Gazebo 初始化 (60s)..."
    sleep 60
fi

# 2. 等待传感器
log "等待传感器话题..."
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
WAITED=0
while [ $WAITED -lt 120 ]; do
    if ros2 topic list 2>/dev/null | grep -q "scan\|Odometry_gazebo"; then
        log "传感器就绪 (${WAITED}s)"
        break
    fi
    sleep 5
    WAITED=$((WAITED + 5))
done

# 3. Source 工作空间
log "加载 ROS2 工作空间..."
source "$REPO_ROOT/ros2_ws/install/setup.bash" 2>/dev/null || {
    err "工作空间未构建，请先运行 ./scripts/build.sh"
    exit 1
}

# 4. 启动 hw_topic_relay
log "启动 hw_topic_relay_node..."
ros2 run hazardwalker_platform hw_topic_relay_node &
sleep 3

# 5. 启动算法节点
log "启动算法节点..."
ros2 launch hazardwalker_bringup simenv_demo.launch.py &

echo ""
echo "=========================================="
echo "  SimEnv + HazardWalker 已启动"
echo "  Docker: $CONTAINER_NAME"
echo "  停止:   $0 stop"
echo "=========================================="
wait
