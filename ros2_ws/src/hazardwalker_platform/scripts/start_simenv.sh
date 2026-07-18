#!/usr/bin/env bash
# ====================================================================
# [已弃用] SimEnv + HazardWalker 旧一键启动脚本
#
# 本文件只保留 ``stop`` 用于清理历史运行；禁止再由它启动旧 hw_topic_relay。
# 正式入口：仓库根目录 scripts/run_official_simenv_ros1_ros2_stack.sh
# 用法: ./scripts/start_simenv.sh stop
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

# 旧流程会启动无控制门禁/看门狗的 hw_topic_relay，并与正式 rosbridge 适配器形成
# 重复发布者。失败优先，避免成员沿用旧手册后污染 /tf、/map 或 /cmd_vel。
err "此启动入口已弃用，未启动 Docker、旧 hw_topic_relay 或业务节点。"
err "请在独占会话中从仓库根目录运行："
err "  bash scripts/run_official_simenv_ros1_ros2_stack.sh"
err "需要导航时再显式追加 start_navigation:=true，并按正式流程开启控制适配。"
exit 2

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
