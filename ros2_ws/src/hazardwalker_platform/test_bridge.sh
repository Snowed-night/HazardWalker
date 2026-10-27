#!/usr/bin/env bash
# ====================================================================
# HazardWalker 接口桥接测试脚本
# 用法: bash test_bridge.sh
# ====================================================================
set -eo pipefail

C='simenv_ros1_hazard_platform'
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
pass=0; fail=0; warn=0

check() { echo -ne "  $1 ... "; }
ok() { echo -e "${GREEN}OK${NC}"; ((pass++)); }
bad() { echo -e "${RED}FAIL${NC} $2"; ((fail++)); }
wrn() { echo -e "${YELLOW}WARN${NC} $2"; ((warn++)); }

echo "============================================"
echo " HazardWalker 接口桥接测试"
echo " $(date)"
echo "============================================"

# ---- Docker 内 ROS1 源接口 ----
echo ""
echo "=== 一、Docker ROS1 源接口 ==="

test_docker_topic() {
    local topic=$1
    check "$topic"
    local result=$(docker exec "$C" bash -c "source /opt/ros/noetic/setup.bash; source devel/setup.bash; timeout 8 rostopic hz $topic -w 2 2>&1" 2>/dev/null || true)
    if echo "$result" | grep -q "average rate"; then
        local hz=$(echo "$result" | grep "average rate" | awk '{print $NF}')
        ok "(~${hz}Hz)"
    else
        bad "(no data or not found)"
    fi
}

test_docker_service() {
    local svc=$1
    check "$svc"
    if docker exec "$C" bash -c "source /opt/ros/noetic/setup.bash; source devel/setup.bash; rosservice list 2>/dev/null" | grep -q "$svc"; then
        ok
    else
        bad "(missing)"
    fi
}

test_docker_topic "/Odometry_gazebo"
test_docker_topic "/livox/Pointcloud2"
test_docker_topic "/livox/imu"
test_docker_topic "/trunk_imu"
test_docker_topic "/tf"
test_docker_topic "/scan"
test_docker_service "/set_door_state"
test_docker_service "/call_elevator"

check "/cmd_vel topic"
if docker exec "$C" bash -c "source /opt/ros/noetic/setup.bash; rostopic list 2>/dev/null" | grep -q "/cmd_vel"; then
    ok
else
    bad
fi

# ---- 宿主机 ROS2 桥接接口 ----
echo ""
echo "=== 二、宿主机 ROS2 桥接接口 ==="

source /opt/ros/jazzy/setup.bash 2>/dev/null || true

test_ros2_topic() {
    local topic=$1
    check "$topic"
    local info=$(ros2 topic info "$topic" 2>&1 || true)
    if echo "$info" | grep -q 'Publisher count: [1-9]'; then
        ok
    elif echo "$info" | grep -q 'Publisher count: 0'; then
        wrn "(no publisher)"
    else
        bad "(not found)"
    fi
}

test_ros2_topic "/Odometry_gazebo"
test_ros2_topic "/livox/Pointcloud2"
test_ros2_topic "/livox/imu"
test_ros2_topic "/trunk_imu"
test_ros2_topic "/tf"
test_ros2_topic "/scan"

check "/cmd_vel"
if ros2 topic list 2>/dev/null | grep -q "/cmd_vel"; then ok; else bad; fi

# ---- 宿主机 /hw/* 中继接口 ----
echo ""
echo "=== 三、宿主机 /hw/* 中继接口 ==="

for t in "/hw/Odometry_gazebo" "/hw/livox/Pointcloud2" "/hw/livox/imu" "/hw/trunk_imu" "/hw/tf" "/hw/real_sense/rgb/image_raw" "/hw/real_sense/depth/points"; do
    test_ros2_topic "$t"
done

check "/hw/cmd_vel topic"
if ros2 topic list 2>/dev/null | grep -q "/hw/cmd_vel"; then ok; else bad; fi

# ---- 服务调用 ----
echo ""
echo "=== 四、服务调用测试 ==="

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SVC_SCRIPT="$SCRIPT_DIR/scripts/hw_service_call.sh"

check "hw_service_call.sh door"
if [ -x "$SVC_SCRIPT" ]; then
    if timeout 10 "$SVC_SCRIPT" door main_entrance true 2>&1 | grep -q "accepted=True"; then
        ok
    else
        bad "(call failed)"
    fi
else
    bad "(script missing)"
fi

check "hw_service_call.sh elevator"
if [ -x "$SVC_SCRIPT" ]; then
    if timeout 10 "$SVC_SCRIPT" elevator elevator_main 0 true 2>&1 | grep -q "accepted=True"; then
        ok
    else
        bad "(call failed)"
    fi
else
    bad "(script missing)"
fi

# ---- /hw/cmd_vel 写入 ----
echo ""
echo "=== 五、/hw/cmd_vel 写入测试 ==="
check "publish to /hw/cmd_vel"
if ros2 topic pub --once /hw/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.5}, angular: {z: 0.1}}' 2>&1 | grep -q "publishing"; then
    ok
else
    bad "(publish failed)"
fi

# ---- 总结 ----
echo ""
echo "============================================"
echo " 结果: ${GREEN}${pass} OK${NC}  ${YELLOW}${warn} WARN${NC}  ${RED}${fail} FAIL${NC}"
echo "============================================"
