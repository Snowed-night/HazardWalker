#!/usr/bin/env bash
# 负责人新增：在不重建容器或删除实验结果的前提下，恢复倒地 A1 的标准站立状态。
set -euo pipefail

WORKSPACE_DIR="/home/ros/simenv_ws"
# Noetic 的 profile 脚本会读取 ROS_MASTER_URI；独立 docker exec 场景不能假定它
# 已由主容器进程继承，因此在启用 nounset 前补齐默认值。
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/.ros1_catkin_ws/devel/setup.bash"

if ! pgrep -x junior_ctrl >/dev/null; then
  echo "ERROR: junior_ctrl is not running; use './auto_docker.sh down' then './auto_docker.sh up'." >&2
  exit 1
fi

echo "Stopping motion commands and pausing physics..."
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null
rosservice call /gazebo/pause_physics >/dev/null

# reset_simulation 恢复本轮模型初始姿态；控制器已经运行，会在解除暂停后保持站立。
echo "Resetting the current simulation round to the standing start pose..."
rosservice call /gazebo/reset_simulation >/dev/null
sleep 1
rosservice call /gazebo/unpause_physics >/dev/null

rm -f "$WORKSPACE_DIR/logs/controller_stand.ready" \
  "$WORKSPACE_DIR/logs/controller_stand.failed"
if python3 "$WORKSPACE_DIR/scripts/controller_stand_probe.py"; then
  touch "$WORKSPACE_DIR/logs/controller_stand.ready"
  echo "A1 recovery completed. Resume motion only after the current controller owner confirms control." 
else
  touch "$WORKSPACE_DIR/logs/controller_stand.failed"
  rosservice call /gazebo/pause_physics >/dev/null 2>&1 || true
  echo "A1 recovery failed; physics is paused. Use a clean './auto_docker.sh down' then './auto_docker.sh up'." >&2
  exit 1
fi
