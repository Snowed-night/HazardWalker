#!/usr/bin/env bash
# 官方 SimEnv ROS1 直连控制验收。仅在平台组确认独占场景后使用，自动落盘可审计证据。
set -euo pipefail

# 负责人：姜晨。此脚本绕过 ROS1/ROS2 适配层，优先证明官方 A1 控制器本身能真实运动。
# 输入：运行中的官方 Docker、ROS1 /cmd_vel、/Odometry_gazebo；输出：前后里程计、控制器信息、summary、CSV 和 README。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${SIMENV_CONTAINER:-simenv_run}"
RUN="${1:-}"
FORWARD_SPEED="${OFFICIAL_SIMENV_FORWARD_SPEED:-0.25}"
FORWARD_SECONDS="${OFFICIAL_SIMENV_FORWARD_SECONDS:-6}"
TURN_SPEED="${OFFICIAL_SIMENV_TURN_SPEED:-0.35}"
TURN_SECONDS="${OFFICIAL_SIMENV_TURN_SECONDS:-4}"
STAMP="$(date +%Y%m%d_%H%M%S)"
EVIDENCE_DIR="${OFFICIAL_SIMENV_EVIDENCE_DIR:-$ROOT/reports/platform/official_simenv_ros1_ros2/${STAMP}_ros1_direct_control}"
VIDEO_REFERENCE="${OFFICIAL_SIMENV_VIDEO_REFERENCE:-}"

usage() {
  cat <<'USAGE'
用法：
  OFFICIAL_SIMENV_EXCLUSIVE_SESSION=1 ./scripts/verify_official_simenv_ros1_direct_control.sh --run

可选环境变量：SIMENV_CONTAINER、OFFICIAL_SIMENV_EVIDENCE_DIR、OFFICIAL_SIMENV_VIDEO_REFERENCE、
OFFICIAL_SIMENV_FORWARD_SPEED、OFFICIAL_SIMENV_FORWARD_SECONDS、OFFICIAL_SIMENV_TURN_SPEED、
OFFICIAL_SIMENV_TURN_SECONDS。脚本只在官方容器内发布 ROS1 /cmd_vel，不经过 /hw/* 或适配层。
USAGE
}

if [[ "$RUN" != "--run" ]]; then
  usage
  exit 2
fi
if [[ "${OFFICIAL_SIMENV_EXCLUSIVE_SESSION:-0}" != "1" ]]; then
  echo '[direct-control] 拒绝执行：必须由平台组确认独占场景，并设置 OFFICIAL_SIMENV_EXCLUSIVE_SESSION=1。' >&2
  exit 2
fi
if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
  echo "[direct-control] 官方 ROS1 容器未运行：$CONTAINER" >&2
  exit 1
fi

mkdir -p "$EVIDENCE_DIR"

ros1() {
  docker exec "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; $*"
}

# 即使中途命令、网络或容器调用失败，也优先尝试送出零速度，避免命令残留。
stop_robot() {
  ros1 "timeout 4 rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'" \
    >/dev/null 2>&1 || true
}
trap stop_robot EXIT INT TERM

snapshot_odom() {
  local destination="$1"
  ros1 'timeout 8 rostopic echo -n 1 -p /Odometry_gazebo' > "$destination"
}

echo '[direct-control] 记录控制器、物理和里程计基线。'
ros1 'rostopic info /cmd_vel' > "$EVIDENCE_DIR/cmd_vel_info_before.txt"
ros1 'rosnode info /unitree_gazebo_servo' > "$EVIDENCE_DIR/unitree_gazebo_servo_before.txt"
ros1 'rosservice call /gazebo/get_physics_properties' > "$EVIDENCE_DIR/physics_before.txt"
snapshot_odom "$EVIDENCE_DIR/odom_before.csv"

echo "[direct-control] 直行：${FORWARD_SPEED} m/s，${FORWARD_SECONDS} s。"
ros1 "timeout $FORWARD_SECONDS rostopic pub -r 20 /cmd_vel geometry_msgs/Twist '{linear: {x: $FORWARD_SPEED}, angular: {z: 0.0}}'" \
  > "$EVIDENCE_DIR/forward_cmd.txt" 2>&1 || true
stop_robot
sleep 1
snapshot_odom "$EVIDENCE_DIR/odom_after_forward.csv"

echo "[direct-control] 原地转向：${TURN_SPEED} rad/s，${TURN_SECONDS} s。"
ros1 "timeout $TURN_SECONDS rostopic pub -r 20 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: $TURN_SPEED}}'" \
  > "$EVIDENCE_DIR/turn_cmd.txt" 2>&1 || true
stop_robot
sleep 1
snapshot_odom "$EVIDENCE_DIR/odom_after_turn.csv"
ros1 'rostopic info /cmd_vel' > "$EVIDENCE_DIR/cmd_vel_info_after.txt"

# 仅以同一轮前后里程计计算位移；发布成功、订阅存在或进程日志都不能替代该结果。
python3 - "$EVIDENCE_DIR" "$VIDEO_REFERENCE" <<'PY'
"""从 rostopic -p CSV 生成直接控制验收摘要；供脚本运行时使用。"""
import csv
import json
import math
import sys
from pathlib import Path

evidence_dir = Path(sys.argv[1])
video_reference = sys.argv[2]


def read_pose(path):
    rows = list(csv.DictReader(path.open(encoding='utf-8')))
    if not rows:
        raise ValueError('%s 没有里程计数据' % path.name)
    row = rows[-1]
    return {
        'x': float(row['field.pose.pose.position.x']),
        'y': float(row['field.pose.pose.position.y']),
        'z': float(row['field.pose.pose.position.z']),
        'qz': float(row['field.pose.pose.orientation.z']),
        'qw': float(row['field.pose.pose.orientation.w']),
    }


def yaw(pose):
    return math.atan2(2.0 * pose['qw'] * pose['qz'], 1.0 - 2.0 * pose['qz'] * pose['qz'])


try:
    before = read_pose(evidence_dir / 'odom_before.csv')
    after_forward = read_pose(evidence_dir / 'odom_after_forward.csv')
    after_turn = read_pose(evidence_dir / 'odom_after_turn.csv')
    displacement = math.hypot(after_forward['x'] - before['x'], after_forward['y'] - before['y'])
    yaw_change = abs((yaw(after_turn) - yaw(after_forward) + math.pi) % (2.0 * math.pi) - math.pi)
    result = {
        'ros1_direct_control': {
            'forward_displacement_m': round(displacement, 4),
            'forward_at_least_1m': displacement >= 1.0,
            'turn_yaw_change_rad': round(yaw_change, 4),
            'turn_observed': yaw_change >= 0.2,
            'stop_command_published': True,
            'video_reference': video_reference or None,
            'video_reference_provided': bool(video_reference),
        },
        'conclusion': 'pass' if displacement >= 1.0 and yaw_change >= 0.2 and video_reference else 'incomplete',
        'note': '结论只基于本轮前后 /Odometry_gazebo；仍需人工核对视频、控制器日志与场景安全。',
    }
except Exception as error:
    result = {
        'conclusion': 'invalid',
        'error': str(error),
        'note': '未得到完整里程计，不能据此宣称直连控制通过。',
    }

(evidence_dir / 'summary.json').write_text(
    json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
record = [
    'record_id,test_item,expected_evidence,status,notes',
    'ROS1-DIRECT-01,直行至少1米,前后Odometry和视频,%s,见summary.json' % result['conclusion'],
    'ROS1-DIRECT-02,原地转向,前后姿态和视频,%s,见summary.json' % result['conclusion'],
    'ROS1-DIRECT-03,停止,零速度命令和视频,%s,见summary.json' % result['conclusion'],
]
(evidence_dir / 'testing_record_platform.csv').write_text('\n'.join(record) + '\n', encoding='utf-8')
readme = '''# 官方 SimEnv ROS1 直连控制验收\n\n负责人：姜晨。该目录由 `verify_official_simenv_ros1_direct_control.sh --run` 自动生成。\n\n- `cmd_vel_info_*.txt`：控制话题订阅端快照。\n- `unitree_gazebo_servo_before.txt`：A1 控制器节点快照。\n- `odom_*.csv`：同轮控制前后里程计。\n- `summary.json`：自动计算的位移、转角和证据完整性。\n- `testing_record_platform.csv`：测试组记录。\n\n没有视频引用、前后里程计或至少 1m 位移时，结论必须保持 `incomplete` 或 `invalid`。\n'''
(evidence_dir / 'README.md').write_text(readme, encoding='utf-8')
PY

echo "[direct-control] 证据已写入：$EVIDENCE_DIR"
cat "$EVIDENCE_DIR/summary.json"
