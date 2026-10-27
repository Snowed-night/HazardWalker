#!/usr/bin/env bash
# 官方随机场景的感知侧一键编排器。
# 仅启动自建激光—IMU定位、RGB-D感知和证据记录；不启动导航、不发布 /cmd_vel。
# 导航层需自行执行探索/返航，并在结束时发布 FINISHED 到 mission_state_topic。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIMENV_ROOT="${SIMENV_ROOT:?请设置官方 SimEnv 根目录}"
OFFICIAL_SIMENV_EXCLUSIVE_SESSION="${OFFICIAL_SIMENV_EXCLUSIVE_SESSION:-0}"
OFFICIAL_SCENARIO_SEED="${OFFICIAL_SCENARIO_SEED:?必须显式记录固定场景 SEED}"
OFFICIAL_CODE_VERSION="${OFFICIAL_CODE_VERSION:?必须显式记录本次代码版本}"
OFFICIAL_EVIDENCE_DIR="${OFFICIAL_EVIDENCE_DIR:?必须指定正式证据目录}"
OFFICIAL_TEST_RECORD_DIR="${OFFICIAL_TEST_RECORD_DIR:?必须指定测试表目录}"
OFFICIAL_RESULT_PATH="${OFFICIAL_RESULT_PATH:-$SIMENV_ROOT/results/detected_danger.json}"
OFFICIAL_MISSION_STATE_TOPIC="${OFFICIAL_MISSION_STATE_TOPIC:-/hazardwalker/mission/state}"
OFFICIAL_FLOOR_INDEX_TOPIC="${OFFICIAL_FLOOR_INDEX_TOPIC:-/hazardwalker/navigation/floor_index}"
OFFICIAL_MAX_RUNTIME_SEC="${OFFICIAL_MAX_RUNTIME_SEC:-600}"
OFFICIAL_PUBLIC_START_X="${OFFICIAL_PUBLIC_START_X:-0.0}"
OFFICIAL_PUBLIC_START_Y="${OFFICIAL_PUBLIC_START_Y:--2.2}"
OFFICIAL_PUBLIC_START_Z="${OFFICIAL_PUBLIC_START_Z:-0.6}"
OFFICIAL_PUBLIC_START_YAW="${OFFICIAL_PUBLIC_START_YAW:-1.5708}"

if [[ "$OFFICIAL_SIMENV_EXCLUSIVE_SESSION" != "1" ]]; then
  echo '[perception-evidence] 需要平台确认 OFFICIAL_SIMENV_EXCLUSIVE_SESSION=1；拒绝在共享场景归档正式成绩。' >&2
  exit 2
fi
if ! [[ "$OFFICIAL_MAX_RUNTIME_SEC" =~ ^[1-9][0-9]*$ ]] || (( OFFICIAL_MAX_RUNTIME_SEC > 600 )); then
  echo '[perception-evidence] OFFICIAL_MAX_RUNTIME_SEC 必须是 1..600 的整数。' >&2
  exit 2
fi

source /opt/ros/noetic/setup.bash
source "$SIMENV_ROOT/devel/setup.bash"
ROS_PYTHON_DIST_PACKAGES="/opt/ros/noetic/lib/python3/dist-packages"
export PYTHONPATH="$ROS_PYTHON_DIST_PACKAGES:$REPO_ROOT/ros2_ws/src/hazardwalker_perception:$REPO_ROOT/ros2_ws/src/hazardwalker_decision:${PYTHONPATH:-}"
cd "$SIMENV_ROOT"

# 相同匿名节点会覆盖私有参数、混写证据，因而预先拒绝残留感知进程。
for node in /hazardwalker_official_lidar_imu_slam /hazardwalker_official_rgbd_perception /hazardwalker_official_evidence_recorder; do
  if rosnode list 2>/dev/null | grep -Fxq "$node"; then
    echo "[perception-evidence] 检测到残留节点 $node；请由其拥有者停止后再运行。" >&2
    exit 3
  fi
done

mkdir -p "$OFFICIAL_EVIDENCE_DIR/runtime_logs" "$OFFICIAL_TEST_RECORD_DIR"
LAUNCH_NOTE="official_random_scene seed=$OFFICIAL_SCENARIO_SEED code=$OFFICIAL_CODE_VERSION mission_topic=$OFFICIAL_MISSION_STATE_TOPIC floor_topic=$OFFICIAL_FLOOR_INDEX_TOPIC"
SLAM_PID=''
PERCEPTION_PID=''
RECORDER_PID=''

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  # 先停感知以生成最终 JSON，再停记录器将该 JSON 原子归档；定位最后退出。
  [[ -n "$PERCEPTION_PID" ]] && kill "$PERCEPTION_PID" 2>/dev/null || true
  [[ -n "$PERCEPTION_PID" ]] && wait "$PERCEPTION_PID" 2>/dev/null || true
  [[ -n "$RECORDER_PID" ]] && kill "$RECORDER_PID" 2>/dev/null || true
  [[ -n "$RECORDER_PID" ]] && wait "$RECORDER_PID" 2>/dev/null || true
  [[ -n "$SLAM_PID" ]] && kill "$SLAM_PID" 2>/dev/null || true
  [[ -n "$SLAM_PID" ]] && wait "$SLAM_PID" 2>/dev/null || true
  exit "$exit_code"
}
trap cleanup EXIT INT TERM

python3 "$REPO_ROOT/scripts/official_simenv_lidar_imu_slam_node.py" \
  _output_topic:=/hazardwalker/slam/odometry \
  _floor_index_topic:="$OFFICIAL_FLOOR_INDEX_TOPIC" \
  >"$OFFICIAL_EVIDENCE_DIR/runtime_logs/lidar_imu_slam.log" 2>&1 &
SLAM_PID=$!

python3 "$REPO_ROOT/scripts/official_simenv_ros1_perception_node.py" \
  _output_path:="$OFFICIAL_RESULT_PATH" \
  _localization_frame:=start \
  _world_frame:=world \
  _camera_axis_convention:=gazebo_link_x_forward \
  _public_start_world_x:="$OFFICIAL_PUBLIC_START_X" \
  _public_start_world_y:="$OFFICIAL_PUBLIC_START_Y" \
  _public_start_world_z:="$OFFICIAL_PUBLIC_START_Z" \
  _public_start_world_yaw:="$OFFICIAL_PUBLIC_START_YAW" \
  _localization_provenance:=lidar_imu_slam+public_floor_action \
  _mission_state_topic:="$OFFICIAL_MISSION_STATE_TOPIC" \
  _auto_activate_cmd_vel:=false \
  >"$OFFICIAL_EVIDENCE_DIR/runtime_logs/rgbd_perception.log" 2>&1 &
PERCEPTION_PID=$!

python3 "$REPO_ROOT/scripts/official_simenv_ros1_evidence_recorder.py" \
  _output_dir:="$OFFICIAL_EVIDENCE_DIR" \
  _test_record_dir:="$OFFICIAL_TEST_RECORD_DIR" \
  _scenario_name:="official_random_seed_$OFFICIAL_SCENARIO_SEED" \
  _run_mode:=official_random_scene \
  _scenario_seed:="$OFFICIAL_SCENARIO_SEED" \
  _code_version:="$OFFICIAL_CODE_VERSION" \
  _legal_pose_topic:=/hazardwalker/slam/odometry \
  _localization_provenance:=lidar_imu_slam+public_floor_action \
  _launch_command:="$LAUNCH_NOTE" \
  _result_json_path:="$OFFICIAL_RESULT_PATH" \
  >"$OFFICIAL_EVIDENCE_DIR/runtime_logs/evidence_recorder.log" 2>&1 &
RECORDER_PID=$!

echo "[perception-evidence] 已启动感知侧证据链；等待导航在 $OFFICIAL_MISSION_STATE_TOPIC 发布 FINISHED（最多 ${OFFICIAL_MAX_RUNTIME_SEC}s）。"
if timeout "$OFFICIAL_MAX_RUNTIME_SEC" rostopic echo "$OFFICIAL_MISSION_STATE_TOPIC" 2>/dev/null | grep -m 1 -F 'FINISHED' >/dev/null; then
  echo '[perception-evidence] 收到 FINISHED，开始按感知→记录器→定位顺序归档。'
  exit 0
fi
echo '[perception-evidence] 未在时限内收到 FINISHED；已执行失败归档，不能作为完成场景。' >&2
exit 4
