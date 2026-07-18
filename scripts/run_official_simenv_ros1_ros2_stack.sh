#!/usr/bin/env bash
# 官方 SimEnv + ROS2 业务层统一入口。默认不越权启动或重置共享仿真。
set -euo pipefail

# 负责人：姜晨。先确认官方容器，再启动独立适配，再启动不含 fake/Harmonic 的 ROS2 业务层。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${SIMENV_CONTAINER:-simenv_run}"
export HAZARDWALKER_ROOT="$ROOT"

# Jazzy 的 setup.bash 会读取若干可选变量；在 set -u 下未定义时会提前退出。
# 与独立适配器入口保持同一加载方式，避免一键业务栈还未启动就中断。
set +u
source /opt/ros/jazzy/setup.bash
# 部分官方工作站通过离线解包提供 Cartographer，而不是安装到 /opt/ros。
# 调用方只需显式传 ROS 前缀；入口统一补齐 ament、可执行文件和动态库搜索路径，
# 避免依赖某个交互 shell 里偶然残留的环境变量。
CARTOGRAPHER_PREFIX="${OFFICIAL_SIMENV_CARTOGRAPHER_PREFIX:-}"
if [[ -n "$CARTOGRAPHER_PREFIX" ]]; then
  if [[ ! -d "$CARTOGRAPHER_PREFIX/share/cartographer_ros" ]]; then
    echo "[stack] Cartographer 前缀无效：$CARTOGRAPHER_PREFIX" >&2
    echo '[stack] 需要包含 share/cartographer_ros；请修正 OFFICIAL_SIMENV_CARTOGRAPHER_PREFIX。' >&2
    exit 1
  fi
  export AMENT_PREFIX_PATH="$CARTOGRAPHER_PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export CMAKE_PREFIX_PATH="$CARTOGRAPHER_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export PATH="$CARTOGRAPHER_PREFIX/bin${PATH:+:$PATH}"
  export LD_LIBRARY_PATH="$CARTOGRAPHER_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  for PACKAGE_SETUP in \
    "$CARTOGRAPHER_PREFIX/share/cartographer_ros_msgs/local_setup.bash" \
    "$CARTOGRAPHER_PREFIX/share/cartographer_ros/local_setup.bash"; do
    if [[ -f "$PACKAGE_SETUP" ]]; then
      source "$PACKAGE_SETUP"
    fi
  done
  if [[ -n "${OFFICIAL_SIMENV_CARTOGRAPHER_LIBRARY_PATH:-}" ]]; then
    export LD_LIBRARY_PATH="${OFFICIAL_SIMENV_CARTOGRAPHER_LIBRARY_PATH}:$LD_LIBRARY_PATH"
  fi
fi
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:?请先设置与官方 dynamic_bridge 相同的 ROS_DOMAIN_ID}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

NAVIGATION_REQUESTED=0
SLAM_REQUESTED=0
SLAM_BACKEND=cartographer
NAV_MODE=frontier
PERCEPTION_OUTPUT_FRAME=map
LOCALIZATION_PROVENANCE=unverified
PERCEPTION_REQUESTED=1
DECISION_REQUESTED=1
EVIDENCE_REQUESTED=0
SCENARIO_SEED_VALUE=
CODE_VERSION_VALUE=
EVIDENCE_OUTPUT_DIR_VALUE=
TEST_RECORD_DIR_VALUE=
OFFICIAL_RESULT_VALUE=results/detected_danger.json
for ARG in "$@"; do
  case "$ARG" in
    start_navigation:=true|start_navigation:=True|start_navigation:=1)
      NAVIGATION_REQUESTED=1
      ;;
    start_slam:=true|start_slam:=True|start_slam:=1)
      SLAM_REQUESTED=1
      ;;
    slam_backend:=*)
      SLAM_BACKEND="${ARG#slam_backend:=}"
      ;;
    nav_mode:=*)
      NAV_MODE="${ARG#nav_mode:=}"
      ;;
    perception_output_frame:=*)
      PERCEPTION_OUTPUT_FRAME="${ARG#perception_output_frame:=}"
      ;;
    localization_provenance:=*)
      LOCALIZATION_PROVENANCE="${ARG#localization_provenance:=}"
      ;;
    start_perception:=false|start_perception:=False|start_perception:=0)
      PERCEPTION_REQUESTED=0
      ;;
    start_decision:=false|start_decision:=False|start_decision:=0)
      DECISION_REQUESTED=0
      ;;
    start_evidence_recorder:=true|start_evidence_recorder:=True|start_evidence_recorder:=1)
      EVIDENCE_REQUESTED=1
      ;;
    scenario_seed:=*)
      SCENARIO_SEED_VALUE="${ARG#scenario_seed:=}"
      ;;
    code_version:=*)
      CODE_VERSION_VALUE="${ARG#code_version:=}"
      ;;
    evidence_output_dir:=*)
      EVIDENCE_OUTPUT_DIR_VALUE="${ARG#evidence_output_dir:=}"
      ;;
    test_record_dir:=*)
      TEST_RECORD_DIR_VALUE="${ARG#test_record_dir:=}"
      ;;
    official_result_path:=*)
      OFFICIAL_RESULT_VALUE="${ARG#official_result_path:=}"
      ;;
  esac
done

if [[ "$SLAM_REQUESTED" == 1 && "$SLAM_BACKEND" == cartographer ]]; then
  if ! ros2 pkg prefix cartographer_ros >/dev/null 2>&1; then
    echo '[stack] 已请求 Cartographer SLAM，但当前环境找不到 cartographer_ros。' >&2
    echo '[stack] 请安装该包，或设置 OFFICIAL_SIMENV_CARTOGRAPHER_PREFIX 指向离线 ROS 前缀。' >&2
    exit 1
  fi
fi

case "${OFFICIAL_SIMENV_ENABLE_CONTROL:-0}" in
  1|true|True|TRUE|yes|YES|on|ON) CONTROL_REQUESTED=1 ;;
  *) CONTROL_REQUESTED=0 ;;
esac
if [[ "$NAVIGATION_REQUESTED" == 1 && "$CONTROL_REQUESTED" != 1 ]]; then
  echo '[stack] start_navigation=true 但控制适配未显式开启；拒绝启动“看似运行、实际不可控”的导航。' >&2
  echo '[stack] 独占验收时请设置 OFFICIAL_SIMENV_ENABLE_CONTROL=1。' >&2
  exit 1
fi
if [[ "$NAVIGATION_REQUESTED" == 1 && "$SLAM_REQUESTED" != 1 ]]; then
  echo '[stack] Frontier 导航要求本轮显式 start_slam=true；拒绝启动无地图导航。' >&2
  exit 1
fi
if [[ "$NAVIGATION_REQUESTED" == 1 ]]; then
  if [[ "$NAV_MODE" != frontier ]]; then
    echo '[stack] 正式一键任务只允许 nav_mode=frontier；固定航点仅可作为独立诊断。' >&2
    exit 1
  fi
  if [[ "$PERCEPTION_REQUESTED" != 1 || "$DECISION_REQUESTED" != 1 ]]; then
    echo '[stack] 正式 Frontier 任务必须同时启动感知和决策节点。' >&2
    exit 1
  fi
  if [[ "$PERCEPTION_OUTPUT_FRAME" != world ]]; then
    echo '[stack] 正式结果要求 perception_output_frame=world；拒绝把 map/start 坐标冒充提交坐标。' >&2
    exit 1
  fi
  case "$LOCALIZATION_PROVENANCE" in
    lidar_imu_slam|visual_inertial_slam|lidar_imu_slam+public_floor_action) ;;
    *)
      echo '[stack] 正式结果要求白名单内的合法 SLAM localization_provenance。' >&2
      exit 1
      ;;
  esac
  if [[ "$EVIDENCE_REQUESTED" != 1 || -z "$SCENARIO_SEED_VALUE" \
        || -z "$CODE_VERSION_VALUE" || -z "$EVIDENCE_OUTPUT_DIR_VALUE" \
        || -z "$TEST_RECORD_DIR_VALUE" ]]; then
    echo '[stack] 正式 Frontier 任务必须开启证据记录并提供 SEED、代码版本和输出目录。' >&2
    exit 1
  fi
fi

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
  if [[ -z "${SIMENV_START_COMMAND:-}" ]]; then
    echo "[stack] 官方容器未运行：$CONTAINER" >&2
    echo "[stack] 为避免错误重置共享场景，请由平台组启动；或显式提供 SIMENV_START_COMMAND。" >&2
    exit 1
  fi
  echo "[stack] 执行调用方明确提供的官方启动命令。"
  bash -lc "$SIMENV_START_COMMAND"
fi

bash "$ROOT/scripts/check_official_simenv_exclusive_session.sh" \
  --container "$CONTAINER" --require-exclusive

# 生成器声明 main_entrance initial_open=true，但当前官方门控进程实测不会在
# 启动时把该状态同步到 Gazebo 模型：起点深度会在约 2.0 m 处被门板完全挡住。
# 正式导航只通过赛题公开 /set_door_state 服务幂等开门；不读取 door_config、
# 场景布局或 Gazebo 状态。服务拒绝或不可用时 fail-closed，避免机器人绕楼外。
case "${OFFICIAL_SIMENV_OPEN_MAIN_ENTRANCE:-1}" in
  1|true|True|TRUE|yes|YES|on|ON) OPEN_MAIN_ENTRANCE=1 ;;
  *) OPEN_MAIN_ENTRANCE=0 ;;
esac
if [[ "$NAVIGATION_REQUESTED" == 1 && "$OPEN_MAIN_ENTRANCE" == 1 ]]; then
  echo '[stack] 通过公开 /set_door_state 请求打开 main_entrance。'
  if ! docker exec -i "$CONTAINER" bash -lc \
      'source /opt/ros/noetic/setup.bash; source /home/ros/simenv_ws/devel/setup.bash; python3 -' <<'PY'
import sys

import rospy
from building_generator_interfaces.srv import SetDoorState

rospy.init_node('hazardwalker_open_main_entrance', anonymous=True)
rospy.wait_for_service('/set_door_state', timeout=15.0)
response = rospy.ServiceProxy('/set_door_state', SetDoorState)(
    'main_entrance', True,
)
print(
    '[stack] main_entrance accepted=%s state=%s'
    % (response.accepted, response.state)
)
if not response.accepted or str(response.state).lower() != 'open':
    sys.exit(2)
PY
  then
    echo '[stack] 公开 main_entrance 开门请求失败；拒绝启动导航。' >&2
    exit 1
  fi
fi

if ros2 node list 2>/dev/null | grep -qx '/hazardwalker_official_rosbridge_adapter'; then
  echo '[stack] ROS_DOMAIN_ID 内已存在官方适配器；请由该进程所有者收尾后重试，避免重复 /clock、/scan 或控制转发。' >&2
  exit 1
fi
# 显式经 bash 调用，不依赖 Git checkout、共享目录或 Windows 挂载是否保留可执行位。
bash "$ROOT/scripts/run_official_simenv_rosbridge_adapter.sh" &
ADAPTER_PID=$!
cleanup_adapter() {
  if [[ "${CLEANUP_DONE:-0}" == 1 ]]; then
    return
  fi
  CLEANUP_DONE=1
  # ros2 launch 会派生导航、感知和决策子进程；只杀 launch 父进程会留下多个
  # /hw/cmd_vel 发布者。setsid 为业务栈创建独立进程组后统一回收。
  if [[ -n "${BUSINESS_PID:-}" ]]; then
    kill -TERM -- "-$BUSINESS_PID" 2>/dev/null || true
    for _ in {1..50}; do
      kill -0 -- "-$BUSINESS_PID" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL -- "-$BUSINESS_PID" 2>/dev/null || true
    wait "$BUSINESS_PID" 2>/dev/null || true
  fi
  kill -TERM "$ADAPTER_PID" 2>/dev/null || true
  for _ in {1..50}; do
    kill -0 "$ADAPTER_PID" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL "$ADAPTER_PID" 2>/dev/null || true
  wait "$ADAPTER_PID" 2>/dev/null || true
}
trap cleanup_adapter EXIT INT TERM
# 业务节点统一使用仿真时间；必须等到适配器转发出两帧递增的 /clock。
# 单个非零值可能只是冻结的旧消息，不能证明仿真仍在推进。
CLOCK_READY=0
LAST_CLOCK_NS=
for _ in {1..20}; do
  CLOCK_SAMPLE="$(timeout 2 ros2 topic echo /clock --once 2>/dev/null || true)"
  CLOCK_SEC="$(printf '%s\n' "$CLOCK_SAMPLE" | awk '/sec:/{print $2; exit}')"
  CLOCK_NSEC="$(printf '%s\n' "$CLOCK_SAMPLE" | awk '/nanosec:/{print $2; exit}')"
  if [[ "$CLOCK_SEC" =~ ^[0-9]+$ ]] && [[ "$CLOCK_NSEC" =~ ^[0-9]+$ ]]; then
    CLOCK_NS=$((10#$CLOCK_SEC * 1000000000 + 10#$CLOCK_NSEC))
    if [[ -n "$LAST_CLOCK_NS" ]] && (( CLOCK_NS > LAST_CLOCK_NS )); then
      CLOCK_READY=1
      break
    fi
    if (( CLOCK_NS > 0 )); then
      LAST_CLOCK_NS="$CLOCK_NS"
    fi
  fi
done
if [[ "$CLOCK_READY" != "1" ]]; then
  echo '[stack] 20 次采样仍未收到连续递增的 /clock；拒绝启动混合时间域业务栈。' >&2
  exit 1
fi
if [[ "$NAVIGATION_REQUESTED" == 1 ]]; then
  ADAPTER_STATUS="$(
    timeout 3 ros2 topic echo /hw/platform/official_simenv_adapter_status \
      --field data --once 2>/dev/null || true
  )"
  if [[ "$ADAPTER_STATUS" != *'"enable_cmd_vel_relay": true'* ]]; then
    echo '[stack] 适配器状态未确认控制转发已开启；拒绝启动导航。' >&2
    exit 1
  fi
fi
echo '[stack] 启动 ROS2 业务层（不含 fake 平台；固定航点导航默认关闭）。'
setsid ros2 launch hazardwalker_bringup official_simenv_business.launch.py "$@" &
BUSINESS_PID=$!
case "$OFFICIAL_RESULT_VALUE" in
  /*) OFFICIAL_RESULT_PATH="$OFFICIAL_RESULT_VALUE" ;;
  *) OFFICIAL_RESULT_PATH="$ROOT/$OFFICIAL_RESULT_VALUE" ;;
esac
RUN_START_EPOCH="$(date +%s)"
case "${OFFICIAL_SIMENV_AUTO_STOP_ON_FINISHED:-1}" in
  1|true|True|TRUE|yes|YES|on|ON) AUTO_STOP_ON_FINISHED=1 ;;
  *) AUTO_STOP_ON_FINISHED=0 ;;
esac
if [[ "$AUTO_STOP_ON_FINISHED" != 1 ]]; then
  wait "$BUSINESS_PID"
  exit $?
fi

# FINISHED 必须同时伴随本轮新写出的官方结果；旧文件不能让脚本误报成功。
STACK_TIMEOUT_SEC="${OFFICIAL_SIMENV_STACK_TIMEOUT_SEC:-3600}"
DEADLINE_EPOCH=$((RUN_START_EPOCH + STACK_TIMEOUT_SEC))
while kill -0 "$BUSINESS_PID" 2>/dev/null; do
  MISSION_STATE="$(
    timeout 3 ros2 topic echo /hw/mission/state --field data --once 2>/dev/null || true
  )"
  if [[ "$MISSION_STATE" == *FINISHED* ]]; then
    for _ in {1..20}; do
      if [[ -f "$OFFICIAL_RESULT_PATH" ]]; then
        RESULT_MTIME="$(stat -c %Y "$OFFICIAL_RESULT_PATH" 2>/dev/null || echo 0)"
        if (( RESULT_MTIME >= RUN_START_EPOCH )) \
          && python3 -c 'import json,sys; json.load(open(sys.argv[1], encoding="utf-8"))' \
            "$OFFICIAL_RESULT_PATH" 2>/dev/null; then
          echo "[stack] 本轮任务完成，官方结果已写出：$OFFICIAL_RESULT_PATH"
          exit 0
        fi
      fi
      sleep 0.25
    done
    echo '[stack] 收到 FINISHED，但未找到本轮新写出的有效官方结果。' >&2
    exit 1
  fi
  if (( $(date +%s) >= DEADLINE_EPOCH )); then
    echo "[stack] 超过 ${STACK_TIMEOUT_SEC}s 壁钟上限，任务未完成。" >&2
    exit 124
  fi
  sleep 1
done
wait "$BUSINESS_PID"
