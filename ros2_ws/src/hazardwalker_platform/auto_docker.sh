#!/usr/bin/env bash
# SimEnv ROS 1 host entry (hxbl Ubuntu 24.04) — runs Classic Gazebo inside Docker 20.04.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
CATKIN_WORKSPACE="$ROOT/.ros1_catkin_ws"
HAZARDWALKER_ROOT="${HAZARDWALKER_ROOT:-$(cd "$ROOT/../../.." && pwd)}"
ADAPTER_MANAGER="$HAZARDWALKER_ROOT/scripts/manage_official_simenv_rosbridge_adapter.sh"
CONTROL_MANAGER="$HAZARDWALKER_ROOT/scripts/manage_official_simenv_command_mux.sh"
RUNTIME_ASSET_BOOTSTRAP="$ROOT/docker/bootstrap_runtime_assets.sh"
export HAZARDWALKER_ROOT
export SIMENV_CONTAINER="${SIMENV_CONTAINER:-simenv_ros1_${DOCKER_SIMENV_USER:-${USER:-default}}}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${OFFICIAL_SIMENV_ROS_DOMAIN_ID:-42}}"

# 正式平台默认会启动 junior_ctrl，因此同一入口拉起的受管适配器也必须默认
# 开启 /hw/cmd_vel -> /cmd_vel。独立适配器脚本仍保持“控制默认关闭”的安全
# 语义；用户显式设置 OFFICIAL_SIMENV_ENABLE_CONTROL=0 时同样不会被覆盖。
if [[ -z "${OFFICIAL_SIMENV_ENABLE_CONTROL+x}" ]]; then
  case "${START_CONTROLLER:-1}" in
    0|false|False|FALSE|no|NO|off|OFF)
      export OFFICIAL_SIMENV_ENABLE_CONTROL=0
      ;;
    *)
      export OFFICIAL_SIMENV_ENABLE_CONTROL=1
      ;;
  esac
fi

# 宇树 move_base 与容器同生命周期。只有显式启动该官方局部规划器时，
# 受管适配器才桥接其目标和隔离速度话题；普通平台启动不增加 ROS 数据流。
if [[ -z "${OFFICIAL_SIMENV_ENABLE_UNITREE_MOVE_BASE_BRIDGE+x}" ]]; then
  case "${START_UNITREE_MOVE_BASE:-0}" in
    1|true|True|TRUE|yes|YES|on|ON)
      export OFFICIAL_SIMENV_ENABLE_UNITREE_MOVE_BASE_BRIDGE=1
      ;;
    *)
      export OFFICIAL_SIMENV_ENABLE_UNITREE_MOVE_BASE_BRIDGE=0
      ;;
  esac
fi

# 当前赛事仓库的 Mid-360 xacro 位于 ENABLE_LIDAR 分支内。用户只请求宇树
# move_base 时自动补齐两个公开传感器开关；若用户显式关闭任一项，容器内
# auto.sh 仍会 fail-closed，而不是启动没有障碍物输入的局部规划器。
case "${START_UNITREE_MOVE_BASE:-0}" in
  1|true|True|TRUE|yes|YES|on|ON)
    : "${ENABLE_LIDAR:=true}"
    : "${ENABLE_LIVOX_3D:=true}"
    export ENABLE_LIDAR ENABLE_LIVOX_3D
    ;;
esac

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
  "$ROOT/scripts/recover_a1_gazebo.py" \
  "$ROOT/docker/auto_noetic.sh" "$ROOT/docker/build_catkin.sh" \
  "$ROOT/docker/gui_client.sh" "$ROOT/docker/first_person_client.sh" \
  2>/dev/null || true

# `.ros1_catkin_ws` 是 Docker 内 Catkin 的独立构建工作区，而不是源码目录。
# 若它被清理、首次克隆尚未构建，`up` 必须先恢复产物；否则容器入口加载
# `devel/setup.bash` 会立即退出，留下一个反复重启的无效容器。
runtime_ready() {
  [[ -f "$CATKIN_WORKSPACE/devel/setup.bash" && \
     -x "$CATKIN_WORKSPACE/devel/lib/unitree_guide/junior_ctrl" ]]
}

ensure_runtime() {
  # 控制策略和预编译 SDK 均被 Git 忽略；即使 junior_ctrl 已编译，也必须
  # 在每次启动前确认这些运行资产仍然存在。
  bash "$RUNTIME_ASSET_BOOTSTRAP"
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

adapter_enabled() {
  case "${OFFICIAL_SIMENV_AUTO_ADAPTER:-1}" in
    0|false|False|FALSE|no|NO|off|OFF) return 1 ;;
    *) return 0 ;;
  esac
}

manage_adapter() {
  local action="$1"
  if [[ ! -f "$ADAPTER_MANAGER" ]]; then
    echo "ERROR: ROS2 adapter manager missing: $ADAPTER_MANAGER" >&2
    return 1
  fi
  bash "$ADAPTER_MANAGER" "$action"
}

manage_control() {
  local action="$1"
  shift || true
  if [[ ! -f "$CONTROL_MANAGER" ]]; then
    echo "ERROR: ROS2 control manager missing: $CONTROL_MANAGER" >&2
    return 1
  fi
  bash "$CONTROL_MANAGER" "$action" "$@"
}

read_container_scenario_seed() {
  docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$SIMENV_CONTAINER" \
    2>/dev/null | sed -n 's/^SEED=//p' | tail -n 1
}

wait_for_container_ready() {
  local timeout_sec="${OFFICIAL_SIMENV_CONTAINER_READY_TIMEOUT_SEC:-180}"
  local deadline=$((SECONDS + timeout_sec)) running health health_configured
  while (( SECONDS < deadline )); do
    running="$(docker inspect -f '{{.State.Running}}' "$SIMENV_CONTAINER" 2>/dev/null || true)"
    health="$(docker inspect -f '{{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}' \
      "$SIMENV_CONTAINER" 2>/dev/null || true)"
    health_configured="$(docker inspect -f \
      '{{if .Config.Healthcheck}}true{{else}}false{{end}}' \
      "$SIMENV_CONTAINER" 2>/dev/null || true)"
    # Docker 创建容器后的极短窗口里，Config 已声明健康检查，但 State.Health
    # 还可能暂时不存在。此时 health=none 不是就绪，只能等待明确的 healthy。
    if [[ "$running" == true && \
          ( ( "$health_configured" == true && "$health" == healthy ) || \
            ( "$health_configured" == false && "$health" == none ) ) ]]; then
      return 0
    fi
    if [[ "$running" == false || "$health" == unhealthy ]]; then
      echo "ERROR: container failed before adapter startup: running=$running health=$health" >&2
      return 1
    fi
    sleep 1
  done
  echo "ERROR: container was not ready within ${timeout_sec}s: $SIMENV_CONTAINER" >&2
  return 1
}

case "${1:-up}" in
  build)
    bash "$RUNTIME_ASSET_BOOTSTRAP"
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
    container_was_running="$(docker inspect -f '{{.State.Running}}' "$SIMENV_CONTAINER" 2>/dev/null || true)"
    "$ROOT/docker/auto_noetic.sh" up
    if ! wait_for_container_ready; then
      [[ "$container_was_running" == true ]] || "$ROOT/docker/auto_noetic.sh" down
      exit 1
    fi
    # 适配器签名和运行时状态必须绑定当前容器真正使用的 SEED。即使容器被
    # 外部替换但名称不变，管理器也会因签名变化而重启适配器。
    export OFFICIAL_SIMENV_SCENARIO_SEED="$(read_container_scenario_seed)"
    if adapter_enabled && ! manage_adapter start; then
      echo 'ERROR: container is ready but the ROS2 adapter failed to start.' >&2
      [[ "$container_was_running" == true ]] || "$ROOT/docker/auto_noetic.sh" down
      exit 1
    fi
    # 键盘、导航和辅助对准都只写各自输入话题；平台必须同时托管唯一
    # command_mux_node，才能把它们安全汇总到适配器订阅的 /hw/cmd_vel。
    if ! manage_control start; then
      echo 'ERROR: container and adapter are ready but the ROS2 command mux failed to start.' >&2
      if [[ "$container_was_running" != true ]]; then
        manage_adapter stop || true
        "$ROOT/docker/auto_noetic.sh" down
      fi
      exit 1
    fi
    ;;
  down|stop)
    # 仲裁器退出时先发布零速度；随后回收适配器，最后停止容器。
    manage_control stop
    manage_adapter stop
    exec "$ROOT/docker/auto_noetic.sh" down
    ;;
  logs)
    exec "$ROOT/docker/auto_noetic.sh" logs
    ;;
  shell)
    exec "$ROOT/docker/auto_noetic.sh" shell
    ;;
  status)
    "$ROOT/docker/auto_noetic.sh" status
    manage_adapter status || true
    manage_control status || true
    ;;
  recover)
    # 仅用于仿真平台维护：保留当前随机场景和机器人平面位置，将倒地 A1
    # 扶正并让 headless FSM 回到 fixed stand。该操作不得计入正式比赛结果。
    if [[ "$(docker inspect -f '{{.State.Running}}' "$SIMENV_CONTAINER" 2>/dev/null || true)" != true ]]; then
      echo "ERROR: container is not running: $SIMENV_CONTAINER" >&2
      exit 1
    fi
    if ! docker exec "$SIMENV_CONTAINER" pgrep -x junior_ctrl >/dev/null; then
      echo "ERROR: junior_ctrl is not running; use './auto_docker.sh up' first." >&2
      exit 1
    fi
    controller_binary="$CATKIN_WORKSPACE/devel/lib/unitree_guide/junior_ctrl"
    controller_source_root="$ROOT/src/unitree_guide"
    if [[ ! -x "$controller_binary" ]] || find "$controller_source_root" -type f \
        \( -name '*.cpp' -o -name '*.h' -o -name 'CMakeLists.txt' \) \
        -newer "$controller_binary" -print -quit | grep -q .; then
      echo "ERROR: recovery-capable junior_ctrl has not been built." >&2
      echo "Run './auto_docker.sh down && ./auto_docker.sh build force', then start again." >&2
      exit 1
    fi
    # recover 可能由一个全新的终端调用。先按当前正式 profile 校验受管适配器
    # 的配置签名；若旧实例是以 control=false 启动，管理器会在扶正前替换它，
    # 避免恢复完成后 /hw/cmd_vel 仍被适配器静默丢弃。
    export OFFICIAL_SIMENV_SCENARIO_SEED="$(read_container_scenario_seed)"
    if adapter_enabled && ! manage_adapter start; then
      echo 'ERROR: A1 recovery aborted because the managed ROS2 adapter is not control-ready.' >&2
      exit 1
    fi
    if ! manage_control start; then
      echo 'ERROR: A1 recovery aborted because the ROS2 command mux is not ready.' >&2
      exit 1
    fi
    docker exec "$SIMENV_CONTAINER" bash -lc \
      'source /opt/ros/noetic/setup.bash && source /home/ros/simenv_ws/.ros1_catkin_ws/devel/setup.bash &&
       for _ in 1 2 3; do rostopic pub -1 /cmd_vel geometry_msgs/Twist -- "{}" >/dev/null; done &&
       touch "${SIMENV_RECOVERY_REQUEST_FILE:-/tmp/hazardwalker-controller-recover.request}" &&
       python3 /home/ros/simenv_ws/scripts/recover_a1_gazebo.py'
    ;;
  control-mode)
    manage_control mode "${2:-}"
    ;;
  gui)
    # GUI sidecar 只连接现有 Master，不会重启或重建正式仿真容器。
    exec "$ROOT/docker/gui_client.sh" "${@:2}"
    ;;
  first_person)
    # 第一人称 sidecar 只读相机和 GUI 状态，不发布控制指令。
    exec "$ROOT/docker/first_person_client.sh" "${@:2}"
    ;;
  image)
    # 仅重建镜像；`--no-cache` 等参数原样转交，供平台管理员执行干净验收。
    exec "$ROOT/docker/auto_noetic.sh" image "${@:2}"
    ;;
  *)
    echo "Usage: $0 {build|image|up|down|logs|shell|status|recover|control-mode|gui|first_person}" >&2
    exit 1
    ;;
esac
