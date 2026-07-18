#!/usr/bin/env bash
# 官方 SimEnv 场景独占预检：只读取 Docker 状态，不停止、不删除任何容器。
# 负责人：姜晨。用于防止多套 Gazebo/ROS1 master 并发造成速度、里程计和实时性验收互相污染。
set -euo pipefail

CONTAINER="${SIMENV_CONTAINER:-simenv_run}"
REQUIRE_EXCLUSIVE=0
ALLOW_ISOLATED_PARALLEL="${OFFICIAL_SIMENV_ALLOW_ISOLATED_PARALLEL:-0}"

usage() {
  cat <<'USAGE'
用法：
  ./scripts/check_official_simenv_exclusive_session.sh [--container 名称] [--require-exclusive]

默认仅报告。传入 --require-exclusive 时，目标容器与运行中名称以 simenv 开头的容器必须恰好只有目标一个；
否则以非零退出。脚本绝不停止或删除容器，清理由拥有者执行。
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --container) CONTAINER="${2:?--container 需要容器名}"; shift 2 ;;
    --require-exclusive) REQUIRE_EXCLUSIVE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[exclusive-preflight] 未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done

if ! command -v docker >/dev/null 2>&1; then
  echo '[exclusive-preflight] 未找到 docker，无法审查官方场景。' >&2
  exit 1
fi
if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
  echo "[exclusive-preflight] 目标官方容器未运行：$CONTAINER" >&2
  exit 1
fi

# 正式容器通常以 simenv 命名；隔离实验为避免被其他成员的旧清理脚本误删，
# 可以使用非 simenv 名称，但目标本身必须始终纳入独占计数。
mapfile -t RUNNING_CONTAINERS < <(docker ps --format '{{.Names}}')
SIMENV_CONTAINERS=()
for NAME in "${RUNNING_CONTAINERS[@]}"; do
  if [[ "$NAME" == "$CONTAINER" || "$NAME" =~ ^simenv($|[_-]) ]]; then
    SIMENV_CONTAINERS+=("$NAME")
  fi
done
printf '[exclusive-preflight] 目标容器：%s；运行中的 SimEnv 容器：%s\n' \
  "$CONTAINER" "${SIMENV_CONTAINERS[*]:-(无)}"
docker stats --no-stream --format '[exclusive-preflight] {{.Name}} cpu={{.CPUPerc}} mem={{.MemUsage}} pids={{.PIDs}}' \
  "${SIMENV_CONTAINERS[@]}" 2>/dev/null || true

ISOLATED_OK=0
if [[ ${#SIMENV_CONTAINERS[@]} -ne 1 || "${SIMENV_CONTAINERS[0]:-}" != "$CONTAINER" ]]; then
  if [[ "$ALLOW_ISOLATED_PARALLEL" == 1 ]]; then
    TARGET_ENV="$(
      docker inspect "$CONTAINER" \
        --format '{{range .Config.Env}}{{println .}}{{end}}'
    )"
    TARGET_ROS_MASTER="$(
      printf '%s\n' "$TARGET_ENV" | awk -F= '/^ROS_MASTER_URI=/{print $2; exit}'
    )"
    TARGET_GAZEBO_MASTER="$(
      printf '%s\n' "$TARGET_ENV" | awk -F= '/^GAZEBO_MASTER_URI=/{print $2; exit}'
    )"
    if [[ -n "$TARGET_ROS_MASTER" && -n "$TARGET_GAZEBO_MASTER" \
          && "$TARGET_ROS_MASTER" != 'http://127.0.0.1:11311' \
          && "$TARGET_GAZEBO_MASTER" != 'http://127.0.0.1:11345' ]]; then
      ISOLATED_OK=1
      for OTHER in "${SIMENV_CONTAINERS[@]}"; do
        [[ "$OTHER" == "$CONTAINER" ]] && continue
        OTHER_ENV="$(
          docker inspect "$OTHER" \
            --format '{{range .Config.Env}}{{println .}}{{end}}'
        )"
        if [[ "$OTHER_ENV" == *"ROS_MASTER_URI=$TARGET_ROS_MASTER"* \
              || "$OTHER_ENV" == *"GAZEBO_MASTER_URI=$TARGET_GAZEBO_MASTER"* ]]; then
          ISOLATED_OK=0
          break
        fi
      done
    fi
  fi
fi

if [[ ${#SIMENV_CONTAINERS[@]} -ne 1 || "${SIMENV_CONTAINERS[0]:-}" != "$CONTAINER" ]] \
    && [[ "$ISOLATED_OK" != 1 ]]; then
  echo '[exclusive-preflight] 检测到多个 SimEnv 容器或目标不唯一：本轮不能作为控制、导航或闭环验收。' >&2
  echo '[exclusive-preflight] 请由各容器拥有者停止遗留实例；本脚本不会替你执行 docker stop/rm。' >&2
  if [[ "$REQUIRE_EXCLUSIVE" == 1 ]]; then
    exit 3
  fi
elif [[ "$ISOLATED_OK" == 1 ]]; then
  echo '[exclusive-preflight] 隔离并行通过：目标使用独立 ROS/Gazebo master；该轮仍需标注存在算力竞争。'
else
  echo '[exclusive-preflight] 通过：当前仅目标 SimEnv 容器运行，可进入独占验收。'
fi
