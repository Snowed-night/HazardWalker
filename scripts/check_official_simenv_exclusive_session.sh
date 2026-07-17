#!/usr/bin/env bash
# 官方 SimEnv 场景独占预检：只读取 Docker 状态，不停止、不删除任何容器。
# 负责人：姜晨。用于防止多套 Gazebo/ROS1 master 并发造成速度、里程计和实时性验收互相污染。
set -euo pipefail

CONTAINER="${SIMENV_CONTAINER:-simenv_run}"
REQUIRE_EXCLUSIVE=0

usage() {
  cat <<'USAGE'
用法：
  ./scripts/check_official_simenv_exclusive_session.sh [--container 名称] [--require-exclusive]

默认仅报告。传入 --require-exclusive 时，运行中的名称以 simenv 开头的容器必须恰好为目标容器一个；
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

# 官方验收容器统一以 simenv 命名；限制匹配范围，避免把无关 Docker 工作负载当作阻塞项。
mapfile -t SIMENV_CONTAINERS < <(docker ps --format '{{.Names}}' | grep -E '^simenv($|[_-])' || true)
printf '[exclusive-preflight] 目标容器：%s；运行中的 SimEnv 容器：%s\n' \
  "$CONTAINER" "${SIMENV_CONTAINERS[*]:-(无)}"
docker stats --no-stream --format '[exclusive-preflight] {{.Name}} cpu={{.CPUPerc}} mem={{.MemUsage}} pids={{.PIDs}}' \
  "${SIMENV_CONTAINERS[@]}" 2>/dev/null || true

if [[ ${#SIMENV_CONTAINERS[@]} -ne 1 || "${SIMENV_CONTAINERS[0]:-}" != "$CONTAINER" ]]; then
  echo '[exclusive-preflight] 检测到多个 SimEnv 容器或目标不唯一：本轮不能作为控制、导航或闭环验收。' >&2
  echo '[exclusive-preflight] 请由各容器拥有者停止遗留实例；本脚本不会替你执行 docker stop/rm。' >&2
  if [[ "$REQUIRE_EXCLUSIVE" == 1 ]]; then
    exit 3
  fi
else
  echo '[exclusive-preflight] 通过：当前仅目标 SimEnv 容器运行，可进入独占验收。'
fi
