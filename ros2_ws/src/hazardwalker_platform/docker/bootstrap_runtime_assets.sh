#!/usr/bin/env bash
# 从本机官方 SimEnv 工作区补齐 Git 忽略的 Unitree 运行库和策略模型。
set -euo pipefail

PLATFORM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST_SRC="$PLATFORM_ROOT/src"

runtime_assets_ready() {
  [[ -f "$DEST_SRC/unitree_guide/logs/policy_act_inference_plane.pt" && \
     -f "$DEST_SRC/unitree_guide/logs/policy_act_inference_stair.pt" && \
     -f "$DEST_SRC/unitree_guide/unitree_guide/unitree_guide/library/unitree_legged_sdk-3.8.0/lib/cpp/amd64/libunitree_legged_sdk.a" && \
     -f "$DEST_SRC/unitree_guide/unitree_guide/unitree_actuator_sdk/unitree_motor_ctrl/lib/libUnitree_motor_SDK_Linux.so" ]]
}

runtime_assets_ready && exit 0

SOURCE_ROOT="${OFFICIAL_SIMENV_SOURCE_ROOT:-}"
if [[ -z "$SOURCE_ROOT" ]]; then
  for candidate in "$HOME/桌面/SimEnv" "$HOME/Desktop/SimEnv"; do
    if [[ -d "$candidate/src/unitree_guide" ]]; then
      SOURCE_ROOT="$candidate"
      break
    fi
  done
fi

if [[ ! -d "$SOURCE_ROOT/src/unitree_guide" ]]; then
  echo "ERROR: Git 忽略的 Unitree 运行资产缺失，且未找到官方 SimEnv 工作区。" >&2
  echo "请设置 OFFICIAL_SIMENV_SOURCE_ROOT=/path/to/SimEnv 后重试。" >&2
  exit 1
fi

SOURCE_SRC="$SOURCE_ROOT/src"
while IFS= read -r -d '' source_file; do
  relative_path="${source_file#"$SOURCE_SRC/"}"
  destination="$DEST_SRC/$relative_path"
  mkdir -p "$(dirname "$destination")"
  cp -p "$source_file" "$destination"
done < <(find "$SOURCE_SRC/unitree_guide" -type f \
  \( -name '*.so' -o -name '*.a' -o -name '*.pt' -o -name '*.pth' \) -print0)

if ! runtime_assets_ready; then
  echo "ERROR: $SOURCE_ROOT 不包含完整的 Unitree 控制运行资产。" >&2
  exit 1
fi
echo "Unitree 运行资产已从官方 SimEnv 工作区同步；这些二进制不会写入 Git。"
