#!/usr/bin/env bash
# 构建六个 ROS2 包和官方 ROS1 控制器，运行离线检查。
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ROOT/scripts/reproduction/build.sh" "$@"
