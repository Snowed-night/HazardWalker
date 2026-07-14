#!/usr/bin/env bash
# 兼容旧入口：官方容器没有 dynamic_bridge，实际实现已迁移为 ROS2 主机 rosbridge 适配器。
set -euo pipefail

# 负责人：姜晨。保留文件名防止旧文档/自动化失效，不再向 ROS1 容器复制错误的动态桥代码。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo '[adapter] 旧入口已转向 ROS2 主机 rosbridge 适配器。'
exec "$ROOT/scripts/run_official_simenv_rosbridge_adapter.sh" "$@"
