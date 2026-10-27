#!/usr/bin/env bash
# 核验旧版源码和环境；--source-only 可在没有 ROS/Docker 时审阅源码。
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$ROOT/scripts/reproduction/verify.py" "$@"
