#!/usr/bin/env bash
# 每次调用启动一轮三层实验；缺省使用旧版历史成功种子202609102。
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$ROOT/scripts/reproduction/run.py" "$@"
