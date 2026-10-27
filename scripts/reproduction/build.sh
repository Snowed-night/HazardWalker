#!/usr/bin/env bash
# 构建源码并运行核心离线测试。ROS1只使用已准备的镜像，不在此下载大型依赖。
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ "${1:-}" == --help ]]; then
  echo 'Usage: bash build.sh [--ros2-only]'; exit 0
fi
MODE="${1:-all}"
[[ "$MODE" == all || "$MODE" == --ros2-only ]] || { echo '参数错误'; exit 2; }
cd "$ROOT"
python3 scripts/reproduction/verify.py --source-only
python3 scripts/reproduction/prepare_assets.py
python3 - "$ROOT" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1])/'ros2_ws/src/hazardwalker_platform/src/CMakeLists.txt'
target='/opt/ros/noetic/share/catkin/cmake/toplevel.cmake'
if p.is_symlink():
    assert str(p.readlink())==target
else:
    assert p.is_file() and p.read_text().strip()==target
    p.unlink();p.symlink_to(target)
PY
set +u
source /opt/ros/jazzy/setup.bash
set -u
unset COLCON_CURRENT_PREFIX
colcon build --symlink-install --parallel-workers 2 --base-paths ros2_ws/src --packages-select \
  hazardwalker_bringup hazardwalker_decision hazardwalker_msgs hazardwalker_nav hazardwalker_perception hazardwalker_platform
PYTHONDONTWRITEBYTECODE=1 python3 scripts/run_offline_tests.py
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests/reproduction -v
if [[ "$MODE" == all ]]; then
  python3 scripts/reproduction/verify.py
  python3 scripts/reproduction/build_noetic.py
fi
python3 scripts/reproduction/verify.py --source-only
echo '构建完成。完整实验入口：bash run.sh 202609102'
