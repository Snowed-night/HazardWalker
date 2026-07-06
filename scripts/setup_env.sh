#!/usr/bin/env bash
set -euo pipefail

echo "HazardWalker 主力机环境检查"
echo
echo "目标环境：Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic"
echo "本脚本只做检查和提示，不自动安装 NVIDIA 驱动、ROS 或 Gazebo。"
echo "大型依赖请按 docs/environment/主力机环境使用指南.md 中的说明手动安装。"
echo

check_cmd() {
  local name="$1"
  local cmd="$2"
  if command -v "$cmd" >/dev/null 2>&1; then
    echo "[OK] $name: $(command -v "$cmd")"
  else
    echo "[MISS] $name: 未找到 $cmd"
  fi
}

echo "== 系统信息 =="
lsb_release -a 2>/dev/null || true
uname -a
echo

echo "== 硬件与驱动 =="
check_cmd "NVIDIA 驱动检查工具" nvidia-smi
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi
else
  echo "未检测到 nvidia-smi。请先安装或修复 NVIDIA 驱动。"
fi
echo

echo "== 基础工具 =="
check_cmd "Git" git
check_cmd "Python 3" python3
check_cmd "pip" pip3
check_cmd "colcon" colcon
check_cmd "rosdep" rosdep
echo

echo "== ROS 2 / Gazebo =="
if [ -f /opt/ros/jazzy/setup.bash ]; then
  echo "[OK] ROS 2 Jazzy: /opt/ros/jazzy/setup.bash"
else
  echo "[MISS] ROS 2 Jazzy: 未找到 /opt/ros/jazzy/setup.bash"
fi
check_cmd "Gazebo" gz
echo

echo "== 项目验证命令 =="
echo "1. python3 scripts/run_offline_tests.py"
echo "2. ./scripts/build.sh"
echo "3. ./scripts/run_minimal_demo.sh"
