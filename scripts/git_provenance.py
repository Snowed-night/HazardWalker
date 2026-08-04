#!/usr/bin/env python3
"""统一读取实验代码的 Git 来源，并排除规范成果与构建产物。

所属组：感知定位组。负责人：姜晨。
文件作用：为正式感知回放、人工标注评估和横向比较提供一致的分支、提交与
代码脏状态定义。实验图片、测试表和 ROS 构建目录不属于算法代码改动。
"""

from __future__ import annotations

from pathlib import Path
import subprocess


ARTIFACT_PREFIXES = (
    'reports/perception/',
    'results/',
    'ros2_ws/build/',
    'ros2_ws/install/',
    'ros2_ws/log/',
    # 官方 auto.sh 会按固定 SEED 覆盖这些挂载回宿主机的运行目录，其中还
    # 包含仓库历史上误跟踪的场景快照。它们不是算法代码；生成器源码、启动
    # 参数和配置仍不在排除范围内。感知程序也不得读取其中的真值文件。
    'ros2_ws/src/hazardwalker_platform/generated_building/',
    'ros2_ws/src/hazardwalker_platform/results/',
    'ros2_ws/src/hazardwalker_platform/logs/',
    'ros2_ws/src/hazardwalker_platform/.ros1_catkin_ws/',
)


def _normalize_status_path(value: str) -> str:
    """规范化 porcelain 路径；兼容 Windows 分隔符和 rename 记录。"""

    path = str(value).strip().strip('"').replace('\\', '/')
    if ' -> ' in path:
        path = path.rsplit(' -> ', 1)[-1].strip().strip('"')
    return path.removeprefix('./')


def code_dirty_entries(porcelain_output: str) -> list[str]:
    """从 Git porcelain 输出筛出会改变算法或运行合同的文件。"""

    entries = []
    for raw_line in str(porcelain_output).splitlines():
        if not raw_line.strip():
            continue
        # porcelain v1 的前两列是 XY 状态，第三列起才是路径。
        path = _normalize_status_path(
            raw_line[3:] if len(raw_line) >= 4 else raw_line)
        if not path or any(path.startswith(prefix) for prefix in ARTIFACT_PREFIXES):
            continue
        entries.append(path)
    return sorted(set(entries))


def _git_value(repo_root: Path, *args: str) -> str:
    try:
        return subprocess.check_output(
            ['git', *args], cwd=repo_root, text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return ''


def read_git_state(repo_root: Path) -> dict:
    """返回可写入实验清单的代码来源；无法解析 Git 时按不可信处理。"""

    root = Path(repo_root).resolve()
    commit = _git_value(root, 'rev-parse', 'HEAD')
    porcelain = _git_value(root, 'status', '--porcelain', '--untracked-files=all')
    entries = code_dirty_entries(porcelain)
    return {
        'branch': _git_value(root, 'branch', '--show-current'),
        'commit': commit,
        'dirty': bool(entries) or not bool(commit),
        'dirty_entries': entries,
    }
