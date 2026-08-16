#!/usr/bin/env python3
"""分析一次导航 run 的时间分布，定位探索耗时瓶颈。

读取 NavRecorder 输出的 JSONL，计算：
- 总耗时、各状态时长占比
- 前进 / 原地转向 / 完全停等的时间占比（原地转向对应环视与 grace 空转）
- 路径总长、回头路（>120° 折返）次数
- 卡死 / 安全阻塞等失败次数

用法：
  python3 scripts/analyze_nav_run.py [run_dir]
  缺省 run_dir 时自动选择 reports/nav/ 下最新的 run_*。
"""

from __future__ import annotations

import json
import math
import os
import sys
from collections import Counter


def load_jsonl(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def find_latest_run(repo_root):
    base = os.path.join(repo_root, 'reports', 'nav')
    if not os.path.isdir(base):
        return None
    runs = [d for d in os.listdir(base) if d.startswith('run_')]
    if not runs:
        return None
    runs.sort()
    return os.path.join(base, runs[-1])


def analyze(run_dir):
    meta_path = os.path.join(run_dir, 'run_meta.json')
    meta = {}
    if os.path.exists(meta_path):
        with open(meta_path, encoding='utf-8') as f:
            meta = json.load(f)

    transitions = load_jsonl(os.path.join(run_dir, 'state_transitions.jsonl'))
    cmd = load_jsonl(os.path.join(run_dir, 'cmd_vel.jsonl'))
    traj = load_jsonl(os.path.join(run_dir, 'trajectory.jsonl'))
    failures = load_jsonl(os.path.join(run_dir, 'failures.jsonl'))

    print('=' * 62)
    print(f'Run: {os.path.basename(run_dir)}')
    print(f'  final_state: {meta.get("final_state", "?")}')
    print(f'  total_frontiers_visited: '
          f'{meta.get("total_frontiers_visited", "?")}')

    total = None
    if traj:
        t0 = traj[0].get('ros_sec', 0.0)
        t1 = traj[-1].get('ros_sec', 0.0)
        total = t1 - t0
        if total > 0:
            print(f'\n总耗时（轨迹首尾）: {total:.1f}s = {total / 60:.2f}min')

    # 状态时长：trajectory 是 10Hz 降采样，用 ros_sec 差更准确。
    if traj and total:
        print('\n状态时长分布（按 ros_sec 连续段）:')
        seg = {'state': traj[0].get('state', '?'), 't0': traj[0].get('ros_sec', 0.0)}
        durations = Counter()
        for r in traj[1:]:
            st = r.get('state', '?')
            ts = r.get('ros_sec', 0.0)
            if st != seg['state']:
                durations[seg['state']] += max(0.0, ts - seg['t0'])
                seg = {'state': st, 't0': ts}
        durations[seg['state']] += max(0.0, t1 - seg['t0'])
        for state, dur in durations.most_common():
            print(f'  {state:15s}: {dur:8.1f}s ({dur / total * 100:4.1f}%)')

    # 速度指令分布：原地转向 ≈ 环视 / grace 空转 / 对准。
    if cmd:
        n = len(cmd)
        forward = sum(1 for r in cmd if abs(r.get('linear_x', 0)) > 0.02)
        turn_only = sum(
            1 for r in cmd
            if abs(r.get('linear_x', 0)) <= 0.02
            and abs(r.get('angular_z', 0)) > 0.05
        )
        still = sum(1 for r in cmd if not r.get('moving'))
        # cmd_vel 记录频率不是固定 10Hz（实测约 38Hz），不能用 0.1s/帧估算；
        # 用 ros_sec 首尾跨度除以间隔数得到真实帧间隔，秒数才可信。
        if n >= 2:
            t0 = cmd[0].get('ros_sec', 0.0)
            t1 = cmd[-1].get('ros_sec', 0.0)
            dt = (t1 - t0) / max(1, n - 1)
        else:
            dt = 0.1
        print(f'\n速度指令分布（实际≈{1.0 / dt:.1f}Hz，每帧≈{dt:.3f}s）:')
        print(f'  前进:     {forward * dt:7.1f}s '
              f'({forward / n * 100:4.1f}%)')
        print(f'  原地转向: {turn_only * dt:7.1f}s '
              f'({turn_only / n * 100:4.1f}%)')
        print(f'  完全停等: {still * dt:7.1f}s '
              f'({still / n * 100:4.1f}%)')

    # 路径指标 + 回头路。
    if traj and len(traj) > 1:
        path_len = 0.0
        backtrack = 0
        skipped = 0
        prev_dx = prev_dy = None
        # 轨迹 10Hz 降采样、前进 0.35m/s 时单步 ≈0.035m；SLAM 返航漂移会制造
        # 几米级的定位跳变，把路径总长虚高到物理不可能。超过 1m 的单步按跳变
        # 跳过，并断开回头路检测的前后衔接。
        for i in range(1, len(traj)):
            dx = traj[i].get('x', 0.0) - traj[i - 1].get('x', 0.0)
            dy = traj[i].get('y', 0.0) - traj[i - 1].get('y', 0.0)
            seg = math.hypot(dx, dy)
            if seg > 1.0:
                skipped += 1
                prev_dx = prev_dy = None
                continue
            path_len += seg
            if prev_dx is not None and seg > 0.02:
                prev_len = math.hypot(prev_dx, prev_dy)
                if prev_len > 0.02:
                    cosang = (prev_dx * dx + prev_dy * dy) / (prev_len * seg)
                    if cosang < -0.5:  # >120° 折返
                        backtrack += 1
            if seg > 0.02:
                prev_dx, prev_dy = dx, dy
        start_end = math.hypot(
            traj[-1].get('x', 0.0) - traj[0].get('x', 0.0),
            traj[-1].get('y', 0.0) - traj[0].get('y', 0.0),
        )
        print('\n路径指标:')
        print(f'  路径总长: {path_len:.1f}m')
        print(f'  起点→终点直线: {start_end:.1f}m')
        print(f'  回头路(>120°折返)次数: {backtrack}')
        if skipped:
            print(f'  跳过的定位跳变帧(>1m): {skipped}')

    if transitions:
        print(f'\n状态变迁次数: {len(transitions)}')
        print('  时间线:')
        for tr in transitions:
            print(f'    {tr.get("ros_sec", 0):8.1f}s  '
                  f'{tr.get("prev_state", "?")} → {tr.get("new_state", "?")}')

    if failures:
        fcounter = Counter(f.get('type', '?') for f in failures)
        print('\n失败/异常:')
        for ftype, cnt in fcounter.most_common():
            print(f'  {ftype}: {cnt} 次')

    print('=' * 62)


def main():
    if len(sys.argv) > 1:
        run_dir = sys.argv[1]
    else:
        repo_root = os.environ.get(
            'HAZARDWALKER_ROOT',
            os.path.join(os.path.expanduser('~'), 'HazardWalker'),
        )
        run_dir = find_latest_run(repo_root)
    if not run_dir or not os.path.isdir(run_dir):
        print('未找到 run 目录。用法: python3 scripts/analyze_nav_run.py <run_dir>')
        sys.exit(1)
    analyze(run_dir)


if __name__ == '__main__':
    main()
