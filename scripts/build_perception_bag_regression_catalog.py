#!/usr/bin/env python3
"""汇总多个固定 SEED 人工巡检 rosbag，形成动态感知回归集目录。

所属组：感知定位组。负责人：姜晨。
文件作用：只登记已完成且逐话题校验通过的数据集，并强制每个 SEED 只有一
份基准巡检，输出 JSON/CSV 索引。原始 rosbag 仍留在主机数据盘，不提交 Git。
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / 'scripts'
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from run_perception_replay_experiment import load_valid_session  # noqa: E402


def build_catalog(session_dirs, *, minimum_seed_count=3):
    """校验并按 SEED 排序；正式动态回归集至少覆盖三个不同场景。"""

    if int(minimum_seed_count) < 2:
        raise ValueError('minimum_seed_count 必须至少为 2')

    entries = []
    seen_seeds = {}
    for value in session_dirs:
        session_dir = Path(value).expanduser().resolve()
        manifest = load_valid_session(session_dir)
        git_state = manifest.get('git', {})
        if not str(git_state.get('commit', '')).strip():
            raise ValueError(f'数据集缺少 Git 提交：{session_dir}')
        if git_state.get('dirty') is not False:
            raise ValueError(f'数据集来自未提交代码：{session_dir}')
        seed = str(manifest.get('scenario_seed', '')).strip()
        if not seed:
            raise ValueError(f'数据集缺少 scenario_seed：{session_dir}')
        if seed in seen_seeds:
            raise ValueError(
                f'SEED {seed} 出现多份基准巡检：'
                f'{seen_seeds[seed]}；{session_dir}')
        seen_seeds[seed] = session_dir
        bag_dir = session_dir / str(manifest.get('bag_relative_path', 'bag'))
        bag_size = sum(
            path.stat().st_size for path in bag_dir.rglob('*') if path.is_file())
        counts = manifest.get('bag_validation', {}).get('message_counts', {})
        coverage = manifest.get('patrol_coverage', {}).get('metrics', {})
        entries.append({
            'scenario_seed': seed,
            'run_id': str(manifest.get('run_id', session_dir.name)),
            'session_dir': str(session_dir),
            'bag_relative_path': str(manifest.get('bag_relative_path', 'bag')),
            'bag_size_bytes': bag_size,
            'bag_fingerprint_sha256': str(
                manifest.get('bag_validation', {}).get(
                    'content_fingerprint_sha256', '')),
            'started_at_utc': manifest.get('started_at_utc'),
            'finished_at_utc': manifest.get('finished_at_utc'),
            'operator': str(manifest.get('operator', '')),
            'git_commit': str(manifest.get('git', {}).get('commit', '')),
            'git_dirty': bool(manifest.get('git', {}).get('dirty', False)),
            'localization_provenance': str(
                manifest.get('localization_provenance', 'unverified')),
            'runtime_localization_provenance': str(
                manifest.get('runtime_localization_provenance', '')),
            'historical_localization_reuse_eligible': bool(
                manifest.get('historical_localization_reuse_eligible', False)),
            'rgb_message_count': int(counts.get('/hw/camera/image_raw', 0)),
            'depth_message_count': int(counts.get('/hw/camera/depth_image', 0)),
            'scan_message_count': int(counts.get('/hw/scan', 0)),
            'imu_message_count': int(counts.get('/hw/trunk_imu', 0)),
            'slam_odometry_message_count': int(
                counts.get('/hazardwalker/slam/odometry', 0)),
            'slam_provenance_message_count': int(counts.get(
                '/hazardwalker/slam/localization_provenance', 0)),
            'map_message_count': int(counts.get('/map', 0)),
            'detection_message_count': int(
                counts.get('/hw/perception/hazard_detections', 0)),
            'control_message_count': int(counts.get('/hw/cmd_vel', 0)),
            'patrol_coverage_message_count': int(
                counts.get('/hw/perception/patrol_coverage', 0)),
            'patrol_path_length_m': float(
                coverage.get('planar_path_length_m', 0.0)),
            'patrol_planar_span_m': float(
                coverage.get('planar_span_m', 0.0)),
            'patrol_vertical_span_m': float(
                coverage.get('vertical_span_m', 0.0)),
            'duration_sec': float(
                manifest.get('bag_validation', {}).get('duration_sec', 0.0)),
            'validation_status': 'passed',
        })
    entries.sort(key=lambda item: item['scenario_seed'])
    if len(entries) < int(minimum_seed_count):
        raise ValueError(
            f'动态回归集至少需要 {int(minimum_seed_count)} 个不同 SEED，'
            f'当前只有 {len(entries)} 个')
    return {
        'schema': 'hazardwalker_perception_bag_regression_v1',
        'generated_at_utc': datetime.now(timezone.utc).isoformat(),
        'seed_count': len(entries),
        'one_baseline_patrol_per_seed': True,
        'truth_inputs_used': False,
        'sessions': entries,
    }


def write_catalog(output_dir: Path, catalog: dict) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'regression_catalog.json').write_text(
        json.dumps(catalog, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    rows = catalog['sessions']
    fieldnames = list(rows[0]) if rows else [
        'scenario_seed', 'run_id', 'session_dir', 'validation_status']
    with (output_dir / 'regression_catalog.csv').open(
            'w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', action='append', required=True)
    parser.add_argument('--min-seeds', type=int, default=3)
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    try:
        catalog = build_catalog(
            args.session, minimum_seed_count=args.min_seeds)
    except (ValueError, json.JSONDecodeError) as exc:
        raise SystemExit(str(exc)) from exc
    write_catalog(Path(args.output_dir).expanduser().resolve(), catalog)
    print(json.dumps(catalog, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
