#!/usr/bin/env python3
"""横向汇总同一 rosbag、同一人工标注下的感知回放结果。

所属组：感知定位组。负责人：姜晨。
文件作用：严格检查源会话、SEED 和标注文件哈希一致，再输出不同算法或参数
的候选、确认、耗时和定位指标 JSON/CSV；不把跨数据集结果伪装成控制变量实验。
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path


def compare_runs(run_dirs):
    rows = []
    expected_contract = None
    seen_variants = set()
    for value in run_dirs:
        run_dir = Path(value).expanduser().resolve()
        evaluation_path = run_dir / 'evaluation.json'
        replay_path = run_dir / 'replay_experiment_manifest.json'
        if not evaluation_path.is_file() or not replay_path.is_file():
            raise ValueError(f'结果目录缺少评估或回放清单：{run_dir}')
        evaluation = json.loads(evaluation_path.read_text(encoding='utf-8'))
        replay = json.loads(replay_path.read_text(encoding='utf-8'))
        inputs = evaluation.get('evaluation_inputs', {})
        if replay.get('status') != 'complete':
            raise ValueError(f'回放实验未完成：{run_dir}')
        if not inputs.get('replay_control_contract_verified'):
            raise ValueError(f'评估未验证实际回放控制变量：{run_dir}')
        if str(inputs.get('source_session', '')) != str(
                replay.get('source_session', '')):
            raise ValueError(f'评估与回放清单的源会话不一致：{run_dir}')
        if str(inputs.get('scenario_seed', '')) != str(
                replay.get('scenario_seed', '')):
            raise ValueError(f'评估与回放清单的 SEED 不一致：{run_dir}')
        if str(inputs.get('source_bag_fingerprint_sha256', '')) != str(
                replay.get('source_bag_fingerprint_sha256', '')):
            raise ValueError(f'评估与回放清单的 rosbag 指纹不一致：{run_dir}')
        contract = (
            str(replay.get('source_bag_fingerprint_sha256', '')),
            str(replay.get('scenario_seed', '')),
            str(inputs.get('annotation_file', {}).get('sha256', '')),
        )
        if not all(contract):
            raise ValueError(f'控制变量合同字段不完整：{run_dir}')
        if expected_contract is None:
            expected_contract = contract
        elif contract != expected_contract:
            raise ValueError('各轮不是同一 rosbag、同一 SEED 和同一标注文件')

        algorithm = evaluation.get('algorithm_run', {})
        git_state = algorithm.get('git', {})
        if not str(git_state.get('commit', '')).strip():
            raise ValueError(f'算法结果缺少 Git 提交：{run_dir}')
        if git_state.get('dirty') is not False:
            raise ValueError(f'算法结果来自未提交代码，禁止正式比较：{run_dir}')
        replay_git = replay.get('git', {})
        if replay_git.get('dirty') is not False:
            raise ValueError(f'实际回放来自未提交代码，禁止正式比较：{run_dir}')
        if replay_git.get('commit') != git_state.get('commit'):
            raise ValueError(f'评估与实际回放的 Git 提交不一致：{run_dir}')
        parameters = algorithm.get('parameter_files', [])
        parameter_hashes = tuple(item.get('sha256', '') for item in parameters)
        variant = (str(algorithm.get('label', '')), parameter_hashes)
        if variant in seen_variants:
            raise ValueError(f'重复算法/参数组合：{variant[0]}')
        seen_variants.add(variant)
        candidate = evaluation['candidate_metrics']
        confirmed = evaluation['confirmed_output_metrics']
        latency = evaluation['candidate_to_confirmation_latency_sec']
        localization = evaluation['localization_error_m']
        processing = evaluation['processing_time_ms']
        rows.append({
            'algorithm_label': variant[0],
            'parameter_sha256': ';'.join(parameter_hashes),
            'git_commit': str(algorithm.get('git', {}).get('commit', '')),
            'git_dirty': bool(algorithm.get('git', {}).get('dirty', False)),
            'candidate_true_positive': candidate.get('true_positive'),
            'candidate_false_positive': candidate.get('false_positive'),
            'candidate_false_negative': candidate.get('false_negative'),
            'candidate_precision': candidate['precision'],
            'candidate_recall': candidate['recall'],
            'candidate_f1_score': candidate['f1_score'],
            'confirmed_true_positive': confirmed.get('true_positive'),
            'confirmed_false_positive': confirmed.get('false_positive'),
            'confirmed_false_negative': confirmed.get('false_negative'),
            'confirmed_precision': confirmed['precision'],
            'confirmed_recall': confirmed['recall'],
            'confirmed_f1_score': confirmed['f1_score'],
            'confirmation_latency_count': latency.get('count'),
            'confirmation_latency_mean_sec': latency['mean'],
            'confirmation_latency_max_sec': latency.get('max'),
            'localization_reference_count': localization.get(
                'reference_count'),
            'localization_prediction_count': localization.get(
                'prediction_count'),
            'localization_error_count': localization.get('count'),
            'localization_coverage': localization['coverage'],
            'localization_error_mean_m': localization['mean'],
            'localization_error_median_m': localization.get('median'),
            'localization_error_max_m': localization.get('max'),
            'processing_time_count': processing.get('count'),
            'processing_time_mean_ms': processing['mean'],
            'processing_time_p95_ms': processing['p95'],
            'processing_time_max_ms': processing.get('max'),
            'observed_output_rate_hz': evaluation['observed_output_rate_hz'],
            'run_dir': str(run_dir),
        })
    rows.sort(key=lambda row: (
        -float(row['confirmed_f1_score']),
        -float(row['candidate_recall']),
        float('inf') if row['localization_error_mean_m'] is None
        else float(row['localization_error_mean_m']),
        float('inf') if row['processing_time_mean_ms'] is None
        else float(row['processing_time_mean_ms']),
        row['algorithm_label'],
    ))
    for index, row in enumerate(rows, start=1):
        row['comparison_rank'] = index
    source_fingerprint, seed, annotation_sha = (
        expected_contract or ('', '', ''))
    return {
        'schema': 'hazardwalker_perception_replay_comparison_v1',
        'generated_at_utc': datetime.now(timezone.utc).isoformat(),
        'source_bag_fingerprint_sha256': source_fingerprint,
        'scenario_seed': seed,
        'annotation_sha256': annotation_sha,
        'run_count': len(rows),
        'ranking_note': (
            '排序仅便于回归审阅：确认 F1、候选召回、定位误差、处理耗时；'
            '不替代官方总分。'),
        'runs': rows,
    }


def write_comparison(output_dir, comparison):
    output_dir = Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'replay_comparison.json').write_text(
        json.dumps(comparison, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    rows = comparison['runs']
    fieldnames = list(rows[0]) if rows else ['comparison_rank', 'algorithm_label']
    with (output_dir / 'replay_comparison.csv').open(
            'w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', action='append', required=True)
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    try:
        comparison = compare_runs(args.run_dir)
    except (ValueError, json.JSONDecodeError) as exc:
        raise SystemExit(str(exc)) from exc
    write_comparison(args.output_dir, comparison)
    print(json.dumps(comparison, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
