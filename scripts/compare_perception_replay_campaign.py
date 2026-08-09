#!/usr/bin/env python3
"""跨多个固定 SEED 汇总同一批感知算法回放结果。

所属组：感知定位组。负责人：姜晨。
文件作用：先复用单 SEED 比较器验证每个场景的 rosbag、标注和回放合同，
再要求每个算法/参数/提交组合完整覆盖同一组 SEED，输出微平均指标、最差
场景指标和逐场景明细。缺场景或事后只挑成功运行会直接失败。
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from compare_perception_replay_runs import compare_runs


def compare_campaign(run_dirs, *, minimum_seed_count=3, minimum_variant_count=2):
    """验证完整场景×算法矩阵并生成跨 SEED 汇总。"""

    if int(minimum_seed_count) < 2:
        raise ValueError('minimum_seed_count 必须至少为 2')
    if int(minimum_variant_count) < 2:
        raise ValueError('minimum_variant_count 必须至少为 2')
    grouped: dict[str, list[Path]] = {}
    for value in run_dirs:
        run_dir = Path(value).expanduser().resolve()
        manifest_path = run_dir / 'replay_experiment_manifest.json'
        if not manifest_path.is_file():
            raise ValueError(f'结果目录缺少回放清单：{run_dir}')
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        seed = str(manifest.get('scenario_seed', '')).strip()
        if not seed:
            raise ValueError(f'回放清单缺少 SEED：{run_dir}')
        grouped.setdefault(seed, []).append(run_dir)

    if len(grouped) < int(minimum_seed_count):
        raise ValueError(
            f'跨场景比较至少需要 {int(minimum_seed_count)} 个不同 SEED，'
            f'当前只有 {len(grouped)} 个')

    per_seed = {}
    expected_variants = None
    variant_rows: dict[tuple[str, str, str], list[dict]] = {}
    for seed in sorted(grouped):
        comparison = compare_runs(grouped[seed])
        rows = comparison['runs']
        variants = {
            _variant_key(row) for row in rows
        }
        if len(variants) < int(minimum_variant_count):
            raise ValueError(
                f'SEED {seed} 至少需要 {int(minimum_variant_count)} 个算法/参数组合')
        if expected_variants is None:
            expected_variants = variants
        elif variants != expected_variants:
            missing = sorted(expected_variants - variants)
            extra = sorted(variants - expected_variants)
            raise ValueError(
                f'SEED {seed} 的算法矩阵不完整；缺少={missing}，额外={extra}')
        per_seed[seed] = {
            'source_bag_fingerprint_sha256': comparison[
                'source_bag_fingerprint_sha256'],
            'annotation_sha256': comparison['annotation_sha256'],
            'run_count': comparison['run_count'],
        }
        for row in rows:
            variant_rows.setdefault(_variant_key(row), []).append({
                **row,
                'scenario_seed': seed,
            })

    seed_set = set(grouped)
    summaries = []
    for variant, rows in variant_rows.items():
        observed = {row['scenario_seed'] for row in rows}
        if observed != seed_set or len(rows) != len(seed_set):
            raise ValueError(
                f'算法 {variant[0]} 未完整覆盖全部 SEED：'
                f'{sorted(observed)} != {sorted(seed_set)}')
        summaries.append(_summarize_variant(variant, rows))

    summaries.sort(key=lambda row: (
        -float(row['confirmed_f1_score']),
        -float(row['candidate_recall']),
        -float(row['worst_seed_confirmed_f1']),
        _none_last(row['localization_error_mean_m']),
        _none_last(row['processing_time_mean_ms']),
        row['algorithm_label'],
    ))
    for rank, summary in enumerate(summaries, start=1):
        summary['campaign_rank'] = rank
    return {
        'schema': 'hazardwalker_perception_replay_campaign_v1',
        'generated_at_utc': datetime.now(timezone.utc).isoformat(),
        'seed_count': len(seed_set),
        'variant_count': len(summaries),
        'complete_seed_variant_matrix': True,
        'truth_inputs_used_at_runtime': False,
        'ranking_note': (
            '排序仅用于跨随机场景回归审阅；优先确认 F1、候选召回、'
            '最差场景确认 F1、定位误差和处理耗时，不替代官方总分。'),
        'seed_contracts': per_seed,
        'variants': summaries,
    }


def _variant_key(row):
    return (
        str(row.get('algorithm_label', '')),
        str(row.get('parameter_sha256', '')),
        str(row.get('git_commit', '')),
    )


def _summarize_variant(variant, rows):
    candidate_tp = sum(int(row.get('candidate_true_positive') or 0) for row in rows)
    candidate_fp = sum(int(row.get('candidate_false_positive') or 0) for row in rows)
    candidate_fn = sum(int(row.get('candidate_false_negative') or 0) for row in rows)
    confirmed_tp = sum(int(row.get('confirmed_true_positive') or 0) for row in rows)
    confirmed_fp = sum(int(row.get('confirmed_false_positive') or 0) for row in rows)
    confirmed_fn = sum(int(row.get('confirmed_false_negative') or 0) for row in rows)
    candidate = _micro_metrics(candidate_tp, candidate_fp, candidate_fn)
    confirmed = _micro_metrics(confirmed_tp, confirmed_fp, confirmed_fn)
    latency_mean = _weighted_mean(
        rows, 'confirmation_latency_mean_sec', 'confirmation_latency_count')
    localization_mean = _weighted_mean(
        rows, 'localization_error_mean_m', 'localization_error_count')
    processing_mean = _weighted_mean(
        rows, 'processing_time_mean_ms', 'processing_time_count')
    rates = [float(row['observed_output_rate_hz']) for row in rows]
    return {
        'algorithm_label': variant[0],
        'parameter_sha256': variant[1],
        'git_commit': variant[2],
        'seed_count': len(rows),
        'candidate_true_positive': candidate_tp,
        'candidate_false_positive': candidate_fp,
        'candidate_false_negative': candidate_fn,
        'candidate_precision': candidate['precision'],
        'candidate_recall': candidate['recall'],
        'candidate_f1_score': candidate['f1_score'],
        'confirmed_true_positive': confirmed_tp,
        'confirmed_false_positive': confirmed_fp,
        'confirmed_false_negative': confirmed_fn,
        'confirmed_precision': confirmed['precision'],
        'confirmed_recall': confirmed['recall'],
        'confirmed_f1_score': confirmed['f1_score'],
        'confirmation_latency_mean_sec': latency_mean,
        'confirmation_latency_worst_max_sec': _finite_max(
            rows, 'confirmation_latency_max_sec'),
        'localization_reference_count': sum(
            int(row.get('localization_reference_count') or 0) for row in rows),
        'localization_prediction_count': sum(
            int(row.get('localization_prediction_count') or 0) for row in rows),
        'localization_error_count': sum(
            int(row.get('localization_error_count') or 0) for row in rows),
        'localization_error_mean_m': localization_mean,
        'localization_error_worst_max_m': _finite_max(
            rows, 'localization_error_max_m'),
        'processing_time_mean_ms': processing_mean,
        'processing_time_worst_p95_ms': _finite_max(
            rows, 'processing_time_p95_ms'),
        'processing_time_worst_max_ms': _finite_max(
            rows, 'processing_time_max_ms'),
        'output_rate_mean_hz': round(sum(rates) / len(rates), 6),
        'output_rate_min_hz': round(min(rates), 6),
        'worst_seed_candidate_recall': min(
            float(row['candidate_recall']) for row in rows),
        'worst_seed_confirmed_f1': min(
            float(row['confirmed_f1_score']) for row in rows),
        'worst_seed_localization_coverage': min(
            float(row['localization_coverage']) for row in rows),
        'seed_metrics': [
            {
                'scenario_seed': row['scenario_seed'],
                'candidate_recall': row['candidate_recall'],
                'confirmed_f1_score': row['confirmed_f1_score'],
                'localization_coverage': row['localization_coverage'],
                'localization_error_mean_m': row['localization_error_mean_m'],
                'processing_time_p95_ms': row['processing_time_p95_ms'],
                'observed_output_rate_hz': row['observed_output_rate_hz'],
                'run_dir': row['run_dir'],
            }
            for row in sorted(rows, key=lambda item: item['scenario_seed'])
        ],
    }


def _micro_metrics(true_positive, false_positive, false_negative):
    precision = _safe_divide(true_positive, true_positive + false_positive)
    recall = _safe_divide(true_positive, true_positive + false_negative)
    return {
        'precision': round(precision, 6),
        'recall': round(recall, 6),
        'f1_score': round(_safe_divide(2 * precision * recall, precision + recall), 6),
    }


def _weighted_mean(rows, value_key, count_key):
    weighted_sum = 0.0
    total_count = 0
    for row in rows:
        value = row.get(value_key)
        count = int(row.get(count_key) or 0)
        if value is None or count <= 0:
            continue
        number = float(value)
        if not math.isfinite(number):
            continue
        weighted_sum += number * count
        total_count += count
    return round(weighted_sum / total_count, 6) if total_count else None


def _finite_max(rows, key):
    values = []
    for row in rows:
        value = row.get(key)
        if value is None:
            continue
        number = float(value)
        if math.isfinite(number):
            values.append(number)
    return round(max(values), 6) if values else None


def _safe_divide(numerator, denominator):
    return float(numerator) / float(denominator) if denominator else 0.0


def _none_last(value):
    return float('inf') if value is None else float(value)


def write_campaign(output_dir, campaign):
    output_dir = Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'replay_campaign_comparison.json').write_text(
        json.dumps(campaign, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    rows = []
    for item in campaign['variants']:
        rows.append({key: value for key, value in item.items()
                     if key != 'seed_metrics'})
    fieldnames = list(rows[0]) if rows else [
        'campaign_rank', 'algorithm_label', 'seed_count']
    with (output_dir / 'replay_campaign_comparison.csv').open(
            'w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', action='append', required=True)
    parser.add_argument('--min-seeds', type=int, default=3)
    parser.add_argument('--min-variants', type=int, default=2)
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    try:
        campaign = compare_campaign(
            args.run_dir,
            minimum_seed_count=args.min_seeds,
            minimum_variant_count=args.min_variants,
        )
    except (ValueError, json.JSONDecodeError) as exc:
        raise SystemExit(str(exc)) from exc
    write_campaign(args.output_dir, campaign)
    print(json.dumps(campaign, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
