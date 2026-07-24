#!/usr/bin/env python3
"""汇总官方随机场景感知活动，并阻止漏跑 SEED 或挑选成功结果。

本脚本只在各轮运行和独立赛后评测结束后读取归档文件，不读取仿真运行中的
真值。活动清单必须在运行前登记至少三个 SEED、同一 Git 提交和参数文件哈希。
"""

import argparse
import csv
import hashlib
import importlib.util
import json
import re
from pathlib import Path


SCHEMA = 'hazardwalker_official_random_perception_campaign_v1'
GIT_SHA_PATTERN = re.compile(r'^[0-9a-f]{7,40}$')
SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')


def sha256_file(path):
    """返回文件 SHA-256，供活动清单绑定实际参数快照。"""

    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def summarize_campaign(manifest_path, structure_validator=None):
    """校验活动清单及逐轮证据，返回可直接写入 JSON 的汇总。"""

    manifest_path = Path(manifest_path).expanduser().resolve()
    manifest = _read_json(manifest_path)
    root = manifest_path.parent
    errors = []
    if structure_validator is None:
        structure_validator = _load_structure_validator()

    if manifest.get('schema') != SCHEMA:
        errors.append('unexpected_campaign_schema')
    campaign_id = str(manifest.get('campaign_id', '')).strip()
    if not campaign_id:
        errors.append('missing_campaign_id')

    seeds = [str(value).strip() for value in manifest.get('pre_registered_seeds', [])]
    if len(seeds) < 3:
        errors.append('fewer_than_three_pre_registered_seeds')
    if any(not seed for seed in seeds) or len(set(seeds)) != len(seeds):
        errors.append('invalid_or_duplicate_pre_registered_seeds')

    code_version = str(manifest.get('code_version', '')).strip().lower()
    if not GIT_SHA_PATTERN.fullmatch(code_version):
        errors.append('code_version_is_not_a_clean_git_commit')

    config_sha256 = str(manifest.get('config_sha256', '')).strip().lower()
    if not SHA256_PATTERN.fullmatch(config_sha256):
        errors.append('invalid_config_sha256')

    runs = manifest.get('runs', [])
    if not isinstance(runs, list):
        errors.append('runs_not_a_list')
        runs = []
    run_seeds = [str(run.get('seed', '')).strip() for run in runs if isinstance(run, dict)]
    if sorted(run_seeds) != sorted(seeds):
        errors.append('runs_do_not_exactly_match_pre_registered_seeds')

    run_reports = []
    for run in runs:
        if not isinstance(run, dict):
            errors.append('invalid_run_entry')
            continue
        report = _summarize_run(
            root=root,
            run=run,
            expected_code_version=code_version,
            expected_config_sha256=config_sha256,
            structure_validator=structure_validator,
        )
        run_reports.append(report)
        errors.extend(
            'seed_%s:%s' % (report['seed'], error)
            for error in report['errors']
        )

    valid_metrics = [
        report for report in run_reports
        if report['official_metrics_available']
    ]
    aggregate = _aggregate_metrics(valid_metrics)
    campaign_pass = (
        not errors
        and len(run_reports) == len(seeds)
        and all(report['run_pass'] for report in run_reports)
    )
    return {
        'schema': SCHEMA,
        'campaign_id': campaign_id,
        'pre_registered_seeds': seeds,
        'code_version': code_version,
        'config_sha256': config_sha256,
        'run_count': len(run_reports),
        'campaign_pass': campaign_pass,
        'errors': sorted(set(errors)),
        'aggregate': aggregate,
        'runs': run_reports,
        'scope_limitations': [
            '本汇总不在运行期读取真值；evaluation_result.json 必须由任务停止后的官方评测脚本生成。',
            '活动通过仅证明预注册随机场景结果满足结构和官方客观指标门槛，不替代导航全楼覆盖审计。',
        ],
    }


def _summarize_run(
        root,
        run,
        expected_code_version,
        expected_config_sha256,
        structure_validator):
    seed = str(run.get('seed', '')).strip()
    errors = []
    evidence_dir = _resolve_inside(root, run.get('evidence_dir'), errors, 'evidence_dir')
    test_record_dir = _resolve_inside(
        root,
        run.get('test_record_dir', run.get('evidence_dir')),
        errors,
        'test_record_dir',
    )
    config_snapshot = _resolve_inside(
        root,
        run.get('config_snapshot'),
        errors,
        'config_snapshot',
    )

    required = {}
    if evidence_dir is not None:
        required.update({
            'run_manifest': evidence_dir / 'run_manifest.json',
            'summary': evidence_dir / 'summary.json',
            'result': evidence_dir / 'detected_danger.json',
            'validation': evidence_dir / 'independent_post_evaluation.json',
            'evaluation': evidence_dir / 'evaluation_result.json',
        })
    if test_record_dir is not None:
        required.update({
            'test_record_csv': test_record_dir / 'testing_record_perception.csv',
            'test_record_json': test_record_dir / 'testing_record_perception.json',
        })
    for name, path in required.items():
        if not path.is_file() or path.stat().st_size <= 0:
            errors.append('missing_or_empty_' + name)

    if config_snapshot is None or not config_snapshot.is_file():
        errors.append('missing_config_snapshot')
    elif sha256_file(config_snapshot) != expected_config_sha256:
        errors.append('config_snapshot_hash_mismatch')

    if errors:
        return _run_report(seed, errors=errors)

    run_manifest = _read_json(required['run_manifest'])
    summary = _read_json(required['summary'])
    result = _read_json(required['result'])
    validation = _read_json(required['validation'])
    evaluation = _read_json(required['evaluation'])
    rebuilt_validation = structure_validator(
        evidence_dir,
        required['result'],
        True,
    )

    contract = run_manifest.get('evidence_contract', {})
    if str(contract.get('scenario_seed', '')).strip() != seed:
        errors.append('manifest_seed_mismatch')
    if str(contract.get('code_version', '')).strip().lower() != expected_code_version:
        errors.append('manifest_code_version_mismatch')
    if not contract.get('formal_evidence_eligible'):
        errors.append('formal_evidence_contract_rejected')
    if contract.get('truth_inputs_used') is not False:
        errors.append('truth_input_declaration_invalid')
    if summary.get('mission_completed') is not True:
        errors.append('mission_not_completed')
    if validation.get('structural_evidence_complete') is not True:
        errors.append('independent_structure_validation_failed')
    if validation.get('active_reobservation_required') is not True:
        errors.append('active_reobservation_was_not_required')
    if validation.get('errors'):
        errors.append('independent_validation_has_errors')
    if rebuilt_validation.get('structural_evidence_complete') is not True:
        errors.append('rebuilt_structure_validation_failed')
    if rebuilt_validation.get('active_reobservation_required') is not True:
        errors.append('rebuilt_validation_did_not_require_active_reobservation')
    if rebuilt_validation.get('errors'):
        errors.append('rebuilt_validation_has_errors')

    result_sources = result.get('detected_danger_sources', [])
    if not isinstance(result_sources, list) or not result_sources:
        errors.append('official_result_has_no_confirmed_hazard')

    metrics = evaluation.get('metrics', {})
    scores = evaluation.get('scores', {})
    official_metrics = _validate_official_metrics(metrics, scores, errors)
    recall = official_metrics.get('recall')
    far = official_metrics.get('false_alarm_rate')
    elapsed = official_metrics.get('exploration_time_sec')
    run_pass = (
        not errors
        and recall is not None and recall > 0.60
        and far is not None and far <= 0.10
        and elapsed is not None and elapsed <= 600.0
    )
    return _run_report(
        seed,
        errors=errors,
        run_pass=run_pass,
        evidence_dir=str(evidence_dir),
        official_metrics=official_metrics,
        official_scores=scores,
    )


def _validate_official_metrics(metrics, scores, errors):
    integer_names = ('truth_count', 'detected_count', 'correct', 'missed', 'false_alarms')
    parsed = {}
    for name in integer_names:
        value = metrics.get(name)
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            errors.append('invalid_official_metric_' + name)
        else:
            parsed[name] = value
    elapsed = metrics.get('exploration_time')
    if not isinstance(elapsed, (int, float)) or isinstance(elapsed, bool) or elapsed < 0:
        errors.append('invalid_official_metric_exploration_time')
        elapsed = None
    if len(parsed) == len(integer_names):
        if parsed['correct'] + parsed['missed'] != parsed['truth_count']:
            errors.append('official_truth_count_inconsistent')
        if parsed['correct'] + parsed['false_alarms'] != parsed['detected_count']:
            errors.append('official_detected_count_inconsistent')
    if not isinstance(scores.get('technical_objective_total'), (int, float)):
        errors.append('missing_official_objective_score')

    truth_count = parsed.get('truth_count', 0)
    detected_count = parsed.get('detected_count', 0)
    correct = parsed.get('correct', 0)
    false_alarms = parsed.get('false_alarms', 0)
    return {
        'truth_count': truth_count,
        'detected_count': detected_count,
        'correct': correct,
        'missed': parsed.get('missed', 0),
        'false_alarms': false_alarms,
        'recall': None if truth_count == 0 else correct / float(truth_count),
        'false_alarm_rate': (
            None if detected_count == 0 else false_alarms / float(detected_count)
        ),
        'exploration_time_sec': elapsed,
    }


def _aggregate_metrics(run_reports):
    if not run_reports:
        return {
            'evaluated_run_count': 0,
            'minimum_recall': None,
            'maximum_false_alarm_rate': None,
            'maximum_exploration_time_sec': None,
            'mean_objective_score': None,
        }
    recalls = [
        run['official_metrics']['recall'] for run in run_reports
        if run['official_metrics']['recall'] is not None
    ]
    fars = [
        run['official_metrics']['false_alarm_rate'] for run in run_reports
        if run['official_metrics']['false_alarm_rate'] is not None
    ]
    times = [
        run['official_metrics']['exploration_time_sec'] for run in run_reports
        if run['official_metrics']['exploration_time_sec'] is not None
    ]
    scores = [
        float(run['official_scores']['technical_objective_total'])
        for run in run_reports
    ]
    return {
        'evaluated_run_count': len(run_reports),
        'minimum_recall': min(recalls) if recalls else None,
        'maximum_false_alarm_rate': max(fars) if fars else None,
        'maximum_exploration_time_sec': max(times) if times else None,
        'mean_objective_score': sum(scores) / len(scores),
    }


def _run_report(
        seed,
        errors,
        run_pass=False,
        evidence_dir='',
        official_metrics=None,
        official_scores=None,
):
    return {
        'seed': seed,
        'evidence_dir': evidence_dir,
        'run_pass': run_pass,
        'official_metrics_available': official_metrics is not None,
        'official_metrics': official_metrics or {},
        'official_scores': official_scores or {},
        'errors': sorted(set(errors)),
    }


def _resolve_inside(root, value, errors, name):
    if not isinstance(value, str) or not value.strip():
        errors.append('missing_' + name)
        return None
    path = (root / value).resolve()
    try:
        path.relative_to(root)
    except ValueError:
        errors.append(name + '_outside_campaign_root')
        return None
    return path


def _read_json(path):
    with Path(path).open('r', encoding='utf-8') as stream:
        return json.load(stream)


def _load_structure_validator():
    """从当前工作区加载独立校验器，避免信任旧归档的通过标志。"""

    validator_path = (
        Path(__file__).resolve().parent
        / 'validate_official_random_perception_evidence.py'
    )
    spec = importlib.util.spec_from_file_location(
        'hazardwalker_official_random_validator',
        validator_path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    def run_validator(evidence_dir, result_path, require_active_reobservation):
        return module.validate(
            evidence_dir,
            result_path,
            require_active_reobservation=require_active_reobservation,
        )

    return run_validator


def write_report(report, output_dir):
    """原子写出活动 JSON/CSV；CSV 只保留逐轮核心客观指标。"""

    output_dir = Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / 'campaign_summary.json'
    json_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    csv_path = output_dir / 'campaign_summary.csv'
    fieldnames = [
        'seed', 'run_pass', 'truth_count', 'detected_count', 'correct',
        'missed', 'false_alarms', 'recall', 'false_alarm_rate',
        'exploration_time_sec', 'technical_objective_total', 'errors',
    ]
    with csv_path.open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for run in report['runs']:
            metrics = run['official_metrics']
            writer.writerow({
                'seed': run['seed'],
                'run_pass': run['run_pass'],
                'truth_count': metrics.get('truth_count', ''),
                'detected_count': metrics.get('detected_count', ''),
                'correct': metrics.get('correct', ''),
                'missed': metrics.get('missed', ''),
                'false_alarms': metrics.get('false_alarms', ''),
                'recall': metrics.get('recall', ''),
                'false_alarm_rate': metrics.get('false_alarm_rate', ''),
                'exploration_time_sec': metrics.get('exploration_time_sec', ''),
                'technical_objective_total': run['official_scores'].get(
                    'technical_objective_total', '',
                ),
                'errors': ';'.join(run['errors']),
            })
    return json_path, csv_path


def main():
    parser = argparse.ArgumentParser(
        description='校验并汇总预注册的官方随机场景感知活动。',
    )
    parser.add_argument('--campaign-manifest', required=True)
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()

    report = summarize_campaign(args.campaign_manifest)
    write_report(report, args.output_dir)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['campaign_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
