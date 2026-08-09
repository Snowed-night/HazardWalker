#!/usr/bin/env python3
"""按预注册矩阵依次回放多 SEED 感知数据并生成跨场景比较。

所属组：感知定位组。负责人：姜晨。
文件作用：读取已经校验的 rosbag 回归集目录和算法方案，强制每个 SEED
运行完全相同的算法/参数矩阵；任一组合失败即停止并保留执行清单，避免漏跑
困难场景或事后挑选结果。所有回放仍由 run_perception_replay_experiment.py
完成，真值标注只进入赛后评估，不进入运行时感知链路。
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / 'scripts'
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from compare_perception_replay_campaign import (  # noqa: E402
    compare_campaign,
    write_campaign,
)


CATALOG_SCHEMA = 'hazardwalker_perception_bag_regression_v1'
PLAN_SCHEMA = 'hazardwalker_perception_replay_campaign_plan_v1'
EXECUTION_SCHEMA = 'hazardwalker_perception_replay_campaign_execution_v1'
SAFE_NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]*$')


def _read_json(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(payload, dict):
        raise ValueError(f'JSON 顶层必须是对象：{path}')
    return payload


def _resolve_repo_path(value: str, *, field: str) -> Path:
    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = REPO_ROOT / path
    path = path.resolve()
    if not path.is_file():
        raise ValueError(f'{field} 文件不存在：{path}')
    return path


def _resolve_plan_path(value: str, *, plan_dir: Path, field: str) -> Path:
    """解析随活动方案保存在数据盘上的标注文件。"""

    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = plan_dir / path
    path = path.resolve()
    if not path.is_file():
        raise ValueError(f'{field} 文件不存在：{path}')
    return path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _safe_name(value: object, *, field: str) -> str:
    name = str(value or '').strip()
    if not SAFE_NAME.fullmatch(name):
        raise ValueError(
            f'{field} 只能包含字母、数字、点、下划线和连字符：{name!r}')
    return name


def build_execution_matrix(
        catalog: dict, plan: dict, *, output_root: Path,
        domain_id_start: int = 142,
        plan_dir: Path | None = None) -> list[dict]:
    """验证预注册合同，并生成确定性的 SEED×算法回放矩阵。"""

    if catalog.get('schema') != CATALOG_SCHEMA:
        raise ValueError('回归集目录 schema 不受支持')
    if catalog.get('one_baseline_patrol_per_seed') is not True:
        raise ValueError('回归集未证明每个 SEED 只有一份基准巡检')
    sessions = catalog.get('sessions')
    if not isinstance(sessions, list) or len(sessions) < 3:
        raise ValueError('正式回放活动至少需要三个不同 SEED')

    if plan.get('schema') != PLAN_SCHEMA:
        raise ValueError('回放方案 schema 不受支持')
    _safe_name(plan.get('campaign_id'), field='campaign_id')
    variants = plan.get('variants')
    if not isinstance(variants, list) or len(variants) < 2:
        raise ValueError('正式回放活动至少需要两个算法/参数方案')
    annotations = plan.get('annotations_by_seed')
    if not isinstance(annotations, dict):
        raise ValueError('annotations_by_seed 必须逐 SEED 预注册')

    seeds = []
    session_by_seed = {}
    for session in sessions:
        if not isinstance(session, dict):
            raise ValueError('回归集 sessions 项必须是对象')
        seed = _safe_name(session.get('scenario_seed'), field='scenario_seed')
        if seed in session_by_seed:
            raise ValueError(f'回归集包含重复 SEED：{seed}')
        if session.get('validation_status') != 'passed':
            raise ValueError(f'SEED {seed} 的基准巡检未通过')
        session_dir = Path(str(session.get('session_dir', ''))).expanduser().resolve()
        session_by_seed[seed] = session_dir
        seeds.append(seed)
    if set(annotations) != set(seeds):
        raise ValueError(
            '标注集合必须与回归集 SEED 完全一致；'
            f'缺少={sorted(set(seeds) - set(annotations))}，'
            f'额外={sorted(set(annotations) - set(seeds))}')

    normalized_variants = []
    labels = set()
    parameter_hashes = set()
    for variant in variants:
        if not isinstance(variant, dict):
            raise ValueError('variants 项必须是对象')
        label = _safe_name(variant.get('algorithm_label'), field='algorithm_label')
        if label in labels:
            raise ValueError(f'算法标签重复：{label}')
        labels.add(label)
        parameter_file = _resolve_repo_path(
            variant.get('parameter_file', ''), field=f'{label}.parameter_file')
        parameter_sha256 = _sha256(parameter_file)
        if parameter_sha256 in parameter_hashes:
            raise ValueError(
                f'算法 {label} 与另一方案使用完全相同的参数快照，不能伪装为独立变量')
        parameter_hashes.add(parameter_sha256)
        normalized_variants.append({
            'algorithm_label': label,
            'parameter_file': parameter_file,
            'parameter_sha256': parameter_sha256,
        })

    matrix = []
    # 参数文件属于受版本控制的算法合同，继续以仓库根目录为基准；人工标注
    # 与活动方案一起保存在数据盘，允许相对于 campaign_plan.json 所在目录。
    annotation_base = (plan_dir or REPO_ROOT).expanduser().resolve()
    # ROS 2 默认 DDS domain id 的可移植范围为 0..232；每个组合使用独立域，
    # 防止上一轮尚在退出的节点污染下一轮。
    domain_count = 233
    if int(domain_id_start) < 0 or int(domain_id_start) + len(seeds) * len(
            normalized_variants) >= domain_count:
        raise ValueError('domain_id_start 无法为完整矩阵分配有效 ROS_DOMAIN_ID')
    for seed in sorted(seeds):
        annotation_file = _resolve_plan_path(
            annotations[seed], plan_dir=annotation_base,
            field=f'annotations_by_seed[{seed}]')
        for variant in normalized_variants:
            index = len(matrix)
            run_dir = output_root / f'seed_{seed}' / variant['algorithm_label']
            command = [
                sys.executable,
                str(SCRIPTS_DIR / 'run_perception_replay_experiment.py'),
                '--session', str(session_by_seed[seed]),
                '--output-dir', str(run_dir),
                '--parameter-file', str(variant['parameter_file']),
                '--algorithm-label', variant['algorithm_label'],
                '--annotations', str(annotation_file),
                '--scenario', f'seed_{seed}',
                '--domain-id', str(int(domain_id_start) + index),
            ]
            if bool(plan.get('recompute_localization', False)):
                command.extend([
                    '--recompute-localization',
                    '--localization-provenance', str(
                        plan.get('localization_provenance', 'lidar_imu_slam')),
                ])
            rate = float(plan.get('rate', 1.0))
            if rate <= 0.0:
                raise ValueError('rate 必须大于 0')
            command.extend(['--rate', str(rate)])
            matrix.append({
                'scenario_seed': seed,
                'session_dir': str(session_by_seed[seed]),
                'algorithm_label': variant['algorithm_label'],
                'parameter_file': str(variant['parameter_file']),
                'parameter_sha256': variant['parameter_sha256'],
                'annotation_file': str(annotation_file),
                'annotation_sha256': _sha256(annotation_file),
                'domain_id': int(domain_id_start) + index,
                'run_dir': str(run_dir),
                'command': command,
            })
    return matrix


def _resume_run_is_complete(item: dict) -> bool:
    manifest_path = Path(item['run_dir']) / 'replay_experiment_manifest.json'
    if not manifest_path.is_file():
        return False
    manifest = _read_json(manifest_path)
    parameter = manifest.get('parameter_snapshot', {})
    annotation = manifest.get('annotation_snapshot', {})
    return (
        manifest.get('status') == 'complete'
        and str(manifest.get('scenario_seed')) == item['scenario_seed']
        and manifest.get('algorithm_label') == item['algorithm_label']
        and str(manifest.get('parameter_sha256', parameter.get('sha256', '')))
        == item['parameter_sha256']
        and str(manifest.get('annotation_sha256', annotation.get('sha256', '')))
        == item['annotation_sha256']
    )


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    temporary.replace(path)


def execute_campaign(
        matrix: list[dict], *, output_root: Path, campaign_id: str,
        resume: bool = False, runner=subprocess.run) -> dict:
    """顺序执行完整矩阵；失败后保留清单，续跑时只复用严格匹配的完成项。"""

    output_root = output_root.resolve()
    manifest_path = output_root / 'campaign_execution_manifest.json'
    attempt_history = []
    if manifest_path.exists() and not resume:
        raise ValueError(f'活动目录已存在执行清单；续跑须使用 --resume：{output_root}')
    if manifest_path.exists():
        previous = _read_json(manifest_path)
        attempt_history = list(previous.get('attempt_history', []))
        attempt_history.append({
            'started_at_utc': previous.get('started_at_utc'),
            'finished_at_utc': previous.get('finished_at_utc'),
            'status': previous.get('status'),
            'completed_count': previous.get('completed_count'),
            'failed_item': previous.get('failed_item'),
        })
    execution = {
        'schema': EXECUTION_SCHEMA,
        'campaign_id': campaign_id,
        'status': 'running',
        'started_at_utc': datetime.now(timezone.utc).isoformat(),
        'finished_at_utc': None,
        'matrix_size': len(matrix),
        'completed_count': 0,
        'failed_item': None,
        'attempt_history': attempt_history,
        'runs': [],
    }
    _write_json(manifest_path, execution)
    run_dirs = []
    for item in matrix:
        run_dir = Path(item['run_dir'])
        if run_dir.exists():
            if not resume or not _resume_run_is_complete(item):
                execution['status'] = 'failed'
                execution['failed_item'] = {
                    **item, 'reason': '已有结果与预注册方案不匹配或未完成'}
                execution['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
                _write_json(manifest_path, execution)
                raise RuntimeError(execution['failed_item']['reason'])
            result = {'returncode': 0, 'reused_complete_run': True}
        else:
            completed = runner(item['command'], cwd=REPO_ROOT)
            result = {
                'returncode': int(completed.returncode),
                'reused_complete_run': False,
            }
        execution['runs'].append({
            **{key: value for key, value in item.items() if key != 'command'},
            'command': shlex.join(item['command']),
            **result,
        })
        if result['returncode'] != 0:
            execution['status'] = 'failed'
            execution['failed_item'] = execution['runs'][-1]
            execution['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
            _write_json(manifest_path, execution)
            raise RuntimeError(
                f"回放失败：SEED={item['scenario_seed']} "
                f"algorithm={item['algorithm_label']}")
        execution['completed_count'] += 1
        run_dirs.append(run_dir)
        _write_json(manifest_path, execution)

    comparison_dir = output_root / 'comparison'
    try:
        campaign = compare_campaign(run_dirs)
        write_campaign(comparison_dir, campaign)
    except (ValueError, json.JSONDecodeError) as exc:
        execution['status'] = 'failed'
        execution['failed_item'] = {
            'stage': 'campaign_comparison',
            'reason': str(exc),
        }
        execution['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        _write_json(manifest_path, execution)
        raise RuntimeError(f'跨 SEED 比较失败：{exc}') from exc
    execution['status'] = 'complete'
    execution['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
    execution['comparison_dir'] = str(comparison_dir)
    _write_json(manifest_path, execution)
    return execution


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--catalog', required=True)
    parser.add_argument('--plan', required=True)
    parser.add_argument('--output-root', required=True)
    parser.add_argument('--domain-id-start', type=int, default=142)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()

    try:
        catalog_path = Path(args.catalog).expanduser().resolve()
        plan_path = Path(args.plan).expanduser().resolve()
        output_root = Path(args.output_root).expanduser().resolve()
        perception_root = (REPO_ROOT / 'reports' / 'perception').resolve()
        try:
            relative_output = output_root.relative_to(perception_root)
        except ValueError as exc:
            raise ValueError(
                f'正式活动输出必须位于 {perception_root} 下') from exc
        if not relative_output.parts:
            raise ValueError('正式活动输出必须使用 reports/perception 下的独立子目录')
        catalog = _read_json(catalog_path)
        plan = _read_json(plan_path)
        campaign_id = _safe_name(plan.get('campaign_id'), field='campaign_id')
        matrix = build_execution_matrix(
            catalog, plan, output_root=output_root,
            domain_id_start=args.domain_id_start,
            plan_dir=plan_path.parent)
    except (ValueError, json.JSONDecodeError) as exc:
        raise SystemExit(str(exc)) from exc
    if args.dry_run:
        print(json.dumps({
            'campaign_id': campaign_id,
            'matrix_size': len(matrix),
            'runs': [
                {**{key: value for key, value in item.items() if key != 'command'},
                 'command': shlex.join(item['command'])}
                for item in matrix
            ],
        }, ensure_ascii=False, indent=2))
        return 0
    try:
        result = execute_campaign(
            matrix, output_root=output_root, campaign_id=campaign_id,
            resume=args.resume)
    except (ValueError, RuntimeError, json.JSONDecodeError) as exc:
        raise SystemExit(str(exc)) from exc
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
