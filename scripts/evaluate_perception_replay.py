#!/usr/bin/env python3
"""用人工标注计算固定 SEED 感知回放指标并写入成果目录。"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / 'scripts'
PERCEPTION_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
if str(PERCEPTION_SRC) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SRC))

from git_provenance import read_git_state  # noqa: E402
from hazardwalker_perception.replay_evaluation import (  # noqa: E402
    evaluate_labeled_replay,
)


def load_jsonl(path):
    records = []
    with path.open(encoding='utf-8') as handle:
        for line_number, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f'{path}:{line_number} JSON 损坏') from exc
            if not isinstance(value, dict):
                raise ValueError(f'{path}:{line_number} 必须是 JSON 对象')
            records.append(value)
    return records


def write_outputs(output_dir, evaluation, scenario):
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'evaluation.json').write_text(
        json.dumps(evaluation, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    (output_dir / 'algorithm_run_manifest.json').write_text(
        json.dumps(evaluation['algorithm_run'], ensure_ascii=False, indent=2)
        + '\n',
        encoding='utf-8',
    )
    candidates = evaluation['candidate_metrics']
    confirmed = evaluation['confirmed_output_metrics']
    location = evaluation['localization_error_m']
    confirmation = evaluation['candidate_to_confirmation_latency_sec']
    processing = evaluation['processing_time_ms']
    row = {
        'scenario': scenario,
        'labeled_frame_count': evaluation['labeled_frame_count'],
        'candidate_true_positive': candidates['true_positive'],
        'candidate_false_positive': candidates['false_positive'],
        'candidate_false_negative': candidates['false_negative'],
        'candidate_precision': candidates['precision'],
        'candidate_recall': candidates['recall'],
        'candidate_f1_score': candidates['f1_score'],
        'confirmed_true_positive': confirmed['true_positive'],
        'confirmed_false_positive': confirmed['false_positive'],
        'confirmed_false_negative': confirmed['false_negative'],
        'confirmed_precision': confirmed['precision'],
        'confirmed_recall': confirmed['recall'],
        'confirmed_f1_score': confirmed['f1_score'],
        # 兼容测试组既有字段，语义明确等同于最终确认输出。
        'true_positive': confirmed['true_positive'],
        'false_positive': confirmed['false_positive'],
        'false_negative': confirmed['false_negative'],
        'precision': confirmed['precision'],
        'recall': confirmed['recall'],
        'f1_score': confirmed['f1_score'],
        'localization_reference_count': location['reference_count'],
        'localization_prediction_count': location['prediction_count'],
        'localization_coverage': location['coverage'],
        'localization_error_mean_m': location['mean'],
        'localization_error_median_m': location['median'],
        'localization_error_max_m': location['max'],
        'confirmation_latency_count': confirmation['count'],
        'confirmation_latency_mean_sec': confirmation['mean'],
        'confirmation_latency_max_sec': confirmation['max'],
        'processing_time_count': processing['count'],
        'processing_time_mean_ms': processing['mean'],
        'processing_time_p95_ms': processing['p95'],
        'processing_time_max_ms': processing['max'],
        'observed_output_rate_hz': evaluation['observed_output_rate_hz'],
        'annotation_provenance': evaluation['annotation_provenance'],
        'truth_inputs_used_at_runtime': False,
        'algorithm_label': evaluation['algorithm_run']['label'],
        'algorithm_commit': evaluation['algorithm_run']['git']['commit'],
        'algorithm_worktree_dirty': evaluation['algorithm_run']['git']['dirty'],
        'source_session': evaluation.get('evaluation_inputs', {}).get(
            'source_session', ''),
        'annotation_sha256': evaluation.get('evaluation_inputs', {}).get(
            'annotation_file', {}).get('sha256', ''),
        'replay_contract_verified': evaluation.get(
            'evaluation_inputs', {}).get(
                'replay_control_contract_verified', False),
    }
    json_path = output_dir / 'testing_record_perception_labeled.json'
    json_path.write_text(
        json.dumps([row], ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    with (output_dir / 'testing_record_perception_labeled.csv').open(
            'w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(row))
        writer.writeheader()
        writer.writerow(row)
    _update_readme_with_evaluation(output_dir, evaluation, scenario)


def _update_readme_with_evaluation(output_dir, evaluation, scenario):
    """将人工标注评测摘要幂等写入本轮 README。"""

    start = '<!-- labeled-evaluation:start -->'
    end = '<!-- labeled-evaluation:end -->'
    candidates = evaluation['candidate_metrics']
    confirmed = evaluation['confirmed_output_metrics']
    location = evaluation['localization_error_m']
    confirmation = evaluation['candidate_to_confirmation_latency_sec']
    section = '\n'.join([
        start,
        '## 人工标注回放评测',
        '',
        f"- 场景：`{scenario}`",
        f"- 标注帧数：{evaluation['labeled_frame_count']}",
        f"- 候选 Precision / Recall / F1：{candidates['precision']} / {candidates['recall']} / {candidates['f1_score']}",
        f"- 最终确认 Precision / Recall / F1：{confirmed['precision']} / {confirmed['recall']} / {confirmed['f1_score']}",
        f"- 平均候选到确认耗时：{confirmation['mean']} s",
        f"- 三维定位覆盖率：{location['coverage']}",
        f"- 三维定位平均 / 中位 / 最大误差：{location['mean']} / {location['median']} / {location['max']} m",
        '- 真值仅来自人工图像标注或赛后公开参考，运行期未读取 Gazebo 真值。',
        end,
        '',
    ])
    readme_path = output_dir / 'README.md'
    current = readme_path.read_text(encoding='utf-8') if readme_path.is_file() else (
        f'# {scenario}\n\n')
    if start in current and end in current:
        before, tail = current.split(start, 1)
        _old, after = tail.split(end, 1)
        current = before.rstrip() + '\n\n' + section + after.lstrip('\n')
    else:
        current = current.rstrip() + '\n\n' + section
    readme_path.write_text(current, encoding='utf-8')


def build_algorithm_metadata(label, parameter_files):
    """记录本次回放算法版本和参数哈希，确保控制变量可复核。"""

    normalized_label = str(label).strip()
    if not normalized_label:
        raise ValueError('algorithm-label 不能为空')
    snapshots = []
    for value in parameter_files:
        path = Path(value)
        if not path.is_absolute():
            path = REPO_ROOT / path
        path = path.resolve()
        if not path.is_file():
            raise ValueError(f'参数文件不存在：{path}')
        try:
            display_path = str(path.relative_to(REPO_ROOT))
        except ValueError:
            display_path = str(path)
        snapshots.append({
            'path': display_path.replace('\\', '/'),
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'size_bytes': path.stat().st_size,
        })

    return {
        'schema_version': 1,
        'label': normalized_label,
        'git': read_git_state(REPO_ROOT),
        'parameter_files': snapshots,
    }


def build_evaluation_input_metadata(frames_path, annotation_path, output_dir):
    """记录标注哈希和源 rosbag；同源对比不能只依赖相似目录名。"""

    frames_path = Path(frames_path).resolve()
    annotation_path = Path(annotation_path).resolve()
    output_dir = Path(output_dir).resolve()

    def snapshot(path):
        return {
            'path': str(path),
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'size_bytes': path.stat().st_size,
        }

    result = {
        'frames_file': snapshot(frames_path),
        'annotation_file': snapshot(annotation_path),
        'source_session': '',
        'source_bag_fingerprint_sha256': '',
        'scenario_seed': '',
        'replay_experiment_manifest_verified': False,
    }
    manifest_path = output_dir / 'replay_experiment_manifest.json'
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        result.update({
            'source_session': str(manifest.get('source_session', '')),
            'source_bag_fingerprint_sha256': str(
                manifest.get('source_bag_fingerprint_sha256', '')),
            'scenario_seed': str(manifest.get('scenario_seed', '')),
            'replay_algorithm_label': str(
                manifest.get('algorithm_label', '')),
            'replay_parameter_sha256': str(
                manifest.get('parameter_sha256', '')),
            'replay_git': manifest.get('git', {}),
            'replay_experiment_manifest_verified': True,
        })
    return result


def verify_replay_control_contract(algorithm_run, evaluation_inputs):
    """确认评估标签和参数文件与实际回放进程一致。"""

    if not evaluation_inputs.get('replay_experiment_manifest_verified'):
        return False
    if algorithm_run['label'] != evaluation_inputs.get('replay_algorithm_label'):
        raise ValueError('评估 algorithm-label 与实际回放标签不一致')
    snapshots = algorithm_run.get('parameter_files', [])
    if len(snapshots) != 1:
        raise ValueError('受控回放当前要求恰好一个实际感知参数文件')
    if snapshots[0]['sha256'] != evaluation_inputs.get(
            'replay_parameter_sha256'):
        raise ValueError('评估参数哈希与实际回放参数不一致')
    replay_git = evaluation_inputs.get('replay_git', {})
    algorithm_git = algorithm_run.get('git', {})
    if not str(replay_git.get('commit', '')).strip():
        raise ValueError('实际回放清单缺少 Git 提交')
    if replay_git.get('dirty') is not False:
        raise ValueError('实际回放使用了未提交代码')
    if algorithm_git.get('dirty') is not False:
        raise ValueError('评估阶段代码存在未提交改动')
    if algorithm_git.get('commit') != replay_git.get('commit'):
        raise ValueError('评估 Git 提交与实际回放不一致')
    if not evaluation_inputs.get('source_session'):
        raise ValueError('实际回放清单缺少源 rosbag 会话')
    fingerprint = str(
        evaluation_inputs.get('source_bag_fingerprint_sha256', ''))
    if len(fingerprint) != 64:
        raise ValueError('实际回放清单缺少有效 rosbag 内容指纹')
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', required=True)
    parser.add_argument('--annotations', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--iou-threshold', type=float, default=0.5)
    parser.add_argument('--algorithm-label', default='hsv_depth_tf')
    parser.add_argument(
        '--parameter-file', action='append', default=[],
        help='参与本轮控制变量测试的参数文件，可重复指定',
    )
    args = parser.parse_args()
    frames_path = Path(args.frames).expanduser().resolve()
    annotation_path = Path(args.annotations).expanduser().resolve()
    records = load_jsonl(frames_path)
    annotations = json.loads(annotation_path.read_text(encoding='utf-8'))
    evaluation = evaluate_labeled_replay(
        records, annotations, iou_threshold=args.iou_threshold)
    parameter_files = args.parameter_file or ['config/perception.yaml']
    evaluation['algorithm_run'] = build_algorithm_metadata(
        args.algorithm_label, parameter_files)
    output_dir = Path(args.output_dir).expanduser().resolve()
    evaluation['evaluation_inputs'] = build_evaluation_input_metadata(
        frames_path, annotation_path, output_dir)
    evaluation['evaluation_inputs']['replay_control_contract_verified'] = (
        verify_replay_control_contract(
            evaluation['algorithm_run'], evaluation['evaluation_inputs']))
    write_outputs(output_dir, evaluation, args.scenario)
    print(json.dumps(evaluation, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
