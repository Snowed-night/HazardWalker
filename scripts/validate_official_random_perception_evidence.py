#!/usr/bin/env python3
"""校验一个官方随机场景的感知证据归档是否可复核。

本脚本只读取运行结束后已经落盘的证据和 ``detected_danger.json``，不会连接 ROS、
读取场景布局或真值文件。它检查资料完整性和结果结构，不能替代主办方按真值计算
召回率、虚警率与定位误差的独立评测。
"""

import argparse
import json
import math
from pathlib import Path


def validate(evidence_dir, result_path):
    """返回结构化校验报告；任何缺失或违规都会写入 errors。"""

    root = Path(evidence_dir)
    result_file = Path(result_path)
    errors = []
    expected_archived_result = root / 'detected_danger.json'
    try:
        result_is_archived = result_file.resolve() == expected_archived_result.resolve()
    except OSError:
        result_is_archived = False
    if not result_is_archived:
        errors.append('result_not_archived_in_evidence_dir')
    required = {
        'run_manifest': root / 'run_manifest.json',
        'summary': root / 'summary.json',
        'frames': root / 'frames.jsonl',
        'failure_reasons': root / 'failure_reasons.json',
        'trajectory': root / 'trajectory.jsonl',
        'detected_danger': result_file,
    }
    for name, path in required.items():
        if not path.is_file() or path.stat().st_size == 0:
            errors.append('missing_or_empty_' + name)
    if errors:
        return _report(root, result_file, errors, 0, 0, 0)

    manifest = _read_json(required['run_manifest'], errors, 'run_manifest')
    summary = _read_json(required['summary'], errors, 'summary')
    failure_reasons = _read_json(required['failure_reasons'], errors, 'failure_reasons')
    result = _read_json(result_file, errors, 'detected_danger')
    frames = _read_json_lines(required['frames'], errors)
    trajectory = _read_json_lines(required['trajectory'], errors)
    if errors:
        return _report(root, result_file, errors, len(frames), len(trajectory), 0)

    if manifest.get('schema') != 'hazardwalker_perception_official_evidence_v1':
        errors.append('unexpected_manifest_schema')
    contract = manifest.get('evidence_contract', {})
    if not contract.get('formal_evidence_eligible', False):
        errors.append('formal_evidence_contract_rejected')
    if contract.get('truth_inputs_used') is not False:
        errors.append('truth_input_declaration_invalid')
    if summary.get('evidence_contract') != contract:
        errors.append('summary_manifest_contract_mismatch')
    if not isinstance(failure_reasons.get('observed_failure_reasons'), list):
        errors.append('failure_reasons_not_list')

    image_count = 0
    depth_count = 0
    candidate_seen = False
    confirmation_seen = False
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict):
            errors.append('invalid_frame_record_' + str(index))
            continue
        candidate_seen = candidate_seen or bool(frame.get('detections_2d'))
        confirmation_seen = confirmation_seen or any(
            item.get('status') == 'confirmed' for item in frame.get('hazards', [])
        )
        image_path = str(frame.get('evidence_image', '')).strip()
        depth_path = str(frame.get('evidence_depth', '')).strip()
        if image_path:
            image_count += 1
            if not (root / image_path).is_file():
                errors.append('missing_rgb_evidence_' + str(index))
        if depth_path:
            depth_count += 1
            if not (root / depth_path).is_file():
                errors.append('missing_depth_evidence_' + str(index))
    if not candidate_seen:
        errors.append('no_candidate_to_review')
    if not confirmation_seen:
        errors.append('no_confirmed_red_ball')
    if image_count == 0:
        errors.append('no_key_rgb_evidence')
    if depth_count == 0:
        errors.append('no_time_paired_depth_evidence')
    if not trajectory:
        errors.append('empty_legal_slam_trajectory')

    _validate_official_result(result, errors)
    return _report(root, result_file, errors, len(frames), len(trajectory), image_count,
                   depth_evidence_count=depth_count)


def _validate_official_result(result, errors):
    if not isinstance(result, dict):
        errors.append('result_not_object')
        return
    duration = result.get('exploration_time')
    if not isinstance(duration, (int, float)) or not math.isfinite(float(duration)):
        errors.append('invalid_exploration_time')
    elif duration > 600.0:
        errors.append('exploration_time_exceeds_600_seconds')
    sources = result.get('detected_danger_sources')
    if not isinstance(sources, list):
        errors.append('detected_danger_sources_not_list')
        return
    for index, source in enumerate(sources):
        position = source.get('position') if isinstance(source, dict) else None
        if (not isinstance(position, list) or len(position) != 3
                or not all(isinstance(value, (int, float)) and math.isfinite(float(value))
                           for value in position)):
            errors.append('invalid_danger_position_' + str(index))


def _read_json(path, errors, name):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        errors.append('invalid_json_' + name)
        return {}


def _read_json_lines(path, errors):
    rows = []
    try:
        for index, line in enumerate(path.read_text(encoding='utf-8').splitlines(), start=1):
            if line.strip():
                rows.append(json.loads(line))
    except (OSError, json.JSONDecodeError):
        errors.append('invalid_jsonl_frames_or_trajectory')
    return rows


def _report(root, result_file, errors, frame_count, trajectory_count, image_count,
            depth_evidence_count=0):
    return {
        'schema': 'hazardwalker_official_random_perception_evidence_validation_v1',
        'evidence_dir': str(root),
        'result_path': str(result_file),
        'frame_count': frame_count,
        'trajectory_sample_count': trajectory_count,
        'rgb_evidence_count': image_count,
        'depth_evidence_count': depth_evidence_count,
        'structural_evidence_complete': not errors,
        'errors': sorted(set(errors)),
        'scope_limitations': [
            '本校验器不读取真值，不能计算召回率、虚警率或定位误差。',
            '正式成绩须另用主办方赛后评测程序在运行结束后独立计算。',
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('evidence_dir', type=Path)
    parser.add_argument('--result-json', type=Path, required=True)
    parser.add_argument('--write-report', type=Path)
    args = parser.parse_args()
    report = validate(args.evidence_dir, args.result_json)
    target = args.write_report or args.evidence_dir / 'independent_post_evaluation.json'
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['structural_evidence_complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
