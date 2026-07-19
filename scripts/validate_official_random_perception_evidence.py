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


ALLOWED_DETECTION_SOURCES = frozenset({
    'hsv_depth_tf',
    'official_ros1_rgbd',
})
MIN_ELIGIBLE_OBSERVATIONS = 3
MIN_DISTINCT_VIEWS = 3
MIN_SPHERICAL_VIEWS = 2
MIN_VIEW_BEARING_SPAN_DEG = 25.0
MAX_RESULT_TRACK_POSITION_DELTA_M = 0.05


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
    if manifest.get('mission_completion_required') is not True:
        errors.append('mission_completion_contract_missing')
    if summary.get('mission_completed') is not True:
        errors.append('mission_not_completed')
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

    _validate_official_result(result, frames, errors)
    return _report(root, result_file, errors, len(frames), len(trajectory), image_count,
                   depth_evidence_count=depth_count)


def _validate_official_result(result, frames, errors):
    """同时复核结果条目、最终 track 快照和逐帧 RGB-D 观测。"""

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
    used_track_ids = set()
    for index, source in enumerate(sources):
        prefix = 'danger_' + str(index)
        if not isinstance(source, dict):
            errors.append('invalid_danger_position_' + str(index))
            continue
        if set(source) != {'position'}:
            errors.append('unexpected_danger_fields_' + str(index))
        position = _validated_position(source.get('position'))
        if position is None:
            errors.append('invalid_danger_position_' + str(index))
        if position is None:
            continue

        matching_tracks = _matching_confirmed_tracks(frames, position)
        if not matching_tracks:
            errors.append('danger_not_linked_to_frame_evidence_' + str(index))
            continue
        if len(matching_tracks) != 1:
            errors.append('danger_matches_multiple_tracks_' + str(index))
            continue
        track_id, final_hazard = next(iter(matching_tracks.items()))
        if track_id in used_track_ids:
            errors.append('danger_track_reused_' + str(index))
            continue
        used_track_ids.add(track_id)
        _validate_multiview_record(final_hazard, errors, prefix + '_linked_track')
        _validate_frame_observations(frames, track_id, final_hazard, errors, index)


def _validate_multiview_record(record, errors, prefix):
    """对齐 result_builder 的严格多视角球面证据门禁。"""

    if str(record.get('evidence_status', '')) != 'multi_view_sphere_consistent':
        errors.append('invalid_evidence_status_' + prefix)
    if str(record.get('source', '')).strip() not in ALLOWED_DETECTION_SOURCES:
        errors.append('invalid_evidence_source_' + prefix)

    eligible_view_ids = _unique_nonempty_strings(record.get('eligible_view_ids'))
    spherical_view_ids = _unique_nonempty_strings(record.get('spherical_view_ids'))
    if eligible_view_ids is None:
        errors.append('invalid_eligible_view_ids_' + prefix)
        eligible_view_ids = []
    if spherical_view_ids is None:
        errors.append('invalid_spherical_view_ids_' + prefix)
        spherical_view_ids = []
    if not set(spherical_view_ids).issubset(set(eligible_view_ids)):
        errors.append('spherical_views_not_eligible_' + prefix)

    distinct_view_count = _validated_integer(record.get('distinct_view_count'))
    eligible_observation_count = _validated_integer(record.get('eligible_observation_count'))
    required_observations = _validated_integer(
        record.get('required_min_eligible_observations'),
    )
    required_views = _validated_integer(record.get('required_min_distinct_views'))
    required_spherical = _validated_integer(record.get('required_min_spherical_views'))
    if distinct_view_count is None or distinct_view_count != len(eligible_view_ids):
        errors.append('invalid_distinct_view_count_' + prefix)
    if eligible_observation_count is None:
        errors.append('invalid_eligible_observation_count_' + prefix)
    if required_observations is None:
        errors.append('invalid_required_observation_count_' + prefix)
        required_observations = MIN_ELIGIBLE_OBSERVATIONS
    if required_views is None:
        errors.append('invalid_required_distinct_views_' + prefix)
        required_views = MIN_DISTINCT_VIEWS
    if required_spherical is None:
        errors.append('invalid_required_spherical_views_' + prefix)
        required_spherical = MIN_SPHERICAL_VIEWS

    required_observations = max(required_observations, MIN_ELIGIBLE_OBSERVATIONS)
    required_views = max(required_views, MIN_DISTINCT_VIEWS)
    required_spherical = max(required_spherical, MIN_SPHERICAL_VIEWS)
    if eligible_observation_count is None or eligible_observation_count < required_observations:
        errors.append('insufficient_eligible_observations_' + prefix)
    if distinct_view_count is None or distinct_view_count < required_views:
        errors.append('insufficient_distinct_views_' + prefix)
    if len(spherical_view_ids) < required_spherical:
        errors.append('insufficient_spherical_views_' + prefix)

    bearing_span = _finite_number(record.get('view_bearing_span_deg'))
    required_bearing_span = _finite_number(record.get('required_min_view_bearing_span_deg'))
    if required_bearing_span is None:
        errors.append('invalid_required_bearing_span_' + prefix)
        required_bearing_span = MIN_VIEW_BEARING_SPAN_DEG
    required_bearing_span = max(required_bearing_span, MIN_VIEW_BEARING_SPAN_DEG)
    if bearing_span is None or bearing_span < required_bearing_span:
        errors.append('insufficient_bearing_span_' + prefix)


def _validate_frame_observations(frames, track_id, linked_hazard, errors, index):
    """不信任累计计数，直接从逐帧检测记录重建有效视角和球面深度证据。"""

    observations = []
    for frame in frames:
        if not isinstance(frame, dict):
            continue
        for detection in frame.get('detections_2d', []):
            if not isinstance(detection, dict):
                continue
            if _normalized_id(detection.get('track_id')) != track_id:
                continue
            if not _is_eligible_detection(detection):
                continue
            observations.append(detection)

    view_ids = {
        str(item.get('view_id', '')).strip()
        for item in observations
        if str(item.get('view_id', '')).strip()
    }
    spherical_view_ids = {
        str(item.get('view_id', '')).strip()
        for item in observations
        if str(item.get('view_id', '')).strip()
        and isinstance(item.get('depth_shape'), dict)
        and item['depth_shape'].get('status') == 'spherical'
    }
    if len(observations) < MIN_ELIGIBLE_OBSERVATIONS:
        errors.append('insufficient_frame_observations_' + str(index))
    if len(view_ids) < MIN_DISTINCT_VIEWS:
        errors.append('insufficient_frame_views_' + str(index))
    if len(spherical_view_ids) < MIN_SPHERICAL_VIEWS:
        errors.append('insufficient_frame_spherical_views_' + str(index))

    hazard_views = _unique_nonempty_strings(linked_hazard.get('eligible_view_ids')) or []
    hazard_spherical_views = _unique_nonempty_strings(
        linked_hazard.get('spherical_view_ids'),
    ) or []
    if not set(hazard_views).issubset(view_ids):
        errors.append('track_views_not_found_in_frames_' + str(index))
    if not set(hazard_spherical_views).issubset(spherical_view_ids):
        errors.append('track_spherical_views_not_found_in_frames_' + str(index))

    bearings = [
        value for value in (
            _finite_number(item.get('view_bearing_deg')) for item in observations
        )
        if value is not None
    ]
    if _bearing_span_deg(bearings) < MIN_VIEW_BEARING_SPAN_DEG:
        errors.append('insufficient_frame_bearing_span_' + str(index))


def _is_eligible_detection(detection):
    """有效观测必须具备同步深度、同步 TF、世界定位和稳定视角。"""

    return (
        detection.get('confirmation_eligible') is True
        and detection.get('depth_synchronized') is True
        and detection.get('tf_synchronized') is True
        and str(detection.get('localization_status', '')) == 'localized'
        and bool(str(detection.get('view_id', '')).strip())
        and _validated_position(detection.get('localized_position')) is not None
    )


def _matching_confirmed_tracks(frames, result_position):
    """返回位置邻域内最终的合法 world-frame confirmed track，按 ID 去重。"""

    latest_by_track = {}
    for frame in frames:
        if not isinstance(frame, dict):
            continue
        for hazard in frame.get('hazards', []):
            if not isinstance(hazard, dict):
                continue
            track_id = _normalized_id(hazard.get('track_id', hazard.get('id')))
            if track_id:
                # frames.jsonl 按时间写入，必须以该 track 最后一条状态为准，不能
                # 用之后已回退/拒绝的轨迹早期 confirmed 快照冒充正式结果。
                latest_by_track[track_id] = hazard
    matches = {}
    for track_id, hazard in latest_by_track.items():
        position = _validated_position(hazard.get('position'))
        if (
            position is not None
            and str(hazard.get('position_frame_id', '')).strip() == 'world'
            and str(hazard.get('source', '')).strip() in ALLOWED_DETECTION_SOURCES
            and str(hazard.get('status', '')) == 'confirmed'
            and _distance_m(result_position, position) <= MAX_RESULT_TRACK_POSITION_DELTA_M
        ):
            matches[track_id] = hazard
    return matches


def _unique_nonempty_strings(value):
    if not isinstance(value, (list, tuple)):
        return None
    normalized = [str(item).strip() for item in value]
    if any(not item for item in normalized) or len(set(normalized)) != len(normalized):
        return None
    return normalized


def _validated_integer(value):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def _validated_position(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    numbers = [_finite_number(item) for item in value]
    return tuple(numbers) if all(item is not None for item in numbers) else None


def _finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _distance_m(first, second):
    return math.sqrt(sum(
        (float(first[index]) - float(second[index])) ** 2 for index in range(3)
    ))


def _normalized_id(value):
    if value is None or isinstance(value, bool):
        return ''
    return str(value).strip()


def _bearing_span_deg(bearings):
    """返回任意两个稳定视角的最大水平夹角，正确处理 -180/180 环绕。"""

    max_span = 0.0
    for first_index, first in enumerate(bearings):
        for second in bearings[first_index + 1:]:
            delta_rad = math.atan2(
                math.sin(math.radians(second - first)),
                math.cos(math.radians(second - first)),
            )
            max_span = max(max_span, abs(math.degrees(delta_rad)))
    return max_span


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
