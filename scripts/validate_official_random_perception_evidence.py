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
STRONG_RGBD_MIN_DISTINCT_VIEWS = 2
STRONG_RGBD_MIN_VIEW_BEARING_SPAN_DEG = 5.0
EXPECTED_SPHERE_DIAMETER_M = 0.30
MAX_SPHERE_DIAMETER_RELATIVE_ERROR = 0.35
MIN_SPHERE_ASPECT_RATIO = 0.82
MIN_NORMALIZED_DEPTH_CURVATURE = 0.10
MAX_NORMALIZED_DEPTH_CURVATURE = 0.30
MAX_DIAMETER_CV = 0.35
MAX_DEPTH_CURVATURE_CV = 0.65
MAX_RESULT_TRACK_POSITION_DELTA_M = 0.05
MIN_REOBSERVATION_TRANSLATION_M = 0.05
MIN_REOBSERVATION_YAW_DEG = 5.0
PHYSICAL_REOBSERVATION_ACTIONS = frozenset({
    'turn_left',
    'turn_right',
    'move_left',
    'move_right',
    'move_forward',
    'move_backward',
})


def validate(evidence_dir, result_path, require_active_reobservation=False):
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
        return _report(
            root, result_file, errors, 0, 0, 0,
            active_reobservation_required=require_active_reobservation,
        )

    manifest = _read_json(required['run_manifest'], errors, 'run_manifest')
    summary = _read_json(required['summary'], errors, 'summary')
    failure_reasons = _read_json(required['failure_reasons'], errors, 'failure_reasons')
    result = _read_json(result_file, errors, 'detected_danger')
    frames = _read_json_lines(required['frames'], errors)
    trajectory = _read_json_lines(required['trajectory'], errors)
    if errors:
        return _report(
            root, result_file, errors, len(frames), len(trajectory), 0,
            active_reobservation_required=require_active_reobservation,
        )

    schema = manifest.get('schema')
    if schema not in {
        'hazardwalker_perception_official_evidence_v1',
        'hazardwalker_perception_official_evidence_v2',
    }:
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
    raw_image_count = 0
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
        raw_image_path = str(frame.get('evidence_raw_image', '')).strip()
        depth_path = str(frame.get('evidence_depth', '')).strip()
        if image_path:
            image_count += 1
            if not (root / image_path).is_file():
                errors.append('missing_rgb_evidence_' + str(index))
        if raw_image_path:
            raw_image_count += 1
            if not (root / raw_image_path).is_file():
                errors.append('missing_raw_rgb_evidence_' + str(index))
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
    if schema == 'hazardwalker_perception_official_evidence_v2':
        if raw_image_count == 0:
            errors.append('no_raw_rgb_evidence')
        if raw_image_count != image_count:
            errors.append('raw_annotated_rgb_count_mismatch')
    if depth_count == 0:
        errors.append('no_time_paired_depth_evidence')
    if not trajectory:
        errors.append('empty_legal_slam_trajectory')

    _validate_official_result(result, frames, errors)
    if require_active_reobservation:
        _validate_active_reobservation(frames, errors)
    return _report(
        root, result_file, errors, len(frames), len(trajectory), image_count,
        raw_rgb_evidence_count=raw_image_count,
        depth_evidence_count=depth_count,
        active_reobservation_required=require_active_reobservation,
    )


def _validate_active_reobservation(frames, errors):
    """确认至少存在一个“局部候选→真实移动→完整观测→确认”的同目标闭环。

    ``view_recommendation`` 只是感知层建议，不能证明控制器执行成功。因此本门禁还
    要求合法 SLAM 位姿在候选与确认之间发生可测变化，并要求后续完整观测具备同步
    RGB-D/TF 和三维定位。正式多视角强度与去重仍由 ``_validate_official_result``
    逐帧重建校验。
    """

    attempts = []
    for start_index, frame in enumerate(frames):
        recommendation = frame.get('view_recommendation', {})
        if not isinstance(recommendation, dict):
            continue
        action = str(recommendation.get('action', '')).strip()
        target_id = _normalized_id(recommendation.get('target_id'))
        if action not in PHYSICAL_REOBSERVATION_ACTIONS or not target_id:
            continue
        partial_detection = _matching_detection(
            frame.get('detections_2d', []), target_id, require_partial=True,
        )
        if partial_detection is None:
            continue

        attempt = {
            'motion_command': False,
            'observed_motion': False,
            'complete_detection': False,
            'confirmation': False,
        }
        start_pose = _validated_robot_pose(frame.get('robot_pose'))
        latest_pose = start_pose
        for later in frames[start_index:]:
            later_recommendation = later.get('view_recommendation', {})
            if isinstance(later_recommendation, dict):
                later_action = str(later_recommendation.get('action', '')).strip()
                later_target = _normalized_id(later_recommendation.get('target_id'))
                if (
                    later_action in PHYSICAL_REOBSERVATION_ACTIONS
                    and _identities_overlap(target_id, later_target)
                ):
                    attempt['motion_command'] = True

            pose = _validated_robot_pose(later.get('robot_pose'))
            if pose is not None:
                latest_pose = pose
            complete = _matching_detection(
                later.get('detections_2d', []), target_id, require_partial=False,
            )
            if complete is not None and _is_eligible_detection(complete):
                # “先看到完整球，随后机器人因其他原因移动”不属于主动复查成功。
                # 完整观测必须发生在相对局部候选起点已有可测位姿变化之后。
                if (
                    start_pose is not None
                    and latest_pose is not None
                    and _poses_show_reobservation_motion(start_pose, latest_pose)
                ):
                    attempt['complete_detection'] = True
                    target_id = _preferred_detection_identity(complete, target_id)
            if _has_confirmed_target(later.get('hazards', []), target_id):
                attempt['confirmation'] = True
                if start_pose is not None and latest_pose is not None:
                    attempt['observed_motion'] = _poses_show_reobservation_motion(
                        start_pose, latest_pose,
                    )
                break
        attempts.append(attempt)
        if all(attempt.values()):
            return

    if not attempts:
        errors.append('no_partial_reobservation_start')
        return
    errors.append('no_complete_active_reobservation_episode')
    for field, error in (
        ('motion_command', 'no_reobservation_motion_command'),
        ('observed_motion', 'no_observed_robot_motion'),
        ('complete_detection', 'no_reobserved_complete_detection'),
        ('confirmation', 'no_reobservation_confirmation'),
    ):
        if not any(attempt[field] for attempt in attempts):
            errors.append(error)


def _matching_detection(detections, target_id, require_partial):
    """返回与目标别名相符的局部或完整检测。"""

    for detection in detections:
        if not isinstance(detection, dict):
            continue
        identities = _detection_identities(detection)
        if not any(_identities_overlap(target_id, value) for value in identities):
            continue
        is_partial = bool(
            detection.get('is_partial')
            or detection.get('requires_reobservation')
        )
        if is_partial == require_partial:
            return detection
    return None


def _preferred_detection_identity(detection, fallback):
    """候选升级为三维 track 后优先使用 track_id，同时保留候选别名匹配。"""

    for key in ('track_id', 'candidate_id', 'id'):
        value = _normalized_id(detection.get(key))
        if value:
            return value
    return fallback


def _detection_identities(detection):
    return [
        value for value in (
            _normalized_id(detection.get('id')),
            _normalized_id(detection.get('track_id')),
            _normalized_id(detection.get('candidate_id')),
        ) if value
    ]


def _has_confirmed_target(hazards, target_id):
    for hazard in hazards:
        if not isinstance(hazard, dict) or hazard.get('status') != 'confirmed':
            continue
        identities = [
            _normalized_id(hazard.get('id')),
            _normalized_id(hazard.get('track_id')),
        ]
        aliases = hazard.get('candidate_ids', [])
        if isinstance(aliases, (list, tuple)):
            identities.extend(_normalized_id(value) for value in aliases)
        if any(_identities_overlap(target_id, value) for value in identities if value):
            return True
    return False


def _identities_overlap(first, second):
    """兼容 ``candidate-N``、``untracked:candidate-N`` 与升级后的 track 别名。"""

    first_text = _normalized_id(first)
    second_text = _normalized_id(second)
    if first_text.startswith('untracked:'):
        first_text = first_text.split(':', 1)[1]
    if second_text.startswith('untracked:'):
        second_text = second_text.split(':', 1)[1]
    return bool(first_text and second_text and first_text == second_text)


def _validated_robot_pose(value):
    if not isinstance(value, dict):
        return None
    position = value.get('position')
    orientation = value.get('orientation')
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        return None
    xyz = tuple(_finite_number(position.get(axis)) for axis in ('x', 'y', 'z'))
    quaternion = tuple(
        _finite_number(orientation.get(axis)) for axis in ('x', 'y', 'z', 'w')
    )
    if any(item is None for item in xyz + quaternion):
        return None
    return xyz, quaternion


def _poses_show_reobservation_motion(first, second):
    first_position, first_quaternion = first
    second_position, second_quaternion = second
    translation = _distance_m(first_position, second_position)
    yaw_delta = abs(math.degrees(math.atan2(
        math.sin(_quaternion_yaw(second_quaternion) - _quaternion_yaw(first_quaternion)),
        math.cos(_quaternion_yaw(second_quaternion) - _quaternion_yaw(first_quaternion)),
    )))
    return (
        translation >= MIN_REOBSERVATION_TRANSLATION_M
        or yaw_delta >= MIN_REOBSERVATION_YAW_DEG
    )


def _quaternion_yaw(quaternion):
    x, y, z, w = quaternion
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


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

    confirmation_path = str(
        record.get('confirmation_path', 'regular_multiview')
    ).strip()
    if confirmation_path == 'regular_multiview':
        expected_status = 'multi_view_sphere_consistent'
        minimum_views = MIN_DISTINCT_VIEWS
        minimum_bearing_span = MIN_VIEW_BEARING_SPAN_DEG
    elif confirmation_path == 'strong_rgbd_geometry':
        expected_status = 'strong_rgbd_sphere_geometry_consistent'
        minimum_views = STRONG_RGBD_MIN_DISTINCT_VIEWS
        minimum_bearing_span = STRONG_RGBD_MIN_VIEW_BEARING_SPAN_DEG
    else:
        errors.append('invalid_confirmation_path_' + prefix)
        expected_status = 'multi_view_sphere_consistent'
        minimum_views = MIN_DISTINCT_VIEWS
        minimum_bearing_span = MIN_VIEW_BEARING_SPAN_DEG
    if str(record.get('evidence_status', '')) != expected_status:
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
    required_views = max(required_views, minimum_views)
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
    required_bearing_span = max(required_bearing_span, minimum_bearing_span)
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
    confirmation_path = str(
        linked_hazard.get('confirmation_path', 'regular_multiview')
    ).strip()
    strong_rgbd_path = confirmation_path == 'strong_rgbd_geometry'
    minimum_views = (
        STRONG_RGBD_MIN_DISTINCT_VIEWS
        if strong_rgbd_path else MIN_DISTINCT_VIEWS
    )
    minimum_bearing_span = (
        STRONG_RGBD_MIN_VIEW_BEARING_SPAN_DEG
        if strong_rgbd_path else MIN_VIEW_BEARING_SPAN_DEG
    )
    if len(observations) < MIN_ELIGIBLE_OBSERVATIONS:
        errors.append('insufficient_frame_observations_' + str(index))
    if len(view_ids) < minimum_views:
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
    if _bearing_span_deg(bearings) < minimum_bearing_span:
        errors.append('insufficient_frame_bearing_span_' + str(index))
    if strong_rgbd_path:
        _validate_strong_rgbd_geometry(observations, errors, index)


def _validate_strong_rgbd_geometry(observations, errors, index):
    """从原始逐帧字段复算强球面路径，禁止仅伪造累计标签。"""

    prefix = str(index)
    spherical_by_view = {}
    for item in observations:
        depth_shape = item.get('depth_shape')
        if not isinstance(depth_shape, dict):
            continue
        status = str(depth_shape.get('status', 'unknown'))
        if status in ('flat', 'anisotropic', 'non_spherical'):
            errors.append('strong_rgbd_has_non_spherical_frame_' + prefix)
        if status != 'spherical':
            continue
        diameter = _finite_number(item.get('apparent_diameter_m'))
        shape = item.get('shape')
        aspect = _finite_number(
            shape.get('aspect_ratio') if isinstance(shape, dict) else None
        )
        curvature = _finite_number(depth_shape.get('curvature_m'))
        if diameter is None or diameter <= 0.0:
            errors.append('invalid_strong_rgbd_diameter_' + prefix)
            continue
        if abs(diameter - EXPECTED_SPHERE_DIAMETER_M) / (
                EXPECTED_SPHERE_DIAMETER_M) > MAX_SPHERE_DIAMETER_RELATIVE_ERROR:
            errors.append('inconsistent_strong_rgbd_diameter_' + prefix)
        if aspect is None or aspect < MIN_SPHERE_ASPECT_RATIO:
            errors.append('inconsistent_strong_rgbd_aspect_' + prefix)
        normalized_curvature = (
            curvature / diameter if curvature is not None else None
        )
        if (normalized_curvature is None
                or normalized_curvature < MIN_NORMALIZED_DEPTH_CURVATURE
                or normalized_curvature > MAX_NORMALIZED_DEPTH_CURVATURE):
            errors.append('inconsistent_strong_rgbd_curvature_' + prefix)
        view_id = str(item.get('view_id', '')).strip()
        if view_id:
            spherical_by_view.setdefault(view_id, []).append(
                (diameter, curvature),
            )

    if len(spherical_by_view) < STRONG_RGBD_MIN_DISTINCT_VIEWS:
        errors.append('insufficient_strong_rgbd_views_' + prefix)
        return
    diameter_medians = [
        _median([sample[0] for sample in samples])
        for samples in spherical_by_view.values()
    ]
    curvature_medians = [
        _median([sample[1] for sample in samples if sample[1] is not None])
        for samples in spherical_by_view.values()
    ]
    if _coefficient_of_variation(diameter_medians) > MAX_DIAMETER_CV:
        errors.append('unstable_strong_rgbd_diameter_' + prefix)
    if (len(curvature_medians) != len(spherical_by_view)
            or _coefficient_of_variation(curvature_medians)
            > MAX_DEPTH_CURVATURE_CV):
        errors.append('unstable_strong_rgbd_curvature_' + prefix)


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


def _median(values):
    """返回有限数值中位数；空输入返回 ``None``。"""

    finite = sorted(
        number for number in (_finite_number(value) for value in values)
        if number is not None
    )
    if not finite:
        return None
    middle = len(finite) // 2
    if len(finite) % 2:
        return finite[middle]
    return 0.5 * (finite[middle - 1] + finite[middle])


def _coefficient_of_variation(values):
    """按总体标准差计算有限正数的变异系数。"""

    finite = [
        number for number in (_finite_number(value) for value in values)
        if number is not None and number > 0.0
    ]
    if len(finite) <= 1:
        return 0.0
    mean = sum(finite) / len(finite)
    variance = sum((value - mean) ** 2 for value in finite) / len(finite)
    return math.sqrt(variance) / mean


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
            raw_rgb_evidence_count=0, depth_evidence_count=0,
            active_reobservation_required=False):
    return {
        'schema': 'hazardwalker_official_random_perception_evidence_validation_v1',
        'evidence_dir': str(root),
        'result_path': str(result_file),
        'frame_count': frame_count,
        'trajectory_sample_count': trajectory_count,
        'rgb_evidence_count': image_count,
        'raw_rgb_evidence_count': raw_rgb_evidence_count,
        'depth_evidence_count': depth_evidence_count,
        'active_reobservation_required': active_reobservation_required,
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
    parser.add_argument(
        '--require-active-reobservation',
        action='store_true',
        help='额外要求局部候选经真实位姿变化后完成同目标确认闭环。',
    )
    args = parser.parse_args()
    report = validate(
        args.evidence_dir,
        args.result_json,
        require_active_reobservation=args.require_active_reobservation,
    )
    target = args.write_report or args.evidence_dir / 'independent_post_evaluation.json'
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['structural_evidence_complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
