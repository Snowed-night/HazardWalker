"""人工确认后的红球辅助居中纯逻辑测试。"""

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PLATFORM_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform'
if str(PLATFORM_SRC) not in sys.path:
    sys.path.insert(0, str(PLATFORM_SRC))

from hazardwalker_platform.assist_alignment import (  # noqa: E402
    AlignmentConfig,
    compute_alignment_decision,
    evaluate_control_takeover,
)


def _payload(x_min, x_max, *, target_id='candidate-1'):
    return {
        'image_width': 1000,
        'view_recommendation': {'target_id': target_id},
        'detections_2d': [{
            'id': 'untracked:candidate-1',
            'candidate_id': 'candidate-1',
            'confidence': 0.9,
            'requires_reobservation': True,
            'bbox': {'x_min': x_min, 'x_max': x_max, 'y_min': 10, 'y_max': 90},
        }],
    }


def test_left_candidate_requests_positive_yaw_and_right_negative_yaw():
    left = compute_alignment_decision(_payload(100, 200))
    right = compute_alignment_decision(_payload(800, 900))
    assert left.valid and left.reason == 'turn_left' and left.angular_z > 0.0
    assert right.valid and right.reason == 'turn_right' and right.angular_z < 0.0


def test_centered_candidate_stops_without_false_confirmation():
    decision = compute_alignment_decision(
        _payload(470, 530), AlignmentConfig(center_tolerance_ratio=0.08))
    assert decision.valid and decision.centered
    assert decision.angular_z == 0.0
    assert decision.reason == 'target_centered'


def test_named_target_must_be_visible_and_unknown_target_is_rejected():
    decision = compute_alignment_decision(
        _payload(100, 200, target_id='candidate-99'))
    assert not decision.valid
    assert decision.reason == 'target_not_visible'


def test_missing_image_geometry_fails_closed():
    decision = compute_alignment_decision({'detections_2d': []})
    assert not decision.valid
    assert decision.angular_z == 0.0
    assert decision.reason == 'missing_image_width'


def test_confirmed_assist_target_override_cannot_switch_to_new_recommendation():
    payload = {
        'image_width': 1000,
        'view_recommendation': {'target_id': 'candidate-2'},
        'detections_2d': [
            {
                'candidate_id': 'candidate-1',
                'confidence': 0.7,
                'requires_reobservation': True,
                'bbox': {
                    'x_min': 100, 'x_max': 200, 'y_min': 10, 'y_max': 90},
            },
            {
                'candidate_id': 'candidate-2',
                'confidence': 0.99,
                'requires_reobservation': True,
                'bbox': {
                    'x_min': 800, 'x_max': 900, 'y_min': 10, 'y_max': 90},
            },
        ],
    }
    decision = compute_alignment_decision(
        payload, target_id_override='candidate-1')
    assert decision.valid
    assert decision.target_id == 'candidate-1'
    assert decision.reason == 'turn_left'


def test_confirmed_assist_target_loss_stops_instead_of_switching_targets():
    payload = _payload(800, 900, target_id='candidate-1')
    decision = compute_alignment_decision(
        payload, target_id_override='candidate-99')
    assert not decision.valid
    assert decision.target_id == 'candidate-99'
    assert decision.reason == 'target_not_visible'


def test_non_finite_or_reversed_bbox_fails_closed():
    for x_min, x_max in ((float('nan'), 200), (300, 200), (1100, 1200)):
        decision = compute_alignment_decision(_payload(x_min, x_max))
        assert not decision.valid
        assert decision.angular_z == 0.0
        assert decision.reason == 'invalid_bbox'


def test_rejected_non_sphere_cannot_be_selected_by_stale_recommendation():
    payload = _payload(100, 200)
    payload['detections_2d'][0]['track_status'] = 'rejected_non_spherical'
    decision = compute_alignment_decision(payload)
    assert not decision.valid
    assert decision.reason == 'target_not_visible'


def test_non_finite_alignment_parameters_are_rejected():
    try:
        compute_alignment_decision(
            _payload(100, 160),
            AlignmentConfig(angular_kp=float('nan')),
        )
    except ValueError as exc:
        assert '有限' in str(exc)
    else:
        raise AssertionError('非有限对准参数必须被拒绝')


def test_assist_waits_for_mux_confirmation_and_retries_before_timeout():
    waiting = evaluate_control_takeover(
        'keyboard', elapsed_sec=0.1, since_request_sec=0.1,
        timeout_sec=1.5, retry_sec=0.2)
    assert not waiting.ready and not waiting.failed
    assert not waiting.should_retry

    retry = evaluate_control_takeover(
        '', elapsed_sec=0.4, since_request_sec=0.25,
        timeout_sec=1.5, retry_sec=0.2)
    assert retry.should_retry and retry.reason == 'waiting_for_control_takeover'

    ready = evaluate_control_takeover(
        'assist', elapsed_sec=0.5, since_request_sec=0.1,
        timeout_sec=1.5, retry_sec=0.2)
    assert ready.ready and not ready.failed and not ready.should_retry


def test_assist_takeover_timeout_and_invalid_time_fail_closed():
    timeout = evaluate_control_takeover(
        'keyboard', elapsed_sec=1.5, since_request_sec=0.2,
        timeout_sec=1.5, retry_sec=0.2)
    assert timeout.failed and timeout.reason == 'control_takeover_timeout'

    for kwargs in (
        {'elapsed_sec': -0.1, 'since_request_sec': 0.0},
        {'elapsed_sec': 0.1, 'since_request_sec': float('nan')},
    ):
        try:
            evaluate_control_takeover(
                'keyboard', timeout_sec=1.5, retry_sec=0.2, **kwargs)
        except ValueError:
            pass
        else:
            raise AssertionError('非法控制接管时间必须被拒绝')
