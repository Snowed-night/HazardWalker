"""感知主动复查建议到导航动作的纯契约回归。"""

import os
import sys


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_nav'))

from hazardwalker_nav.reobservation_contract import (
    action_has_scan_clearance,
    bearing_change_deg,
    find_target_detection,
    parse_reobservation_request,
    reobservation_request_is_eligible,
)


def test_directional_lateral_action_reaches_navigation_unchanged():
    request = parse_reobservation_request({
        'view_recommendation': {
            'action': 'move_right',
            'reason': '侧视复查圆柱',
            'priority': 94,
            'target_id': 'track-3',
        },
    })

    assert request == {
        'action': 'move_right',
        'reason': '侧视复查圆柱',
        'priority': 94,
        'target_id': 'track-3',
    }


def test_continue_or_unknown_action_never_interrupts_exploration():
    assert parse_reobservation_request({
        'view_recommendation': {'action': 'continue_exploring'},
    }) is None
    assert parse_reobservation_request({
        'view_recommendation': {'action': 'teleport', 'target_id': '1'},
    }) is None


def test_same_target_has_bounded_reobservation_attempts_and_state_gate():
    request = {
        'action': 'move_left',
        'target_id': '7',
    }
    assert reobservation_request_is_eligible(request, 'EXPLORING', {}, 4) is True
    assert reobservation_request_is_eligible(
        request, 'REOBSERVING', {}, 4,
    ) is False
    assert reobservation_request_is_eligible(
        request, 'EXPLORING', {'7': 4}, 4,
    ) is False


def test_reobservation_motion_requires_clear_relevant_scan_sector():
    ranges = [float('inf')] * 360
    # -180° 起始、1° 分辨率时，0° 正前方在索引 180。
    ranges[179:182] = [0.35, 0.35, 0.35]
    assert action_has_scan_clearance(
        'move_forward', ranges, -3.141592653589793,
        3.141592653589793 / 180.0, 0.6,
    ) is False
    assert action_has_scan_clearance(
        'move_left', ranges, -3.141592653589793,
        3.141592653589793 / 180.0, 0.6,
    ) is True


def test_reobservation_motion_fails_closed_without_valid_scan_samples():
    assert action_has_scan_clearance(
        'move_right', [], -1.0, 0.01, 0.6,
    ) is False
    assert action_has_scan_clearance(
        'turn_left', [None, float('nan')], -1.0, 0.01, 0.6,
    ) is False
    assert action_has_scan_clearance(
        'hold_observation', [], 0.0, 0.0, 0.6,
    ) is True


def test_isolated_scan_spikes_do_not_stop_but_three_close_returns_do():
    ranges = [float('inf')] * 360
    ranges[100] = 0.20
    ranges[240] = 0.25
    assert action_has_scan_clearance(
        'turn_left', ranges, -3.141592653589793,
        3.141592653589793 / 180.0, 0.6,
    ) is True
    ranges[179:182] = [0.35, 0.35, 0.35]
    assert action_has_scan_clearance(
        'turn_left', ranges, -3.141592653589793,
        3.141592653589793 / 180.0, 0.6,
    ) is False


def test_turn_checks_side_obstacle_while_forward_uses_front_sector_only():
    ranges = [float('inf')] * 360
    # 左侧约 +90° 对应索引 270。
    ranges[269:272] = [0.35, 0.35, 0.35]
    assert action_has_scan_clearance(
        'move_forward', ranges, -3.141592653589793,
        3.141592653589793 / 180.0, 0.6,
    ) is True
    assert action_has_scan_clearance(
        'turn_right', ranges, -3.141592653589793,
        3.141592653589793 / 180.0, 0.6,
    ) is False
    assert action_has_scan_clearance(
        'move_forward', ranges, -3.141592653589793, 0.0, 0.6,
    ) is False


def test_request_carries_live_target_bearing_for_closed_loop_lateral_motion():
    payload = {
        'required_min_view_bearing_span_deg': 25.0,
        'view_recommendation': {
            'action': 'move_right',
            'target_id': 'track-3',
        },
        'detections_2d': [{
            'id': 1,
            'track_id': 'track-3',
            'view_bearing_deg': 172.0,
            'localized_position': [3.0, 4.0, 0.3],
        }],
    }

    request = parse_reobservation_request(payload)

    assert request['view_bearing_deg'] == 172.0
    assert request['required_bearing_change_deg'] == 25.0
    assert request['target_position'] == [3.0, 4.0, 0.3]
    assert find_target_detection(payload, 'track-3')['id'] == 1


def test_untracked_candidate_matches_the_track_created_during_reobservation():
    payload = {
        'detections_2d': [{
            'id': 1,
            'track_id': '1',
            'view_bearing_deg': -170.0,
        }],
    }

    assert find_target_detection(payload, 'untracked:1')['track_id'] == '1'
    assert abs(bearing_change_deg(170.0, -170.0) - 20.0) < 1e-9


def test_reobservation_uses_sim_time_and_has_feedback_bounded_lateral_motion():
    source_path = os.path.join(
        REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_nav',
        'hazardwalker_nav', 'frontier_explorer_node.py',
    )
    source = open(source_path, encoding='utf-8').read()
    trigger = source.split('def _trigger_reobservation', 1)[1].split(
        'def _update_reobservation_feedback', 1,
    )[0]
    handler = source.split('def _handle_reobserving', 1)[1].split(
        'def _handle_returning', 1,
    )[0]

    assert "declare_parameter('reobserve_lateral_motion_duration_s', 10.0)" in source
    assert 'now = self._ros_time_sec()' in trigger
    assert 'time.monotonic()' not in trigger
    assert 'now = self._ros_time_sec()' in handler
    assert 'time.monotonic()' not in handler
    assert 'Reobservation bearing goal reached' in source
