"""感知主动复查建议到导航动作的纯契约回归。"""

import os
import sys


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_nav'))

from hazardwalker_nav.reobservation_contract import (
    action_has_scan_clearance,
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
