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
    reobservation_actions_conflict,
    reobservation_request_is_eligible,
    target_centered_in_image,
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
    assert reobservation_request_is_eligible(
        {'action': 'move_left', 'target_id': 'untracked:7'},
        'EXPLORING', {'7': 4}, 4,
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

    assert find_target_detection(
        payload, '1', allow_untracked_upgrade=True,
    )['track_id'] == '1'
    assert abs(bearing_change_deg(170.0, -170.0) - 20.0) < 1e-9

    request = parse_reobservation_request({
        'view_recommendation': {
            'action': 'move_left',
            'target_id': 'untracked:1',
        },
    })
    assert request['target_id'] == '1'
    assert request['target_was_untracked'] is True


def test_established_track_never_consumes_a_new_untracked_alias():
    payload = {
        'detections_2d': [{
            'id': 1,
            'track_id': 'untracked:1',
            'view_bearing_deg': -65.0,
        }],
    }

    assert find_target_detection(payload, '1') is None
    assert find_target_detection(
        payload, '1', allow_untracked_upgrade=True,
    )['track_id'] == 'untracked:1'


def test_turn_feedback_stops_when_target_enters_center_band():
    centered = {
        'bbox': {'x_min': 270, 'x_max': 350, 'y_min': 100, 'y_max': 180},
    }
    edge = {
        'bbox': {'x_min': 600, 'x_max': 639, 'y_min': 100, 'y_max': 180},
    }

    assert target_centered_in_image(centered, 640, 0.18)
    assert not target_centered_in_image(edge, 640, 0.18)
    assert reobservation_actions_conflict('move_left', 'move_right')
    assert not reobservation_actions_conflict('move_left', 'move_forward')


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

    assert "declare_parameter('reobserve_lateral_motion_duration_s', 3.0)" in source
    assert "declare_parameter('reobserve_lateral_max_distance_m', 0.80)" in source
    assert "declare_parameter('reobserve_target_loss_timeout_s', 0.40)" in source
    assert "declare_parameter('reobserve_lateral_speed', 0.45)" in source
    assert "declare_parameter('reobserve_forward_speed', 0.30)" in source
    assert 'now = self._ros_time_sec()' in trigger
    assert 'time.monotonic()' not in trigger
    assert 'now = self._ros_time_sec()' in handler
    assert 'time.monotonic()' not in handler
    assert 'Reobservation bearing goal reached' in source
    assert 'target_centered_in_image(' in source
    assert 'reobservation_actions_conflict(' in source
    assert 'self._reobserve_allow_untracked_upgrade = False' in source
    assert 'lateral_distance >= maximum_distance' in handler


def test_returning_replans_on_sim_time_and_recovers_without_nonzero_cmd():
    source_path = os.path.join(
        REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_nav',
        'hazardwalker_nav', 'frontier_explorer_node.py',
    )
    source = open(source_path, encoding='utf-8').read()
    handler = source.split('def _handle_returning', 1)[1].split(
        'def _return_progress_watchdog_expired', 1,
    )[0]
    watchdog = source.split(
        'def _return_progress_watchdog_expired', 1,
    )[1].split('def _update_pose', 1)[0]

    assert 'now = self._ros_time_sec()' in handler
    assert 'time.monotonic()' not in handler
    assert 'now - self._last_return_plan_time >= replan_interval' not in handler
    assert 'or len(self.current_path) == 0' in handler
    assert 'goal_search_radius_m=0.0' in handler
    assert 'append_exact_goal=True' in handler
    assert 'start_search_radius_m=0.50' in handler
    assert 'Return progress watchdog expired' in watchdog
    assert 'self.current_path = []' in watchdog
    assert 'return_pose_has_progress(' in watchdog
    assert 'net_progress_expired' in watchdog
    assert 'self._return_last_net_progress_time' in watchdog
    assert 'self._return_net_progress_reference_distance' in watchdog
    assert 'if stationary_expired:' in watchdog
    assert 'return_recovery_turn_command(' in watchdog
    assert "declare_parameter('return_recovery_turn_speed', 0.80)" in source
    assert (
        "declare_parameter('return_recovery_turn_duration_s', 2.0)"
        in source
    )
    assert 'def _return_recovery_command_for_now' in source
    assert 'Return recovery turn blocked by the full-circle' in source
    assert (
        'dist_home <= self._return_best_distance_home - progress_distance'
        not in watchdog
    )
    # 看门狗直接观察实际位移，不能依赖经 scan 安全门禁后的 cmd_vel。
    assert 'cmd.' not in watchdog


def test_exploring_stuck_target_is_suppressed_before_path_is_cleared():
    source_path = os.path.join(
        REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_nav',
        'hazardwalker_nav', 'frontier_explorer_node.py',
    )
    source = open(source_path, encoding='utf-8').read()
    stuck_handler = source.split('def _update_stuck_detection', 1)[1].split(
        'def _transition', 1,
    )[0]

    mark_index = stuck_handler.index(
        'self._mark_frontier_unreachable(self.current_target)'
    )
    clear_index = stuck_handler.index('self.current_target = None')
    assert mark_index < clear_index
