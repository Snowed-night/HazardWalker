"""感知主动复查建议到导航动作的纯契约回归。"""

import os
import sys


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_nav'))

from hazardwalker_nav.reobservation_contract import (
    action_has_scan_clearance,
    bearing_change_deg,
    bounded_planar_pose_increment,
    find_target_detection,
    find_target_status,
    lateral_centering_angular_velocity,
    live_reobservation_action_update_allowed,
    parse_reobservation_request,
    reobservation_actions_conflict,
    reobservation_request_is_eligible,
    select_followup_reobservation_request,
    select_live_reobservation_update,
    strict_room_reobservation_allowed,
    target_centered_in_image,
    target_horizontal_error_ratio,
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


def test_strict_room_reobservation_only_runs_at_algorithmic_capture_pose():
    assert strict_room_reobservation_allowed(False, 'corridor_outbound', '')
    assert strict_room_reobservation_allowed(
        True, 'room_inspection', 'CAPTURE', camera_stable=True)
    assert not strict_room_reobservation_allowed(
        True, 'room_inspection', 'CAPTURE', camera_stable=False)
    assert not strict_room_reobservation_allowed(
        True, 'room_inspection', 'MOVE')
    assert not strict_room_reobservation_allowed(
        True, 'room_inspection', 'ORIENT')
    assert not strict_room_reobservation_allowed(
        True, 'room_cross', 'CAPTURE')
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
    assert reobservation_request_is_eligible(
        request, 'RETURNING', {}, 2,
    ) is False
    assert reobservation_request_is_eligible(
        request, 'RETURNING', {}, 2, allow_returning=True,
    ) is True
    assert reobservation_request_is_eligible(
        request, 'RETURNING', {'7': 2}, 2, allow_returning=True,
    ) is False


def test_official_returning_reobservation_is_bounded_and_resumes_returning():
    source_path = os.path.join(
        REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_nav',
        'hazardwalker_nav', 'frontier_explorer_node.py',
    )
    source = open(source_path, encoding='utf-8').read()

    assert "declare_parameter('reobserve_during_returning', False)" in source
    assert "'reobserve_returning_max_attempts_per_target', 2" in source
    assert "'RETURNING' if self.state == 'RETURNING' else 'EXPLORING'" in source
    assert 'self._transition(resume_state)' in source
    launch_path = os.path.join(
        REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_bringup',
        'launch', 'official_simenv_business.launch.py',
    )
    launch = open(launch_path, encoding='utf-8').read()
    assert "'reobserve_during_returning': False" in launch
    assert "'reobserve_returning_max_attempts_per_target': 0" in launch
    assert "'reobserve_max_attempts_per_target': 4" in launch


def test_official_reobservation_cannot_preempt_corridor_or_door_entry():
    source_path = os.path.join(
        REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_nav',
        'hazardwalker_nav', 'frontier_explorer_node.py',
    )
    source = open(source_path, encoding='utf-8').read()
    assert "'reobserve_only_inside_active_room', False" in source
    assert "and self._active_room_sector is None" in source
    assert "self._deterministic_route_phase == 'room_inspection'" in source
    hazard_callback = source.split('def on_hazard', 1)[1].split(
        'def on_inspection_result', 1)[0]
    assert 'strict_room_reobservation_allowed(' in hazard_callback
    assert 'inspection_phase = execution.phase' in hazard_callback
    assert "payload.get('camera_stable', False)" in hazard_callback
    assert hazard_callback.index(
        "strict_room_inspection_enabled').value") < hazard_callback.index(
            'request = parse_reobservation_request(payload)')

    launch_path = os.path.join(
        REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_bringup',
        'launch', 'official_simenv_business.launch.py',
    )
    launch = open(launch_path, encoding='utf-8').read()
    assert "'reobserve_only_inside_active_room': True" in launch


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


def test_ineligible_partial_box_stays_reobserve_and_archive_uses_real_views():
    """黄色复查候选不能冒充已确认目标，证据也不能伪造多视角字段。"""

    detector_path = os.path.join(
        REPO_ROOT,
        'ros2_ws',
        'src',
        'hazardwalker_perception',
        'hazardwalker_perception',
        'hsv_detector_node.py',
    )
    with open(detector_path, encoding='utf-8') as handle:
        detector = handle.read()
    assert 'or not confirmation_eligible' in detector
    assert 'shape_complete_for_3d_tracking' in detector
    assert 'if localization and shape_complete_for_3d_tracking' in detector

    capture_path = os.path.join(
        REPO_ROOT,
        'scripts',
        'capture_official_simenv_rgbd_case.py',
    )
    with open(capture_path, encoding='utf-8') as handle:
        capture = handle.read()
    assert "'view_bearing_span_deg'" in capture
    assert "'required_min_distinct_views'" in capture
    assert "'source_ids'" not in capture
    assert "if track_status == 'confirmed'" in capture


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


def test_stable_candidate_alias_links_partial_request_to_later_track_status():
    payload = {
        'detections_2d': [{
            'id': 7,
            'track_id': '7',
            'candidate_id': 'candidate-3',
            'candidate_aliases': ['candidate-3'],
        }],
        'hazards': [{
            'id': 7,
            'track_id': 7,
            'candidate_ids': ['candidate-3'],
            'status': 'confirmed',
        }],
    }

    detection = find_target_detection(
        payload, 'candidate-3', allow_untracked_upgrade=True,
    )

    assert detection['track_id'] == '7'
    assert find_target_status(payload, 'candidate-3') == 'confirmed'


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
    assert target_horizontal_error_ratio(centered, 640) == -0.03125
    assert target_horizontal_error_ratio(edge, 640) > 0.9


def test_lateral_arc_centering_turns_toward_target_with_limits():
    assert lateral_centering_angular_velocity(-0.5, 0.8, 0.6, 0.05) == 0.4
    assert lateral_centering_angular_velocity(0.9, 0.8, 0.6, 0.05) == -0.6
    assert lateral_centering_angular_velocity(0.02, 0.8, 0.6, 0.05) == 0.0
    assert lateral_centering_angular_velocity(float('nan'), 0.8, 0.6) == 0.0


def test_live_action_updates_do_not_oscillate_between_lateral_and_turning():
    assert live_reobservation_action_update_allowed('turn_left', 'move_left')
    assert live_reobservation_action_update_allowed('turn_right', 'move_right')
    assert not live_reobservation_action_update_allowed('turn_left', 'move_right')
    assert not live_reobservation_action_update_allowed('move_left', 'turn_left')
    assert not live_reobservation_action_update_allowed('move_right', 'turn_right')
    assert live_reobservation_action_update_allowed('turn_left', 'move_forward')


def test_reobservation_pose_increment_accepts_realistic_motion():
    increment = bounded_planar_pose_increment(
        (18.32, 0.63), (18.36, 0.65), 0.30,
    )

    assert abs(increment - (0.04 ** 2 + 0.02 ** 2) ** 0.5) < 1e-9


def test_reobservation_pose_increment_rejects_slam_map_jump():
    assert bounded_planar_pose_increment(
        (18.32, 0.63), (-3.60, 18.49), 0.30,
    ) is None


def test_filtered_reobservation_distance_accumulates_only_valid_steps():
    poses = [(0.0, 0.0), (0.2, 0.0), (23.8, 0.0), (24.0, 0.0), (24.2, 0.0)]
    distance = 0.0
    previous = poses[0]
    for current in poses[1:]:
        increment = bounded_planar_pose_increment(previous, current, 0.30)
        previous = current
        if increment is not None:
            distance += increment

    assert abs(distance - 0.60) < 1e-9


def test_same_target_live_recommendation_updates_active_action():
    payload = {
        'view_recommendation': {
            'action': 'move_forward',
            'target_id': 'candidate-1',
            'reason': '目标已离开边缘，靠近补充面积',
        },
    }

    update = select_live_reobservation_update(
        payload, 'candidate-1', 'turn_right',
    )

    assert update['action'] == 'move_forward'
    assert update['target_id'] == 'candidate-1'


def test_same_target_live_lateral_recommendation_updates_active_action():
    payload = {
        'view_recommendation': {
            'action': 'move_right',
            'target_id': 'untracked:candidate-2',
        },
    }

    update = select_live_reobservation_update(
        payload, 'candidate-2', 'turn_right',
    )

    assert update['action'] == 'move_right'


def test_live_update_follows_explicit_candidate_alias_after_track_upgrade():
    payload = {
        'view_recommendation': {
            'action': 'move_right',
            'target_id': 'track-7',
        },
        'detections_2d': [{
            'track_id': 'track-7',
            'candidate_aliases': ['candidate-2'],
        }],
    }

    update = select_live_reobservation_update(
        payload, 'candidate-2', 'turn_right',
    )

    assert update['action'] == 'move_right'
    assert update['target_id'] == 'track-7'


def test_live_update_rejects_unlinked_track_even_with_similar_suffix():
    payload = {
        'view_recommendation': {
            'action': 'move_right',
            'target_id': 'track-2',
        },
        'detections_2d': [{
            'track_id': 'track-2',
            'candidate_aliases': ['candidate-9'],
        }],
    }

    assert select_live_reobservation_update(
        payload, 'candidate-2', 'turn_right',
    ) is None


def test_live_recommendation_cannot_switch_target_or_repeat_action():
    other_target = {
        'view_recommendation': {
            'action': 'move_forward',
            'target_id': 'candidate-9',
        },
    }
    same_action = {
        'view_recommendation': {
            'action': 'turn_right',
            'target_id': 'candidate-1',
        },
    }

    assert select_live_reobservation_update(
        other_target, 'candidate-1', 'turn_right',
    ) is None
    assert select_live_reobservation_update(
        same_action, 'candidate-1', 'turn_right',
    ) is None


def test_followup_request_allows_same_action_only_for_same_target_alias():
    payload = {
        'view_recommendation': {
            'action': 'move_left',
            'target_id': '1',
        },
        'detections_2d': [{
            'track_id': '1',
            'candidate_aliases': ['candidate-10'],
        }],
    }
    assert select_followup_reobservation_request(
        payload, 'candidate-10')['action'] == 'move_left'
    assert select_followup_reobservation_request(
        payload, 'unrelated-track') is None


def test_invalid_or_continue_live_recommendation_never_changes_action():
    for action in ('teleport', 'continue_exploring'):
        assert select_live_reobservation_update(
            {
                'view_recommendation': {
                    'action': action,
                    'target_id': 'candidate-1',
                },
            },
            'candidate-1',
            'turn_right',
        ) is None


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
    assert "declare_parameter('reobserve_pose_jump_reject_m', 0.30)" in source
    assert "declare_parameter('reobserve_target_loss_timeout_s', 0.40)" in source
    assert "declare_parameter('reobserve_lateral_speed', 0.45)" in source
    assert "declare_parameter('reobserve_lateral_centering_gain', 0.80)" in source
    assert "declare_parameter('reobserve_forward_speed', 0.45)" in source
    assert 'now = self._ros_time_sec()' in trigger
    assert 'time.monotonic()' not in trigger
    assert 'now = self._ros_time_sec()' in handler
    assert 'time.monotonic()' not in handler
    assert 'Reobservation bearing goal reached' in source
    assert 'target_centered_in_image(' in source
    assert 'reobservation_actions_conflict(' in source
    assert 'select_live_reobservation_update(' in source
    assert 'select_followup_reobservation_request(' in source
    assert 'starting the next bounded segment for the same target' in source
    assert 'resume_state=resume_state' in source
    assert 'Reobservation action updated for target=' in source
    assert 'self.reobserve_end_time =' not in source.split(
        'def _update_active_reobservation_action', 1,
    )[1].split('def _stop_reobservation_motion', 1)[0]
    assert 'self._reobserve_allow_untracked_upgrade = False' in source
    assert 'lateral_distance >= maximum_distance' in handler
    assert 'bounded_planar_pose_increment(' in handler
    assert 'self._reobserve_lateral_distance_m += increment' in handler
    assert "turn_action = 'turn_left' if angular > 0.0 else 'turn_right'" in handler
    assert 'cmd.angular.z = angular' in handler
    assert 'lateral_centering_angular_velocity(' in handler
    assert 'live_reobservation_action_update_allowed(' in source
    assert 'self._reobserve_start_pose' not in source


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
    recovery = source.split(
        'def _return_recovery_command_for_now', 1,
    )[1].split('def _return_progress_watchdog_expired', 1)[0]
    planner = source.split('def _hybrid_return_path', 1)[1].split(
        'def _verified_reverse_return_path', 1)[0]

    assert 'now = self._ros_time_sec()' in handler
    assert 'time.monotonic()' not in handler
    assert 'now - self._last_return_plan_time >= replan_interval' not in handler
    assert 'or len(self.current_path) == 0' in handler
    assert 'self._hybrid_return_path(' in handler
    assert 'goal_search_radius_m=0.0' in planner
    assert 'append_exact_goal=True' in planner
    assert 'start_search_radius_m=0.50' in planner
    assert 'self._verified_reverse_return_path(home_x, home_y)' in planner
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
    assert 'self._elevator_odom_path = []' not in recovery
    assert 'self._elevator_odom_path_index + 2' in recovery


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
    # Gazebo 可能只有 0.1 左右实时因子；卡死判断必须使用仿真时钟，不能让
    # 墙钟看门狗提前打断按仿真时间工作的房间目标收缩恢复器。
    assert 'now = self._ros_time_sec()' in stuck_handler


def test_deterministic_room_new_goals_use_filtered_unitree_backend():
    source_path = os.path.join(
        REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_nav',
        'hazardwalker_nav', 'frontier_explorer_node.py',
    )
    source = open(source_path, encoding='utf-8').read()
    handler = source.split(
        'def _handle_deterministic_room_route', 1,
    )[1].split('def _handle_exploring', 1)[0]

    room_approach = handler.split(
        "if phase == 'room_approach':", 1,
    )[1].split("if phase == 'room_cross':", 1)[0]
    room_waypoints = handler.split(
        "if phase == 'room_cross':", 1,
    )[1]
    assert 'else self._follow_path()' in room_approach
    assert 'else self._follow_path()' in room_waypoints


def test_deterministic_doorways_freeze_after_corridor_outbound():
    source_path = os.path.join(
        REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_nav',
        'hazardwalker_nav', 'frontier_explorer_node.py',
    )
    source = open(source_path, encoding='utf-8').read()
    handler = source.split(
        'def _handle_deterministic_room_route', 1,
    )[1].split('def _handle_exploring', 1)[0]

    refresh = handler.split(
        'self._refresh_deterministic_doorways()', 1,
    )[0].rsplit('\n', 8)[-8:]
    refresh_context = '\n'.join(refresh)
    assert "self._deterministic_route_phase == 'corridor_outbound'" in (
        refresh_context
    )
    assert 'select_symmetric_doorway_stations(' in source


def test_deterministic_corridor_rejects_entry_empty_before_calibrated_depth():
    """入口空地即使形成两排候选，也不能让确定性路线提前返程。"""

    source_path = os.path.join(
        REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_nav',
        'hazardwalker_nav', 'frontier_explorer_node.py',
    )
    source = open(source_path, encoding='utf-8').read()
    assert "'deterministic_corridor_min_progress_m', 30.0" in source
    assert "'deterministic_corridor_max_lateral_m', 1.0" in source
    assert "'deterministic_corridor_center_lateral_m', 0.0" in source
    assert "'deterministic_corridor_inflation_radius_m', 0.20" in source
    assert "'deterministic_entry_direct_until_progress_m', 0.0" in source
    assert "'deterministic_entry_clearance_m', -1.0" in source
    assert "'deterministic_entry_rotation_clearance_m', 0.30" in source
    assert "'deterministic_calibrated_doorways_enabled', False" in source
    assert "'deterministic_near_door_progress_m', 18.9" in source
    assert "'deterministic_far_door_progress_m', 32.7" in source
    assert "'deterministic_corridor_hard_limit_m', 35.5" in source
    assert "'deterministic_corridor_waypoint_tolerance_m', 0.8" in source
    assert 'minimum_outbound_reached' in source
    assert source.count('and minimum_outbound_reached') >= 2
    corridor_planner = source.split(
        'def _plan_reachable_corridor_goal', 1,
    )[1].split('def _update_deterministic_room_distance', 1)[0]
    assert "'deterministic_corridor_max_lateral_m'" in corridor_planner
    assert "'deterministic_corridor_inflation_radius_m'" in corridor_planner
    assert 'fallback_path = [fallback]' in corridor_planner
    assert 'center_goal = self._deterministic_world_point(' in corridor_planner
    assert 'goal_progress = min(progress, hard_limit)' in corridor_planner
    assert 'progress > robot_progress + minimum_goal_gap' in corridor_planner
    assert "label == 'corridor_outbound'" in source
    assert 'path.append(resolved_goal)' in source
    assert 'for lateral_limit in (narrow, wide)' not in corridor_planner
    route_handler = source.split(
        'def _handle_deterministic_room_route', 1,
    )[1].split('def _handle_exploring', 1)[0]
    assert "'deterministic_corridor_center_lateral_m'" in route_handler
    assert 'self.start_x - self._entry_axis[1] * lateral_offset' in route_handler
    assert "phase == 'corridor_outbound'" in route_handler
    assert "'deterministic_entry_direct_until_progress_m'" in route_handler
    assert 'navigation_clearance_override=clearance' in route_handler
    assert 'rotation_clearance_override=rotation_clearance' in route_handler
    assert 'self._seed_calibrated_doorways()' in route_handler
    assert "'use_official_odom_for_room_control'" in source
    assert "path = [resolved_goal]" in source
