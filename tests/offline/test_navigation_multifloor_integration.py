"""导航组多楼层代码的集成回归，防止记录污染和控制循环阻塞。"""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def _frontier_source() -> str:
    return (
        REPO_ROOT
        / 'ros2_ws'
        / 'src'
        / 'hazardwalker_nav'
        / 'hazardwalker_nav'
        / 'frontier_explorer_node.py'
    ).read_text(encoding='utf-8')


def test_floor_change_records_distinct_source_and_destination():
    source = _frontier_source()

    assert 'previous_floor = self._current_floor' in source
    assert 'previous_floor, int(next_floor)' in source
    assert 'now_ros, self._current_floor, next_floor' not in source


def test_elevator_service_does_not_block_control_timer():
    source = _frontier_source()

    assert 'ThreadPoolExecutor(' in source
    assert 'self._elevator_executor.submit(' in source
    assert 'if future is None or not future.done()' in source
    transition = source.split('def _handle_floor_transition', 1)[1]
    assert 'result = call_elevator(' not in transition


def test_manual_elevator_assist_requires_confirmation_and_3d_height_change():
    source = _frontier_source()

    assert "'/hazardwalker/navigation/elevator_ready'" in source
    assert "self._floor_transition_phase = (" in source
    assert "'manual_returning_lobby'" in source
    assert 'home_x, home_y = self._home_map_position()' in source
    assert "self._floor_transition_phase = 'manual_waiting'" in source
    assert "self._request_control_mode('keyboard')" in source
    assert "self._request_control_mode('navigation')" in source
    assert "self._floor_transition_phase = 'manual_waiting_height'" in source
    assert 'abs(self.robot_z - start_z) >= required_height' in source
    assert 'self._publish_floor_index(self._current_floor)' in source
    assert "'elevator_payload_not_transported'" in source
    assert "declare_parameter('automatic_elevator_entry', True)" in source
    assert "declare_parameter('platform_unloads_elevator_payload', True)" in source
    assert 'target-floor safe lobby; starting Frontier.' in source
    assert 'starting local door relocalization before' in source
    assert "declare_parameter('elevator_entry_forward_offset_m', 2.10)" in source
    assert "declare_parameter('elevator_entry_right_offset_m', 2.80)" in source
    assert "declare_parameter('elevator_exit_right_offset_m', 1.45)" in source
    assert "'elevator_upper_floor_entry_tolerance_m', 1.50" in source
    assert "declare_parameter('elevator_entry_stall_timeout_s', 20.0)" in source
    assert "declare_parameter('elevator_entry_min_clearance_m', 0.28)" in source
    assert "declare_parameter('elevator_exit_min_clearance_m', -1.0)" in source
    assert "declare_parameter('elevator_entry_linear_speed', 0.30)" in source
    assert "declare_parameter('elevator_exit_linear_speed', 0.45)" in source
    assert "declare_parameter('elevator_angular_speed', 1.00)" in source
    assert "self._floor_transition_phase = 'auto_opening_entry'" in source
    assert "self._floor_transition_phase = 'auto_entering'" in source
    assert "'auto_entry_open'" in source
    assert "container, elevator_id, open_doors=True" in source
    assert 'doorway_odom = (' in source
    assert 'target_odom = (' in source
    assert 'else ELEVATOR_ENTRY_ALIGNMENT_OFFSETS' in source
    assert 'self._auto_entry_alignment_index += 1' in source
    auto_open = source.split(
        "if self._floor_transition_phase == 'auto_opening_entry':", 1,
    )[1].split("if self._floor_transition_phase == 'auto_entering':", 1)[0]
    assert 'self._auto_entry_alignment_index = 0' not in auto_open
    assert 'Elevator entry stalled; retrying doorway alignment' in source
    assert 'self._auto_entry_last_pose' in source
    assert 'self._auto_entry_backoff_until_ros = now_ros + 4.0' in source
    assert 'command.linear.x = -0.45' in source
    assert '>= 0.12' in source
    assert 'navigation_clearance_override' in source
    assert 'rotation_clearance_override' in source
    assert 'linear_speed_override' in source
    assert 'angular_speed_override' in source
    assert 'navigation_clearance >= 0.0' in source
    assert 'rotation_clearance >= 0.0' in source

    manual_return = source.split(
        "if self._floor_transition_phase == 'manual_returning_lobby':", 1,
    )[1].split("if self._floor_transition_phase == 'manual_waiting':", 1)[0]
    assert 'self._return_progress_watchdog_expired(' in manual_return
    assert 'self._hybrid_return_path(' in manual_return
    assert 'self._distance_home_m() <= 1.50' in manual_return
    assert 'now_ros - self._last_return_plan_time >= replan_period' not in manual_return


def test_multifloor_time_slice_and_layer_switch_are_explicit():
    source = _frontier_source()

    assert "declare_parameter('per_floor_exploration_s', 120.0)" in source
    assert '_validated_elevator_cabin_odom' in source
    assert 'reusing the previously platform-validated cabin' in source
    assert 'Stored platform-validated elevator cabin odom anchor' in source
    assert 'room_complete and floor_elapsed >= early_finish_min' in source
    assert 'now_ros - self._floor_started_ros' in source
    assert 'self._publish_floor_index(self._current_floor)' in source
    assert 'self._home_odom = self._robot_odom' in source
    assert 'self._entry_axis = None' in source
    assert 'self._visited_pose_history.clear()' in source
    assert "declare_parameter('return_goal_tolerance_m', 0.50)" in source
    assert "self._floor_transition_phase = 'auto_exiting_floor'" in source
    assert 'New-floor elevator exit completed; starting Frontier.' in source
    assert 'def _home_relative_map_position(' in source
    assert 'target_odom_x = self._home_odom[0] + float(forward_m)' in source
    assert 'def _follow_elevator_odom_path(' in source
    assert 'self._robot_odom_yaw' in source
    assert 'self._elevator_odom_path' in source
    assert 'self._elevator_anchor_odom' in source
    assert 'self._elevator_cabin_odom_by_floor' in source
    assert 'known_upper_cabin' in source
    assert "'auto_door_localizing_close'" in source
    assert "'auto_door_capture_closed'" in source
    assert "'auto_door_localizing_open'" in source
    assert "'auto_door_capture_open'" in source
    assert "'auto_door_search_rotating'" in source
    assert "'auto_door_approaching'" in source
    assert 'detect_opened_door_from_scans(' in source
    assert 'Door approach reached the physical threshold' in source
    assert 'requesting platform validation before approach' in source
    assert 'Platform rejected lobby pose; approaching the' in source
    assert 'now_ros - self._door_approach_started_ros' in source
    assert 'now_ros - self._door_rotation_started_ros' in source
    assert 'upper_floor_entry = source_floor > 0' in source
    assert 'exit_lateral_distance = max(' in source
    assert 'build_reverse_history_path(' in source
    assert 'def _hybrid_return_path(' in source
    assert 'def _direct_home_odom_command(' in source
    assert 'def _verified_return_odom_command(' in source
    assert 'self._elevator_odom_path = []' in source
    assert 'if distance <= 2.0:' in source
    assert 'self._visited_odom_history' in source
    assert "self.state in ('EXPLORING', 'REOBSERVING')" in source
    assert 'append_loop_erased_history(' in source
    assert 'def _verified_reverse_return_path(' in source


def test_final_floor_returns_by_elevator_before_home():
    source = _frontier_source()

    floor_complete = source.split(
        'def _handle_floor_complete', 1)[1].split(
        'def _start_elevator_request', 1)[0]
    assert "'official_return_floor_index'" in floor_complete
    assert 'returning_home=True' in floor_complete
    assert 'def _start_floor_transition(' in floor_complete
    assert "'return_elevator' if returning_home else 'elevator'" in floor_complete
    begin_floor = source.split(
        'def _begin_new_floor_exploration', 1)[1].split(
        'def _handle_manual_floor_transition', 1)[0]
    assert 'if self._return_after_floor_transition:' in begin_floor
    assert "self._transition('RETURNING')" in begin_floor
    assert 'continuing to the official task home' in begin_floor
