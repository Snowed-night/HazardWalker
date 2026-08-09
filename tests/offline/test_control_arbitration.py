"""统一控制仲裁器离线安全契约测试。"""

import math
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PLATFORM_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform'
if str(PLATFORM_SRC) not in sys.path:
    sys.path.insert(0, str(PLATFORM_SRC))

from hazardwalker_platform.control_arbitration import (  # noqa: E402
    ControlArbitrator,
    should_publish_status,
)


def test_only_selected_fresh_source_reaches_output():
    arbitrator = ControlArbitrator(default_mode='keyboard')
    arbitrator.update_source(
        'keyboard', linear_x=0.4, linear_y=0.0, angular_z=0.1,
        received_monotonic_sec=10.0,
    )
    arbitrator.update_source(
        'navigation', linear_x=0.2, linear_y=0.0, angular_z=-0.3,
        received_monotonic_sec=10.0,
    )
    result = arbitrator.resolve(10.1)
    assert result.source_fresh
    assert result.mode == 'keyboard'
    assert (result.linear_x, result.angular_z) == (0.4, 0.1)


def test_mode_switch_never_reuses_previous_source():
    arbitrator = ControlArbitrator(default_mode='keyboard')
    arbitrator.update_source(
        'keyboard', linear_x=0.4, linear_y=0.0, angular_z=0.0,
        received_monotonic_sec=3.0,
    )
    arbitrator.select_mode('assist')
    result = arbitrator.resolve(3.1)
    assert not result.source_fresh
    assert result.reason == 'source_never_received'
    assert result.linear_x == result.angular_z == 0.0


def test_mode_switch_discards_target_source_command_sent_before_takeover():
    arbitrator = ControlArbitrator(default_mode='keyboard')
    arbitrator.update_source(
        'assist', linear_x=0.0, linear_y=0.0, angular_z=0.6,
        received_monotonic_sec=3.0,
    )
    arbitrator.select_mode('assist')
    assert arbitrator.resolve(3.1).reason == 'source_never_received'
    arbitrator.update_source(
        'assist', linear_x=0.0, linear_y=0.0, angular_z=0.5,
        received_monotonic_sec=3.2,
    )
    assert arbitrator.resolve(3.3).angular_z == 0.5


def test_source_timeout_and_explicit_stop_fail_closed():
    arbitrator = ControlArbitrator(
        default_mode='assist', source_timeouts_sec={'assist': 0.2})
    arbitrator.update_source(
        'assist', linear_x=0.0, linear_y=0.0, angular_z=0.5,
        received_monotonic_sec=1.0,
    )
    assert arbitrator.resolve(1.21).reason == 'source_timeout'
    arbitrator.select_mode('stopped')
    assert arbitrator.resolve(1.22).reason == 'explicit_stop'


def test_invalid_modes_and_non_finite_commands_are_rejected():
    arbitrator = ControlArbitrator()
    try:
        arbitrator.select_mode('automatic_magic')
    except ValueError:
        pass
    else:
        raise AssertionError('未知控制模式必须被拒绝')
    try:
        arbitrator.update_source(
            'keyboard', linear_x=math.nan, linear_y=0.0, angular_z=0.0,
            received_monotonic_sec=1.0,
        )
    except ValueError:
        pass
    else:
        raise AssertionError('NaN 速度必须被拒绝')


def test_invalid_source_timeout_configuration_is_rejected():
    for configuration, expected in (
        ({'assist': math.nan}, '有限正数'),
        ({'perception': 0.5}, '未知控制源'),
    ):
        try:
            ControlArbitrator(source_timeouts_sec=configuration)
        except ValueError as exc:
            assert expected in str(exc)
        else:
            raise AssertionError('非法超时配置必须被拒绝')


def test_clearing_a_faulty_selected_source_stops_without_waiting_for_timeout():
    arbitrator = ControlArbitrator(default_mode='keyboard')
    arbitrator.update_source(
        'keyboard', linear_x=0.4, linear_y=0.0, angular_z=0.0,
        received_monotonic_sec=10.0,
    )
    assert arbitrator.resolve(10.1).linear_x == 0.4

    arbitrator.clear_source('keyboard')
    stopped = arbitrator.resolve(10.1)
    assert stopped.source_fresh is False
    assert stopped.reason == 'source_never_received'
    assert stopped.linear_x == 0.0


def test_status_is_republished_on_change_or_heartbeat_for_late_subscribers():
    assert should_publish_status(
        'keyboard:fresh', None,
        now_monotonic_sec=10.0,
        previous_publish_monotonic_sec=0.0,
        heartbeat_sec=1.0,
    )
    assert not should_publish_status(
        'keyboard:fresh', 'keyboard:fresh',
        now_monotonic_sec=10.5,
        previous_publish_monotonic_sec=10.0,
        heartbeat_sec=1.0,
    )
    assert should_publish_status(
        'keyboard:fresh', 'keyboard:fresh',
        now_monotonic_sec=11.0,
        previous_publish_monotonic_sec=10.0,
        heartbeat_sec=1.0,
    )


def test_assist_cancel_while_idle_does_not_request_a_control_mode_change():
    source = (
        PLATFORM_SRC / 'hazardwalker_platform' / 'assist_alignment_node.py'
    ).read_text(encoding='utf-8')
    cancel_body = source.split(
        'def on_cancel(self, _request, response):', 1
    )[1].split('def on_timer', 1)[0]
    assert 'if was_active:' in cancel_body
    assert "self._publish_status('idle', 'assist_not_running')" in cancel_body


def test_rejected_assist_start_publishes_reason_for_gui_feedback():
    source = (
        PLATFORM_SRC / 'hazardwalker_platform' / 'assist_alignment_node.py'
    ).read_text(encoding='utf-8')
    start_body = source.split(
        'def on_start(self, _request, response):', 1
    )[1].split('def on_cancel', 1)[0]
    invalid_body = start_body.split('if not decision.valid:', 1)[1].split(
        'self.active = True', 1)[0]
    assert "self._publish_status('idle', decision.reason, decision)" in invalid_body
