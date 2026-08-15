"""电梯 + 门控调用模块的离线回归测试。

所属组：导航组。
文件作用：验证 rosservice 输出解析、电梯入口坐标解析、docker exec 命令构造，
通过 mock subprocess.run 避免真实调用容器，不依赖 ROS2 / Docker。

对应实现：ros2_ws/src/hazardwalker_nav/hazardwalker_nav/elevator_controller.py
"""

from pathlib import Path
from types import SimpleNamespace
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
NAV_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav'
if str(NAV_SRC) not in sys.path:
    sys.path.insert(0, str(NAV_SRC))

from hazardwalker_nav import elevator_controller as ec  # noqa: E402


def test_parse_rosservice_output_bool_and_string():
    parsed = ec._parse_rosservice_output('accepted: True\nstate: idle\n')

    assert parsed['accepted'] is True
    assert parsed['state'] == 'idle'


def test_parse_rosservice_output_false():
    parsed = ec._parse_rosservice_output('accepted: False\n')

    assert parsed['accepted'] is False


def test_parse_rosservice_output_positive_and_negative_int():
    parsed = ec._parse_rosservice_output(
        'current_floor: 2\nanother: -1\n')

    assert parsed['current_floor'] == 2
    assert parsed['another'] == -1


def test_parse_rosservice_output_strips_quotes():
    parsed = ec._parse_rosservice_output('message: "ok"\n')

    assert parsed['message'] == 'ok'


def test_parse_rosservice_output_ignores_lines_without_colon():
    parsed = ec._parse_rosservice_output(
        'no_colon_here\naccepted: True\n')

    assert parsed == {'accepted': True}


def test_parse_rosservice_output_empty():
    assert ec._parse_rosservice_output('') == {}


def test_elevator_approach_position_default_is_zero():
    assert ec.elevator_approach_position(0) == (0.0, 0.0)
    assert ec.elevator_approach_position(1) == (0.0, 0.0)


def test_elevator_approach_position_uses_injected_positions():
    custom = {0: (1.65, 2.6), 1: (3.0, 4.0)}

    assert ec.elevator_approach_position(0, custom) == (1.65, 2.6)
    assert ec.elevator_approach_position(1, custom) == (3.0, 4.0)
    # 未注入的楼层回退到 (0, 0)
    assert ec.elevator_approach_position(2, custom) == (0.0, 0.0)


def test_elevator_door_id():
    assert ec.elevator_door_id(0) == 'elevator_floor_0'
    assert ec.elevator_door_id(2) == 'elevator_floor_2'


def test_call_elevator_builds_command_and_parses_result():
    captured = {}

    def fake_run(cmd, capture_output, text, timeout):
        captured['cmd'] = cmd
        captured['timeout'] = timeout
        return SimpleNamespace(
            returncode=0,
            stdout='accepted: True\ncurrent_floor: 1\n'
                   'state: moving\nmessage: ok\n',
            stderr='',
        )

    original = ec.subprocess.run
    ec.subprocess.run = fake_run
    try:
        result = ec.call_elevator(
            'container_x', 'elevator_main', 1, True)
    finally:
        ec.subprocess.run = original

    assert result.accepted is True
    assert result.current_floor == 1
    assert result.state == 'moving'
    assert result.message == 'ok'
    # docker exec 的 bash 片段应包含完整的 rosservice call 命令。
    bash_cmd = captured['cmd'][-1]
    assert 'rosservice call /call_elevator elevator_main 1 true' in bash_cmd


def test_set_door_state_builds_command_and_parses_result():
    captured = {}

    def fake_run(cmd, capture_output, text, timeout):
        captured['cmd'] = cmd
        return SimpleNamespace(
            returncode=0,
            stdout='accepted: True\nstate: open\n',
            stderr='',
        )

    original = ec.subprocess.run
    ec.subprocess.run = fake_run
    try:
        result = ec.set_door_state('container_x', 'main_entrance', True)
    finally:
        ec.subprocess.run = original

    assert result.accepted is True
    assert result.state == 'open'
    bash_cmd = captured['cmd'][-1]
    assert 'rosservice call /set_door_state main_entrance true' in bash_cmd


def test_docker_exec_raises_on_nonzero_returncode():
    def fake_run(cmd, capture_output, text, timeout):
        return SimpleNamespace(
            returncode=1,
            stdout='',
            stderr='some docker error',
        )

    original = ec.subprocess.run
    ec.subprocess.run = fake_run
    try:
        raised = False
        try:
            ec._docker_exec('container_x', 'rosservice call /x')
        except RuntimeError as exc:
            raised = True
            assert 'docker exec' in str(exc)
        assert raised
    finally:
        ec.subprocess.run = original
