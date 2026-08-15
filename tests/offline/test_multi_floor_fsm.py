"""多楼层探索状态机的离线回归测试。

所属组：导航组。
文件作用：验证 FrontierExplorerNode 多楼层扩展的纯状态逻辑——楼层序列推进、
覆盖达标判定、跨层 from_floor 语义、电梯退避、超时逃生重置地图，不依赖 ROS2。

实现要点：
- 用最小 stub 顶替 rclpy / tf2_ros / 各 msgs 包，使 frontier_explorer_node.py
  可以在离线环境 import；状态机 handler 通过 types.SimpleNamespace 模拟 `self`
  的必要字段后直接调用，断言修复后的语义。
"""

from pathlib import Path
from types import SimpleNamespace
import sys
import types

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
NAV_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav'
if str(NAV_SRC) not in sys.path:
    sys.path.insert(0, str(NAV_SRC))


def _install_ros_stubs():
    """用最小 stub 顶替 ROS 模块，允许离线 import frontier_explorer_node。"""
    if 'rclpy' in sys.modules:
        return

    rclpy = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')

    class _Node:
        pass
    node_mod.Node = _Node

    clock_mod = types.ModuleType('rclpy.clock')

    class _ClockType:
        STEADY_TIME = 'STEADY_TIME'
        ROS_TIME = 'ROS_TIME'

    class _Clock:
        def __init__(self, clock_type=None):
            self.clock_type = clock_type
    clock_mod.Clock = _Clock
    clock_mod.ClockType = _ClockType

    exec_mod = types.ModuleType('rclpy.executors')

    class _ExternalShutdownException(Exception):
        pass
    exec_mod.ExternalShutdownException = _ExternalShutdownException

    rclpy.node = node_mod
    rclpy.clock = clock_mod
    rclpy.executors = exec_mod
    sys.modules['rclpy'] = rclpy
    sys.modules['rclpy.node'] = node_mod
    sys.modules['rclpy.clock'] = clock_mod
    sys.modules['rclpy.executors'] = exec_mod

    sys.modules['tf2_ros'] = types.ModuleType('tf2_ros')

    for pkg, msgs in (
        ('geometry_msgs', ('Twist',)),
        ('nav_msgs', ('OccupancyGrid',)),
        ('sensor_msgs', ('LaserScan',)),
        ('std_msgs', ('Int32', 'String')),
    ):
        pkg_mod = types.ModuleType(pkg)
        msg_mod = types.ModuleType(f'{pkg}.msg')
        for name in msgs:
            setattr(msg_mod, name, type(name, (), {}))
        pkg_mod.msg = msg_mod
        sys.modules[pkg] = pkg_mod
        sys.modules[f'{pkg}.msg'] = msg_mod


_install_ros_stubs()

from hazardwalker_nav import frontier_explorer_node as fen  # noqa: E402


class _Logger:
    def info(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass

    def debug(self, *a, **k):
        pass


class _FakeCoverage:
    def __init__(self, ratio):
        self._ratio = ratio

    def floor_coverage_ratio(self, grid):
        return self._ratio


def _bind(func, obj):
    return types.MethodType(func, obj)


def _make_node():
    """构造带多楼层字段与真实方法绑定的 mock self。"""
    node = SimpleNamespace()
    node._target_floors = [0, 1, 2]
    node._current_floor = 0
    node._coverage = None
    node.grid = None
    node._elevator_initiated = False
    node._elevator_floor_reached = False
    node._floor_complete_since_ros = None
    node._floor_transition_phase = ''
    node._floor_transition_start_ros = None
    node._floor_transition_from_floor = None
    node._elevator_positions = {}
    node._last_elevator_call_ros = None
    node.robot_x = 0.0
    node.robot_y = 0.0
    node.current_target = None
    node.current_path = []
    node.path_index = 0
    node._visited_frontiers = set()
    node._unreachable_frontiers = {}
    node._detour_deferred_frontiers = {}
    node._now = [100.0]
    node.get_logger = lambda: _Logger()
    node.recorder = SimpleNamespace(
        record_floor_change=lambda *a: None,
        record_elevator_call=lambda *a: None,
        record_state_transition=lambda *a: None,
    )
    node.floor_index_pub = SimpleNamespace(publish=lambda msg: None)
    node._reset_frontier_progress_watchdog = lambda: None
    node._ros_time_sec = lambda: node._now[0]
    node._transition = lambda s: None
    node._publish_floor_index = _bind(
        fen.FrontierExplorerNode._publish_floor_index, node)
    node._next_floor = _bind(fen.FrontierExplorerNode._next_floor, node)
    node._floor_is_covered = _bind(
        fen.FrontierExplorerNode._floor_is_covered, node)
    node._handle_floor_complete = _bind(
        fen.FrontierExplorerNode._handle_floor_complete, node)
    node._handle_floor_transition = _bind(
        fen.FrontierExplorerNode._handle_floor_transition, node)
    return node


def _make_get_parameter():
    params = {
        'simenv_container': 'simenv_ros1_hazard_platform',
        'elevator_id': 'elevator_main',
        'goal_tolerance_m': 0.25,
        'elevator_retry_interval_s': 2.0,
    }

    def get_parameter(name):
        return SimpleNamespace(value=params[name])
    return get_parameter


# ---- 楼层序列推进 ----

def test_next_floor_advances_through_sorted_targets():
    node = SimpleNamespace(_target_floors=[2, 0, 1], _current_floor=0)
    assert fen.FrontierExplorerNode._next_floor(node) == 1


def test_next_floor_returns_none_at_last_floor():
    node = SimpleNamespace(_target_floors=[0, 1, 2], _current_floor=2)
    assert fen.FrontierExplorerNode._next_floor(node) is None


def test_next_floor_returns_none_for_empty_targets():
    node = SimpleNamespace(_target_floors=[], _current_floor=0)
    assert fen.FrontierExplorerNode._next_floor(node) is None


def test_next_floor_returns_none_when_current_not_in_targets():
    node = SimpleNamespace(_target_floors=[0, 1, 2], _current_floor=5)
    assert fen.FrontierExplorerNode._next_floor(node) is None


# ---- 覆盖达标判定 ----

def test_floor_is_covered_false_when_no_target_floors():
    node = _make_node()
    node._target_floors = []
    node._coverage = _FakeCoverage(ratio=1.0)
    node.grid = np.zeros((10, 10), dtype=np.int8)

    assert node._floor_is_covered() is False


def test_floor_is_covered_true_when_coverage_unavailable():
    node = _make_node()
    node._target_floors = [0, 1]
    node._coverage = None
    node.grid = np.zeros((10, 10), dtype=np.int8)

    assert node._floor_is_covered() is True


def test_floor_is_covered_uses_threshold():
    node = _make_node()
    node._target_floors = [0, 1]
    node.grid = np.zeros((10, 10), dtype=np.int8)
    node.get_parameter = lambda name: SimpleNamespace(value=0.90)

    node._coverage = _FakeCoverage(ratio=0.95)
    assert node._floor_is_covered() is True

    node._coverage = _FakeCoverage(ratio=0.80)
    assert node._floor_is_covered() is False


# ---- FLOOR_COMPLETE 转移语义 ----

def test_handle_floor_complete_preserves_from_floor_semantics():
    node = _make_node()
    node._target_floors = [0, 1, 2]
    node._current_floor = 0
    transitions = []
    node._transition = lambda s: transitions.append(s)
    recorded = []
    node.recorder = SimpleNamespace(
        record_floor_change=lambda *a: recorded.append(a))

    node._now[0] = 100.0
    node._handle_floor_complete()
    assert transitions == []  # 2 秒稳定期未过，不推进

    node._now[0] = 103.0
    node._handle_floor_complete()

    assert transitions == ['FLOOR_TRANSITION']
    assert node._current_floor == 1
    assert node._floor_transition_from_floor == 0
    assert node._floor_transition_phase == 'navigating'
    # record_floor_change(ros_sec, from_floor, to_floor, trigger)
    assert recorded[0][1] == 0
    assert recorded[0][2] == 1
    assert recorded[0][1] != recorded[0][2]


def test_handle_floor_complete_returns_when_all_floors_done():
    node = _make_node()
    node._target_floors = [0]
    node._current_floor = 0
    transitions = []
    node._transition = lambda s: transitions.append(s)

    node._now[0] = 100.0
    node._handle_floor_complete()
    node._now[0] = 103.0
    node._handle_floor_complete()

    assert transitions == ['RETURNING']


# ---- FLOOR_TRANSITION 转移语义 ----

def test_handle_floor_transition_aborts_without_elevator_position():
    node = _make_node()
    node._current_floor = 1
    node._floor_transition_from_floor = 0
    node._floor_transition_phase = 'navigating'
    node._elevator_positions = {}  # 未注入旧楼层电梯坐标
    node.get_parameter = _make_get_parameter()
    transitions = []
    node._transition = lambda s: transitions.append(s)

    node._handle_floor_transition()

    assert transitions == ['RETURNING']


def test_handle_floor_transition_calling_backs_off_and_targets_from_floor():
    node = _make_node()
    node._current_floor = 1
    node._floor_transition_from_floor = 0
    node._floor_transition_phase = 'calling'
    node._floor_transition_start_ros = 100.0
    node._elevator_initiated = False
    node._last_elevator_call_ros = None
    node.get_parameter = _make_get_parameter()
    node.recorder = SimpleNamespace(record_elevator_call=lambda *a: None)

    calls = []

    def fake_call_elevator(*a, **k):
        calls.append(a)
        return SimpleNamespace(
            accepted=False, current_floor=-1, state='idle', message='rejected')

    original = fen.call_elevator
    fen.call_elevator = fake_call_elevator
    try:
        node._now[0] = 100.0
        node._handle_floor_transition()
        assert len(calls) == 1
        # 电梯应叫到旧楼层 from_floor，而非已更新的目标楼层。
        assert calls[0][2] == 0

        # 0.1 秒内重试被退避，不重复调用。
        node._now[0] = 100.1
        node._handle_floor_transition()
        assert len(calls) == 1

        # 间隔满足 retry_interval 后再次调用。
        node._now[0] = 102.1
        node._handle_floor_transition()
        assert len(calls) == 2
    finally:
        fen.call_elevator = original


def test_handle_floor_transition_timeout_escape_publishes_floor_index():
    node = _make_node()
    node._current_floor = 1
    node._floor_transition_from_floor = 0
    node._floor_transition_phase = 'calling'
    node._floor_transition_start_ros = 0.0
    node._elevator_initiated = False
    node._last_elevator_call_ros = None
    node.get_parameter = _make_get_parameter()
    node.recorder = SimpleNamespace(record_elevator_call=lambda *a: None)
    published = []
    node.floor_index_pub = SimpleNamespace(
        publish=lambda msg: published.append(msg.data))
    transitions = []
    node._transition = lambda s: transitions.append(s)

    def fake_call_elevator(*a, **k):
        return SimpleNamespace(
            accepted=False, current_floor=-1, state='idle', message='rejected')

    original = fen.call_elevator
    fen.call_elevator = fake_call_elevator
    try:
        # 已远超 60 秒：电梯始终未就绪，走超时逃生，仍须重置地图再探索。
        node._now[0] = 100.0
        node._handle_floor_transition()
    finally:
        fen.call_elevator = original

    assert published == [1]  # 逃生路径补发目标楼层 floor_index
    assert transitions == ['EXPLORING']


def test_frontier_node_declares_multi_floor_parameters():
    source = (
        NAV_SRC / 'hazardwalker_nav' / 'frontier_explorer_node.py'
    ).read_text(encoding='utf-8')

    assert "declare_parameter('target_floors', [])" in source
    assert "declare_parameter('elevator_positions', {})" in source
    assert "declare_parameter('elevator_retry_interval_s', 2.0)" in source
    # from_floor 语义：跨层阶段不得提前用目标楼层覆盖当前楼层。
    assert 'from_floor = self._current_floor' in source
    assert 'self._floor_transition_from_floor = from_floor' in source
    # 覆盖阈值必须真正接入完成判定。
    assert 'if self._floor_is_covered():' in source
