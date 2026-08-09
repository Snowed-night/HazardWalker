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
    assert 'now_ros, previous_floor, next_floor' in source
    assert 'now_ros, self._current_floor, next_floor' not in source


def test_elevator_service_does_not_block_control_timer():
    source = _frontier_source()

    assert 'ThreadPoolExecutor(' in source
    assert 'self._elevator_executor.submit(' in source
    assert 'if future is None or not future.done()' in source
    transition = source.split('def _handle_floor_transition', 1)[1]
    assert 'result = call_elevator(' not in transition
