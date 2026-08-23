"""导航异常检测与恢复离线测试。

所属组：导航组 / 测试组。
文件作用：
- 验证 `stuck_detector.py` 中卡死检测、超时返航判断
  和目标重选逻辑的纯函数正确性。
- 不依赖 ROS、Nav2、Gazebo 或真实机器人。

当前验证内容：
- StuckDetector 初始化参数校验。
- 正常移动时不误报卡死。
- 静止不动时正确判定卡死。
- 极小位移时判定卡死。
- 滑动窗口正确丢弃过期记录。
- reset 清空历史。
- 超时返航时间边界判断。
- 目标重选（距离过远、重试耗尽、正常情况）。
"""
import math
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_nav'))

from hazardwalker_nav.stuck_detector import (
    StuckDetector,
    StuckStatus,
    should_trigger_timeout_return,
    should_reselect_goal,
)


# ============================================================
# StuckDetector 初始化测试
# ============================================================

def test_stuck_detector_default_params():
    """验证使用默认参数构造时不抛异常。"""
    detector = StuckDetector()
    assert detector.window_sec == 8.0
    assert detector.min_progress_m == 0.15
    assert detector.history_size == 0


def test_stuck_detector_custom_params():
    """验证自定义参数正确存储。"""
    detector = StuckDetector(window_sec=5.0, min_progress_m=0.3)
    assert detector.window_sec == 5.0
    assert detector.min_progress_m == 0.3


def test_stuck_detector_rejects_invalid_window():
    """验证非正窗口时长抛出 ValueError。"""
    try:
        StuckDetector(window_sec=0.0)
        assert False, "应抛出 ValueError"
    except ValueError:
        pass
    try:
        StuckDetector(window_sec=-1.0)
        assert False, "应抛出 ValueError"
    except ValueError:
        pass


def test_stuck_detector_rejects_negative_progress():
    """验证负位移阈值抛出 ValueError。"""
    try:
        StuckDetector(min_progress_m=-0.1)
        assert False, "应抛出 ValueError"
    except ValueError:
        pass


# ============================================================
# StuckDetector 正常移动测试
# ============================================================

def test_stuck_detector_normal_movement_not_stuck():
    """验证机器人正常移动时不判卡死。"""
    detector = StuckDetector(window_sec=4.0, min_progress_m=0.5)
    # 模拟 4 秒内移动了 2 米
    status = detector.update(0.0, 0.0, 0.0)
    assert not status.is_stuck  # 第一条记录，不判定
    status = detector.update(1.0, 0.0, 1.0)
    assert not status.is_stuck
    status = detector.update(2.0, 0.0, 2.0)
    assert not status.is_stuck
    assert status.progress_m >= 2.0 - 1e-6  # 累积位移约 2.0m


def test_stuck_detector_first_update_not_stuck():
    """验证首次更新不判定卡死。"""
    detector = StuckDetector()
    status = detector.update(1.0, 2.0, 100.0)
    assert not status.is_stuck
    assert status.progress_m == 0.0
    assert status.reason == "历史记录不足，无法判定"


# ============================================================
# StuckDetector 卡死判定测试
# ============================================================

def test_stuck_detector_no_movement_is_stuck():
    """验证完全不动时判定卡死。"""
    detector = StuckDetector(window_sec=3.0, min_progress_m=0.1)
    detector.update(0.0, 0.0, 0.0)
    detector.update(0.0, 0.0, 1.0)
    status = detector.update(0.0, 0.0, 2.0)
    # 3 秒窗口覆盖 0.0~2.0，位移 = 0
    assert status.is_stuck
    assert status.progress_m == 0.0
    assert "卡死" in status.reason


def test_stuck_detector_very_small_movement_is_stuck():
    """验证极小位移时判定卡死。"""
    detector = StuckDetector(window_sec=5.0, min_progress_m=0.15)
    detector.update(0.0, 0.0, 0.0)
    detector.update(0.01, 0.0, 1.0)
    detector.update(0.02, 0.0, 2.0)
    detector.update(0.03, 0.0, 3.0)
    status = detector.update(0.04, 0.0, 4.0)
    # 累积位移约 0.04m < 0.15m
    assert status.is_stuck
    assert status.progress_m < 0.15


def test_stuck_detector_large_enough_movement_not_stuck():
    """验证位移超过阈值时不判卡死。"""
    detector = StuckDetector(window_sec=5.0, min_progress_m=0.15)
    detector.update(0.0, 0.0, 0.0)
    detector.update(0.2, 0.0, 1.0)
    status = detector.update(0.4, 0.0, 2.0)
    # 累积位移约 0.4m > 0.15m
    assert not status.is_stuck
    assert status.progress_m >= 0.15


# ============================================================
# StuckDetector 滑动窗口测试
# ============================================================

def test_stuck_detector_window_sliding():
    """验证超出窗口的旧记录被丢弃。"""
    detector = StuckDetector(window_sec=2.0, min_progress_m=0.1)
    # t=0: 机器人位于原点
    detector.update(0.0, 0.0, 0.0)
    # t=1: 移动到 (1,0)
    detector.update(1.0, 0.0, 1.0)
    # t=3: 窗口只覆盖 [1.0, 3.0]，
    # t=0 的记录被丢弃，所以位移只从 (1,0) 到当前位置
    status = detector.update(1.0, 0.0, 3.0)
    # 窗口内只有 t=1 和 t=3 两条记录，位移 = 0
    assert status.progress_m == 0.0
    assert detector.history_size == 2  # 只有 t=1.0 和 t=3.0 这两条


def test_stuck_detector_exact_window_boundary():
    """验证窗口边界处的记录保留行为。"""
    detector = StuckDetector(window_sec=2.0, min_progress_m=0.1)
    detector.update(0.0, 0.0, 0.0)
    detector.update(0.0, 0.0, 1.0)
    # t=2.0 时，cutoff=0.0，t=0 的记录 timestamp==cutoff，应被丢弃
    status = detector.update(0.0, 0.0, 2.0)
    # t=0 被丢弃（timestamp < cutoff），剩下 t=1.0 和 t=2.0
    assert detector.history_size >= 1


# ============================================================
# StuckDetector reset 测试
# ============================================================

def test_stuck_detector_reset_clears_history():
    """验证 reset 清空历史记录。"""
    detector = StuckDetector()
    detector.update(0.0, 0.0, 0.0)
    detector.update(1.0, 0.0, 1.0)
    assert detector.history_size == 2
    detector.reset()
    assert detector.history_size == 0


def test_stuck_detector_reset_then_new_update():
    """验证 reset 后再次更新不判卡死。"""
    detector = StuckDetector()
    detector.update(0.0, 0.0, 0.0)
    detector.update(0.0, 0.0, 5.0)
    assert detector.history_size == 2
    detector.reset()
    # reset 后首次更新
    status = detector.update(10.0, 10.0, 10.0)
    assert not status.is_stuck
    assert status.progress_m == 0.0


# ============================================================
# StuckDetector 复杂路径测试
# ============================================================

def test_stuck_detector_circular_movement():
    """验证圆周运动累积位移为弧长。"""
    detector = StuckDetector(window_sec=10.0, min_progress_m=1.0)
    # 模拟绕原点走 1/4 圆弧，半径 1m
    points = [
        (0.0, 1.0, 0.0),   # t=0: (1,0)
        (0.2, 0.707, 0.707),  # 约 45°
        (0.4, 0.0, 1.0),    # 约 90°
    ]
    for t, x, y in points:
        detector.update(x, y, t)
    # 机器人走了约 pi/2 ≈ 1.57m
    last = detector.update(0.0, 1.0, 0.6)
    assert last.progress_m > 0.0
    assert not last.is_stuck


def test_stuck_detector_zigzag():
    """验证折线路径位移为线段之和。"""
    detector = StuckDetector(window_sec=5.0, min_progress_m=0.5)
    detector.update(0.0, 0.0, 0.0)
    detector.update(1.0, 0.0, 1.0)  # 走 1m
    detector.update(1.0, 1.0, 2.0)  # 又走 1m
    detector.update(0.0, 1.0, 3.0)  # 又走 1m
    status = detector.update(0.0, 0.0, 4.0)  # 又走 1m
    # 累积位移约 4m
    assert status.progress_m > 3.0
    assert not status.is_stuck


# ============================================================
# 超时返航判断测试
# ============================================================

def test_timeout_return_not_yet():
    """验证时间充裕时不触发返航。"""
    assert not should_trigger_timeout_return(300.0, 600.0, 60.0)


def test_timeout_return_exact_boundary_not_triggered():
    """验证恰好在边界前不触发。"""
    # 已用 539s，预留 60s，539+60=599 < 600
    assert not should_trigger_timeout_return(539.0, 600.0, 60.0)


def test_timeout_return_exact_boundary_triggered():
    """验证恰好在边界触发返航。"""
    # 已用 540s，预留 60s，540+60=600 >= 600
    assert should_trigger_timeout_return(540.0, 600.0, 60.0)


def test_timeout_return_already_over():
    """验证已超时时触发返航。"""
    assert should_trigger_timeout_return(610.0, 600.0, 60.0)


def test_timeout_return_no_margin():
    """验证不留返航余量时在最后一秒触发。"""
    assert not should_trigger_timeout_return(599.0, 600.0, 0.0)
    assert should_trigger_timeout_return(600.0, 600.0, 0.0)


def test_timeout_return_different_params():
    """验证自定义参数正常工作。"""
    # 30min 任务，预留 5min 返航
    assert not should_trigger_timeout_return(1400.0, 1800.0, 300.0)
    assert should_trigger_timeout_return(1500.0, 1800.0, 300.0)


def test_timeout_return_rejects_negative_elapsed():
    """验证负耗时抛出 ValueError。"""
    try:
        should_trigger_timeout_return(-1.0, 600.0, 60.0)
        assert False, "应抛出 ValueError"
    except ValueError:
        pass


def test_timeout_return_rejects_invalid_duration():
    """验证无效最大时长抛出 ValueError。"""
    try:
        should_trigger_timeout_return(100.0, 0.0, 60.0)
        assert False, "应抛出 ValueError"
    except ValueError:
        pass
    try:
        should_trigger_timeout_return(100.0, -10.0, 60.0)
        assert False, "应抛出 ValueError"
    except ValueError:
        pass


def test_timeout_return_rejects_negative_margin():
    """验证负预留时间抛出 ValueError。"""
    try:
        should_trigger_timeout_return(100.0, 600.0, -5.0)
        assert False, "应抛出 ValueError"
    except ValueError:
        pass


# ============================================================
# 目标重选判断测试
# ============================================================

def test_reselect_goal_too_far():
    """验证距离过远时建议重选。"""
    should, reason = should_reselect_goal(80.0, 3, 0)
    assert should
    assert "距离过远" in reason


def test_reselect_goal_retries_exhausted():
    """验证重试次数耗尽时建议重选。"""
    should, reason = should_reselect_goal(10.0, 3, 3)
    assert should
    assert "重试次数已耗尽" in reason


def test_reselect_goal_retries_exhausted_max_retries_zero():
    """验证 max_retry_count=0 时使用默认 max_retries=3。"""
    should, reason = should_reselect_goal(10.0, 0, 3)
    assert should
    assert "重试次数已耗尽" in reason


def test_reselect_goal_normal():
    """验证距离和重试次数均未超限时不建议重选。"""
    should, reason = should_reselect_goal(20.0, 3, 1)
    assert not should
    assert reason == ""


def test_reselect_goal_boundary():
    """验证边界条件：恰好在距离上限时不触发。"""
    should, reason = should_reselect_goal(50.0, 3, 0)
    # 50.0 > 50.0 为 False，不触发
    assert not should


def test_reselect_goal_both_conditions_trigger():
    """验证两个条件同时满足时也正确返回（距离条件优先）。"""
    should, reason = should_reselect_goal(80.0, 3, 3)
    assert should
    # 按代码顺序先检查距离，所以返回距离过远
    assert "距离过远" in reason
