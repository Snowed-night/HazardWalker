"""导航异常检测与恢复模块。

所属组：导航组。
文件作用：
- 提供不依赖 ROS 的导航异常检测纯函数。
- 卡死检测：监控机器人位置历史，判断是否长时间无明显位移。
- 超时返航判断：根据已用时间和比赛时限，决定是否应触发返航。
- 目标重选判断：当目标路径过长或多次不可达时，建议重新选点。

当前函数职责：
- `StuckDetector`：可复用类，维护位置历史并判定卡死状态。
- `should_trigger_timeout_return`：纯函数，判断是否接近比赛时限。
- `should_reselect_goal`：纯函数，判断是否应跳过当前目标重选。

后续扩展方式：
- `StuckDetector` 当前基于欧氏距离，后续可改用路径里程或 Nav2 反馈。
- 超时返航可与决策组 FSM 联动，由决策模块在 RETURNING 状态下调用。
- 目标跳过判定当前基于直线距离+重试次数，后续可接入 Nav2 路径长度。

验证方式：
- 用 `tests/offline/test_stuck_detector.py` 验证卡死判定、
  超时返航逻辑和目标重选条件。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass


@dataclass
class StuckStatus:
    """卡死检测结果。

    Attributes:
        is_stuck: 是否判定为卡死。
        progress_m: 检测窗口内的累积位移（米）。
        window_sec: 实际使用的窗口时长（秒）。
        elapsed_sec: 从首次记录到当前的时间跨度。
        reason: 判定理由，便于调试和日志。
    """

    is_stuck: bool
    progress_m: float
    window_sec: float
    elapsed_sec: float
    reason: str = ""


class StuckDetector:
    """卡死检测器。

    维护机器人位置的时间序列，在滑动窗口内统计累积位移。
    当窗口内位移低于阈值时，判定为卡死。

    用法：
        detector = StuckDetector(window_sec=8.0, min_progress_m=0.15)
        for each control cycle:
            status = detector.update(x, y, timestamp_sec)
            if status.is_stuck:
                # 触发重规划或恢复行为
                detector.reset()

    Attributes:
        window_sec: 检测窗口时长（秒）。
        min_progress_m: 窗口内最小位移阈值（米），低于此值判定卡死。
        _history: 内部位置历史队列。
    """

    def __init__(self, window_sec: float = 8.0, min_progress_m: float = 0.15):
        if window_sec <= 0:
            raise ValueError(f"window_sec 必须 > 0，当前值 {window_sec}")
        if min_progress_m < 0:
            raise ValueError(f"min_progress_m 不能为负，当前值 {min_progress_m}")
        self.window_sec = float(window_sec)
        self.min_progress_m = float(min_progress_m)
        self._history: deque = deque()

    def update(self, x: float, y: float, timestamp_sec: float) -> StuckStatus:
        """输入最新位姿和时间戳，返回卡死判定。

        内部维护一个滑动窗口：丢弃超出 window_sec 的旧记录，
        然后计算窗口内累积位移（相邻点欧氏距离之和）。

        Args:
            x, y: 机器人当前位置（世界坐标系，米）。
            timestamp_sec: 当前时间戳（秒），应与里程计时间戳一致。

        Returns:
            StuckStatus，包含是否卡死、累积位移和判定理由。

        Note:
            首次调用只记录位置不判定卡死（历史不足）。
            窗口内记录少于 2 条时也无法计算位移，不判定卡死。
        """
        self._history.append((timestamp_sec, x, y))
        # 丢弃窗口外的旧记录
        cutoff = timestamp_sec - self.window_sec
        while self._history and self._history[0][0] < cutoff:
            self._history.popleft()

        if len(self._history) < 2:
            elapsed = 0.0 if len(self._history) == 1 else 0.0
            return StuckStatus(
                is_stuck=False,
                progress_m=0.0,
                window_sec=self.window_sec,
                elapsed_sec=elapsed,
                reason="历史记录不足，无法判定",
            )

        # 计算窗口内累积位移（相邻点欧氏距离之和）
        progress = 0.0
        for i in range(1, len(self._history)):
            t0, x0, y0 = self._history[i - 1]
            t1, x1, y1 = self._history[i]
            dx = x1 - x0
            dy = y1 - y0
            progress += (dx * dx + dy * dy) ** 0.5

        elapsed = self._history[-1][0] - self._history[0][0]
        is_stuck = progress < self.min_progress_m

        if is_stuck:
            reason = (
                f"卡死：{elapsed:.1f}秒内位移{progress:.3f}m "
                f"< 阈值{self.min_progress_m:.3f}m"
            )
        else:
            reason = (
                f"正常：{elapsed:.1f}秒内位移{progress:.3f}m "
                f">= 阈值{self.min_progress_m:.3f}m"
            )

        return StuckStatus(
            is_stuck=is_stuck,
            progress_m=progress,
            window_sec=self.window_sec,
            elapsed_sec=elapsed,
            reason=reason,
        )

    def reset(self):
        """清空位置历史，在卡死恢复后调用。"""
        self._history.clear()

    @property
    def history_size(self) -> int:
        """当前窗口内的位置记录数（调试用）。"""
        return len(self._history)


# ---- 超时返航 ----

def should_trigger_timeout_return(
    elapsed_sec: float,
    max_duration_sec: float = 600.0,
    return_margin_sec: float = 60.0,
) -> bool:
    """判断是否应因时间不足而触发返航。

    当 已用时间 + 返航预留时间 >= 最大任务时间 时，返回 True。
    这给机器人留出 return_margin_sec 秒用于实际返航。

    Args:
        elapsed_sec: 从任务开始到当前的耗时（秒）。
        max_duration_sec: 比赛最大任务时间（秒），默认 600 秒（10 分钟）。
        return_margin_sec: 返航预留时间（秒），默认 60 秒。
                           即在比赛结束前至少 60 秒就应开始返航。

    Returns:
        True 表示应立即停止探索并触发返航。

    Example:
        >>> should_trigger_timeout_return(550.0, 600.0, 60.0)
        True
        >>> should_trigger_timeout_return(500.0, 600.0, 60.0)
        False
    """
    if elapsed_sec < 0:
        raise ValueError(f"elapsed_sec 不能为负，当前值 {elapsed_sec}")
    if max_duration_sec <= 0:
        raise ValueError(f"max_duration_sec 必须 > 0，当前值 {max_duration_sec}")
    if return_margin_sec < 0:
        raise ValueError(f"return_margin_sec 不能为负，当前值 {return_margin_sec}")
    return elapsed_sec + return_margin_sec >= max_duration_sec


# ---- 目标重选 ----

def should_reselect_goal(
    distance_to_goal: float,
    max_retry_count: int,
    current_retry: int,
    max_path_distance: float = 50.0,
    max_retries: int = 3,
) -> tuple:
    """判断是否应跳过当前目标重新选点。

    触发条件（满足任一即建议重选）：
    1. 直线距离超过 max_path_distance（路径过长）。
    2. 当前目标的重试次数已超过 max_retries（多次不可达）。

    Args:
        distance_to_goal: 机器人到当前目标的直线距离（米）。
        max_retry_count: 当前目标允许的最大重试次数。
        current_retry: 当前目标已有的重试次数。
        max_path_distance: 路径长度上限（米），超过此值建议重选。
        max_retries: 最大重试次数上限，用作 max_retry_count 的默认值。

    Returns:
        (should_reselect: bool, reason: str)
        should_reselect 为 True 表示建议跳过当前目标。

    Example:
        >>> should_reselect_goal(80.0, 3, 0)
        (True, '目标距离过远（80.0m > 50.0m），建议重选')
        >>> should_reselect_goal(10.0, 3, 3)
        (True, '目标重试次数已耗尽（3/3），建议跳过')
        >>> should_reselect_goal(10.0, 3, 1)
        (False, '')
    """
    if distance_to_goal > max_path_distance:
        return True, (
            f"目标距离过远（{distance_to_goal:.1f}m > {max_path_distance:.1f}m），"
            f"建议重选"
        )
    effective_max_retry = max_retry_count if max_retry_count > 0 else max_retries
    if current_retry >= effective_max_retry:
        return True, (
            f"目标重试次数已耗尽（{current_retry}/{effective_max_retry}），"
            f"建议跳过"
        )
    return False, ""
