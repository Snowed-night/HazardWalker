"""统一速度控制仲裁的纯逻辑。

文件作用：
- 在键盘、导航和辅助对准三个速度源之间进行显式切换；
- 对每个速度源执行独立超时保护；
- 不依赖 ROS，便于离线验证控制优先级和失联停车行为。

感知模块只产生复查建议，不能直接写入本仲裁器的最终输出。
"""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple


VALID_CONTROL_MODES = ('keyboard', 'navigation', 'assist', 'stopped')


@dataclass(frozen=True)
class VelocityCommand:
    """控制源的一帧二维速度及接收时间。"""

    linear_x: float
    linear_y: float
    angular_z: float
    received_monotonic_sec: float


@dataclass(frozen=True)
class ArbitrationResult:
    """一次仲裁结果；``reason`` 用于 GUI、日志和测试记录。"""

    mode: str
    linear_x: float
    linear_y: float
    angular_z: float
    source_fresh: bool
    reason: str


class ControlArbitrator:
    """按显式模式选择唯一控制源，任何异常都输出零速度。"""

    def __init__(
        self,
        *,
        default_mode: str = 'keyboard',
        source_timeouts_sec: Optional[Dict[str, float]] = None,
    ) -> None:
        self._validate_mode(default_mode)
        self.mode = default_mode
        self.source_timeouts_sec = {
            'keyboard': 0.50,
            'navigation': 0.50,
            'assist': 0.30,
        }
        if source_timeouts_sec:
            unknown_sources = set(source_timeouts_sec) - set(
                self.source_timeouts_sec)
            if unknown_sources:
                raise ValueError(
                    f'未知控制源超时配置：{sorted(unknown_sources)}')
            self.source_timeouts_sec.update(source_timeouts_sec)
        if any(
            not _is_finite_number(value) or float(value) <= 0.0
            for value in self.source_timeouts_sec.values()
        ):
            raise ValueError('控制源超时必须为有限正数')
        self.commands: Dict[str, VelocityCommand] = {}

    @staticmethod
    def _validate_mode(mode: str) -> None:
        if mode not in VALID_CONTROL_MODES:
            raise ValueError(
                f'未知控制模式 {mode!r}；允许值为 {VALID_CONTROL_MODES}'
            )

    def select_mode(self, mode: str) -> None:
        """切换模式并清除目标源旧命令。

        调用层仍应在切换瞬间额外发布一次零速度。清除目标源缓存可保证
        新模式必须在接管后重新发送命令，不能复用切换前恰好仍未超时的速度。
        """

        self._validate_mode(mode)
        if mode != self.mode and mode in self.commands:
            del self.commands[mode]
        self.mode = mode

    def update_source(
        self,
        source: str,
        *,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        received_monotonic_sec: float,
    ) -> None:
        """更新一个控制源；不允许把 ``stopped`` 伪装成输入源。"""

        if source not in self.source_timeouts_sec:
            raise ValueError(f'未配置控制源 {source!r}')
        values = (linear_x, linear_y, angular_z, received_monotonic_sec)
        if not all(_is_finite_number(value) for value in values):
            raise ValueError('速度和时间必须是有限数值')
        self.commands[source] = VelocityCommand(
            linear_x=float(linear_x),
            linear_y=float(linear_y),
            angular_z=float(angular_z),
            received_monotonic_sec=float(received_monotonic_sec),
        )

    def clear_source(self, source: str) -> None:
        """清除异常或退出的控制源，使当前模式下一周期立即归零。"""

        if source not in self.source_timeouts_sec:
            raise ValueError(f'未配置控制源 {source!r}')
        self.commands.pop(source, None)

    def resolve(self, now_monotonic_sec: float) -> ArbitrationResult:
        """返回当前唯一允许输出的速度；断流、倒序时间和停止模式均归零。"""

        if not _is_finite_number(now_monotonic_sec):
            raise ValueError('当前时间必须是有限数值')
        if self.mode == 'stopped':
            return _stopped_result(self.mode, 'explicit_stop')
        command = self.commands.get(self.mode)
        if command is None:
            return _stopped_result(self.mode, 'source_never_received')
        age_sec = float(now_monotonic_sec) - command.received_monotonic_sec
        if age_sec < 0.0:
            return _stopped_result(self.mode, 'non_monotonic_source_time')
        if age_sec > self.source_timeouts_sec[self.mode]:
            return _stopped_result(self.mode, 'source_timeout')
        return ArbitrationResult(
            mode=self.mode,
            linear_x=command.linear_x,
            linear_y=command.linear_y,
            angular_z=command.angular_z,
            source_fresh=True,
            reason='selected_source_fresh',
        )


def _stopped_result(mode: str, reason: str) -> ArbitrationResult:
    return ArbitrationResult(
        mode=mode,
        linear_x=0.0,
        linear_y=0.0,
        angular_z=0.0,
        source_fresh=False,
        reason=reason,
    )


def _is_finite_number(value) -> bool:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return False
    return number == number and number not in (float('inf'), float('-inf'))


def should_publish_status(
    current: str,
    previous: Optional[str],
    *,
    now_monotonic_sec: float,
    previous_publish_monotonic_sec: float,
    heartbeat_sec: float,
) -> bool:
    """判断状态是否需要发布：内容变化或心跳到期均返回真。"""

    values = (
        now_monotonic_sec,
        previous_publish_monotonic_sec,
        heartbeat_sec,
    )
    if not all(_is_finite_number(value) for value in values):
        raise ValueError('状态发布时间必须是有限数值')
    if heartbeat_sec <= 0.0:
        raise ValueError('状态心跳周期必须为正数')
    if now_monotonic_sec < previous_publish_monotonic_sec:
        return True
    return (
        current != previous
        or now_monotonic_sec - previous_publish_monotonic_sec >= heartbeat_sec
    )
