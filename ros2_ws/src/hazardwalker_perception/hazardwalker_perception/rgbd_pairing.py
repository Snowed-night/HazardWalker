"""独立 RGB/Depth 话题的延迟配对策略。

官方 rosbridge 为 RGB 和深度使用不同 WebSocket；两者即使拥有相同仿真时间戳，
到达 ROS2 的先后顺序也不固定。该模块让 RGB 等待下一帧深度，避免永远拿“上一帧
深度”做形状与定位，同时保证缺深度时至多延迟到下一帧 RGB 就降级发布候选。
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RgbdDispatch:
    """一次可交给检测器处理的 RGB，以及当时是否已有同步深度。"""

    payload: object
    depth_synchronized: bool
    depth_stamp_delta_sec: object


class DeferredRgbDepthPairer:
    """缓存最多一帧 RGB，优先等待相同仿真时间的深度后再处理。"""

    def __init__(self, max_delta_sec):
        threshold = float(max_delta_sec)
        if not math.isfinite(threshold) or threshold < 0.0:
            raise ValueError('max_delta_sec must be finite and non-negative.')
        self.max_delta_sec = threshold
        self.latest_depth_stamp_sec = None
        self.pending_rgb = None

    def push_rgb(self, stamp_sec, payload):
        """加入 RGB；旧 RGB 不得无限等待，当前已有配对深度则立即派发。"""

        stamp = _finite_stamp(stamp_sec)
        dispatched = []
        if self.pending_rgb is not None:
            dispatched.append(self._dispatch_pending())
        self.pending_rgb = (stamp, payload)
        if self._pending_matches_latest_depth():
            dispatched.append(self._dispatch_pending())
        return dispatched

    def push_depth(self, stamp_sec):
        """更新深度时间戳；若它匹配等待中的 RGB，则立即派发该 RGB。"""

        self.latest_depth_stamp_sec = _finite_stamp(stamp_sec)
        if self.pending_rgb is None or not self._pending_matches_latest_depth():
            return []
        return [self._dispatch_pending()]

    def _pending_matches_latest_depth(self):
        if self.pending_rgb is None or self.latest_depth_stamp_sec is None:
            return False
        return (
            abs(self.pending_rgb[0] - self.latest_depth_stamp_sec)
            <= self.max_delta_sec
        )

    def _dispatch_pending(self):
        stamp, payload = self.pending_rgb
        delta = (
            abs(stamp - self.latest_depth_stamp_sec)
            if self.latest_depth_stamp_sec is not None else None
        )
        synchronized = (
            delta is not None and delta <= self.max_delta_sec
        )
        self.pending_rgb = None
        return RgbdDispatch(
            payload=payload,
            depth_synchronized=synchronized,
            depth_stamp_delta_sec=delta,
        )


def _finite_stamp(value):
    stamp = float(value)
    if not math.isfinite(stamp) or stamp < 0.0:
        raise ValueError('stamp_sec must be finite and non-negative.')
    return stamp
