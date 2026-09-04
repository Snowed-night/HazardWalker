"""房间观察点与感知新鲜帧之间的严格请求/应答门控。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Optional


@dataclass(frozen=True)
class InspectionCaptureRequest:
    goal_id: str
    obstacle_id: str
    floor: int
    sector: str
    accepted_after_frame_index: int


class InspectionCaptureGate:
    """只有请求后的稳定 RGB-D+TF 帧才能确认观察完成。"""

    def __init__(self):
        self.pending: Optional[InspectionCaptureRequest] = None
        self.completed_goal_ids = set()

    def accept_request(
            self, payload: Mapping, current_frame_index: int,
    ) -> bool:
        try:
            goal_id = str(payload.get('goal_id', '')).strip()
            obstacle_id = str(payload.get('obstacle_id', '')).strip()
            floor = int(payload.get('floor', 0))
            sector = str(payload.get('sector', '')).strip()
            frame_index = int(current_frame_index)
        except (TypeError, ValueError):
            return False
        if not goal_id or goal_id in self.completed_goal_ids:
            return False
        if self.pending is not None and self.pending.goal_id == goal_id:
            # 导航可靠重发不得刷新帧门槛，否则高频请求会让目标永远等不到
            # “请求之后”的新帧。
            return True
        self.pending = InspectionCaptureRequest(
            goal_id=goal_id,
            obstacle_id=obstacle_id,
            floor=floor,
            sector=sector,
            accepted_after_frame_index=frame_index,
        )
        return True

    def observe_frame(
            self,
            *,
            frame_index: int,
            stamp_sec: float,
            camera_stable: bool,
            depth_synchronized: bool,
            tf_synchronized: bool,
            localization_ready: bool,
            hazard_count: int,
            detection_count: int,
    ) -> Optional[dict]:
        request = self.pending
        if request is None:
            return None
        try:
            index = int(frame_index)
            stamp = float(stamp_sec)
            hazards = max(0, int(hazard_count))
            detections = max(0, int(detection_count))
        except (TypeError, ValueError):
            return None
        if not math.isfinite(stamp):
            return None
        if index <= request.accepted_after_frame_index:
            return None
        if not (
                bool(camera_stable)
                and bool(depth_synchronized)
                and bool(tf_synchronized)
                and bool(localization_ready)):
            return None
        result = {
            'goal_id': request.goal_id,
            'obstacle_id': request.obstacle_id,
            'floor': request.floor,
            'sector': request.sector,
            'success': True,
            'stamp_sec': stamp,
            'frame_index': index,
            'hazard_count': hazards,
            'detection_count': detections,
            'localization_ready': True,
        }
        self.completed_goal_ids.add(request.goal_id)
        self.pending = None
        return result
