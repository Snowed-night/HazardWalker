"""危险源多帧确认和空间去重纯函数。

所属组：感知组。
文件作用：
把每帧三维定位结果合并成稳定危险源列表，降低重复上报和单帧误检。
当前实现边界：
按三维欧氏距离做最近邻合并，达到确认观测次数后标记为 confirmed。
暂不做卡尔曼滤波、跨相机重识别或复杂数据关联，后续可在保持输出字段稳定的前提下升级。
验证方式：
使用 tests/offline/test_track_hazards.py 构造多帧观测，验证合并、确认、missed 计数和新目标创建。
"""

from dataclasses import dataclass, field
import math


@dataclass
class HazardObservation:
    """单帧危险源三维观测。"""

    position: tuple
    confidence: float
    stamp_sec: float = 0.0
    source_id: str = ''
    view_id: str = ''


@dataclass
class HazardTrack:
    """跨帧合并后的危险源轨迹。"""

    track_id: int
    position: tuple
    confidence: float
    status: str = 'tentative'
    observation_count: int = 1
    missed_count: int = 0
    first_seen_sec: float = 0.0
    last_seen_sec: float = 0.0
    source_ids: list = field(default_factory=list)
    view_ids: list = field(default_factory=list)


@dataclass
class HazardTrackerConfig:
    """危险源跟踪参数。"""

    confirm_observation_count: int = 3
    reject_after_missed_count: int = 10
    merge_distance_m: float = 0.5
    min_distinct_views: int = 1


class HazardTracker:
    """维护危险源轨迹状态，逐帧更新观测并输出去重结果。"""

    def __init__(self, config=None):
        self.config = config or HazardTrackerConfig()
        self.tracks = []
        self._next_track_id = 1

    def update(self, observations, stamp_sec=0.0):
        matched_track_ids = set()
        for observation in observations:
            normalized = _normalize_observation(observation, stamp_sec)
            track = self._find_matching_track(normalized, matched_track_ids)
            if track is None:
                track = self._create_track(normalized)
            else:
                _merge_observation_into_track(track, normalized)
            matched_track_ids.add(track.track_id)

        for track in self.tracks:
            if track.track_id not in matched_track_ids:
                track.missed_count += 1

        self._refresh_statuses()
        return self.active_tracks()

    def active_tracks(self):
        return [
            track for track in self.tracks
            if track.status != 'rejected'
        ]

    def confirmed_tracks(self):
        return [
            track for track in self.active_tracks()
            if track.status == 'confirmed'
        ]

    def to_hazard_dicts(self):
        return [track_to_hazard_dict(track) for track in self.active_tracks()]

    def _find_matching_track(self, observation, already_matched):
        best_track = None
        best_distance = None
        for track in self.active_tracks():
            if track.track_id in already_matched:
                continue
            distance = distance_m(track.position, observation.position)
            if distance > self.config.merge_distance_m:
                continue
            if best_distance is None or distance < best_distance:
                best_track = track
                best_distance = distance
        return best_track

    def _create_track(self, observation):
        track = HazardTrack(
            track_id=self._next_track_id,
            position=tuple(float(v) for v in observation.position),
            confidence=float(observation.confidence),
            observation_count=1,
            missed_count=0,
            first_seen_sec=float(observation.stamp_sec),
            last_seen_sec=float(observation.stamp_sec),
            source_ids=[observation.source_id] if observation.source_id else [],
            view_ids=[observation.view_id] if observation.view_id else [],
        )
        self._next_track_id += 1
        self.tracks.append(track)
        return track

    def _refresh_statuses(self):
        for track in self.tracks:
            if track.missed_count >= self.config.reject_after_missed_count:
                track.status = 'rejected'
            elif (track.observation_count >= self.config.confirm_observation_count
                  and _distinct_view_count(track) >= self.config.min_distinct_views):
                track.status = 'confirmed'
            else:
                track.status = 'tentative'


"""计算两个三维位置之间的欧氏距离。"""
def distance_m(a, b):
    if len(a) != 3 or len(b) != 3:
        raise ValueError('Position must contain exactly 3 values.')
    return math.sqrt(
        (float(a[0]) - float(b[0])) ** 2 +
        (float(a[1]) - float(b[1])) ** 2 +
        (float(a[2]) - float(b[2])) ** 2
    )


"""将 HazardTrack 转为后续 JSON/结果文件可直接使用的 dict。"""
def track_to_hazard_dict(track):
    return {
        'id': track.track_id,
        'position': [round(float(v), 4) for v in track.position],
        'position_frame_id': 'start',
        'confidence': round(float(track.confidence), 4),
        'status': track.status,
        'observation_count': track.observation_count,
        'missed_count': track.missed_count,
        'first_seen_sec': track.first_seen_sec,
        'last_seen_sec': track.last_seen_sec,
        'source_ids': list(track.source_ids),
        'distinct_view_count': _distinct_view_count(track),
        'view_ids': list(track.view_ids),
    }


"""把 dict 或 HazardObservation 统一成观测对象。"""
def _normalize_observation(observation, default_stamp_sec):
    if isinstance(observation, HazardObservation):
        stamp = observation.stamp_sec if observation.stamp_sec else default_stamp_sec
        return HazardObservation(
            position=tuple(float(v) for v in observation.position),
            confidence=float(observation.confidence),
            stamp_sec=float(stamp),
            source_id=observation.source_id,
            view_id=observation.view_id,
        )

    position = observation.get('position')
    if position is None:
        raise ValueError('Observation must contain position.')
    stamp = observation.get('stamp_sec', default_stamp_sec)
    return HazardObservation(
        position=tuple(float(v) for v in position),
        confidence=float(observation.get('confidence', 0.0)),
        stamp_sec=float(stamp),
        source_id=str(observation.get('source_id', observation.get('id', ''))),
        view_id=str(observation.get('view_id', '')),
    )


"""用加权平均更新轨迹位置和置信度。"""
def _merge_observation_into_track(track, observation):
    old_count = max(1, int(track.observation_count))
    new_count = old_count + 1
    track.position = tuple(
        (float(track.position[index]) * old_count + float(observation.position[index])) / new_count
        for index in range(3)
    )
    track.confidence = max(float(track.confidence), float(observation.confidence))
    track.observation_count = new_count
    track.missed_count = 0
    track.last_seen_sec = float(observation.stamp_sec)
    if observation.source_id and observation.source_id not in track.source_ids:
        track.source_ids.append(observation.source_id)
    if observation.view_id and observation.view_id not in track.view_ids:
        track.view_ids.append(observation.view_id)


"""没有提供视角标识时按兼容旧链路的单一视角计数。"""
def _distinct_view_count(track):
    return len(track.view_ids) if track.view_ids else 1
