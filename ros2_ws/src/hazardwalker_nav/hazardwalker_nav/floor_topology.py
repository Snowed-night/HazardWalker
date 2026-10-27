"""从在线门口观测建立楼层拓扑并生成房间访问顺序。

本模块只使用 SLAM 坐标中的入口轴和门口观测，不假设楼层长度、房间坐标、
左右对称关系或固定房间数量。它是 ROS 无关的纯逻辑，可用合成楼型做测试。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class DoorwayObservation:
    """某一时刻从地图或激光中得到的门口世界坐标观测。"""

    x_m: float
    y_m: float
    confidence: float = 1.0


@dataclass(frozen=True)
class RoomDoorway:
    """聚类后的真实房间入口。"""

    doorway_id: str
    x_m: float
    y_m: float
    progress_m: float
    lateral_m: float
    support_count: int
    confidence: float

    @property
    def side(self) -> str:
        return 'left' if self.lateral_m > 0.0 else 'right'


@dataclass(frozen=True)
class RoomCompletionEvidence:
    """房间任务完成所需的可测量证据。"""

    crossed_door: bool
    maximum_door_depth_m: float
    visibility_coverage_ratio: float
    observation_count: int
    returned_to_door: bool
    robot_upright: bool = True


def _normalised_axis(axis: Sequence[float]) -> Tuple[float, float]:
    axis_x, axis_y = float(axis[0]), float(axis[1])
    norm = math.hypot(axis_x, axis_y)
    if not math.isfinite(norm) or norm <= 1e-9:
        raise ValueError('entry_axis 必须是非零有限向量')
    return axis_x / norm, axis_y / norm


def _coerce_observation(raw) -> Optional[DoorwayObservation]:
    if isinstance(raw, DoorwayObservation):
        observation = raw
    else:
        try:
            confidence = 1.0 if len(raw) < 3 else float(raw[2])
            observation = DoorwayObservation(
                float(raw[0]), float(raw[1]), confidence)
        except (IndexError, TypeError, ValueError):
            return None
    if not all(math.isfinite(value) for value in (
            observation.x_m, observation.y_m, observation.confidence)):
        return None
    if observation.confidence <= 0.0:
        return None
    return observation


def _project_observation(
        observation: DoorwayObservation,
        origin: Sequence[float],
        axis: Sequence[float],
) -> Tuple[float, float]:
    axis_x, axis_y = _normalised_axis(axis)
    normal_x, normal_y = -axis_y, axis_x
    delta_x = observation.x_m - float(origin[0])
    delta_y = observation.y_m - float(origin[1])
    return (
        delta_x * axis_x + delta_y * axis_y,
        delta_x * normal_x + delta_y * normal_y,
    )


def discover_room_doorways(
        observations: Iterable[DoorwayObservation],
        entry_origin: Sequence[float],
        entry_axis: Sequence[float],
        minimum_progress_m: float = 0.0,
        minimum_lateral_m: float = 0.60,
        maximum_lateral_m: float = 6.0,
        longitudinal_cluster_radius_m: float = 1.0,
        lateral_cluster_radius_m: float = 0.8,
        minimum_support_count: int = 2,
) -> Tuple[RoomDoorway, ...]:
    """把重复、带噪声的门口观测聚类为任意数量房间入口。

    左右两侧分别聚类，因此右侧缺门时不会根据左侧坐标镜像补造。聚类 ID
    由沿走廊顺序和侧别生成，只用于单次楼层任务，不承诺跨地图重建持久化。
    """

    axis_x, axis_y = _normalised_axis(entry_axis)
    origin_x, origin_y = float(entry_origin[0]), float(entry_origin[1])
    if not all(math.isfinite(value) for value in (origin_x, origin_y)):
        raise ValueError('entry_origin 必须包含有限坐标')
    minimum_progress = float(minimum_progress_m)
    minimum_lateral = max(0.0, float(minimum_lateral_m))
    maximum_lateral = max(minimum_lateral, float(maximum_lateral_m))
    progress_radius = max(0.01, float(longitudinal_cluster_radius_m))
    lateral_radius = max(0.01, float(lateral_cluster_radius_m))
    minimum_support = max(1, int(minimum_support_count))

    annotated = []
    for raw in observations or ():
        observation = _coerce_observation(raw)
        if observation is None:
            continue
        progress, lateral = _project_observation(
            observation, (origin_x, origin_y), (axis_x, axis_y))
        if progress < minimum_progress:
            continue
        if not minimum_lateral <= abs(lateral) <= maximum_lateral:
            continue
        annotated.append((observation, progress, lateral))

    clusters: List[List[Tuple[DoorwayObservation, float, float]]] = []
    for item in sorted(annotated, key=lambda value: (value[2] < 0.0, value[1])):
        _, progress, lateral = item
        same_side_clusters = [
            cluster for cluster in clusters
            if cluster[0][2] * lateral > 0.0
        ]
        nearest = None
        nearest_distance = math.inf
        for cluster in same_side_clusters:
            total_weight = sum(value[0].confidence for value in cluster)
            mean_progress = sum(
                value[1] * value[0].confidence for value in cluster
            ) / total_weight
            mean_lateral = sum(
                value[2] * value[0].confidence for value in cluster
            ) / total_weight
            progress_gap = abs(progress - mean_progress)
            lateral_gap = abs(lateral - mean_lateral)
            if progress_gap > progress_radius or lateral_gap > lateral_radius:
                continue
            distance = math.hypot(
                progress_gap / progress_radius,
                lateral_gap / lateral_radius,
            )
            if distance < nearest_distance:
                nearest = cluster
                nearest_distance = distance
        if nearest is None:
            clusters.append([item])
        else:
            nearest.append(item)

    normal_x, normal_y = -axis_y, axis_x
    doorways = []
    for cluster in clusters:
        if len(cluster) < minimum_support:
            continue
        total_weight = sum(value[0].confidence for value in cluster)
        progress = sum(
            value[1] * value[0].confidence for value in cluster
        ) / total_weight
        lateral = sum(
            value[2] * value[0].confidence for value in cluster
        ) / total_weight
        confidence = min(1.0, total_weight / max(1.0, minimum_support))
        doorways.append((progress, lateral, len(cluster), confidence))

    doorways.sort(key=lambda item: (item[0], item[1] < 0.0))
    result = []
    for index, (progress, lateral, support, confidence) in enumerate(doorways):
        side = 'left' if lateral > 0.0 else 'right'
        result.append(RoomDoorway(
            doorway_id=f'room_{index:02d}_{side}',
            x_m=origin_x + axis_x * progress + normal_x * lateral,
            y_m=origin_y + axis_y * progress + normal_y * lateral,
            progress_m=progress,
            lateral_m=lateral,
            support_count=support,
            confidence=confidence,
        ))
    return tuple(result)


def build_far_to_near_visit_order(
        doorways: Iterable[RoomDoorway],
        same_station_tolerance_m: float = 1.0,
        initial_lateral_m: float = 0.0,
) -> Tuple[RoomDoorway, ...]:
    """生成“先到走廊尽头、返程逐房”的动态访问顺序。

    纵向由远到近不可逆；同一门排内选择离上一个出口横向位置更近的一侧，
    减少穿越走廊和大幅转向。函数不要求门成对，也不限制房间数量。
    """

    remaining = list(doorways or ())
    if not remaining:
        return tuple()
    tolerance = max(0.01, float(same_station_tolerance_m))
    remaining.sort(key=lambda doorway: doorway.progress_m, reverse=True)
    stations: List[List[RoomDoorway]] = []
    for doorway in remaining:
        if not stations or abs(
                stations[-1][0].progress_m - doorway.progress_m) > tolerance:
            stations.append([doorway])
        else:
            stations[-1].append(doorway)

    ordered = []
    current_lateral = float(initial_lateral_m)
    for station in stations:
        pending = list(station)
        while pending:
            selected = min(
                pending,
                key=lambda doorway: (
                    abs(doorway.lateral_m - current_lateral),
                    abs(doorway.lateral_m),
                    doorway.doorway_id,
                ),
            )
            ordered.append(selected)
            current_lateral = selected.lateral_m
            pending.remove(selected)
    return tuple(ordered)


def room_completion_is_valid(
        evidence: RoomCompletionEvidence,
        minimum_door_depth_m: float,
        minimum_visibility_coverage_ratio: float,
        minimum_observation_count: int,
) -> bool:
    """严格验证房间完成，禁止门口经过或原地徘徊被计数。"""

    try:
        depth = float(evidence.maximum_door_depth_m)
        coverage = float(evidence.visibility_coverage_ratio)
        observation_count = int(evidence.observation_count)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(depth) or not math.isfinite(coverage):
        return False
    return (
        bool(evidence.crossed_door)
        and bool(evidence.returned_to_door)
        and bool(evidence.robot_upright)
        and depth >= max(0.0, float(minimum_door_depth_m))
        and coverage >= min(
            1.0, max(0.0, float(minimum_visibility_coverage_ratio)))
        and observation_count >= max(1, int(minimum_observation_count))
    )
