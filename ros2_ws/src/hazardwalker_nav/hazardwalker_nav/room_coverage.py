"""基于在线占据栅格的房间视线覆盖规划，不包含具体楼宇坐标。

该模块刻意不依赖 ROS：调用方只需提供房间自由区掩码、占据栅格、门口
位置和相机参数。规划器从可通行位置中选择少量观察位姿，以覆盖家具遮挡后
的空间，并在评分中惩罚长距离和大角度转向，便于独立单元测试。
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import FrozenSet, Iterable, List, Optional, Sequence, Tuple

import numpy as np


GridCell = Tuple[int, int]


@dataclass(frozen=True)
class GridFrame:
    """二维占据栅格的世界坐标定义。"""

    resolution_m: float
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0.0:
            raise ValueError('resolution_m 必须为正的有限数')
        if not all(math.isfinite(value) for value in (
                self.origin_x_m, self.origin_y_m)):
            raise ValueError('栅格原点必须为有限数')

    def grid_to_world(self, cell: GridCell) -> Tuple[float, float]:
        x_index, y_index = int(cell[0]), int(cell[1])
        return (
            self.origin_x_m + (x_index + 0.5) * self.resolution_m,
            self.origin_y_m + (y_index + 0.5) * self.resolution_m,
        )

    def world_to_grid(self, point: Sequence[float]) -> GridCell:
        return (
            int(math.floor(
                (float(point[0]) - self.origin_x_m) / self.resolution_m)),
            int(math.floor(
                (float(point[1]) - self.origin_y_m) / self.resolution_m)),
        )


@dataclass(frozen=True)
class ObservationPose:
    """机器人停稳采帧时的世界坐标位姿及该位姿新增覆盖数量。"""

    x_m: float
    y_m: float
    yaw_rad: float
    newly_visible_cells: int


@dataclass(frozen=True)
class RoomCoveragePlan:
    """房间视线覆盖结果。"""

    observation_poses: Tuple[ObservationPose, ...]
    coverage_ratio: float
    covered_cell_count: int
    target_cell_count: int

    @property
    def complete(self) -> bool:
        return self.target_cell_count > 0 and self.coverage_ratio >= 0.999


def _angle_difference(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def coverage_candidate_utility(
        visible_gain_cells: int,
        travel_m: float,
        turn_rad: float,
        travel_cost_weight: float,
        turn_cost_weight: float,
) -> float:
    """计算无量纲覆盖效用，避免“格子数”数量级淹没米/弧度代价。"""

    gain = max(0, int(visible_gain_cells))
    denominator = (
        1.0
        + max(0.0, float(travel_cost_weight))
        * max(0.0, float(travel_m))
        + max(0.0, float(turn_cost_weight))
        * max(0.0, float(turn_rad))
    )
    return float(gain) / denominator


def _bresenham_cells(start: GridCell, end: GridCell) -> List[GridCell]:
    """返回包含起点和终点的整数栅格直线。"""

    x0, y0 = int(start[0]), int(start[1])
    x1, y1 = int(end[0]), int(end[1])
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    step_x = 1 if x0 < x1 else -1
    step_y = 1 if y0 < y1 else -1
    error = dx + dy
    cells = []
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return cells
        twice_error = 2 * error
        if twice_error >= dy:
            error += dy
            x0 += step_x
        if twice_error <= dx:
            error += dx
            y0 += step_y


def _inflate_blocked(blocked: np.ndarray, radius_cells: int) -> np.ndarray:
    """以圆形结构元素膨胀障碍，不依赖 scipy。"""

    radius = max(0, int(radius_cells))
    result = np.asarray(blocked, dtype=bool).copy()
    if radius == 0:
        return result
    height, width = result.shape
    source = np.asarray(blocked, dtype=bool)
    for offset_y in range(-radius, radius + 1):
        for offset_x in range(-radius, radius + 1):
            if offset_x * offset_x + offset_y * offset_y > radius * radius:
                continue
            source_y0 = max(0, -offset_y)
            source_y1 = min(height, height - offset_y)
            source_x0 = max(0, -offset_x)
            source_x1 = min(width, width - offset_x)
            target_y0 = source_y0 + offset_y
            target_y1 = source_y1 + offset_y
            target_x0 = source_x0 + offset_x
            target_x1 = source_x1 + offset_x
            result[target_y0:target_y1, target_x0:target_x1] |= (
                source[source_y0:source_y1, source_x0:source_x1])
    return result


def _sample_mask(mask: np.ndarray, stride_cells: int) -> List[GridCell]:
    stride = max(1, int(stride_cells))
    rows, columns = np.nonzero(np.asarray(mask, dtype=bool))
    if len(rows) == 0:
        return []
    # 采样相位相对于当前区域包围盒，而不是全局栅格原点。否则同一房型仅仅
    # 平移几个格子就会得到不同候选集，破坏陌生环境算法应有的坐标无关性。
    row_origin = int(rows.min())
    column_origin = int(columns.min())
    sampled = [
        (int(column), int(row))
        for row, column in zip(rows, columns)
        if ((int(row) - row_origin) % stride == 0
            and (int(column) - column_origin) % stride == 0)
    ]
    if sampled:
        return sampled
    return [
        (int(column), int(row))
        for row, column in zip(rows, columns)
    ]


def grid_shortest_path_distances(
        traversable_mask: np.ndarray,
        start_cell: GridCell,
        resolution_m: float,
) -> np.ndarray:
    """计算起点到全部可通行格的八邻域最短路径距离（米）。

    对角移动禁止穿越墙角，规则与导航 A* 一致。该距离用于覆盖贪心的
    行走代价，避免把隔着家具但欧氏距离很近的观察点误判为便宜目标。
    """

    traversable = np.asarray(traversable_mask, dtype=bool)
    if traversable.ndim != 2:
        raise ValueError('traversable_mask 必须是二维数组')
    resolution = float(resolution_m)
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError('resolution_m 必须为正的有限数')
    height, width = traversable.shape
    start_x, start_y = int(start_cell[0]), int(start_cell[1])
    distances = np.full((height, width), np.inf, dtype=np.float64)
    if (not (0 <= start_x < width and 0 <= start_y < height)
            or not traversable[start_y, start_x]):
        return distances

    distances[start_y, start_x] = 0.0
    queue = [(0.0, start_x, start_y)]
    diagonal = math.sqrt(2.0)
    neighbors = (
        (-1, 0, 1.0), (1, 0, 1.0),
        (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, diagonal), (1, -1, diagonal),
        (-1, 1, diagonal), (1, 1, diagonal),
    )
    while queue:
        distance, cell_x, cell_y = heapq.heappop(queue)
        if distance > distances[cell_y, cell_x] + 1e-12:
            continue
        for offset_x, offset_y, step_cells in neighbors:
            next_x, next_y = cell_x + offset_x, cell_y + offset_y
            if not (0 <= next_x < width and 0 <= next_y < height):
                continue
            if not traversable[next_y, next_x]:
                continue
            if (offset_x != 0 and offset_y != 0
                    and (not traversable[cell_y, next_x]
                         or not traversable[next_y, cell_x])):
                continue
            candidate = distance + step_cells * resolution
            if candidate + 1e-12 >= distances[next_y, next_x]:
                continue
            distances[next_y, next_x] = candidate
            heapq.heappush(queue, (candidate, next_x, next_y))
    return distances


def visible_room_cells(
        occupancy_grid: np.ndarray,
        room_mask: np.ndarray,
        frame: GridFrame,
        viewpoint: GridCell,
        yaw_rad: float,
        target_cells: Iterable[GridCell],
        camera_fov_rad: float,
        camera_range_m: float,
        occupied_threshold: int = 50,
) -> FrozenSet[GridCell]:
    """计算单个位姿可观察到的房间采样格。

    未知格和房间外区域均阻断视线；目标格必须位于已知自由空间。该约束避免
    把“相机射线理论上能穿过尚未建图区域”误算成已覆盖。
    """

    grid = np.asarray(occupancy_grid)
    room = np.asarray(room_mask, dtype=bool)
    if grid.ndim != 2 or room.shape != grid.shape:
        raise ValueError('occupancy_grid 与 room_mask 必须是同尺寸二维数组')
    fov = float(camera_fov_rad)
    maximum_range = float(camera_range_m)
    if not math.isfinite(fov) or not 0.0 < fov <= 2.0 * math.pi:
        raise ValueError('camera_fov_rad 必须位于 (0, 2π]')
    if not math.isfinite(maximum_range) or maximum_range <= 0.0:
        raise ValueError('camera_range_m 必须为正的有限数')

    height, width = grid.shape
    view_x, view_y = int(viewpoint[0]), int(viewpoint[1])
    if not (0 <= view_x < width and 0 <= view_y < height):
        return frozenset()
    blocked = (grid < 0) | (grid >= int(occupied_threshold)) | (~room)
    if blocked[view_y, view_x]:
        return frozenset()

    visible = set()
    for raw_target in target_cells:
        target = (int(raw_target[0]), int(raw_target[1]))
        target_x, target_y = target
        if not (0 <= target_x < width and 0 <= target_y < height):
            continue
        if blocked[target_y, target_x]:
            continue
        delta_x = (target_x - view_x) * frame.resolution_m
        delta_y = (target_y - view_y) * frame.resolution_m
        if math.hypot(delta_x, delta_y) > maximum_range + 1e-9:
            continue
        bearing = math.atan2(delta_y, delta_x)
        if abs(_angle_difference(bearing, float(yaw_rad))) > 0.5 * fov:
            continue
        ray = _bresenham_cells((view_x, view_y), target)
        if any(blocked[cell_y, cell_x] for cell_x, cell_y in ray[1:-1]):
            continue
        visible.add(target)
    return frozenset(visible)


def plan_room_visibility_coverage(
        occupancy_grid: np.ndarray,
        room_mask: np.ndarray,
        frame: GridFrame,
        entrance_world: Sequence[float],
        camera_fov_rad: float,
        camera_range_m: float,
        robot_clearance_m: float = 0.35,
        target_spacing_m: float = 0.30,
        candidate_spacing_m: float = 0.70,
        heading_sample_count: int = 12,
        maximum_viewpoints: int = 12,
        desired_coverage_ratio: float = 0.95,
        travel_cost_weight: float = 0.20,
        turn_cost_weight: float = 0.35,
        occupied_threshold: int = 50,
) -> RoomCoveragePlan:
    """用贪心可见集覆盖生成短而少转向的房间观察路线。

    所有距离均由栅格和相机物理参数决定。规划器不假设房间数量、房间中心、
    走廊方向或楼层高度，因而可在平移、旋转或尺寸变化后的房间继续使用。
    """

    grid = np.asarray(occupancy_grid)
    room = np.asarray(room_mask, dtype=bool)
    if grid.ndim != 2 or room.shape != grid.shape:
        raise ValueError('occupancy_grid 与 room_mask 必须是同尺寸二维数组')
    desired = min(1.0, max(0.0, float(desired_coverage_ratio)))
    heading_count = max(1, int(heading_sample_count))
    viewpoint_limit = max(0, int(maximum_viewpoints))
    if viewpoint_limit == 0:
        return RoomCoveragePlan(tuple(), 0.0, 0, 0)

    free = (grid >= 0) & (grid < int(occupied_threshold)) & room
    # 房间外也作为墙体参与净空计算，避免路线贴着分割边界运行。
    blocked_for_clearance = (~free)
    clearance_cells = int(math.ceil(
        max(0.0, float(robot_clearance_m)) / frame.resolution_m))
    traversable = free & (~_inflate_blocked(
        blocked_for_clearance, clearance_cells))

    target_stride = max(1, int(round(
        max(frame.resolution_m, float(target_spacing_m))
        / frame.resolution_m)))
    candidate_stride = max(1, int(round(
        max(frame.resolution_m, float(candidate_spacing_m))
        / frame.resolution_m)))
    target_cells = _sample_mask(free, target_stride)
    candidate_cells = _sample_mask(traversable, candidate_stride)
    if not target_cells or not candidate_cells:
        return RoomCoveragePlan(tuple(), 0.0, 0, len(target_cells))

    entrance_x, entrance_y = float(entrance_world[0]), float(entrance_world[1])
    if not math.isfinite(entrance_x) or not math.isfinite(entrance_y):
        raise ValueError('entrance_world 必须包含有限坐标')
    heading_values = tuple(
        -math.pi + index * 2.0 * math.pi / heading_count
        for index in range(heading_count)
    )

    visibility = {}
    for candidate in candidate_cells:
        for heading in heading_values:
            cells = visible_room_cells(
                grid, room, frame, candidate, heading, target_cells,
                camera_fov_rad, camera_range_m,
                occupied_threshold=occupied_threshold,
            )
            if cells:
                visibility[(candidate, heading)] = cells

    uncovered = set(target_cells)
    total = len(uncovered)
    selected: List[ObservationPose] = []
    current_x, current_y = entrance_x, entrance_y
    current_yaw: Optional[float] = None
    used_candidates = set()

    while uncovered and len(selected) < viewpoint_limit:
        current_cell = frame.world_to_grid((current_x, current_y))
        if (not (0 <= current_cell[0] < traversable.shape[1]
                 and 0 <= current_cell[1] < traversable.shape[0])
                or not traversable[current_cell[1], current_cell[0]]):
            current_cell = min(
                candidate_cells,
                key=lambda cell: (
                    (cell[0] - current_cell[0]) ** 2
                    + (cell[1] - current_cell[1]) ** 2),
            )
        path_distances = grid_shortest_path_distances(
            traversable, current_cell, frame.resolution_m)
        snapped_x, snapped_y = frame.grid_to_world(current_cell)
        snap_offset_m = math.hypot(
            snapped_x - current_x, snapped_y - current_y)
        best = None
        for (candidate, heading), visible in visibility.items():
            if candidate in used_candidates:
                continue
            gain_cells = visible & uncovered
            gain = len(gain_cells)
            if gain == 0:
                continue
            world_x, world_y = frame.grid_to_world(candidate)
            travel_m = float(path_distances[candidate[1], candidate[0]])
            if not math.isfinite(travel_m):
                continue
            travel_m += snap_offset_m
            turn_rad = 0.0 if current_yaw is None else abs(
                _angle_difference(heading, current_yaw))
            score = coverage_candidate_utility(
                gain,
                travel_m,
                turn_rad,
                travel_cost_weight,
                turn_cost_weight,
            )
            tie_break = (score, gain, -travel_m, -turn_rad)
            if best is None or tie_break > best[0]:
                best = (
                    tie_break, candidate, heading, gain_cells,
                    world_x, world_y,
                )
        if best is None:
            break
        _, candidate, heading, gain_cells, world_x, world_y = best
        selected.append(ObservationPose(
            x_m=world_x,
            y_m=world_y,
            yaw_rad=float(heading),
            newly_visible_cells=len(gain_cells),
        ))
        uncovered.difference_update(gain_cells)
        used_candidates.add(candidate)
        current_x, current_y, current_yaw = world_x, world_y, float(heading)
        if 1.0 - len(uncovered) / total >= desired:
            break

    covered = total - len(uncovered)
    ratio = 0.0 if total == 0 else covered / total
    return RoomCoveragePlan(
        observation_poses=tuple(selected),
        coverage_ratio=float(ratio),
        covered_cell_count=int(covered),
        target_cell_count=int(total),
    )
