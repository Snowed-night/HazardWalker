"""严格的单房间逐障碍巡检计划与完成证据。

本模块组合导航组的 ``room_obstacle_profiler`` 与现有安全 A*。只有具有可达
路径的观察点才进入计划；运行时也只有“物理到达且采帧成功”才能累计完成。
无路径、超时或主动跳过绝不当作已覆盖，避免防御性分支掩盖漏检。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Iterable, Optional, Sequence, Tuple

import numpy as np

from hazardwalker_nav.frontier_detector import a_star_path
from hazardwalker_nav.room_coverage import (
    GridFrame,
    plan_room_visibility_coverage,
)
from hazardwalker_nav.room_obstacle_profiler import (
    ObstacleCluster,
    ViewPoint,
    extract_room_mask,
    extract_room_obstacles,
    plan_obstacle_viewpoints,
)


@dataclass(frozen=True)
class InspectionGoal:
    """一个必须物理到达并完成采帧的观察目标。"""

    goal_id: str
    obstacle_id: str
    direction_bucket: int
    x_m: float
    y_m: float
    face_yaw_rad: float
    path: Tuple[Tuple[float, float], ...]


@dataclass(frozen=True)
class UncoveredObstacle:
    """规划阶段无法形成足够可达观察方向的障碍。"""

    obstacle_id: str
    required_direction_count: int
    reachable_direction_count: int


def visibility_coverage_requirement_met(
        covered_cell_count: int,
        target_cell_count: int,
        required_ratio: float,
        quantization_tolerance_cells: int = 1,
) -> bool:
    """按整数采样格验收覆盖，允许至多一个栅格的离散量化误差。"""

    target = max(0, int(target_cell_count))
    covered = max(0, int(covered_cell_count))
    if target <= 0:
        return False
    ratio = min(1.0, max(0.0, float(required_ratio)))
    required = int(math.ceil(ratio * target - 1e-12))
    tolerance = max(0, int(quantization_tolerance_cells))
    return covered + tolerance >= required


@dataclass(frozen=True)
class RoomInspectionPlan:
    """单房间严格巡检计划。"""

    goals: Tuple[InspectionGoal, ...]
    obstacle_count: int
    uncovered_obstacles: Tuple[UncoveredObstacle, ...]
    room_mask_cell_count: int
    visibility_coverage_ratio: float = 0.0
    visibility_covered_cell_count: int = 0
    visibility_target_cell_count: int = 0
    required_visibility_coverage_ratio: float = 0.0

    @property
    def executable(self) -> bool:
        coverage_required = self.required_visibility_coverage_ratio > 0.0
        coverage_valid = (
            not coverage_required
            or visibility_coverage_requirement_met(
                self.visibility_covered_cell_count,
                self.visibility_target_cell_count,
                self.required_visibility_coverage_ratio,
            )
        )
        return (
            self.room_mask_cell_count > 0
            and not self.uncovered_obstacles
            and coverage_valid
            and (bool(self.goals) or not coverage_required)
        )


def reproject_planar_pose_between_robot_frames(
        point: Sequence[float],
        heading_rad: float,
        source_robot_pose: Sequence[float],
        target_robot_pose: Sequence[float],
) -> Tuple[float, float, float]:
    """以同一时刻的机器人位姿为锚，在两个二维世界坐标系间重投影位姿。"""

    values = tuple(float(value) for value in (
        point[0], point[1], heading_rad,
        source_robot_pose[0], source_robot_pose[1], source_robot_pose[2],
        target_robot_pose[0], target_robot_pose[1], target_robot_pose[2],
    ))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('重投影输入必须为有限数')
    px, py, heading, sx, sy, source_yaw, tx, ty, target_yaw = values
    dx, dy = px - sx, py - sy
    local_x = math.cos(source_yaw) * dx + math.sin(source_yaw) * dy
    local_y = -math.sin(source_yaw) * dx + math.cos(source_yaw) * dy
    output_x = tx + math.cos(target_yaw) * local_x - math.sin(
        target_yaw) * local_y
    output_y = ty + math.sin(target_yaw) * local_x + math.cos(
        target_yaw) * local_y
    output_yaw = math.atan2(
        math.sin(target_yaw + heading - source_yaw),
        math.cos(target_yaw + heading - source_yaw),
    )
    return float(output_x), float(output_y), float(output_yaw)


def bounded_inspection_turn_rate(
        heading_error_rad: float,
        tolerance_rad: float,
        maximum_speed_rad_s: float,
        minimum_speed_rad_s: float,
) -> float:
    """为已到位的观察点生成可收敛的低速比例转向命令。"""

    error = math.atan2(
        math.sin(float(heading_error_rad)),
        math.cos(float(heading_error_rad)),
    )
    tolerance = max(0.0, float(tolerance_rad))
    maximum = max(0.0, float(maximum_speed_rad_s))
    minimum = min(maximum, max(0.0, float(minimum_speed_rad_s)))
    if not all(math.isfinite(value) for value in (
            error, tolerance, maximum, minimum)):
        raise ValueError('转向参数必须为有限数')
    if abs(error) <= tolerance or maximum <= 0.0:
        return 0.0
    command = max(-maximum, min(maximum, error))
    if 0.0 < abs(command) < minimum:
        command = math.copysign(minimum, command)
    return float(command)


class InspectionProgress:
    """仅记录有成功采帧证据的观察目标。"""

    def __init__(self, plan: RoomInspectionPlan):
        self.plan = plan
        self._required = {goal.goal_id for goal in plan.goals}
        self._captured = set()
        self._failures: Dict[str, str] = {}

    def record_capture(self, goal_id: str, succeeded: bool) -> bool:
        """记录一次观察；失败采帧不会改变完成度。"""

        key = str(goal_id)
        if key not in self._required or not bool(succeeded):
            return False
        self._captured.add(key)
        self._failures.pop(key, None)
        return True

    def record_failure(self, goal_id: str, reason: str) -> None:
        key = str(goal_id)
        if key in self._required and key not in self._captured:
            self._failures[key] = str(reason)

    @property
    def completed_goal_count(self) -> int:
        return len(self._captured)

    @property
    def required_goal_count(self) -> int:
        return len(self._required)

    @property
    def complete(self) -> bool:
        return self.plan.executable and self._captured == self._required

    @property
    def pending_goal_ids(self) -> Tuple[str, ...]:
        return tuple(sorted(self._required - self._captured))


class RoomInspectionExecution:
    """严格的观察目标执行状态机。

    状态只允许 ``move → orient → capture → move`` 单向推进。移动失败、朝向
    失败或采帧失败都会停留在当前目标并留下失败原因，调用方可重新规划，但
    不能把失败伪装成完成。
    """

    MOVE = 'move'
    ORIENT = 'orient'
    CAPTURE = 'capture'
    COMPLETE = 'complete'
    FAILED = 'failed'

    def __init__(self, plan: RoomInspectionPlan):
        self.plan = plan
        self.progress = InspectionProgress(plan)
        self.goal_index = 0
        self.phase = (
            self.COMPLETE
            if plan.executable and not plan.goals
            else self.MOVE
            if plan.executable
            else self.FAILED
        )
        self.failure_reason = (
            '' if plan.executable else 'inspection_plan_not_executable')

    @property
    def current_goal(self) -> Optional[InspectionGoal]:
        if 0 <= self.goal_index < len(self.plan.goals):
            return self.plan.goals[self.goal_index]
        return None

    @property
    def complete(self) -> bool:
        return self.phase == self.COMPLETE and self.progress.complete

    def mark_position_reached(self) -> bool:
        if self.phase != self.MOVE or self.current_goal is None:
            return False
        self.phase = self.ORIENT
        return True

    def mark_orientation_reached(self) -> bool:
        if self.phase != self.ORIENT or self.current_goal is None:
            return False
        self.phase = self.CAPTURE
        return True

    def mark_capture(self, succeeded: bool) -> bool:
        if self.phase != self.CAPTURE or self.current_goal is None:
            return False
        goal = self.current_goal
        if not succeeded:
            self.progress.record_failure(goal.goal_id, 'capture_failed')
            return False
        if not self.progress.record_capture(goal.goal_id, True):
            return False
        self.goal_index += 1
        if self.goal_index >= len(self.plan.goals):
            self.phase = self.COMPLETE
        else:
            self.phase = self.MOVE
        return True

    def mark_motion_failure(self, reason: str) -> None:
        if self.current_goal is not None:
            self.progress.record_failure(self.current_goal.goal_id, reason)
        self.phase = self.FAILED
        self.failure_reason = str(reason)

    def retry_current_goal(self) -> bool:
        if self.phase != self.FAILED or self.current_goal is None:
            return False
        self.phase = self.MOVE
        self.failure_reason = ''
        return True


def _direction_bucket(yaw_rad: float, bucket_count: int) -> int:
    count = max(4, int(bucket_count))
    wrapped = math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))
    return int(round(wrapped / (2.0 * math.pi / count))) % count


def _path_length(path: Iterable[Sequence[float]]) -> float:
    points = [(float(point[0]), float(point[1])) for point in path]
    return sum(
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(points, points[1:])
    )


def _obstacle_id(cluster: ObstacleCluster) -> str:
    return f'obstacle_{cluster.grid_centroid[0]}_{cluster.grid_centroid[1]}'


def _select_reachable_views(
        grid: np.ndarray,
        grid_msg,
        cluster: ObstacleCluster,
        candidates: Iterable[ViewPoint],
        start_world: Sequence[float],
        required_count: int,
        direction_bucket_count: int,
        inflation_radius_m: float,
) -> Tuple[Tuple[InspectionGoal, ...], int]:
    """选择路径可达且方向尽量分散的观察点。"""

    required = max(1, int(required_count))
    current_x, current_y = float(start_world[0]), float(start_world[1])
    remaining = list(candidates)
    selected = []
    selected_buckets = set()
    obstacle_id = _obstacle_id(cluster)

    while remaining and len(selected) < required:
        viable = []
        for viewpoint in remaining:
            bucket = _direction_bucket(
                viewpoint.face_yaw, direction_bucket_count)
            if bucket in selected_buckets:
                continue
            path = a_star_path(
                grid, grid_msg,
                current_x, current_y,
                viewpoint.wx, viewpoint.wy,
                inflation_radius_m=max(0.0, float(inflation_radius_m)),
                start_search_radius_m=0.45,
                goal_search_radius_m=0.30,
            )
            if not path:
                continue
            if selected_buckets:
                separation = min(
                    abs(math.atan2(
                        math.sin(viewpoint.face_yaw - goal.face_yaw_rad),
                        math.cos(viewpoint.face_yaw - goal.face_yaw_rad),
                    ))
                    for goal in selected
                )
            else:
                separation = math.pi
            viable.append((
                -separation,
                _path_length(path),
                bucket,
                viewpoint,
                tuple((float(x), float(y)) for x, y in path),
            ))
        if not viable:
            break
        # 优先增加最大的观察方向分离度，同等分离度选择更短路径。
        viable.sort(key=lambda item: (item[0], item[1], item[2]))
        _, _length, bucket, viewpoint, path = viable[0]
        goal = InspectionGoal(
            goal_id=f'{obstacle_id}_view_{bucket}',
            obstacle_id=obstacle_id,
            direction_bucket=bucket,
            x_m=float(viewpoint.wx),
            y_m=float(viewpoint.wy),
            face_yaw_rad=float(viewpoint.face_yaw),
            path=path,
        )
        selected.append(goal)
        selected_buckets.add(bucket)
        current_x, current_y = goal.x_m, goal.y_m
        remaining = [
            viewpoint for viewpoint in remaining
            if _direction_bucket(
                viewpoint.face_yaw, direction_bucket_count) != bucket
        ]
    return tuple(selected), len(selected_buckets)


def build_strict_room_inspection_plan(
        grid: np.ndarray,
        grid_msg,
        entry_world: Sequence[float],
        entry_yaw_rad: float,
        start_world: Sequence[float],
        door_width_m: float = 1.2,
        seed_offset_m: float = 0.5,
        minimum_room_free_cells: int = 80,
        minimum_obstacle_area_m2: float = 0.05,
        wall_margin_m: float = 0.5,
        viewpoint_count: int = 8,
        required_views_per_obstacle: int = 3,
        viewpoint_standoff_m: float = 0.5,
        viewpoint_clearance_m: float = 0.30,
        path_inflation_radius_m: float = 0.25,
        direction_bucket_count: int = 12,
) -> RoomInspectionPlan:
    """从在线地图构造严格的逐障碍观察计划。

    任何障碍若不足 ``required_views_per_obstacle`` 个不同且可达的观察方向，
    都会进入 ``uncovered_obstacles``，计划不可被标记完成。
    """

    occupancy = np.asarray(grid)
    if occupancy.ndim != 2:
        raise ValueError('grid 必须是二维占据栅格')
    entry_x, entry_y = float(entry_world[0]), float(entry_world[1])
    start_x, start_y = float(start_world[0]), float(start_world[1])
    values = (entry_x, entry_y, start_x, start_y, float(entry_yaw_rad))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('入口、起点和朝向必须是有限数')

    room_mask = extract_room_mask(
        occupancy,
        grid_msg,
        entry_x,
        entry_y,
        float(entry_yaw_rad),
        door_width_m=max(0.2, float(door_width_m)),
        seed_offset_m=max(0.1, float(seed_offset_m)),
        min_room_free_cells=max(1, int(minimum_room_free_cells)),
    )
    if room_mask is None:
        return RoomInspectionPlan(
            goals=tuple(),
            obstacle_count=0,
            uncovered_obstacles=(UncoveredObstacle(
                obstacle_id='room_mask_unavailable',
                required_direction_count=max(
                    1, int(required_views_per_obstacle)),
                reachable_direction_count=0,
            ),),
            room_mask_cell_count=0,
        )

    obstacles = extract_room_obstacles(
        occupancy,
        room_mask,
        grid_msg,
        min_area_m2=max(0.0, float(minimum_obstacle_area_m2)),
        wall_margin_m=max(0.0, float(wall_margin_m)),
    )
    # 先访问离门口/当前位置最近的障碍，减少跨房折返。
    pending = list(obstacles)
    current = (start_x, start_y)
    goals = []
    uncovered = []
    required = max(1, int(required_views_per_obstacle))

    while pending:
        cluster = min(
            pending,
            key=lambda item: math.hypot(
                item.centroid[0] - current[0],
                item.centroid[1] - current[1],
            ),
        )
        pending.remove(cluster)
        candidates = plan_obstacle_viewpoints(
            occupancy,
            grid_msg,
            cluster,
            room_mask,
            count=max(required, int(viewpoint_count)),
            standoff_m=max(0.1, float(viewpoint_standoff_m)),
            clearance_m=max(0.0, float(viewpoint_clearance_m)),
        )
        selected, reachable_count = _select_reachable_views(
            occupancy,
            grid_msg,
            cluster,
            candidates,
            current,
            required,
            direction_bucket_count,
            path_inflation_radius_m,
        )
        if reachable_count < required:
            uncovered.append(UncoveredObstacle(
                obstacle_id=_obstacle_id(cluster),
                required_direction_count=required,
                reachable_direction_count=reachable_count,
            ))
        goals.extend(selected)
        if selected:
            current = (selected[-1].x_m, selected[-1].y_m)

    return RoomInspectionPlan(
        goals=tuple(goals),
        obstacle_count=len(obstacles),
        uncovered_obstacles=tuple(uncovered),
        room_mask_cell_count=int(room_mask.sum()),
    )


def build_room_visibility_inspection_plan(
        grid: np.ndarray,
        grid_msg,
        entry_world: Sequence[float],
        entry_yaw_rad: float,
        start_world: Sequence[float],
        door_width_m: float = 1.8,
        seed_offset_m: float = 0.8,
        minimum_room_free_cells: int = 120,
        camera_fov_rad: float = math.radians(87.0),
        camera_range_m: float = 10.0,
        robot_clearance_m: float = 0.35,
        target_spacing_m: float = 0.5,
        candidate_spacing_m: float = 1.0,
        heading_sample_count: int = 12,
        maximum_viewpoints: int = 16,
        desired_coverage_ratio: float = 0.95,
        path_inflation_radius_m: float = 0.25,
        minimum_obstacle_area_m2: float = 0.15,
        wall_margin_m: float = 0.9,
        neighbor_entries_world: Optional[
            Sequence[Sequence[float]]] = None,
        goal_id_prefix: str = '',
) -> RoomInspectionPlan:
    """按真实入门方向规划整房视线覆盖，并为每个位姿生成可达路径。"""

    occupancy = np.asarray(grid)
    if occupancy.ndim != 2:
        raise ValueError('grid 必须是二维占据栅格')
    entry_x, entry_y = float(entry_world[0]), float(entry_world[1])
    start_x, start_y = float(start_world[0]), float(start_world[1])
    values = (entry_x, entry_y, start_x, start_y, float(entry_yaw_rad))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('入口、起点和朝向必须是有限数')

    room_mask = extract_room_mask(
        occupancy,
        grid_msg,
        entry_x,
        entry_y,
        float(entry_yaw_rad),
        door_width_m=max(0.2, float(door_width_m)),
        seed_offset_m=max(0.1, float(seed_offset_m)),
        min_room_free_cells=max(1, int(minimum_room_free_cells)),
        restrict_to_inside_half_plane=True,
        neighbor_entries_world=[
            (float(entry[0]), float(entry[1]))
            for entry in (neighbor_entries_world or tuple())
        ],
    )
    desired = min(1.0, max(0.01, float(desired_coverage_ratio)))
    if room_mask is None:
        return RoomInspectionPlan(
            goals=tuple(),
            obstacle_count=0,
            uncovered_obstacles=(UncoveredObstacle(
                obstacle_id='room_mask_unavailable',
                required_direction_count=1,
                reachable_direction_count=0,
            ),),
            room_mask_cell_count=0,
            required_visibility_coverage_ratio=desired,
        )

    frame = GridFrame(
        resolution_m=float(grid_msg.info.resolution),
        origin_x_m=float(grid_msg.info.origin.position.x),
        origin_y_m=float(grid_msg.info.origin.position.y),
    )
    coverage = plan_room_visibility_coverage(
        occupancy,
        room_mask,
        frame,
        (start_x, start_y),
        camera_fov_rad=float(camera_fov_rad),
        camera_range_m=float(camera_range_m),
        robot_clearance_m=float(robot_clearance_m),
        target_spacing_m=float(target_spacing_m),
        candidate_spacing_m=float(candidate_spacing_m),
        heading_sample_count=int(heading_sample_count),
        maximum_viewpoints=int(maximum_viewpoints),
        desired_coverage_ratio=desired,
    )

    current_x, current_y = start_x, start_y
    goals = []
    prefix = str(goal_id_prefix).strip()
    prefix = f'{prefix}_' if prefix else ''
    for index, pose in enumerate(coverage.observation_poses):
        path = a_star_path(
            occupancy,
            grid_msg,
            current_x,
            current_y,
            pose.x_m,
            pose.y_m,
            inflation_radius_m=max(0.0, float(path_inflation_radius_m)),
            start_search_radius_m=0.45,
            goal_search_radius_m=0.30,
        )
        if not path:
            return RoomInspectionPlan(
                goals=tuple(goals),
                obstacle_count=0,
                uncovered_obstacles=(UncoveredObstacle(
                    obstacle_id=f'room_visibility_path_{index}',
                    required_direction_count=len(
                        coverage.observation_poses),
                    reachable_direction_count=len(goals),
                ),),
                room_mask_cell_count=int(room_mask.sum()),
                visibility_coverage_ratio=float(coverage.coverage_ratio),
                visibility_covered_cell_count=int(
                    coverage.covered_cell_count),
                visibility_target_cell_count=int(coverage.target_cell_count),
                required_visibility_coverage_ratio=desired,
            )
        bucket = _direction_bucket(pose.yaw_rad, heading_sample_count)
        goals.append(InspectionGoal(
            goal_id=f'{prefix}room_visibility_{index}_view_{bucket}',
            obstacle_id='room_visibility',
            direction_bucket=bucket,
            x_m=float(pose.x_m),
            y_m=float(pose.y_m),
            face_yaw_rad=float(pose.yaw_rad),
            path=tuple((float(x), float(y)) for x, y in path),
        ))
        current_x, current_y = float(pose.x_m), float(pose.y_m)

    uncovered = tuple()
    if not visibility_coverage_requirement_met(
            coverage.covered_cell_count,
            coverage.target_cell_count,
            desired):
        required_cells = int(math.ceil(
            desired * coverage.target_cell_count - 1e-12))
        uncovered = (UncoveredObstacle(
            obstacle_id='room_visibility_coverage',
            required_direction_count=required_cells,
            reachable_direction_count=int(coverage.covered_cell_count),
        ),)
    obstacles = extract_room_obstacles(
        occupancy,
        room_mask,
        grid_msg,
        min_area_m2=max(0.0, float(minimum_obstacle_area_m2)),
        wall_margin_m=max(0.0, float(wall_margin_m)),
    )
    return RoomInspectionPlan(
        goals=tuple(goals),
        obstacle_count=len(obstacles),
        uncovered_obstacles=uncovered,
        room_mask_cell_count=int(room_mask.sum()),
        visibility_coverage_ratio=float(coverage.coverage_ratio),
        visibility_covered_cell_count=int(coverage.covered_cell_count),
        visibility_target_cell_count=int(coverage.target_cell_count),
        required_visibility_coverage_ratio=desired,
    )
