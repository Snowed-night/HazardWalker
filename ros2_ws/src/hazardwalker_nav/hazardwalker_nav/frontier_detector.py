"""自主探索：前沿检测与路径规划纯函数。

所属组：导航组。
文件作用：
- 提供不依赖 ROS 的前沿检测、聚类、评分和 A* 路径规划。
- 被 frontier_explorer_node.py 调用。

前沿定义：OccupancyGrid 中概率值 0..49 且四邻域至少有一个 UNKNOWN (-1)。
Cartographer 用 50 作为未知自由/占据概率分界，边缘自由格常落在 26..49；
沿用静态 map_saver 的 25 阈值会把绝大多数实时可通行区误删。
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
import heapq
from typing import Iterable, List, Optional, Tuple

import numpy as np


def body_tilt_degrees_from_quaternion(
        x: float, y: float, z: float, w: float) -> Optional[float]:
    """由公开 IMU 四元数计算机体 z 轴相对世界 z 轴的倾角。"""

    values = tuple(float(value) for value in (x, y, z, w))
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        return None
    qx, qy, _qz, _qw = (value / norm for value in values)
    upright_cosine = 1.0 - 2.0 * (qx * qx + qy * qy)
    upright_cosine = max(-1.0, min(1.0, upright_cosine))
    return math.degrees(math.acos(upright_cosine))


def append_loop_erased_history(
        history, point, spacing_m: float = 0.10,
        loop_radius_m: float = 0.75, min_index_gap: int = 10) -> bool:
    """向轨迹栈追加点；回到旧轨迹邻域时删除中间闭环。"""

    x, y = float(point[0]), float(point[1])
    spacing = max(0.02, float(spacing_m))
    if history and math.hypot(
            x - float(history[-1][0]), y - float(history[-1][1])) < spacing:
        return False
    radius = max(spacing, float(loop_radius_m))
    gap = max(2, int(min_index_gap))
    search_end = len(history) - gap
    for index in range(search_end - 1, -1, -1):
        if math.hypot(
                x - float(history[index][0]),
                y - float(history[index][1])) <= radius:
            while len(history) > index + 1:
                history.pop()
            break
    history.append((x, y))
    return True


def detect_opened_door_from_scans(
        closed_ranges, opened_ranges, angle_min: float,
        angle_increment: float, min_delta_m: float = 0.25,
        min_changed_bins: int = 4, min_closed_range_m: float = 2.0,
        max_closed_range_m: float = 8.0):
    """由门关闭/打开两帧激光差分返回门洞方位、关闭距离和有效束数。"""

    count = min(len(closed_ranges), len(opened_ranges))
    changed = []
    for index in range(count):
        closed = float(closed_ranges[index])
        opened = float(opened_ranges[index])
        if (not math.isfinite(closed)
                or closed < max(0.05, float(min_closed_range_m))
                or closed > max(0.5, float(max_closed_range_m))):
            continue
        if not math.isfinite(opened):
            opened = 40.0
        delta = opened - closed
        if delta >= max(0.05, float(min_delta_m)):
            changed.append((index, closed, delta))
    if not changed:
        return None

    # 只使用最大连续变化簇，排除远处动态物体或单束噪声。
    groups = []
    current = [changed[0]]
    for item in changed[1:]:
        if item[0] <= current[-1][0] + 2:
            current.append(item)
        else:
            groups.append(current)
            current = [item]
    groups.append(current)
    group = max(groups, key=len)
    if len(group) < max(1, int(min_changed_bins)):
        return None

    weighted_sin = 0.0
    weighted_cos = 0.0
    closed_values = []
    for index, closed, delta in group:
        angle = float(angle_min) + index * float(angle_increment)
        weight = max(0.05, float(delta))
        weighted_sin += math.sin(angle) * weight
        weighted_cos += math.cos(angle) * weight
        closed_values.append(float(closed))
    bearing = math.atan2(weighted_sin, weighted_cos)
    closed_values.sort()
    middle = len(closed_values) // 2
    if len(closed_values) % 2:
        distance = closed_values[middle]
    else:
        distance = 0.5 * (
            closed_values[middle - 1] + closed_values[middle])
    return float(bearing), float(distance), len(group)


def build_reverse_history_path(
        visited_positions, current_x, current_y, home_x, home_y,
        spacing_m=0.45):
    """把本层已走轨迹倒序压缩为返航路径，作为占用图断开时的安全保底。"""

    spacing = max(0.05, float(spacing_m))
    path = []
    last_x = float(current_x)
    last_y = float(current_y)
    for position in reversed(list(visited_positions or [])):
        try:
            x_value = float(position[0])
            y_value = float(position[1])
        except (IndexError, TypeError, ValueError):
            continue
        if not math.isfinite(x_value) or not math.isfinite(y_value):
            continue
        if math.hypot(x_value - last_x, y_value - last_y) < spacing:
            continue
        path.append((x_value, y_value))
        last_x, last_y = x_value, y_value
    if not path or math.hypot(
            float(home_x) - path[-1][0],
            float(home_y) - path[-1][1]) >= 0.05:
        path.append((float(home_x), float(home_y)))
    return path

# OccupancyGrid 常量
FREE = 0
FREE_MAX = 49
OCCUPIED = 100
UNKNOWN = -1


@dataclass
class Frontier:
    """一个前沿聚类。"""
    centroid: Tuple[float, float]   # 世界坐标 (cx, cy)
    size: int                       # 格子数量
    points: List[Tuple[int, int]]   # 网格坐标 [(gx, gy), ...]
    info_gain: float                # 信息增益 = size * avg_unknown_neighbors


def simulation_period_elapsed(
        now_sec: float,
        last_sec: Optional[float],
        period_sec: float,
) -> bool:
    """判断仿真时间周期是否到期，并在时钟回退后立即重建状态。

    控制心跳必须使用墙钟，但地图重规划和覆盖统计应跟随 `/clock`。复杂 Gazebo
    实时倍率约 0.1 时，使用墙钟会在同一仿真秒内重复规划近十次，反过来继续
    压低实时倍率。非法或尚未开始的仿真时间返回 False。
    """

    try:
        now = float(now_sec)
        period = float(period_sec)
        previous = None if last_sec is None else float(last_sec)
    except (TypeError, ValueError, OverflowError):
        return False
    if (not math.isfinite(now) or not math.isfinite(period)
            or now <= 0.0 or period <= 0.0):
        return False
    if previous is None or not math.isfinite(previous) or now < previous:
        return True
    return now - previous >= period


def transform_planar_point(
        point_x: float,
        point_y: float,
        translation_x: float,
        translation_y: float,
        yaw_rad: float,
) -> Tuple[float, float]:
    """将二维点应用刚体变换；用于合法 odom 家点重投影到当前 map。"""

    values = (
        point_x, point_y, translation_x, translation_y, yaw_rad)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('二维刚体变换参数必须是有限数')
    cosine = math.cos(float(yaw_rad))
    sine = math.sin(float(yaw_rad))
    return (
        float(translation_x) + cosine * float(point_x)
        - sine * float(point_y),
        float(translation_y) + sine * float(point_x)
        + cosine * float(point_y),
    )


def transform_planar_goal_to_robot_frame(
        goal_x: float, goal_y: float, goal_yaw: float,
        robot_x: float, robot_y: float,
        robot_yaw: float) -> Tuple[float, float, float]:
    """把同一合法地图中的绝对目标转为机器人 base 相对目标。"""

    values = (
        goal_x, goal_y, goal_yaw, robot_x, robot_y, robot_yaw)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('二维相对目标参数必须是有限数')
    dx = float(goal_x) - float(robot_x)
    dy = float(goal_y) - float(robot_y)
    cosine = math.cos(float(robot_yaw))
    sine = math.sin(float(robot_yaw))
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        math.atan2(
            math.sin(float(goal_yaw) - float(robot_yaw)),
            math.cos(float(goal_yaw) - float(robot_yaw)),
        ),
    )


def interpolate_path_lookahead(
        path, start_index: int, current_x: float, current_y: float,
        lookahead_m: float) -> Optional[Tuple[float, float, float]]:
    """从机器人在折线上的最近投影向前取前视点，绝不追逐身后旧点。"""

    raw_points = list(path or [])[max(0, int(start_index)):]
    points = []
    for point in raw_points:
        try:
            point_x, point_y = float(point[0]), float(point[1])
        except (IndexError, TypeError, ValueError):
            continue
        if math.isfinite(point_x) and math.isfinite(point_y):
            points.append((point_x, point_y))
    if not points:
        return None
    current_x, current_y = float(current_x), float(current_y)
    remaining = max(0.01, float(lookahead_m))
    if not all(math.isfinite(value) for value in (
            current_x, current_y, remaining)):
        return None
    if len(points) == 1:
        dx = points[0][0] - current_x
        dy = points[0][1] - current_y
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return points[0][0], points[0][1], 0.0
        ratio = min(1.0, remaining / length)
        yaw = math.atan2(dy, dx)
        return current_x + ratio * dx, current_y + ratio * dy, yaw

    best = None
    for segment_index in range(len(points) - 1):
        start_x, start_y = points[segment_index]
        end_x, end_y = points[segment_index + 1]
        dx, dy = end_x - start_x, end_y - start_y
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            continue
        projection = max(0.0, min(1.0, (
            (current_x - start_x) * dx
            + (current_y - start_y) * dy
        ) / length_sq))
        projected_x = start_x + projection * dx
        projected_y = start_y + projection * dy
        distance_sq = (
            (projected_x - current_x) ** 2
            + (projected_y - current_y) ** 2
        )
        candidate = (
            distance_sq, -segment_index, -projection,
            segment_index, projected_x, projected_y,
        )
        if best is None or candidate < best:
            best = candidate
    if best is None:
        return interpolate_path_lookahead(
            [points[-1]], 0, current_x, current_y, remaining)

    _, _, _, segment_index, projected_x, projected_y = best
    connector_x = projected_x - current_x
    connector_y = projected_y - current_y
    connector_length = math.hypot(connector_x, connector_y)
    if connector_length > 1e-9:
        connector_yaw = math.atan2(connector_y, connector_x)
        if remaining <= connector_length:
            ratio = remaining / connector_length
            return (
                current_x + ratio * connector_x,
                current_y + ratio * connector_y,
                connector_yaw,
            )
        remaining -= connector_length

    previous_x, previous_y = projected_x, projected_y
    last_yaw = 0.0
    for point_index in range(segment_index + 1, len(points)):
        point_x, point_y = points[point_index]
        dx, dy = point_x - previous_x, point_y - previous_y
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            previous_x, previous_y = point_x, point_y
            continue
        last_yaw = math.atan2(dy, dx)
        if remaining <= length:
            ratio = remaining / length
            return (
                previous_x + ratio * dx,
                previous_y + ratio * dy,
                last_yaw,
            )
        remaining -= length
        previous_x, previous_y = point_x, point_y
    return previous_x, previous_y, last_yaw


def occupancy_grid_to_array(grid_msg) -> np.ndarray:
    """将 nav_msgs/OccupancyGrid 转为 (height, width) numpy 数组。

    未知格子映射为 UNKNOWN(-1)，方便前沿检测时使用四邻域查找。
    """
    data = np.array(grid_msg.data, dtype=np.int8).reshape(grid_msg.info.height, grid_msg.info.width)
    return data


def grid_to_world(gx: int, gy: int, grid_msg) -> Tuple[float, float]:
    """网格坐标 → 世界坐标 (地图原点 + 分辨率偏移)。"""
    origin = grid_msg.info.origin
    wx = origin.position.x + (gx + 0.5) * grid_msg.info.resolution
    wy = origin.position.y + (gy + 0.5) * grid_msg.info.resolution
    return wx, wy


def world_to_grid(wx: float, wy: float, grid_msg) -> Tuple[int, int]:
    """世界坐标 → 网格坐标。"""
    origin = grid_msg.info.origin
    res = grid_msg.info.resolution
    gx = int((wx - origin.position.x) / res)
    gy = int((wy - origin.position.y) / res)
    return gx, gy


def find_frontiers(grid: np.ndarray, free_max: int = FREE_MAX) -> np.ndarray:
    """查找所有前沿格子：0..free_max 且四邻域中有 UNKNOWN (-1)。

    Returns:
        (height, width) 布尔数组，True 表示该格子是前沿。
    """
    if grid.ndim != 2:
        raise ValueError('OccupancyGrid 必须是二维数组')
    h, w = grid.shape
    frontiers = np.zeros((h, w), dtype=bool)
    if h < 3 or w < 3:
        return frontiers
    free = (grid >= FREE) & (grid <= int(free_max))
    unknown = grid == UNKNOWN
    adjacent_unknown = np.zeros_like(frontiers)
    adjacent_unknown[1:-1, 1:-1] = (
        unknown[:-2, 1:-1]
        | unknown[2:, 1:-1]
        | unknown[1:-1, :-2]
        | unknown[1:-1, 2:]
    )
    frontiers = free & adjacent_unknown
    # 与旧实现保持一致：地图边界不参与前沿，避免开放地图边缘吸走机器人。
    frontiers[[0, -1], :] = False
    frontiers[:, [0, -1]] = False
    return frontiers


def _bfs_cluster(start_y: int, start_x: int, mask: np.ndarray, visited: np.ndarray,
                 grid: np.ndarray) -> List[Tuple[int, int]]:
    """BFS 聚类：从 (start_y, start_x) 出发收集连通前沿格子。"""
    h, w = mask.shape
    cluster = []
    q = deque()
    q.append((start_y, start_x))
    visited[start_y, start_x] = True
    while q:
        y, x = q.popleft()
        cluster.append((x, y))  # 存为 (gx, gy)
        for ny, nx in [(y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)]:
            if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not visited[ny, nx]:
                visited[ny, nx] = True
                q.append((ny, nx))
    return cluster


def cluster_frontiers(frontier_mask: np.ndarray, grid: np.ndarray,
                      grid_msg) -> List[Frontier]:
    """BFS 聚类前沿格子并计算每个聚类的属性。

    Args:
        frontier_mask: find_frontiers 的输出。
        grid: 原始 occupancy 数组。
        grid_msg: OccupancyGrid 消息（用于坐标转换）。

    Returns:
        Frontier 对象列表，按 size 降序排列。
    """
    h, w = frontier_mask.shape
    visited = np.zeros((h, w), dtype=bool)
    clusters = []

    for y in range(h):
        for x in range(w):
            if frontier_mask[y, x] and not visited[y, x]:
                points = _bfs_cluster(y, x, frontier_mask, visited, grid)
                if len(points) < 3:  # 忽略太小的聚类
                    continue
                # 算术质心可能落进凹形聚类的未知区/障碍区；导航目标必须选取
                # 距质心最近的真实前沿自由格，避免把可见前沿误判为不可达。
                gx_mean = sum(p[0] for p in points) / len(points)
                gy_mean = sum(p[1] for p in points) / len(points)
                representative = min(
                    points,
                    key=lambda point: (
                        (point[0] - gx_mean) ** 2
                        + (point[1] - gy_mean) ** 2
                    ),
                )
                wx, wy = grid_to_world(
                    representative[0], representative[1], grid_msg)

                # 信息增益：每个前沿格子的未知邻居数之和
                info = 0
                for px, py in points:
                    for ny, nx in [(py - 1, px), (py + 1, px), (py, px - 1), (py, px + 1)]:
                        if 0 <= ny < h and 0 <= nx < w and grid[ny, nx] == UNKNOWN:
                            info += 1

                clusters.append(Frontier(
                    centroid=(wx, wy),
                    size=len(points),
                    points=points,
                    info_gain=info,
                ))

    clusters.sort(key=lambda f: f.size, reverse=True)
    return clusters


def select_best_frontier(frontiers: List[Frontier], robot_wx: float, robot_wy: float,
                         last_target: Optional[Tuple[float, float]] = None,
                         min_frontier_size: int = 10,
                         locality_slack_m: Optional[float] = None,
                         robot_yaw: Optional[float] = None,
                         robot_yaw_half_angle_rad: float = math.pi / 2.0,
                         require_robot_yaw_candidate: bool = False,
                         entry_origin: Optional[Tuple[float, float]] = None,
                         entry_axis: Optional[Tuple[float, float]] = None,
                         entry_backtrack_margin_m: float = 0.5,
                         entry_lateral_limit_m: Optional[float] = None,
                         entry_progress_priority_slack_m: Optional[float] = None,
                         visited_positions: Optional[
                             Iterable[Tuple[float, float]]] = None,
                         revisit_penalty_radius_m: float = 1.2,
                         revisit_penalty_strength: float = 0.0,
                         revisit_free_samples: int = 4,
                         revisit_full_penalty_samples: int = 12,
                         ) -> Optional[Frontier]:
    """选择最优前沿：综合距离、信息增益、大小。

    策略：优先选择近距离、高信息增益的前沿。
    如果有上一个目标且它仍然是前沿，优先继续前往。
    """
    if not frontiers:
        return None

    # 先应用合法空间门禁，再在近场候选带内应用尺寸阈值。若先在整张地图上
    # 删除小簇，近处门洞会被远处长射线形成的巨大前沿挤掉。
    valid = list(frontiers)
    if entry_origin is not None and entry_axis is not None:
        # 官方起点在楼外，首个安全前沿给出了“进入建筑”的数据驱动方向。
        # 后续持续排除起点背面的楼外开放区，但不限制入口前方的左右房间。
        # 该门禁只使用 SLAM 前沿和公开起点，不读取楼宇布局或仿真真值。
        axis_x = float(entry_axis[0])
        axis_y = float(entry_axis[1])
        axis_norm = math.hypot(axis_x, axis_y)
        if axis_norm > 1e-6:
            axis_x /= axis_norm
            axis_y /= axis_norm
            margin = max(0.0, float(entry_backtrack_margin_m))
            lateral_limit = (
                None if entry_lateral_limit_m is None
                else max(0.0, float(entry_lateral_limit_m))
            )
            valid = [
                frontier for frontier in valid
                if (
                    (frontier.centroid[0] - float(entry_origin[0])) * axis_x
                    + (frontier.centroid[1] - float(entry_origin[1])) * axis_y
                ) >= -margin
                and (
                    lateral_limit is None
                    or lateral_limit <= 0.0
                    or abs(
                        -(frontier.centroid[0] - float(entry_origin[0]))
                        * axis_y
                        + (frontier.centroid[1] - float(entry_origin[1]))
                        * axis_x
                    ) <= lateral_limit
                )
            ]
            if not valid:
                return None
            # 走廊骨架阶段不再被近处门口的大前沿拖住：只在当前已知地图中
            # 保留纵深接近最远值的候选。该排序只使用入口轴与合法 SLAM
            # 前沿，不读取房间坐标或场景真值；骨架结束后调用方传 None，
            # 自动恢复常规的近场房间探索。
            if entry_progress_priority_slack_m is not None:
                progress_slack = max(
                    0.0, float(entry_progress_priority_slack_m))
                projected = [
                    (
                        (frontier.centroid[0] - float(entry_origin[0]))
                        * axis_x
                        + (frontier.centroid[1] - float(entry_origin[1]))
                        * axis_y,
                        frontier,
                    )
                    for frontier in valid
                ]
                maximum_progress = max(item[0] for item in projected)
                valid = [
                    frontier for progress, frontier in projected
                    if progress >= maximum_progress - progress_slack
                ]
    if robot_yaw is not None:
        # 官方起点位于入口外且朝向建筑内部。若不考虑当前视线，外部无障碍区的
        # 巨大前沿会压倒入口/走廊前沿，机器人随即绕楼外圈。只要前方半平面有
        # 候选，就先保持向前覆盖；前方耗尽后仍允许选择身后区域。
        forward = [
            frontier for frontier in valid
            if abs(_normalized_angle(
                math.atan2(
                    frontier.centroid[1] - robot_wy,
                    frontier.centroid[0] - robot_wx,
                ) - float(robot_yaw)
            )) <= max(
                0.0,
                min(math.pi, float(robot_yaw_half_angle_rad)),
            )
        ]
        if forward:
            valid = forward
        elif require_robot_yaw_candidate:
            return None

    if locality_slack_m is not None and valid:
        # 词典序“近场优先”：只让距离最近前沿一定余量内的候选参与信息增益
        # 竞争。它不读取房间布局或仿真真值，只使用当前合法 SLAM 位姿。
        slack = max(0.0, float(locality_slack_m))
        nearest_distance = min(
            math.hypot(
                frontier.centroid[0] - robot_wx,
                frontier.centroid[1] - robot_wy,
            )
            for frontier in valid
        )
        valid = [
            frontier for frontier in valid
            if math.hypot(
                frontier.centroid[0] - robot_wx,
                frontier.centroid[1] - robot_wy,
            ) <= nearest_distance + slack
        ]

    large_enough = [
        frontier for frontier in valid
        if frontier.size >= min_frontier_size
    ]
    if large_enough:
        valid = large_enough
    if not valid:
        return None

    best = None
    best_score = -float('inf')
    visited = list(visited_positions or ())
    revisit_radius = max(0.0, float(revisit_penalty_radius_m))
    revisit_strength = max(
        0.0, min(0.95, float(revisit_penalty_strength)))
    free_samples = max(0, int(revisit_free_samples))
    full_samples = max(free_samples + 1, int(revisit_full_penalty_samples))

    for f in valid:
        dist = math.hypot(f.centroid[0] - robot_wx, f.centroid[1] - robot_wy)
        dist = max(dist, 0.5)  # 避免除以零

        # 综合评分：信息增益 / 距离 为主，叠加大小因子
        score = f.info_gain / dist + math.log(f.size + 1) * 0.5

        # 轨迹密度惩罚只使用合法 SLAM 位姿历史。首次经过允许少量样本，只有
        # 同一区域长时间停留或多次折返才逐步降权，避免把正常近场前沿全部压低。
        if visited and revisit_radius > 0.0 and revisit_strength > 0.0:
            nearby = sum(
                1 for x, y in visited
                if math.hypot(f.centroid[0] - x, f.centroid[1] - y)
                <= revisit_radius
            )
            excess = max(0, nearby - free_samples)
            density = min(
                1.0,
                excess / float(full_samples - free_samples),
            )
            score *= max(0.05, 1.0 - revisit_strength * density)

        # 上一个目标加成（减小频繁切换）
        if last_target is not None:
            lt_dist = math.hypot(f.centroid[0] - last_target[0],
                                 f.centroid[1] - last_target[1])
            if lt_dist < 1.0:
                score *= 1.5

        if score > best_score:
            best_score = score
            best = f

    return best


def entry_axis_progress_m(robot_wx: float, robot_wy: float,
                          entry_origin: Optional[Tuple[float, float]],
                          entry_axis: Optional[Tuple[float, float]]
                          ) -> Optional[float]:
    """计算机器人沿公开入口轴的有符号纵深，不读取楼宇布局或真值。"""

    if entry_origin is None or entry_axis is None:
        return None
    values = (
        robot_wx, robot_wy,
        entry_origin[0], entry_origin[1],
        entry_axis[0], entry_axis[1],
    )
    if not all(math.isfinite(float(value)) for value in values):
        return None
    axis_x = float(entry_axis[0])
    axis_y = float(entry_axis[1])
    axis_norm = math.hypot(axis_x, axis_y)
    if axis_norm <= 1e-6:
        return None
    return (
        (float(robot_wx) - float(entry_origin[0])) * axis_x
        + (float(robot_wy) - float(entry_origin[1])) * axis_y
    ) / axis_norm


def corridor_room_sector(
        world_x: float, world_y: float,
        entry_origin: Optional[Tuple[float, float]],
        entry_axis: Optional[Tuple[float, float]],
        split_depth_m: float = 14.0,
        lateral_entry_m: float = 2.0) -> Optional[str]:
    """把合法 SLAM 点划分为近/远、左/右四个房间扇区。"""

    if entry_origin is None or entry_axis is None:
        return None
    axis_x = float(entry_axis[0])
    axis_y = float(entry_axis[1])
    axis_norm = math.hypot(axis_x, axis_y)
    if axis_norm <= 1e-6:
        return None
    axis_x /= axis_norm
    axis_y /= axis_norm
    delta_x = float(world_x) - float(entry_origin[0])
    delta_y = float(world_y) - float(entry_origin[1])
    progress = delta_x * axis_x + delta_y * axis_y
    lateral = -delta_x * axis_y + delta_y * axis_x
    threshold = max(0.05, float(lateral_entry_m))
    if progress < 0.0 or abs(lateral) < threshold:
        return None
    depth = 'far' if progress >= max(0.0, float(split_depth_m)) else 'near'
    side = 'left' if lateral > 0.0 else 'right'
    return f'{depth}_{side}'


def select_symmetric_doorway_stations(
        observations,
        entry_origin: Optional[Tuple[float, float]],
        entry_axis: Optional[Tuple[float, float]],
        preferred_lateral_m: float = 1.5,
        pair_progress_gap_m: float = 2.0,
        station_cluster_m: float = 2.0,
        minimum_station_separation_m: float = 6.0,
        maximum_station_count: int = 2,
):
    """从门带观测中聚类左右同排门，并选择最远的两个真实房间站位。

    Frontier 会在入口空地、墙边、走廊尽头和房间内部产生很多短暂候选。
    固定 14m 分界曾把约 19m 的真实近房划成远房，又把入口空地误当近房。
    本函数不再假设任何绝对房门距离，只使用 SLAM 坐标：

    - 先枚举纵向差小于 ``pair_progress_gap_m`` 的左右候选；
    - 再按纵向位置聚类为门排；
    - 只选相互间隔足够大的最远两个门排，因此自动排除入口空地；
    - 每排左右坐标取平均纵深和对称横距，冻结后供返程逐房使用。

    返回值为 ``far_left/far_right/near_left/near_right`` 到世界坐标的映射。
    """

    if entry_origin is None or entry_axis is None:
        return {}
    axis_x = float(entry_axis[0])
    axis_y = float(entry_axis[1])
    norm = math.hypot(axis_x, axis_y)
    if norm <= 1e-6:
        return {}
    axis_x /= norm
    axis_y /= norm
    normal_x, normal_y = -axis_y, axis_x
    origin_x, origin_y = float(entry_origin[0]), float(entry_origin[1])
    preferred = max(0.05, abs(float(preferred_lateral_m)))
    pair_gap = max(0.1, float(pair_progress_gap_m))
    cluster_radius = max(0.2, float(station_cluster_m))
    minimum_separation = max(
        cluster_radius, float(minimum_station_separation_m))
    maximum_count = max(1, int(maximum_station_count))

    annotated = []
    for raw_pose in list(observations or []):
        try:
            world_x = float(raw_pose[0])
            world_y = float(raw_pose[1])
        except (IndexError, TypeError, ValueError):
            continue
        if not math.isfinite(world_x) or not math.isfinite(world_y):
            continue
        delta_x = world_x - origin_x
        delta_y = world_y - origin_y
        progress = delta_x * axis_x + delta_y * axis_y
        lateral = delta_x * normal_x + delta_y * normal_y
        if progress < 0.0 or abs(lateral) < 0.05:
            continue
        annotated.append((progress, lateral, world_x, world_y))

    left = [item for item in annotated if item[1] > 0.0]
    right = [item for item in annotated if item[1] < 0.0]
    pairs = []
    for left_item in left:
        for right_item in right:
            progress_gap = abs(left_item[0] - right_item[0])
            if progress_gap > pair_gap:
                continue
            station_progress = 0.5 * (
                left_item[0] + right_item[0])
            lateral_magnitude = 0.5 * (
                abs(left_item[1]) + abs(right_item[1]))
            quality = (
                progress_gap
                + 0.30 * abs(abs(left_item[1]) - preferred)
                + 0.30 * abs(abs(right_item[1]) - preferred)
            )
            pairs.append({
                'progress': station_progress,
                'lateral': lateral_magnitude,
                'quality': quality,
            })
    if not pairs:
        return {}

    # 同一真实门排会产生多个相邻 Frontier 配对；先聚类，再取每簇质量最好
    # 的代表，避免某一帧的小幅质心漂移被误计成另一排房门。
    clusters = []
    for pair in sorted(pairs, key=lambda item: item['progress']):
        nearest = None
        nearest_gap = math.inf
        for cluster in clusters:
            gap = abs(pair['progress'] - cluster['mean_progress'])
            if gap <= cluster_radius and gap < nearest_gap:
                nearest = cluster
                nearest_gap = gap
        if nearest is None:
            clusters.append({
                'pairs': [pair],
                'mean_progress': pair['progress'],
            })
        else:
            nearest['pairs'].append(pair)
            nearest['mean_progress'] = sum(
                item['progress'] for item in nearest['pairs']) / len(
                    nearest['pairs'])

    representatives = []
    for cluster in clusters:
        representative = min(
            cluster['pairs'], key=lambda item: item['quality'])
        representatives.append(representative)
    representatives.sort(key=lambda item: item['progress'], reverse=True)

    selected = []
    for representative in representatives:
        if any(abs(
                representative['progress'] - existing['progress'])
                < minimum_separation for existing in selected):
            continue
        selected.append(representative)
        if len(selected) >= maximum_count:
            break
    if len(selected) < 2:
        # 四房官方楼层必须观测到两组左右配对；不允许凭单侧点或固定比例
        # 伪造第二排，否则入口空地会再次被算作房间。
        return {}

    result = {}
    for depth, station in zip(('far', 'near'), selected[:2]):
        station_progress = float(station['progress'])
        lateral_magnitude = max(preferred, float(station['lateral']))
        for side, sign in (('left', 1.0), ('right', -1.0)):
            result[f'{depth}_{side}'] = (
                origin_x + axis_x * station_progress
                + normal_x * sign * lateral_magnitude,
                origin_y + axis_y * station_progress
                + normal_y * sign * lateral_magnitude,
            )
    return result


def prioritize_unvisited_room_frontiers(
        frontiers: Iterable[Frontier],
        entry_origin: Optional[Tuple[float, float]],
        entry_axis: Optional[Tuple[float, float]],
        visited_sectors: Iterable[str],
        attempted_sectors: Iterable[str] = (),
        active_sector: Optional[str] = None,
        split_depth_m: float = 14.0,
        candidate_lateral_m: float = 1.2,
        candidate_max_lateral_m: float = 4.0,
        far_depth_margin_m: float = 2.0):
    """优先当前房间闭环，其次按远到近选择尚未完成的房间。"""

    candidates = list(frontiers)
    if not candidates:
        return candidates, (
            'active_room_frontiers_exhausted'
            if active_sector is not None else 'unconstrained'
        )
    if entry_origin is None or entry_axis is None:
        return candidates, 'unconstrained'
    visited = set(str(item) for item in visited_sectors)
    attempted = set(str(item) for item in attempted_sectors)
    axis_x = float(entry_axis[0])
    axis_y = float(entry_axis[1])
    axis_norm = math.hypot(axis_x, axis_y)
    if axis_norm <= 1e-6:
        return candidates, 'unconstrained'
    axis_x /= axis_norm
    axis_y /= axis_norm

    def lateral_of(frontier: Frontier) -> float:
        delta_x = frontier.centroid[0] - float(entry_origin[0])
        delta_y = frontier.centroid[1] - float(entry_origin[1])
        return -delta_x * axis_y + delta_y * axis_x

    maximum_lateral = max(
        float(candidate_lateral_m), float(candidate_max_lateral_m))
    annotated = [
        (
            corridor_room_sector(
                frontier.centroid[0], frontier.centroid[1],
                entry_origin, entry_axis,
                split_depth_m=split_depth_m,
                lateral_entry_m=candidate_lateral_m,
            ),
            frontier,
        )
        for frontier in candidates
    ]
    if active_sector is not None:
        # 一旦真正跨过门口，后续目标必须留在同一房间，直到房间内前沿
        # 耗尽或轨迹闭环。这样不会刚进入两米就被走廊上的大前沿拉出去。
        active = [
            frontier for sector, frontier in annotated
            if sector == str(active_sector)
        ]
        return active, (
            'active_room_perimeter'
            if active else 'active_room_frontiers_exhausted'
        )
    # 门口已经物理到达但尚未跨门时，必须先完成同一房间的深层解锁。
    # 否则左右两侧仍同属“未完成远房间”，信息增益会把机器人立即拉去
    # 对侧门口，形成只看门不入室。优先选门带外的同扇区前沿；地图暂时
    # 尚未显出深层前沿时保留同扇区门口候选等待继续观测。
    attempted_unfinished = [
        frontier for sector, frontier in annotated
        if sector in attempted and sector not in visited
    ]
    if attempted_unfinished:
        attempted_deep = [
            frontier for frontier in attempted_unfinished
            if abs(lateral_of(frontier)) > maximum_lateral
        ]
        return (
            attempted_deep or attempted_unfinished,
            'attempted_room_deep_after_door',
        )
    unvisited_far = [
        frontier for sector, frontier in annotated
        if sector in ('far_left', 'far_right') and sector not in visited
        and (
            abs(lateral_of(frontier)) <= maximum_lateral
            or sector in attempted
        )
    ]
    if unvisited_far:
        return unvisited_far, 'unvisited_far_room'

    missing_far = {'far_left', 'far_right'} - visited
    if missing_far:
        minimum_progress = max(
            0.0,
            float(split_depth_m) - max(0.0, float(far_depth_margin_m)),
        )
        far_zone = [
            frontier for frontier in candidates
            if (
                (frontier.centroid[0] - float(entry_origin[0])) * axis_x
                + (frontier.centroid[1] - float(entry_origin[1])) * axis_y
            ) >= minimum_progress
            and abs(lateral_of(frontier)) < max(
                0.2, float(candidate_lateral_m))
        ]
        if far_zone:
            return far_zone, 'far_room_discovery'

    unvisited_near = [
        frontier for sector, frontier in annotated
        if sector in ('near_left', 'near_right') and sector not in visited
        and (
            abs(lateral_of(frontier)) <= maximum_lateral
            or sector in attempted
        )
    ]
    if unvisited_near:
        return unvisited_near, 'unvisited_near_room'
    return candidates, 'all_room_sectors_covered'


def select_cached_missing_room_doorway(
        cached_poses, visited_sectors, attempted_sectors):
    """按远到近返回尚未尝试房间的合法 SLAM 门带缓存。"""

    cached = dict(cached_poses or {})
    visited = set(str(item) for item in visited_sectors or ())
    attempted = set(str(item) for item in attempted_sectors or ())
    # 已到门但未完成的房间拥有最高优先级，应由实时深层前沿或房间收尾
    # 处理；此时返回另一个缓存门口会再次造成“只看门不入室”。
    if attempted - visited:
        return None
    for sector in ('far_left', 'far_right', 'near_left', 'near_right'):
        if sector in visited or sector in attempted or sector not in cached:
            continue
        pose = cached[sector]
        try:
            x_value = float(pose[0])
            y_value = float(pose[1])
        except (IndexError, TypeError, ValueError):
            continue
        if math.isfinite(x_value) and math.isfinite(y_value):
            return sector, (x_value, y_value)
    return None


def build_counterclockwise_room_loop(
        doorway: Tuple[float, float],
        entry_axis: Tuple[float, float],
        side: str,
        shallow_depth_m: float = 2.0,
        deep_depth_m: float = 5.0,
        half_length_m: float = 2.0,
        corner_radius_m: float = 0.0,
) -> List[Tuple[float, float]]:
    """以房门为基准生成世界坐标逆时针矩形单圈，不读取房间真值。"""

    axis_x = float(entry_axis[0])
    axis_y = float(entry_axis[1])
    norm = math.hypot(axis_x, axis_y)
    if norm <= 1e-6:
        return []
    axis_x /= norm
    axis_y /= norm
    normal_x, normal_y = -axis_y, axis_x
    side_value = str(side).strip().lower()
    if side_value not in ('left', 'right'):
        return []
    sign = 1.0 if side_value == 'left' else -1.0
    shallow = max(0.5, float(shallow_depth_m))
    deep = max(shallow + 0.5, float(deep_depth_m))
    half = max(0.5, float(half_length_m))
    radius = min(
        max(0.0, float(corner_radius_m)),
        half * 0.45,
        (deep - shallow) * 0.45,
    )
    # 左房从门内底边向走廊前方开始；右房镜像后仍保持正面积（逆时针）。
    # 官方DWA使用6点圆角六边形：比四点矩形转角小，又避免八点路线在
    # 圆角附近产生四个短小目标和额外停顿。
    if radius > 1e-6:
        middle = 0.5 * (shallow + deep)
        local_points = (
            (sign * (half - radius), sign * shallow),
            (sign * half, sign * middle),
            (sign * (half - radius), sign * deep),
            (-sign * (half - radius), sign * deep),
            (-sign * half, sign * middle),
            (-sign * (half - radius), sign * shallow),
        )
    else:
        local_points = (
            (sign * half, sign * shallow),
            (sign * half, sign * deep),
            (-sign * half, sign * deep),
            (-sign * half, sign * shallow),
        )
    result = []
    door_x, door_y = float(doorway[0]), float(doorway[1])
    for progress, lateral in local_points:
        result.append((
            door_x + axis_x * progress + normal_x * lateral,
            door_y + axis_y * progress + normal_y * lateral,
        ))
    return result


def scaled_room_waypoint_candidates(
        doorway: Tuple[float, float],
        waypoint: Tuple[float, float],
        scales=(0.8, 0.6, 0.4),
) -> List[Tuple[float, float]]:
    """把被障碍挡住的房间角点沿门口方向收缩，保持原有象限顺序。"""

    door_x, door_y = float(doorway[0]), float(doorway[1])
    goal_x, goal_y = float(waypoint[0]), float(waypoint[1])
    if not all(math.isfinite(value) for value in (
            door_x, door_y, goal_x, goal_y)):
        return []
    result = []
    for raw_scale in scales:
        scale = float(raw_scale)
        if not math.isfinite(scale) or not 0.0 < scale < 1.0:
            continue
        candidate = (
            round(door_x + (goal_x - door_x) * scale, 6),
            round(door_y + (goal_y - door_y) * scale, 6),
        )
        if candidate not in result:
            result.append(candidate)
    return result


def polygon_signed_area(points) -> float:
    """返回闭合二维多边形有符号面积，正值表示逆时针。"""

    values = list(points or [])
    if len(values) < 3:
        return 0.0
    total = 0.0
    for index, point in enumerate(values):
        next_point = values[(index + 1) % len(values)]
        total += (
            float(point[0]) * float(next_point[1])
            - float(next_point[0]) * float(point[1]))
    return 0.5 * total


def physical_room_loop_is_valid(
        samples,
        reached_count: int,
        expected_count: int,
        physical_path_m: float,
        min_path_m: float,
        min_area_m2: float) -> bool:
    """严格验收真实物理房间闭环，防止门口徘徊被计成四个角点。"""

    values = list(samples or [])
    expected = max(4, int(expected_count))
    if int(reached_count) != expected or len(values) < expected:
        return False
    try:
        points = [
            (float(point[0]), float(point[1]))
            for point in values[:expected]
        ]
        path_m = float(physical_path_m)
    except (IndexError, TypeError, ValueError):
        return False
    if not all(math.isfinite(value) for point in points for value in point):
        return False
    loop_perimeter = 0.0
    for index, point in enumerate(points):
        next_point = points[(index + 1) % len(points)]
        loop_perimeter += math.hypot(
            next_point[0] - point[0],
            next_point[1] - point[1],
        )
    # 定时器增量会因 rosbridge批次到达而漏计，四个已经验真的物理角点
    # 本身可给出闭环周长下界；取两者较大值，仍不允许原地重复采样通过。
    effective_path_m = max(path_m, loop_perimeter)
    if (not math.isfinite(effective_path_m)
            or effective_path_m < max(0.0, float(min_path_m))):
        return False
    return abs(polygon_signed_area(points)) >= max(
        0.0, float(min_area_m2))


def entry_ingress_constraint_active(
        entry_axis: Optional[Tuple[float, float]],
        entry_progress_m: Optional[float],
        ingress_depth_m: float) -> bool:
    """决定是否继续限制入楼方向；进度未知时保守保持约束。"""

    if entry_axis is None:
        return True
    depth = max(0.0, float(ingress_depth_m))
    if depth <= 0.0:
        return False
    if entry_progress_m is None or not math.isfinite(entry_progress_m):
        return True
    return float(entry_progress_m) < depth


def entry_ingress_half_angles_deg(configured_deg: float,
                                  relaxed_deg: float,
                                  maximum_deg: float,
                                  constraint_active: bool
                                  ) -> Tuple[Optional[float], ...]:
    """返回确定性的窄到宽入楼锥序列；解除约束后返回全向模式。"""

    if not constraint_active:
        return (None,)
    if not all(math.isfinite(float(value)) for value in (
            configured_deg, relaxed_deg, maximum_deg)):
        return tuple()
    configured = max(5.0, min(90.0, float(configured_deg)))
    relaxed = max(
        configured,
        min(90.0, float(relaxed_deg)),
    )
    maximum = max(
        relaxed,
        min(90.0, float(maximum_deg)),
    )
    return tuple(dict.fromkeys((configured, relaxed, maximum)))


def should_switch_frontier(current_distance_m: float,
                           challenger_distance_m: float,
                           held_duration_s: float,
                           switch_margin_m: float = 1.0,
                           minimum_hold_s: float = 8.0,
                           recent_progress_age_s: Optional[float] = None,
                           progress_protection_s: float = 0.0,
                           progress_protection_max_hold_s: float = 0.0) -> bool:
    """判断是否用新出现的近场前沿替换仍可规划的远目标。

    最短锁定时间和距离滞回共同防止质心抖动；超过锁定期后，只有明显更近的
    候选才能抢占。当前目标近期仍产生净距离进展时继续保护它，避免地图更新
    反复产生“更近”候选并让机器人在走廊两端掉头。距离与进展时间均来自同一
    合法 SLAM/仿真时钟，不使用场景真值。
    """

    if not all(math.isfinite(value) for value in (
            current_distance_m, challenger_distance_m, held_duration_s,
            switch_margin_m, minimum_hold_s, progress_protection_s,
            progress_protection_max_hold_s)):
        return False
    protection = max(0.0, float(progress_protection_s))
    maximum_protected_hold = max(
        0.0,
        float(progress_protection_max_hold_s),
    )
    if recent_progress_age_s is not None:
        if not math.isfinite(recent_progress_age_s):
            return False
        protection_hold_active = (
            maximum_protected_hold <= 0.0
            or held_duration_s < maximum_protected_hold
        )
        if (protection_hold_active
                and max(0.0, float(recent_progress_age_s)) < protection):
            return False
    if held_duration_s < max(0.0, minimum_hold_s):
        return False
    return (
        challenger_distance_m + max(0.0, switch_margin_m)
        < current_distance_m
    )


def a_star_path(grid: np.ndarray, grid_msg,
                start_wx: float, start_wy: float,
                goal_wx: float, goal_wy: float,
                inflation_radius_m: float = 0.45,
                endpoint_search_radius_m: float = 0.50,
                max_expansions: int = 250000,
                start_search_radius_m: Optional[float] = None,
                goal_search_radius_m: Optional[float] = None,
                append_exact_goal: bool = False,
                ) -> List[Tuple[float, float]]:
    """A* 网格路径规划。

    在 OccupancyGrid 上查找从起点到终点的最短路径。
    只允许经过已知自由格；未知格禁止穿越，占用格按机器狗安全半径膨胀，
    对角移动还要防止从两个障碍角之间穿过。

    ``endpoint_search_radius_m`` 保留为兼容旧调用的共同默认值。返航等需要
    精确到达原始目标的场景应单独传入较小的 ``goal_search_radius_m``，
    但仍可给受近场回波污染的机器人起点较大的 ``start_search_radius_m``。
    当目标原始栅格本身可通行时，``append_exact_goal`` 可把真实世界坐标追加
    到路径末尾，避免栅格中心和路径跟随容差叠加后停在目标容差之外。

    Returns:
        世界坐标路径点列表，若不可达返回空列表。
    """
    h, w = grid.shape
    sx, sy = world_to_grid(start_wx, start_wy, grid_msg)
    gx, gy = world_to_grid(goal_wx, goal_wy, grid_msg)
    goal_was_in_bounds = 0 <= gx < w and 0 <= gy < h
    if append_exact_goal and not goal_was_in_bounds:
        # 精确返航目标若尚未落入当前地图，不能先把栅格钳到边界，再追加一段
        # 穿越未知区的直线到原始世界坐标。
        return []

    # 边界钳制
    sx = max(0, min(w - 1, sx))
    sy = max(0, min(h - 1, sy))
    gx = max(0, min(w - 1, gx))
    gy = max(0, min(h - 1, gy))

    traversable = _build_traversable_mask(
        grid,
        float(grid_msg.info.resolution),
        inflation_radius_m,
    )
    # 实测 SLAM 会因机身近场回波或栅格离散把“机器人当前格”标成占用；
    # 在小范围内吸附到最近安全自由格，不能因此把整张可用地图判为不可达。
    start_radius_m = (
        endpoint_search_radius_m
        if start_search_radius_m is None
        else start_search_radius_m
    )
    goal_radius_m = (
        endpoint_search_radius_m
        if goal_search_radius_m is None
        else goal_search_radius_m
    )
    start_cell = _nearest_traversable_cell(
        traversable, sx, sy,
        float(grid_msg.info.resolution), start_radius_m)
    goal_cell = _nearest_traversable_cell(
        traversable, gx, gy,
        float(grid_msg.info.resolution), goal_radius_m)
    if start_cell is None or goal_cell is None:
        return []
    sx, sy = start_cell
    gx, gy = goal_cell

    # A* 数据结构
    open_set = [(0.0, sy, sx)]
    came_from = {}
    g_score = {(sy, sx): 0.0}
    closed = set()
    expansion_count = 0

    while open_set:
        _, cy, cx = heapq.heappop(open_set)

        if (cy, cx) in closed:
            continue
        closed.add((cy, cx))
        expansion_count += 1
        if expansion_count > max(1, int(max_expansions)):
            return []

        if cy == gy and cx == gx:
            # 重建路径
            path = [(gx, gy)]
            while (cy, cx) in came_from:
                cy, cx = came_from[(cy, cx)]
                path.append((cx, cy))
            path.reverse()
            path = simplify_grid_path(path, traversable)
            world_path = [
                grid_to_world(px, py, grid_msg) for px, py in path
            ]
            if append_exact_goal:
                exact_goal = (float(goal_wx), float(goal_wy))
                if (not world_path
                        or math.hypot(
                            world_path[-1][0] - exact_goal[0],
                            world_path[-1][1] - exact_goal[1],
                        ) > 1e-9):
                    world_path.append(exact_goal)
            return world_path

        # 八邻域
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            ny, nx = cy + dy, cx + dx
            if not (0 <= ny < h and 0 <= nx < w):
                continue
            if not traversable[ny, nx]:
                continue
            if dx != 0 and dy != 0 and (
                    not traversable[cy, nx] or not traversable[ny, cx]):
                # 两个正交邻格至少一个不可通行时，禁止对角穿角。
                continue

            cost = 1.414 if dx != 0 and dy != 0 else 1.0
            new_g = g_score.get((cy, cx), float('inf')) + cost

            if new_g < g_score.get((ny, nx), float('inf')):
                came_from[(ny, nx)] = (cy, cx)
                g_score[(ny, nx)] = new_g
                f = new_g + _heuristic(nx, ny, gx, gy)
                heapq.heappush(open_set, (f, ny, nx))

    return []  # 不可达


def _grid_segment_is_traversable(
        start: Tuple[int, int],
        end: Tuple[int, int],
        traversable: np.ndarray) -> bool:
    """以超采样栅格线检查路径捷径，并保持 A* 的禁止对角穿角约束。"""

    x0, y0 = int(start[0]), int(start[1])
    x1, y1 = int(end[0]), int(end[1])
    height, width = traversable.shape
    sample_count = max(abs(x1 - x0), abs(y1 - y0)) * 4 + 1
    xs = np.rint(np.linspace(x0, x1, sample_count)).astype(np.int32)
    ys = np.rint(np.linspace(y0, y1, sample_count)).astype(np.int32)
    cells = []
    for x, y in zip(xs.tolist(), ys.tolist()):
        cell = (x, y)
        if not cells or cell != cells[-1]:
            cells.append(cell)
    previous = None
    for x, y in cells:
        if not (0 <= x < width and 0 <= y < height and traversable[y, x]):
            return False
        if previous is not None:
            px, py = previous
            if x != px and y != py:
                if not traversable[py, x] or not traversable[y, px]:
                    return False
        previous = (x, y)
    return True


def simplify_grid_path(
        path: List[Tuple[int, int]],
        traversable: np.ndarray) -> List[Tuple[int, int]]:
    """用安全视线压缩 A* 栅格折线，避免机器狗逐 5 cm 点停车转向。

    每段捷径都在已经按机体半径膨胀的可通行掩膜内验证；一旦下一候选不可见，
    保留最后安全拐点。这样减少控制振荡，但不会跨未知区或切过障碍角。
    """

    if len(path) <= 2:
        return list(path)
    result = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        best = anchor + 1
        for candidate in range(anchor + 2, len(path)):
            if not _grid_segment_is_traversable(
                    path[anchor], path[candidate], traversable):
                break
            best = candidate
        result.append(path[best])
        anchor = best
    return result


def _build_traversable_mask(grid: np.ndarray, resolution_m: float,
                            inflation_radius_m: float) -> np.ndarray:
    """生成 A1 机体中心可通行掩膜；未知区始终不可通行。"""

    if resolution_m <= 0.0:
        raise ValueError('map resolution must be positive')
    # OccupancyGrid 是概率栅格；Cartographer 的已知自由区包含 1..49，
    # 只接受精确 0 会让实时地图“没有前沿、没有路径”，而 map_saver 离线图正常。
    traversable = (grid >= FREE) & (grid <= FREE_MAX)
    radius_cells = max(0, int(math.ceil(float(inflation_radius_m) / resolution_m)))
    if radius_cells == 0:
        return traversable

    occupied = grid >= 65
    inflated = occupied.copy()
    height, width = grid.shape
    for delta_y in range(-radius_cells, radius_cells + 1):
        for delta_x in range(-radius_cells, radius_cells + 1):
            if delta_x * delta_x + delta_y * delta_y > radius_cells * radius_cells:
                continue
            source_y = slice(max(0, -delta_y), min(height, height - delta_y))
            source_x = slice(max(0, -delta_x), min(width, width - delta_x))
            target_y = slice(max(0, delta_y), min(height, height + delta_y))
            target_x = slice(max(0, delta_x), min(width, width + delta_x))
            inflated[target_y, target_x] |= occupied[source_y, source_x]
    traversable[inflated] = False
    return traversable


def _nearest_traversable_cell(traversable: np.ndarray, x: int, y: int,
                              resolution_m: float,
                              search_radius_m: float):
    """返回端点附近最近的安全自由格；超出吸附半径则失败。"""

    if traversable[y, x]:
        return x, y
    radius_cells = max(
        0, int(math.ceil(max(0.0, float(search_radius_m)) / resolution_m)))
    if radius_cells == 0:
        return None
    height, width = traversable.shape
    x_min, x_max = max(0, x - radius_cells), min(width, x + radius_cells + 1)
    y_min, y_max = max(0, y - radius_cells), min(height, y + radius_cells + 1)
    candidates = np.argwhere(traversable[y_min:y_max, x_min:x_max])
    if candidates.size == 0:
        return None
    candidates[:, 0] += y_min
    candidates[:, 1] += x_min
    squared_distance = (
        (candidates[:, 1] - x) ** 2 + (candidates[:, 0] - y) ** 2
    )
    best_index = int(np.argmin(squared_distance))
    if (math.sqrt(float(squared_distance[best_index])) * resolution_m
            > float(search_radius_m)):
        return None
    best_y, best_x = candidates[best_index]
    return int(best_x), int(best_y)


def _heuristic(x1: int, y1: int, x2: int, y2: int) -> float:
    """八邻域启发函数 (octile distance)。"""
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    return max(dx, dy) + (1.414 - 1.0) * min(dx, dy)


def _normalized_angle(angle: float) -> float:
    """把角差规范到 [-pi, pi]，供前向前沿门禁使用。"""

    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def compute_exploration_time_limit_s(
        configured_timeout_s: float,
        mission_budget_s: float,
        distance_home_m: float,
        return_speed_mps: float,
        minimum_return_reserve_s: float = 120.0,
        return_safety_factor: float = 2.0,
        return_fixed_overhead_s: float = 30.0) -> float:
    """计算当前距离下允许继续探索的仿真时间上限。

    600 秒正式任务至少给返航保留 120 秒；距离增加时，按保守速度、路径绕行
    倍率和固定重规划开销进一步提前返航。返回值始终不超过调用方配置的探索
    上限，也不会侵占动态返航预留。
    """

    budget = max(0.0, float(mission_budget_s))
    configured = max(0.0, float(configured_timeout_s))
    minimum_reserve = max(0.0, float(minimum_return_reserve_s))
    distance = max(0.0, float(distance_home_m))
    speed = max(0.05, abs(float(return_speed_mps)))
    safety_factor = max(1.0, float(return_safety_factor))
    fixed_overhead = max(0.0, float(return_fixed_overhead_s))
    estimated_return_s = fixed_overhead + distance / speed * safety_factor
    reserve_s = min(budget, max(minimum_reserve, estimated_return_s))
    return min(configured, max(0.0, budget - reserve_s))


def return_pose_has_progress(
        previous_x: float,
        previous_y: float,
        current_x: float,
        current_y: float,
        minimum_distance_m: float) -> bool:
    """判断返航是否产生真实平移，不要求每一步都朝家的方向移动。

    复杂楼宇的合法返航路径可能先绕开障碍、短时远离起点。看门狗只应在机体
    没有实际位移时触发，不能用“距家单调减少”作为唯一进展条件。
    """

    threshold = max(0.0, float(minimum_distance_m))
    return math.hypot(
        float(current_x) - float(previous_x),
        float(current_y) - float(previous_y),
    ) >= threshold


def frontier_route_is_excessive_detour(
        path_distance_m: float,
        straight_distance_m: float,
        maximum_ratio: float = 2.8,
        minimum_excess_m: float = 5.0) -> bool:
    """判断前沿路径是否属于应暂缓的隔墙长绕行。

    该门禁只比较合法 SLAM 地图上的 A* 路径长度与直线距离，不读取房间布局。
    同时满足“比例过大”和“绝对多走距离过大”才返回真，避免把正常绕门或短距离
    离散误差错误过滤。输入异常时关闭优化而不是拒绝目标，保持探索完备性。
    """

    try:
        path_distance = float(path_distance_m)
        straight_distance = float(straight_distance_m)
        ratio_limit = float(maximum_ratio)
        excess_limit = float(minimum_excess_m)
    except (TypeError, ValueError):
        return False
    if not all(math.isfinite(value) for value in (
            path_distance,
            straight_distance,
            ratio_limit,
            excess_limit,
    )):
        return False
    if path_distance < 0.0 or straight_distance < 0.0:
        return False
    ratio_limit = max(1.0, ratio_limit)
    excess_limit = max(0.0, excess_limit)
    effective_straight_distance = max(0.25, straight_distance)
    return (
        path_distance > straight_distance + excess_limit
        and path_distance / effective_straight_distance > ratio_limit
    )


def return_recovery_turn_command(
        attempt: int,
        turn_speed_rad_s: float) -> float:
    """生成左右交替且有界的返航脱困转速。"""

    try:
        attempt_index = int(attempt)
        speed = abs(float(turn_speed_rad_s))
    except (TypeError, ValueError):
        return 0.0
    if (attempt_index <= 0
            or not math.isfinite(speed)
            or speed <= 0.0):
        return 0.0
    return speed if attempt_index % 2 == 1 else -speed


def nearest_frontier_basin_key(
        keys: Iterable[Tuple[float, float]],
        point_x: float,
        point_y: float,
        radius_m: float) -> Optional[Tuple[float, float]]:
    """返回抑制半径内最近的失败前沿盆地。

    Cartographer 地图持续更新时，同一墙角前沿的质心会有数厘米抖动，不能
    只靠四舍五入后的精确键判断。该纯函数让节点按真实空间邻域合并失败记录。
    """

    radius = max(0.0, float(radius_m))
    best_key = None
    best_distance = float('inf')
    for key in keys:
        distance = math.hypot(
            float(key[0]) - float(point_x),
            float(key[1]) - float(point_y),
        )
        if distance <= radius and distance < best_distance:
            best_key = key
            best_distance = distance
    return best_key


def compute_frontier_backoff_ttl_s(
        base_ttl_s: float,
        maximum_ttl_s: float,
        failure_count: int) -> float:
    """按同一空间盆地的连续失败次数计算有上限的指数退避时间。"""

    base = max(0.1, float(base_ttl_s))
    maximum = max(base, float(maximum_ttl_s))
    failures = max(1, int(failure_count))
    return min(maximum, base * (2.0 ** min(failures - 1, 16)))
