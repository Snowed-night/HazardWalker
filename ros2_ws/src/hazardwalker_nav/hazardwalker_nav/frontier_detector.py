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
from typing import List, Optional, Tuple

import numpy as np

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
    h, w = grid.shape
    frontiers = np.zeros((h, w), dtype=bool)
    for y in range(1, h - 1):
        for x in range(1, w - 1):
            if FREE <= grid[y, x] <= int(free_max):
                # 检查四邻域
                if (grid[y - 1, x] == UNKNOWN or grid[y + 1, x] == UNKNOWN or
                        grid[y, x - 1] == UNKNOWN or grid[y, x + 1] == UNKNOWN):
                    frontiers[y, x] = True
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
                         robot_yaw: Optional[float] = None,
                         entry_origin: Optional[Tuple[float, float]] = None,
                         entry_axis: Optional[Tuple[float, float]] = None,
                         entry_backtrack_margin_m: float = 0.5) -> Optional[Frontier]:
    """选择最优前沿：综合距离、信息增益、大小。

    策略：优先选择近距离、高信息增益的前沿。
    如果有上一个目标且它仍然是前沿，优先继续前往。
    """
    if not frontiers:
        return None

    # 过滤太小前沿
    valid = [f for f in frontiers if f.size >= min_frontier_size]
    if not valid:
        valid = frontiers  # 都太小时退回到所有前沿
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
            valid = [
                frontier for frontier in valid
                if (
                    (frontier.centroid[0] - float(entry_origin[0])) * axis_x
                    + (frontier.centroid[1] - float(entry_origin[1])) * axis_y
                ) >= -margin
            ]
            if not valid:
                return None
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
            )) <= math.pi / 2.0
        ]
        if forward:
            valid = forward

    best = None
    best_score = -float('inf')

    for f in valid:
        dist = math.hypot(f.centroid[0] - robot_wx, f.centroid[1] - robot_wy)
        dist = max(dist, 0.5)  # 避免除以零

        # 综合评分：信息增益 / 距离 为主，叠加大小因子
        score = f.info_gain / dist + math.log(f.size + 1) * 0.5

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


def a_star_path(grid: np.ndarray, grid_msg,
                start_wx: float, start_wy: float,
                goal_wx: float, goal_wy: float,
                inflation_radius_m: float = 0.25,
                endpoint_search_radius_m: float = 0.50,
                max_expansions: int = 250000) -> List[Tuple[float, float]]:
    """A* 网格路径规划。

    在 OccupancyGrid 上查找从起点到终点的最短路径。
    只允许经过已知自由格；未知格禁止穿越，占用格按机器狗安全半径膨胀，
    对角移动还要防止从两个障碍角之间穿过。

    Returns:
        世界坐标路径点列表，若不可达返回空列表。
    """
    h, w = grid.shape
    sx, sy = world_to_grid(start_wx, start_wy, grid_msg)
    gx, gy = world_to_grid(goal_wx, goal_wy, grid_msg)

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
    start_cell = _nearest_traversable_cell(
        traversable, sx, sy,
        float(grid_msg.info.resolution), endpoint_search_radius_m)
    goal_cell = _nearest_traversable_cell(
        traversable, gx, gy,
        float(grid_msg.info.resolution), endpoint_search_radius_m)
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
            return [grid_to_world(px, py, grid_msg) for px, py in path]

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
