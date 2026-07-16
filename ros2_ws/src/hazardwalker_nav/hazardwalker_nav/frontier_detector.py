"""自主探索：前沿检测与路径规划纯函数。

所属组：导航组。
文件作用：
- 提供不依赖 ROS 的前沿检测、聚类、评分和 A* 路径规划。
- 被 frontier_explorer_node.py 调用。

前沿定义：OccupancyGrid 中值为 FREE (0) 且四邻域至少有一个 UNKNOWN (-1) 的格子。
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

# OccupancyGrid 常量
FREE = 0
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


def find_frontiers(grid: np.ndarray) -> np.ndarray:
    """查找所有前沿格子：FREE (0) 且四邻域中有 UNKNOWN (-1)。

    Returns:
        (height, width) 布尔数组，True 表示该格子是前沿。
    """
    h, w = grid.shape
    frontiers = np.zeros((h, w), dtype=bool)
    for y in range(1, h - 1):
        for x in range(1, w - 1):
            if grid[y, x] == FREE:
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
                # 质心 (网格坐标)
                gx_mean = sum(p[0] for p in points) / len(points)
                gy_mean = sum(p[1] for p in points) / len(points)
                wx, wy = grid_to_world(int(gx_mean), int(gy_mean), grid_msg)

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
                         min_frontier_size: int = 10) -> Optional[Frontier]:
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
                goal_wx: float, goal_wy: float) -> List[Tuple[float, float]]:
    """A* 网格路径规划。

    在 OccupancyGrid 上查找从起点到终点的最短路径。
    未知格子 (-1) 视为可通行（乐观策略），占用格子 (>=65) 视为障碍。

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

    if grid[sy, sx] >= 65 or grid[gy, gx] >= 65:
        return []  # 起点或终点被阻挡

    # A* 数据结构
    open_set = [(0.0, sy, sx)]
    came_from = {}
    g_score = {(sy, sx): 0.0}
    f_score = {(sy, sx): _heuristic(sx, sy, gx, gy)}
    closed = set()

    while open_set:
        open_set.sort(key=lambda x: x[0])
        _, cy, cx = open_set.pop(0)

        if (cy, cx) in closed:
            continue
        closed.add((cy, cx))

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
            if grid[ny, nx] >= 65:  # 障碍
                continue

            cost = 1.414 if dx != 0 and dy != 0 else 1.0
            new_g = g_score.get((cy, cx), float('inf')) + cost

            if new_g < g_score.get((ny, nx), float('inf')):
                came_from[(ny, nx)] = (cy, cx)
                g_score[(ny, nx)] = new_g
                f = new_g + _heuristic(nx, ny, gx, gy)
                f_score[(ny, nx)] = f
                open_set.append((f, ny, nx))

    return []  # 不可达


def _heuristic(x1: int, y1: int, x2: int, y2: int) -> float:
    """八邻域启发函数 (octile distance)。"""
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    return max(dx, dy) + (1.414 - 1.0) * min(dx, dy)
