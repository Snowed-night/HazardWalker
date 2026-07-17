"""自主探索：前沿检测与路径规划纯函数。

所属组：导航组。
- 提供不依赖 ROS 的前沿检测、聚类、评分和 A* 路径规划。
- 被 frontier_explorer_node.py 调用。
- 前沿定义：OccupancyGrid 中值为 FREE (0) 且四邻域至少有一个 UNKNOWN (-1) 的格子。
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

FREE = 0
OCCUPIED = 100
UNKNOWN = -1


@dataclass
class Frontier:
    centroid: Tuple[float, float]
    size: int
    points: List[Tuple[int, int]]
    info_gain: float


def occupancy_grid_to_array(grid_msg) -> np.ndarray:
    data = np.array(grid_msg.data, dtype=np.int8).reshape(
        grid_msg.info.height, grid_msg.info.width)
    return data


def grid_to_world(gx: int, gy: int, grid_msg) -> Tuple[float, float]:
    origin = grid_msg.info.origin
    wx = origin.position.x + (gx + 0.5) * grid_msg.info.resolution
    wy = origin.position.y + (gy + 0.5) * grid_msg.info.resolution
    return wx, wy


def world_to_grid(wx: float, wy: float, grid_msg) -> Tuple[int, int]:
    origin = grid_msg.info.origin
    res = grid_msg.info.resolution
    gx = int((wx - origin.position.x) / res)
    gy = int((wy - origin.position.y) / res)
    return gx, gy


def find_frontiers(grid: np.ndarray) -> np.ndarray:
    h, w = grid.shape
    frontiers = np.zeros((h, w), dtype=bool)
    for y in range(1, h - 1):
        for x in range(1, w - 1):
            if grid[y, x] == FREE:
                if (grid[y - 1, x] == UNKNOWN or grid[y + 1, x] == UNKNOWN or
                        grid[y, x - 1] == UNKNOWN or grid[y, x + 1] == UNKNOWN):
                    frontiers[y, x] = True
    return frontiers


def _bfs_cluster(start_y, start_x, mask, visited, grid):
    h, w = mask.shape
    cluster = []
    q = deque()
    q.append((start_y, start_x))
    visited[start_y, start_x] = True
    while q:
        y, x = q.popleft()
        cluster.append((x, y))
        for ny, nx in [(y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)]:
            if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not visited[ny, nx]:
                visited[ny, nx] = True
                q.append((ny, nx))
    return cluster


def cluster_frontiers(frontier_mask, grid, grid_msg):
    h, w = frontier_mask.shape
    visited = np.zeros((h, w), dtype=bool)
    clusters = []
    for y in range(h):
        for x in range(w):
            if frontier_mask[y, x] and not visited[y, x]:
                points = _bfs_cluster(y, x, frontier_mask, visited, grid)
                if len(points) < 3:
                    continue
                gx_mean = sum(p[0] for p in points) / len(points)
                gy_mean = sum(p[1] for p in points) / len(points)
                wx, wy = grid_to_world(int(gx_mean), int(gy_mean), grid_msg)
                info = 0
                for px, py in points:
                    for ny, nx in [(py - 1, px), (py + 1, px), (py, px - 1), (py, px + 1)]:
                        if 0 <= ny < h and 0 <= nx < w and grid[ny, nx] == UNKNOWN:
                            info += 1
                clusters.append(Frontier(
                    centroid=(wx, wy), size=len(points),
                    points=points, info_gain=info))
    clusters.sort(key=lambda f: f.size, reverse=True)
    return clusters


def select_best_frontier(frontiers, robot_wx, robot_wy,
                         last_target=None, min_frontier_size=10):
    if not frontiers:
        return None
    valid = [f for f in frontiers if f.size >= min_frontier_size]
    if not valid:
        valid = frontiers
    best = None
    best_score = -float('inf')
    for f in valid:
        dist = math.hypot(f.centroid[0] - robot_wx, f.centroid[1] - robot_wy)
        dist = max(dist, 0.5)
        score = f.info_gain / dist + math.log(f.size + 1) * 0.5
        if last_target is not None:
            lt_dist = math.hypot(f.centroid[0] - last_target[0],
                                 f.centroid[1] - last_target[1])
            if lt_dist < 1.0:
                score *= 1.5
        if score > best_score:
            best_score = score
            best = f
    return best


def a_star_path(grid, grid_msg, start_wx, start_wy, goal_wx, goal_wy):
    h, w = grid.shape
    sx, sy = world_to_grid(start_wx, start_wy, grid_msg)
    gx, gy = world_to_grid(goal_wx, goal_wy, grid_msg)
    sx = max(0, min(w - 1, sx)); sy = max(0, min(h - 1, sy))
    gx = max(0, min(w - 1, gx)); gy = max(0, min(h - 1, gy))
    if grid[sy, sx] >= 65 or grid[gy, gx] >= 65:
        return []
    open_set = [(0.0, sy, sx)]
    came_from = {}
    g_score = {(sy, sx): 0.0}
    closed = set()
    while open_set:
        open_set.sort(key=lambda x: x[0])
        _, cy, cx = open_set.pop(0)
        if (cy, cx) in closed:
            continue
        closed.add((cy, cx))
        if cy == gy and cx == gx:
            path = [(gx, gy)]
            while (cy, cx) in came_from:
                cy, cx = came_from[(cy, cx)]
                path.append((cx, cy))
            path.reverse()
            return [grid_to_world(px, py, grid_msg) for px, py in path]
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            ny, nx = cy + dy, cx + dx
            if not (0 <= ny < h and 0 <= nx < w):
                continue
            if grid[ny, nx] >= 65:
                continue
            cost = 1.414 if dx != 0 and dy != 0 else 1.0
            new_g = g_score.get((cy, cx), float('inf')) + cost
            if new_g < g_score.get((ny, nx), float('inf')):
                came_from[(ny, nx)] = (cy, cx)
                g_score[(ny, nx)] = new_g
                open_set.append((new_g + _heuristic(nx, ny, gx, gy), ny, nx))
    return []


def _heuristic(x1, y1, x2, y2):
    dx, dy = abs(x1 - x2), abs(y1 - y2)
    return max(dx, dy) + (1.414 - 1.0) * min(dx, dy)
