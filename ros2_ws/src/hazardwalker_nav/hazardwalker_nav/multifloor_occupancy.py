"""分层二维占用栅格纯算法：Bresenham 光线与有界 log-odds 更新。"""

from __future__ import annotations

import math

import numpy as np


def bresenham_cells(x0: int, y0: int, x1: int, y1: int):
    """返回包含首尾端点的整数栅格直线。"""

    x0, y0, x1, y1 = map(int, (x0, y0, x1, y1))
    cells = []
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return cells
        twice_error = 2 * error
        if twice_error >= dy:
            error += dy
            x0 += sx
        if twice_error <= dx:
            error += dx
            y0 += sy


def update_log_odds_ray(
        scores: np.ndarray,
        seen: np.ndarray,
        start_cell,
        end_cell,
        *,
        endpoint_is_hit: bool,
        free_delta: int = -1,
        hit_delta: int = 3,
        minimum_score: int = -20,
        maximum_score: int = 20) -> bool:
    """把一条激光射线写入分层地图；越界返回 False 且不写部分结果。"""

    if scores.shape != seen.shape or scores.ndim != 2:
        raise ValueError('scores/seen 必须是同形二维数组')
    cells = bresenham_cells(*start_cell, *end_cell)
    height, width = scores.shape
    if any(not (0 <= x < width and 0 <= y < height) for x, y in cells):
        return False
    free_cells = cells[:-1] if endpoint_is_hit else cells
    for x, y in free_cells:
        seen[y, x] = True
        scores[y, x] = max(minimum_score, int(scores[y, x]) + free_delta)
    if endpoint_is_hit:
        x, y = cells[-1]
        seen[y, x] = True
        scores[y, x] = min(maximum_score, int(scores[y, x]) + hit_delta)
    return True


def grid_to_occupancy(
        scores: np.ndarray,
        seen: np.ndarray,
        occupied_score: int = 3) -> np.ndarray:
    """转换为 ROS OccupancyGrid 约定的 -1/0/100。"""

    if scores.shape != seen.shape or scores.ndim != 2:
        raise ValueError('scores/seen 必须是同形二维数组')
    result = np.full(scores.shape, -1, dtype=np.int8)
    result[seen] = 0
    result[seen & (scores >= int(occupied_score))] = 100
    return result


def world_to_cell(x: float, y: float, origin_x: float, origin_y: float,
                  resolution_m: float):
    if (not all(math.isfinite(value) for value in (
            x, y, origin_x, origin_y, resolution_m))
            or resolution_m <= 0.0):
        raise ValueError('世界坐标或分辨率无效')
    return (
        int(math.floor((x - origin_x) / resolution_m)),
        int(math.floor((y - origin_y) / resolution_m)),
    )
