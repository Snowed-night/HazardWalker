"""房内障碍物簇提取与环绕观察点规划：纯 numpy 模块，ROS 无关。

所属组：导航组。
文件作用：
- 在已进入房间后，从 OccupancyGrid 中提取本房间内部的可达 free 区
  （``extract_room_mask``，用"虚拟门板"阻断 flood 回流走廊）。
- 从房内提取独立障碍物簇（桌椅柜架 = 红球可能的遮挡源），排除房间边界墙
  （``extract_room_obstacles``）。
- 为每个障碍簇规划环绕观察点（``plan_obstacle_viewpoints``），机器人逐点到访
  并朝向障碍质心，即可把每个障碍四周都看一遍，覆盖藏在障碍后的红球。

只依赖前端已经读到的 OccupancyGrid 概率值，不读取任何场景真值/房间布局。
occupancy 约定与 frontier_detector.py 保持一致：
  - free:   0..FREE_MAX(49)
  - occupied: >= OCCUPIED_THRESHOLD(65)
  - unknown: UNKNOWN(-1)

后续扩展方式：
- 障碍簇提取目前要求"距房间边界足够远"，对离墙摆放的家具（官方生成器按
  房型摆 4-6 件、距墙数米）稳定；若遇贴墙障碍需重新引入墙带-障碍分离。
- 环绕点目前按等分方位角生成，未考虑障碍长条形时主轴方向加密，可后续改进。

验证方式：
- 用 ``tests/offline/test_room_obstacle_profiler.py`` 构造合成 occupancy，
  验证房间不回流走廊、障碍计数精确、观察点全部落于 free。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

# 与 frontier_detector.py / coverage_tracker.py 一致的 occupancy 约定。
FREE_MAX = 49
OCCUPIED_THRESHOLD = 65  # grid >= 65 视为占用（墙 / 家具）
UNKNOWN = -1


@dataclass
class ObstacleCluster:
    """房内一个独立障碍物簇（可藏红球的家具）。"""

    centroid: Tuple[float, float]  # 世界坐标 (wx, wy)
    grid_centroid: Tuple[int, int]  # 栅格坐标 (gx, gy)
    radius_m: float  # 质心到最远簇点的距离（m），观察点绕此再留余量
    area_m2: float
    bbox: Tuple[int, int, int, int]  # (gx_min, gy_min, gx_max, gy_max)
    points: List[Tuple[int, int]] = field(default_factory=list)


@dataclass
class ViewPoint:
    """环绕障碍物的一个观察点：停到此点、机头转向 face_yaw 即可看到障碍一侧。"""

    wx: float
    wy: float
    face_yaw: float  # 指向障碍质心的朝向（rad）
    gx: int
    gy: int


def _world_to_grid(grid, grid_msg, wx: float, wy: float) -> Tuple[int, int]:
    origin = grid_msg.info.origin
    res = float(grid_msg.info.resolution)
    gx = int((wx - origin.position.x) / res)
    gy = int((wy - origin.position.y) / res)
    return gx, gy


def _grid_to_world(grid_msg, gx: int, gy: int) -> Tuple[float, float]:
    origin = grid_msg.info.origin
    res = float(grid_msg.info.resolution)
    return (
        origin.position.x + (gx + 0.5) * res,
        origin.position.y + (gy + 0.5) * res,
    )


def _free_mask(grid: np.ndarray) -> np.ndarray:
    """已知自由区掩码（0..FREE_MAX）。未知/占用均不可走。"""
    return (grid >= 0) & (grid <= FREE_MAX)


def _occupied_mask(grid: np.ndarray) -> np.ndarray:
    return grid >= OCCUPIED_THRESHOLD


def extract_room_mask(
    grid: np.ndarray,
    grid_msg,
    entry_wx: float,
    entry_wy: float,
    entry_yaw: float,
    door_width_m: float = 1.2,
    seed_offset_m: float = 0.5,
    door_margin_m: float = 0.25,
    min_room_free_cells: int = 80,
    restrict_to_inside_half_plane: bool = False,
) -> Optional[np.ndarray]:
    """估计房间内部可达 free 连通域，阻断经门洞回流走廊。

    原理：房间是墙围出的单连通 free 区，唯一开口是门洞。在入口 pose 处放一条
    垂直进入朝向、宽约门宽的"虚拟门板"，把这一段 free 从 walkable 中扣除后，
    从门口向房内推进 seed_offset_m 的种子做四邻 flood，得到房间内部 free 区。

    Returns:
        (h, w) bool 掩码（True=房间内部可达 free）；地图过小/种子失效返回 None，
        由调用方回退"入口 + room_half_extent_m 方框"。
    """
    h, w = grid.shape
    if h < 4 or w < 4 or float(grid_msg.info.resolution) <= 0.0:
        return None
    res = float(grid_msg.info.resolution)
    free = _free_mask(grid)
    walkable = free.copy()

    ux = np.cos(entry_yaw)
    uy = np.sin(entry_yaw)
    if restrict_to_inside_half_plane:
        # 激光占据图中的开放门洞经常比几何门宽更宽，短“门板”不足以切断
        # 走廊回流。穿门方向已经由实际运动观察得到时，门外半平面本就不属于
        # 当前房间，可直接从本次 flood 候选中排除；不修改原始占据图。
        cell_x = (
            float(grid_msg.info.origin.position.x)
            + (np.arange(w, dtype=float) + 0.5) * res)
        cell_y = (
            float(grid_msg.info.origin.position.y)
            + (np.arange(h, dtype=float) + 0.5) * res)
        world_x, world_y = np.meshgrid(cell_x, cell_y)
        signed_inside = (
            (world_x - float(entry_wx)) * ux
            + (world_y - float(entry_wy)) * uy)
        walkable[signed_inside < -max(0.0, float(door_margin_m))] = False

    # ---- 扣除虚拟门板 ----
    half_door = max(0.1, (door_width_m + door_margin_m) / 2.0)
    plate_depth = max(0.2, 2.0 * res)  # 沿进向的板厚（2 格）
    # 门板只会在入口附近，缩小遍历窗口。
    span = int(np.ceil((half_door + plate_depth) / res)) + 2
    cgx, cgy = _world_to_grid(grid, grid_msg, entry_wx, entry_wy)
    for dy in range(max(0, cgy - span), min(h, cgy + span + 1)):
        for dx in range(max(0, cgx - span), min(w, cgx + span + 1)):
            wx, wy = _grid_to_world(grid_msg, dx, dy)
            rx, ry = wx - entry_wx, wy - entry_wy
            along = abs(rx * ux + ry * uy)      # 沿进入朝向的投影
            across = abs(rx * -uy + ry * ux)    # 垂直朝向的投影
            if along <= plate_depth and across <= half_door:
                walkable[dy, dx] = False

    # ---- 找房内种子：沿进入朝向从入口推进 seed_offset_m ----
    seed_gx, seed_gy = _world_to_grid(
        grid, grid_msg,
        entry_wx + ux * seed_offset_m,
        entry_wy + uy * seed_offset_m,
    )
    if not (0 <= seed_gx < w and 0 <= seed_gy < h):
        return None
    if not walkable[seed_gy, seed_gx]:
        # 种子压到门板/墙/未知：沿进向再向外多找几步。
        steps = 12
        for i in range(1, steps + 1):
            sx, sy = _world_to_grid(
                grid, grid_msg,
                entry_wx + ux * (seed_offset_m + i * res * 2),
                entry_wy + uy * (seed_offset_m + i * res * 2),
            )
            if 0 <= sx < w and 0 <= sy < h and walkable[sy, sx]:
                seed_gx, seed_gy = sx, sy
                break
        else:
            return None

    # ---- 四邻 flood ----
    room = np.zeros((h, w), dtype=bool)
    stack = [(seed_gy, seed_gx)]
    room[seed_gy, seed_gx] = True
    while stack:
        y, x = stack.pop()
        for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
            if (0 <= ny < h and 0 <= nx < w
                    and walkable[ny, nx] and not room[ny, nx]):
                room[ny, nx] = True
                stack.append((ny, nx))

    if int(room.sum()) < min_room_free_cells:
        return None
    return room


def extract_room_obstacles(
    grid: np.ndarray,
    room_mask: np.ndarray,
    grid_msg,
    min_area_m2: float = 0.05,
    wall_margin_m: float = 0.5,
) -> List[ObstacleCluster]:
    """提取房间内的独立障碍物簇，排除房间边界墙。

    判据：官方房间为矩形，家具离墙数米摆放。把 room_mask 中距房间外接矩形四边
    不足 wall_margin_m 的占用当作墙带整片剔除；剩下的占用连通簇即房内家具。

    Returns:
        ObstacleCluster 列表（按面积降序）。
    """
    h, w = grid.shape
    res = float(grid_msg.info.resolution)
    if res <= 0.0 or room_mask is None or room_mask.shape != (h, w):
        return []
    occupied = _occupied_mask(grid)

    ys, xs = np.where(room_mask)
    if ys.size == 0:
        return []
    gx_min, gx_max = int(xs.min()), int(xs.max())
    gy_min, gy_max = int(ys.min()), int(ys.max())
    margin = max(1, int(np.ceil(wall_margin_m / res)))

    # 房间外接矩形向内缩 wall_margin 后的"内部区"：贴边的墙带不属于内部。
    ix0, ix1 = gx_min + margin, gx_max - margin
    iy0, iy1 = gy_min + margin, gy_max - margin
    if ix1 <= ix0 or iy1 <= iy0:
        return []
    interior = np.zeros((h, w), dtype=bool)
    interior[iy0:iy1 + 1, ix0:ix1 + 1] = occupied[
        iy0:iy1 + 1, ix0:ix1 + 1
    ]

    # 只对 interior 占用做连通域 BFS，避免把房外其他障碍算进来。
    visited = np.zeros((h, w), dtype=bool)
    min_cells = max(1, int(min_area_m2 / (res * res)))
    clusters: List[ObstacleCluster] = []

    for y in range(iy0, iy1 + 1):
        for x in range(ix0, ix1 + 1):
            if not interior[y, x] or visited[y, x]:
                continue
            # BFS 收集连通占用簇（4 邻）
            points: List[Tuple[int, int]] = []
            stack = [(y, x)]
            visited[y, x] = True
            while stack:
                cy, cx = stack.pop()
                points.append((cx, cy))
                for ny, nx in ((cy - 1, cx), (cy + 1, cx),
                               (cy, cx - 1), (cy, cx + 1)):
                    if (iy0 <= ny <= iy1 and ix0 <= nx <= ix1
                            and interior[ny, nx] and not visited[ny, nx]):
                        visited[ny, nx] = True
                        stack.append((ny, nx))
            if len(points) < min_cells:
                continue
            area_m2 = len(points) * res * res
            gxs = [p[0] for p in points]
            gys = [p[1] for p in points]
            cgx = sum(gxs) / len(gxs)
            cgy = sum(gys) / len(gys)
            # 质心取距算术中心最近的簇点，避免凹形簇中心悬空。
            nearest = min(
                points,
                key=lambda p: (p[0] - cgx) ** 2 + (p[1] - cgy) ** 2,
            )
            wx, wy = _grid_to_world(grid_msg, nearest[0], nearest[1])
            max_d2 = max(
                (p[0] - nearest[0]) ** 2 + (p[1] - nearest[1]) ** 2
                for p in points
            )
            radius_m = np.sqrt(max_d2) * res
            clusters.append(ObstacleCluster(
                centroid=(wx, wy),
                grid_centroid=(nearest[0], nearest[1]),
                radius_m=float(radius_m),
                area_m2=area_m2,
                bbox=(min(gxs), min(gys), max(gxs), max(gys)),
                points=points,
            ))

    clusters.sort(key=lambda c: c.area_m2, reverse=True)
    return clusters


def plan_obstacle_viewpoints(
    grid: np.ndarray,
    grid_msg,
    cluster: ObstacleCluster,
    room_mask: np.ndarray,
    count: int = 6,
    standoff_m: float = 0.5,
    clearance_m: float = 0.40,
) -> List[ViewPoint]:
    """为单个障碍簇生成环绕观察点。

    以簇质心为圆心等分 count 个方位角，半径 = 簇半径 + standoff_m；候选点需落在
    room_mask 内、本身是 free、且其 clearance_m 邻域内无占用（保证机体能停到点）。
    无法满足的方向（贴墙/出界/落进别的障碍）自动跳过。

    Returns:
        可用 ViewPoint 列表（角度升序）。
    """
    h, w = grid.shape
    res = float(grid_msg.info.resolution)
    if res <= 0.0 or room_mask is None or room_mask.shape != (h, w):
        return []
    if cluster.grid_centroid[0] < 0 or cluster.grid_centroid[1] < 0:
        return []
    if not (0 <= cluster.grid_centroid[0] < w
            and 0 <= cluster.grid_centroid[1] < h):
        return []
    occupied = _occupied_mask(grid)
    cx, cy = cluster.centroid
    radius = float(cluster.radius_m) + max(0.1, float(standoff_m))
    clearance_cells = (
        int(np.ceil(float(clearance_m) / res))
        if clearance_m > 0.0 else 0
    )
    num = max(4, int(count))
    viewpoints: List[ViewPoint] = []
    seen: List[Tuple[int, int]] = []

    for i in range(num):
        ang = 2.0 * np.pi * i / num
        gx, gy = _world_to_grid(
            grid, grid_msg,
            cx + np.cos(ang) * radius,
            cy + np.sin(ang) * radius,
        )
        if not (0 <= gx < w and 0 <= gy < h):
            continue
        if not room_mask[gy, gx]:
            continue
        if occupied[gy, gx] or grid[gy, gx] < 0:
            continue
        # 邻域净空：此点机体中心到点须可通行（clearance_m<=0 表示跳过检查，
        # 由调用方负责；合成测试避免 res 放大失真）
        if clearance_cells > 0:
            y0 = max(0, gy - clearance_cells)
            y1 = min(h, gy + clearance_cells + 1)
            x0 = max(0, gx - clearance_cells)
            x1 = min(w, gx + clearance_cells + 1)
            if bool(occupied[y0:y1, x0:x1].any()):
                continue
        if any((abs(gx - s[0]) < 2 and abs(gy - s[1]) < 2) for s in seen):
            continue
        wx, wy = _grid_to_world(grid_msg, gx, gy)
        # math.atan2 已返回 [-pi, pi]，无需再归一。
        face_yaw = math.atan2(cy - wy, cx - wx)
        seen.append((gx, gy))
        viewpoints.append(ViewPoint(
            wx=float(wx), wy=float(wy),
            face_yaw=float(face_yaw),
            gx=gx, gy=gy,
        ))
    return viewpoints
