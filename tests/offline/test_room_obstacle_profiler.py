"""房内障碍物簇提取与环绕观察点规划的离线回归测试。

所属组：导航组。
验证 extract_room_mask / extract_room_obstacles / plan_obstacle_viewpoints，
构造合成 occupancy（墙围矩形房 + 门洞 + 房内独立障碍），不依赖 ROS2。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
NAV_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav'
if str(NAV_SRC) not in sys.path:
    sys.path.insert(0, str(NAV_SRC))

from hazardwalker_nav.room_obstacle_profiler import (  # noqa: E402
    OCCUPIED_THRESHOLD,
    FREE_MAX,
    extract_room_mask,
    extract_room_obstacles,
    plan_obstacle_viewpoints,
)

# cell 布局（res=1.0）：
#   - 房间内部 free:  x in [5, 14], y in [5, 14]
#   - 左墙: x == 4（除门洞 y in {9, 10} 为 0）
#   - 走廊 free: x in [0, 3]
ROOM_X0, ROOM_X1 = 5, 14
ROOM_Y0, ROOM_Y1 = 5, 14
DOOR_GY0, DOOR_GY1 = 9, 10


def _room_grid():
    grid = np.zeros((20, 20), dtype=np.int8)
    # 房间内部 free（0）
    grid[ROOM_Y0:ROOM_Y1 + 1, ROOM_X0:ROOM_X1 + 1] = 0
    # 左墙（含门洞外的走廊墙）
    grid[0:20, 4] = OCCUPIED_THRESHOLD
    grid[DOOR_GY0:DOOR_GY1 + 1, 4] = 0  # 门洞
    # 上墙下墙右墙（把房间围起来）
    grid[ROOM_Y1 + 1, ROOM_X0 - 1:ROOM_X1 + 2] = OCCUPIED_THRESHOLD
    grid[ROOM_Y0 - 1, ROOM_X0 - 1:ROOM_X1 + 2] = OCCUPIED_THRESHOLD
    grid[ROOM_Y0 - 1:ROOM_Y1 + 2, ROOM_X1 + 1] = OCCUPIED_THRESHOLD
    # 房间上方/下方的走廊墙沿保持隔断（简化：直接让房外非走廊区为墙）
    grid[0:ROOM_Y0, ROOM_X0:ROOM_X1 + 1] = OCCUPIED_THRESHOLD
    grid[ROOM_Y1 + 1:20, ROOM_X0:ROOM_X1 + 1] = OCCUPIED_THRESHOLD
    # 走廊 free（0..3 列）
    grid[0:20, 0:4] = 0
    return grid


def _grid_message(grid, resolution=1.0):
    origin = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0),
    )
    info = SimpleNamespace(
        width=grid.shape[1],
        height=grid.shape[0],
        resolution=resolution,
        origin=origin,
    )
    return SimpleNamespace(info=info, data=grid.reshape(-1).tolist())


def _add_obstacle(grid, gx, gy, cells=1, res=1.0):
    """在房内 (gx, gy) 放一个 cells*cells 的方块障碍。"""
    grid[gy:gy + cells, gx:gx + cells] = OCCUPIED_THRESHOLD


def _door_entry_world():
    # 门洞中心，稍偏走廊侧（世界坐标）
    return (float(ROOM_X0 - 1), (DOOR_GY0 + DOOR_GY1 + 1) / 2.0)


def _setup(obstacles=()):
    grid = _room_grid()
    for (gx, gy, cells) in obstacles:
        _add_obstacle(grid, gx, gy, cells=cells)
    msg = _grid_message(grid)
    return grid, msg


# 合成房约 80~100 free 格（res=1.0），最小房阈值取 20 即可区分"无房"。
_MIN_ROOM_CELLS = 20


def _room_mask(grid, msg):
    return extract_room_mask(
        grid, msg, *_door_entry_world(), entry_yaw=0.0,
        door_width_m=2.0, seed_offset_m=1.5,
        min_room_free_cells=_MIN_ROOM_CELLS,
    )


# ---- 合成四障碍房间（对应远处两房：四角离墙 2-3m 各一） ----
FOUR_CORNERS = (
    (ROOM_X0 + 2, ROOM_Y0 + 2, 2),   # 左下
    (ROOM_X1 - 3, ROOM_Y0 + 2, 2),   # 右下
    (ROOM_X0 + 2, ROOM_Y1 - 3, 2),   # 左上
    (ROOM_X1 - 3, ROOM_Y1 - 3, 2),   # 右上
)


def test_room_mask_does_not_leak_into_corridor():
    grid, msg = _setup(FOUR_CORNERS)
    mask = _room_mask(grid, msg)
    assert mask is not None
    # 走廊区域不应被判为房间内部
    assert not mask[:, 0:3].any()
    # 房间内部应有足够 free（整房 free 约 100，4 个 2x2 障碍占 16）
    assert mask.sum() >= 80


def test_inside_half_plane_blocks_wide_open_doorway_from_rejoining_corridor():
    grid = np.zeros((20, 30), dtype=np.int8)
    msg = _grid_message(grid, resolution=0.5)
    mask = extract_room_mask(
        grid,
        msg,
        entry_wx=5.0,
        entry_wy=5.0,
        entry_yaw=0.0,
        door_width_m=4.0,
        seed_offset_m=1.0,
        min_room_free_cells=20,
        restrict_to_inside_half_plane=True,
    )
    assert mask is not None
    # 即使占据图没有形成窄门，真实穿门方向也能排除门外半平面。
    # 默认保留门外 0.25 m 数值裕量，因此第 9 列边界格可以保留。
    assert not mask[:, :9].any()
    assert mask[:, 12:].any()


def test_neighbor_door_voronoi_partition_blocks_cross_room_slam_gap():
    grid = np.zeros((30, 30), dtype=np.int8)
    msg = _grid_message(grid, resolution=0.5)
    mask = extract_room_mask(
        grid,
        msg,
        entry_wx=5.0,
        entry_wy=5.0,
        entry_yaw=0.0,
        seed_offset_m=1.0,
        min_room_free_cells=20,
        restrict_to_inside_half_plane=True,
        neighbor_entries_world=[(5.0, 12.0)],
    )
    assert mask is not None
    # 两门中垂线 y=8.5；当前房间不能泄漏到更靠近邻门的上半区。
    assert not mask[18:, :].any()
    assert mask[:16, 12:].any()


def test_mirrored_neighbor_doors_bound_an_end_room_on_both_sides():
    grid = np.zeros((40, 30), dtype=np.int8)
    msg = _grid_message(grid, resolution=0.5)
    mask = extract_room_mask(
        grid,
        msg,
        entry_wx=5.0,
        entry_wy=10.0,
        entry_yaw=0.0,
        seed_offset_m=1.0,
        min_room_free_cells=20,
        restrict_to_inside_half_plane=True,
        # 实际邻门 y=16；镜像邻门 y=4 由两门间距推导。
        neighbor_entries_world=[(5.0, 16.0), (5.0, 4.0)],
    )
    assert mask is not None
    # 两条中垂线 y=7 与 y=13 把端部入口限定在自己的拓扑房间内。
    assert not mask[:14, :].any()
    assert mask[15:26, 12:].any()
    assert not mask[27:, :].any()


def test_room_mask_too_small_returns_none():
    # 几乎封闭无空间 → None
    grid = np.full((10, 10), OCCUPIED_THRESHOLD, dtype=np.int8)
    grid[4:6, 4:6] = 0
    msg = _grid_message(grid)
    mask = extract_room_mask(
        grid, msg, 5.0, 5.0, 0.0, door_width_m=1.0,
        min_room_free_cells=_MIN_ROOM_CELLS,
    )
    assert mask is None


def test_extract_four_corner_obstacles():
    grid, msg = _setup(FOUR_CORNERS)
    mask = _room_mask(grid, msg)
    assert mask is not None
    clusters = extract_room_obstacles(grid, mask, msg, min_area_m2=0.5)
    assert len(clusters) == 4
    areas = sorted(round(c.area_m2, 1) for c in clusters)
    # 每个 2x2 障碍 = 4 m²（res=1.0）
    assert areas == [4.0, 4.0, 4.0, 4.0]
    for c in clusters:
        gx, gy = c.grid_centroid
        assert ROOM_X0 + 1 <= gx <= ROOM_X1 - 1
        assert ROOM_Y0 + 1 <= gy <= ROOM_Y1 - 1


def test_extract_five_obstacles_with_center():
    # 房间中心 (9, 9)，与四个角障碍（x7-8 与 x11-12）至少隔 1 格，避免 4 邻连通
    center_obs = (9, 9, 2)
    grid, msg = _setup(FOUR_CORNERS + (center_obs,))
    mask = _room_mask(grid, msg)
    assert mask is not None
    clusters = extract_room_obstacles(grid, mask, msg, min_area_m2=0.5)
    assert len(clusters) == 5


def test_extract_excludes_walls():
    """墙贴 bbox 边缘，不应被计为房内障碍。"""
    grid, msg = _setup(())
    mask = _room_mask(grid, msg)
    assert mask is not None
    clusters = extract_room_obstacles(grid, mask, msg, min_area_m2=0.5)
    assert clusters == []


def test_viewpoints_are_free_inside_room_and_face_centroid():
    grid, msg = _setup(FOUR_CORNERS)
    mask = _room_mask(grid, msg)
    clusters = extract_room_obstacles(grid, mask, msg, min_area_m2=0.5)
    total = 0
    for c in clusters:
        vps = plan_obstacle_viewpoints(
            grid, msg, c, mask, count=6, standoff_m=1.0,
            clearance_m=0.0,
        )
        assert len(vps) >= 4, f'obstacle {c.grid_centroid} too few vps: {len(vps)}'
        for v in vps:
            assert 0 <= v.gx < grid.shape[1] and 0 <= v.gy < grid.shape[0]
            assert mask[v.gy, v.gx], f'vp ({v.gx},{v.gy}) not in room'
            assert grid[v.gy, v.gx] <= FREE_MAX, 'vp on occupied'
            # 机头朝向障碍质心
            expected = math.atan2(
                c.centroid[1] - v.wy, c.centroid[0] - v.wx)
            diff = abs(math.atan2(
                math.sin(v.face_yaw - expected),
                math.cos(v.face_yaw - expected)))
            assert diff < 1e-6
        total += len(vps)
    assert total >= 16  # 4 障碍各至少 4 点
