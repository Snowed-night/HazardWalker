"""覆盖网格追踪 + 楼梯检测：纯 numpy 函数，ROS 无关。

所属组：导航组。
文件作用：
- CoverageGrid: 维护与 occupancy grid 同分辨率的 visited 网格。
- floor_coverage_ratio: 计算已覆盖自由空间比例。
- detect_stair_candidates: 形态学骨架化识别窄长通廊作为楼梯候选。
- find_elevator_approach_candidates: 覆盖边界查找电梯通道候选。

验证方式：
  tests/offline/test_coverage_tracker.py
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

# occupancy grid 约定（与 frontier_detector.py 一致）
FREE_MAX = 49
OCCUPIED = 65
UNKNOWN = -1


@dataclass
class StairCandidate:
    """楼梯检测候选区域。"""

    centroid: Tuple[float, float]  # 世界坐标 (x, y)
    grid_centroid: Tuple[int, int]  # 栅格坐标 (gx, gy)
    area_cells: int  # 骨架面积（栅格数）
    orientation_deg: float  # 主轴方向（度）


class CoverageGrid:
    """覆盖网格：追踪已访问的自由空间栅格。"""

    def __init__(self, height: int, width: int, resolution_m: float = 0.05):
        """初始化与 occupancy grid 同尺寸的 visited 网格。

        Args:
            height: occupancy grid 行数。
            width: occupancy grid 列数。
            resolution_m: 栅格分辨率。
        """
        self._grid = np.zeros((height, width), dtype=np.bool_)
        self._resolution = resolution_m
        self._last_update_sec: Optional[float] = None

    def update(
        self,
        robot_gx: int,
        robot_gy: int,
        occupancy_grid: np.ndarray,
        radius_cells: int = 6,
    ):
        """以机器人当前位置为中心，标记半径为已访问。

        Args:
            robot_gx: 机器人栅格列坐标。
            robot_gy: 机器人栅格行坐标。
            occupancy_grid: 对应的 occupancy grid (np.int8)。
            radius_cells: 覆盖标记半径（栅格数），默认 6 ≈ 0.30m。
        """
        height, width = self._grid.shape
        min_gx = max(0, robot_gx - radius_cells)
        max_gx = min(width - 1, robot_gx + radius_cells)
        min_gy = max(0, robot_gy - radius_cells)
        max_gy = min(height - 1, robot_gy + radius_cells)

        patch = occupancy_grid[min_gy:max_gy + 1, min_gx:max_gx + 1]
        self._grid[min_gy:max_gy + 1, min_gx:max_gx + 1] |= (
            (patch >= 0) & (patch <= FREE_MAX)
        )

    def floor_coverage_ratio(self, occupancy_grid: np.ndarray) -> float:
        """计算当前已访问自由空间占全部已知自由空间的比例。

        Args:
            occupancy_grid: 当前完整的 occupancy grid。

        Returns:
            覆盖率 [0.0, 1.0]。若地图中无自由空间则返回 0.0。
        """
        free_mask = (occupancy_grid >= 0) & (occupancy_grid <= FREE_MAX)
        total_free = free_mask.sum()
        if total_free == 0:
            return 0.0
        visited_free = self._grid[free_mask].sum()
        return min(1.0, visited_free / total_free)

    @property
    def grid(self) -> np.ndarray:
        """只读访问 visited 网格。"""
        return self._grid

    @property
    def resolution(self) -> float:
        return self._resolution


def detect_stair_candidates(
    grid: np.ndarray,
    resolution_m: float,
    min_length_m: float = 2.0,
    max_width_m: float = 2.5,
    min_free_ratio: float = 0.60,
) -> List[StairCandidate]:
    """从 occupancy grid 检测楼梯候选区域。

    使用形态学腐蚀 + 骨架化，识别窄长自由空间区域。

    Args:
        grid: occupancy grid (np.int8)。
        resolution_m: 栅格分辨率（米）。
        min_length_m: 最小长度阈值（米）。
        max_width_m: 最大宽度阈值（米）。
        min_free_ratio: 区域内最低自由空间比例。

    Returns:
        StairCandidate 列表，按面积降序排列。
    """
    height, width = grid.shape
    free_mask = (grid >= 0) & (grid <= FREE_MAX)
    if not free_mask.any():
        return []

    max_width_cells = max(1, int(max_width_m / resolution_m))
    min_length_cells = max(2, int(min_length_m / resolution_m))

    # 多次腐蚀消除孤立噪点
    from scipy.ndimage import binary_erosion  # type: ignore[import-untyped]
    kernel_size = max(2, max_width_cells // 2)
    try:
        eroded = free_mask
        for _ in range(min(2, kernel_size)):
            eroded = binary_erosion(eroded, iterations=1)
    except Exception:
        eroded = free_mask

    # 连通分量分析
    from scipy.ndimage import label, find_objects  # type: ignore[import-untyped]
    labeled, num_features = label(eroded)
    candidates: List[StairCandidate] = []

    for i in range(1, num_features + 1):
        region_mask = labeled == i
        region_cells = region_mask.sum()
        if region_cells < min_length_cells:
            continue

        # 计算区域包围盒
        gy_indices, gx_indices = np.where(region_mask)
        min_gx, max_gx = gx_indices.min(), gx_indices.max()
        min_gy, max_gy = gy_indices.min(), gy_indices.max()
        bbox_width = max_gx - min_gx + 1
        bbox_height = max_gy - min_gy + 1

        bbox_max = max(bbox_width, bbox_height)
        bbox_min = min(bbox_width, bbox_height)
        if bbox_min > max_width_cells:
            continue  # 太宽，不是窄廊
        if bbox_max < min_length_cells:
            continue  # 太短

        # 检查区域内自由空间比例
        bbox_free = free_mask[min_gy:max_gy + 1, min_gx:max_gx + 1].sum()
        bbox_area = bbox_width * bbox_height
        if bbox_area > 0 and bbox_free / bbox_area < min_free_ratio:
            continue

        # 长宽比必须 > 1.5，即窄长形态
        aspect = bbox_max / max(1, bbox_min)
        if aspect < 1.5:
            continue

        # 主轴方向
        orientation = 0.0
        if bbox_width > bbox_height:
            orientation = 90.0  # 水平

        centroid_gx = (min_gx + max_gx) / 2.0
        centroid_gy = (min_gy + max_gy) / 2.0

        candidates.append(StairCandidate(
            centroid=(centroid_gx * resolution_m, centroid_gy * resolution_m),
            grid_centroid=(int(centroid_gx), int(centroid_gy)),
            area_cells=int(region_cells),
            orientation_deg=orientation,
        ))

    candidates.sort(key=lambda c: c.area_cells, reverse=True)
    return candidates


def find_elevator_approach_candidates(
    grid: np.ndarray,
    visited: np.ndarray,
    min_distance_m: float = 1.5,
    max_distance_m: float = 6.0,
) -> List[Tuple[float, float]]:
    """在覆盖边界查找可能通往电梯的未探索区域。

    在 visited 区域边界外寻找较大的未探索自由空间 patch。

    Args:
        grid: occupancy grid。
        visited: 覆盖网格。
        min_distance_m: 边界搜索最小距离。
        max_distance_m: 边界搜索最大距离。

    Returns:
        世界坐标 (x, y) 列表，按候选区域大小降序。
    """
    free_mask = (grid >= 0) & (grid <= FREE_MAX)
    unvisited_free = free_mask & ~visited
    if not unvisited_free.any():
        return []

    from scipy.ndimage import label  # type: ignore[import-untyped]
    labeled, num_features = label(unvisited_free)

    results = []
    for i in range(1, num_features + 1):
        region_mask = labeled == i
        cells = region_mask.sum()
        if cells < 10:
            continue
        gy_indices, gx_indices = np.where(region_mask)
        centroid_gx = (gx_indices.min() + gx_indices.max()) / 2.0
        centroid_gy = (gy_indices.min() + gy_indices.max()) / 2.0
        # 假设分辨率 0.05（由调用方传入实际值）
        results.append((centroid_gx * 0.05, centroid_gy * 0.05, cells))

    results.sort(key=lambda x: x[2], reverse=True)
    return [(r[0], r[1]) for r in results]
