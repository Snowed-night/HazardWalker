"""Frontier 探索目标选择模块。

所属组：导航组。
文件作用：
- 提供不依赖 ROS 的 Frontier 探索目标选择算法。
- 从 OccupancyGrid 地图中提取前沿点（自由空间与未知空间的边界），
  聚类后按信息增益和距离代价选择最佳探索目标。
- 作为离线纯函数模块，可被 ROS 节点直接调用，也可通过离线测试独立验证。

当前函数职责：
- `find_frontier_cells`：扫描栅格地图，标记所有前沿单元格。
- `cluster_frontiers`：用 BFS 连通域聚类前沿单元格。
- `select_exploration_target`：综合信息增益和距离代价，返回最佳探索目标。

后续扩展方式：
- 目前使用欧氏距离估计代价，后续可接 A* 或 Nav2 路径规划替换。
- 目前信息增益仅统计前沿簇邻接的未知格数量，后续可扩展为可见视场覆盖率。
- 不可达代价目前用直线穿过障碍物检测，后续可接入真实路径规划结果。
- 可新增 `GoalProvider` 抽象基类，让 Waypoint 和 Frontier 统一接口，
  详见 `docs/groups/nav/Frontier_Nav2_接入方案.md`。

验证方式：
- 用 `tests/offline/test_frontier_explorer.py` 验证前沿检测、聚类、
  评分排序和边界条件（空地图、无障碍物、全部已知等）。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field


@dataclass
class FrontierGoal:
    """Frontier 探索目标选择结果。

    Attributes:
        target_x: 目标点 x 坐标（栅格坐标系，列索引）。
        target_y: 目标点 y 坐标（栅格坐标系，行索引）。
        score: 综合评分，越大越优先。
        info_gain: 该前沿簇的信息增益（邻接未知格数量）。
        distance: 机器人到目标点的直线距离（栅格单位）。
        cluster_size: 该前沿簇包含的前沿单元格数量。
        reason: 目标选择理由，便于调试和记录。
    """

    target_x: float
    target_y: float
    score: float
    info_gain: int
    distance: float
    cluster_size: int
    reason: str = ""


@dataclass
class FrontierCluster:
    """单个前沿簇的内部表示。

    Attributes:
        cells: 属于该簇的前沿单元格列表，每个元素为 (row, col)。
        centroid_x: 簇的列坐标均值。
        centroid_y: 簇的行坐标均值。
        info_gain: 该簇邻接的未知格数量（去重后）。
        size: 簇包含的前沿单元格数量。
    """

    cells: list = field(default_factory=list)
    centroid_x: float = 0.0
    centroid_y: float = 0.0
    info_gain: int = 0
    size: int = 0


# ---- 内部辅助函数 ----

def _is_valid(grid: list, row: int, col: int) -> bool:
    """判断 (row, col) 是否在栅格地图内。"""
    if not grid:
        return False
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0
    return 0 <= row < rows and 0 <= col < cols


def _is_frontier(grid: list, row: int, col: int) -> bool:
    """判断某个单元格是否为前沿单元格。

    前沿单元格定义：自身为自由格（值为 0），且至少有一个
    4-邻接或 8-邻接单元格为未知格（值为 -1）。
    使用 8-邻接可获得更完整的前沿边界。
    """
    if not _is_valid(grid, row, col):
        return False
    if grid[row][col] != 0:
        return False

    # 8-邻接方向
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            nr, nc = row + dr, col + dc
            if not _is_valid(grid, nr, nc):
                continue
            if grid[nr][nc] == -1:
                return True
    return False


def _line_passes_obstacle(
    grid: list, r0: float, c0: float, r1: float, c1: float
) -> bool:
    """检查从 (r0, c0) 到 (r1, c1) 的直线是否穿过障碍物。

    使用 Bresenham 风格的采样，按步长 1 格进行检测。
    若途中任一单元格值为 100（障碍物），返回 True。

    Args:
        grid: 二维栅格地图。
        r0, c0: 起点（栅格坐标，浮点）。
        r1, c1: 终点（栅格坐标，浮点）。

    Returns:
        True 表示直线穿过了障碍物。
    """
    if not grid:
        return False
    dr = r1 - r0
    dc = c1 - c0
    dist = math.hypot(dr, dc)
    if dist < 1e-6:
        return False
    steps = max(1, int(dist))
    for i in range(steps + 1):
        t = i / steps
        row = int(round(r0 + dr * t))
        col = int(round(c0 + dc * t))
        if _is_valid(grid, row, col) and grid[row][col] >= 100:
            return True
    return False


# ---- 公开接口 ----

def find_frontier_cells(grid: list) -> list:
    """扫描整个栅格地图，返回所有前沿单元格的坐标列表。

    前沿单元格定义：值为 0（自由空间），且 8-邻接至少有一格值为 -1（未知）。

    Args:
        grid: 二维栅格地图，每一行为一个 list，元素取值为：
              -1（未知）、0（自由）、1-100（障碍物概率，>=100 视为障碍物）。

    Returns:
        List[tuple[int, int]]，每个元素为 (row, col) 即前沿单元格的栅格坐标。
        若无前沿单元格，返回空列表。

    Example:
        >>> grid = [[-1, 0], [0, -1]]
        >>> cells = find_frontier_cells(grid)
        >>> len(cells)
        4
    """
    if not grid:
        return []
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0
    frontiers = []
    for r in range(rows):
        for c in range(cols):
            if _is_frontier(grid, r, c):
                frontiers.append((r, c))
    return frontiers


def cluster_frontiers(
    frontier_cells: list,
    min_cluster_size: int = 5,
) -> list:
    """用 8-邻接 BFS 连通域算法对前沿单元格进行聚类。

    每个连通域形成一个 FrontierCluster，根据 min_cluster_size 过滤小簇。

    Args:
        frontier_cells: find_frontier_cells 返回的前沿单元格列表。
        min_cluster_size: 最小簇大小（包含的前沿单元格数量），
                          小于此值的簇会被丢弃。

    Returns:
        List[FrontierCluster]，按簇大小降序排列。每个簇包含质心坐标、
        单元格列表和大小。空列表表示没有满足条件的簇。
    """
    if not frontier_cells:
        return []

    # 用 set 做快速查找和去重
    frontier_set = set(frontier_cells)
    visited = set()
    clusters = []

    for start_r, start_c in frontier_cells:
        if (start_r, start_c) in visited:
            continue

        # BFS 收集同一连通域的所有前沿单元格
        queue = deque()
        queue.append((start_r, start_c))
        visited.add((start_r, start_c))
        cluster_cells = []

        while queue:
            r, c = queue.popleft()
            cluster_cells.append((r, c))

            # 8-邻接扩展
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    if (nr, nc) in visited:
                        continue
                    if (nr, nc) in frontier_set:
                        visited.add((nr, nc))
                        queue.append((nr, nc))

        # 过滤过小的簇
        if len(cluster_cells) < min_cluster_size:
            continue

        # 计算质心
        sum_r = sum(c[0] for c in cluster_cells)
        sum_c = sum(c[1] for c in cluster_cells)
        n = len(cluster_cells)
        centroid_r = sum_r / n
        centroid_c = sum_c / n

        clusters.append(
            FrontierCluster(
                cells=cluster_cells,
                centroid_x=centroid_c,
                centroid_y=centroid_r,
                size=n,
            )
        )

    # 按簇大小降序排列，方便调试和日志输出
    clusters.sort(key=lambda cl: cl.size, reverse=True)
    return clusters


def compute_info_gain(grid: list, cluster: FrontierCluster) -> int:
    """计算一个前沿簇的信息增益。

    信息增益定义为该簇邻接的未知格（值为 -1）数量（去重）。
    这近似表示"前往这个前沿可以观察到多少新区域"。

    Args:
        grid: 二维栅格地图。
        cluster: 已聚类的前沿簇（需包含 cells 列表）。

    Returns:
        去重后的邻接未知格数量。
    """
    if not grid or not cluster.cells:
        return 0

    unknown_neighbors = set()
    for r, c in cluster.cells:
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if _is_valid(grid, nr, nc) and grid[nr][nc] == -1:
                    unknown_neighbors.add((nr, nc))
    return len(unknown_neighbors)


def select_exploration_target(
    grid: list,
    robot_x: float,
    robot_y: float,
    min_frontier_size: int = 5,
    information_gain_weight: float = 1.0,
    distance_cost_weight: float = 0.5,
    unreachable_penalty: float = 10.0,
    visited_areas: set | None = None,
    visited_novelty_weight: float = 0.3,
) -> FrontierGoal | None:
    """从栅格地图中选择最佳探索目标点。

    完整流程：
    1. 扫描地图找到所有前沿单元格。
    2. 聚类前沿单元格，过滤小簇。
    3. 对每个簇计算信息增益（邻接未知格数量）。
    4. 对每个簇计算距离代价（机器人到质心的直线距离）。
    5. 对每个簇检测不可达代价（直线是否穿过障碍物）。
    6. 若提供 visited_areas，计算新颖性奖励（质心到最近已访问格的距离）。
    7. 按 score = info_gain * w_info - distance * w_dist
       + novelty * w_novelty - unreachable_penalty
       综合评分，返回得分最高的目标。

    Args:
        grid: 二维栅格地图，格式同 `find_frontier_cells`。
        robot_x: 机器人当前 x 坐标（栅格坐标系，列索引）。
        robot_y: 机器人当前 y 坐标（栅格坐标系，行索引）。
        min_frontier_size: 最小前沿簇大小，小于此值的簇被忽略。
        information_gain_weight: 信息增益权重，越高越优先探索大未知区域。
        distance_cost_weight: 距离代价权重，越高越倾向选择近处目标。
        unreachable_penalty: 不可达惩罚值，当目标在障碍物后方时从评分中扣除。
        visited_areas: 已访问单元格集合，元素为 (row, col) 栅格坐标。
                       用于计算新颖性奖励，倾向选择远离已访问区域的前沿。
                       传 None 时跳过新颖性计算，等价于无已访问区域。
        visited_novelty_weight: 新颖性奖励权重，越高越倾向探索未涉足区域。

    Returns:
        FrontierGoal 或 None。None 表示未找到任何有效探索目标
        （地图中无前沿或无满足最小大小的簇），此时探索可终止。

    Example:
        >>> grid = [[-1, -1, -1], [0, 0, 0], [100, 100, 100]]
        >>> goal = select_exploration_target(grid, 0.0, 1.0)
        >>> goal is not None
        True
        >>> goal.target_y  # 第一行（row 0）的前沿应被选中
        0.0
    """
    if not grid:
        return None

    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0
    if rows == 0 or cols == 0:
        return None

    # 第一步：找出所有前沿单元格
    frontier_cells = find_frontier_cells(grid)
    if not frontier_cells:
        return None

    # 第二步：聚类并过滤
    clusters = cluster_frontiers(frontier_cells, min_frontier_size)
    if not clusters:
        return None

    # 第三步：对每个簇计算信息增益
    for cluster in clusters:
        cluster.info_gain = compute_info_gain(grid, cluster)

    # 第四、五、六步：评分
    best_goal = None
    best_score = float("-inf")
    has_visited = visited_areas is not None and len(visited_areas) > 0

    for cluster in clusters:
        # 距离代价（欧氏距离）
        dx = cluster.centroid_x - robot_x
        dy = cluster.centroid_y - robot_y
        distance = math.hypot(dx, dy)

        # 基础评分
        score = (
            cluster.info_gain * information_gain_weight
            - distance * distance_cost_weight
        )

        # 新颖性奖励：质心到最近已访问格的直线距离
        novelty_bonus = 0.0
        if has_visited:
            min_visited_dist = float("inf")
            c_col = cluster.centroid_x
            c_row = cluster.centroid_y
            for vr, vc in visited_areas:
                d = math.hypot(c_col - vc, c_row - vr)
                if d < min_visited_dist:
                    min_visited_dist = d
            if min_visited_dist != float("inf"):
                novelty_bonus = min_visited_dist * visited_novelty_weight
            score += novelty_bonus

        # 不可达检测：直线穿过障碍物则施加惩罚
        unreachable = _line_passes_obstacle(
            grid, robot_y, robot_x, cluster.centroid_y, cluster.centroid_x
        )
        if unreachable:
            score -= unreachable_penalty

        if score > best_score:
            best_score = score
            # 构建选择理由
            parts = [
                f"信息增益={cluster.info_gain}",
                f"距离={distance:.1f}格",
                f"簇大小={cluster.size}",
            ]
            if has_visited:
                parts.append(f"新颖性={novelty_bonus:.1f}")
            if unreachable:
                parts.append("直线路径有障碍")
            reason = "；".join(parts)

            best_goal = FrontierGoal(
                target_x=cluster.centroid_x,
                target_y=cluster.centroid_y,
                score=score,
                info_gain=cluster.info_gain,
                distance=distance,
                cluster_size=cluster.size,
                reason=reason,
            )

    return best_goal
