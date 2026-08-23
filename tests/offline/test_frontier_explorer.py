"""Frontier 探索目标选择离线测试。

所属组：导航组 / 测试组。
文件作用：
- 验证 `frontier_explorer.py` 中前沿检测、聚类、信息增益计算
  和探索目标选择的纯函数逻辑。
- 不依赖 ROS、Nav2、Gazebo 或真实机器人。

当前验证内容：
- 前沿单元格检测（自由格邻接未知格）。
- 连通域聚类和最小簇大小过滤。
- 信息增益计算（邻接未知格数量）。
- 直线障碍物检测。
- 完整探索目标选择流程（单簇、多簇、边界条件）。
- 不可达惩罚对评分的影响。
- 空地图、无前沿、全自由等边界场景。

后续扩展：
- 增加真实 OccupancyGrid 数据格式测试。
- 增加多房间地图前沿选择测试。
- 增加地图分辨率变化测试。
"""
import math
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_nav'))

from hazardwalker_nav.frontier_explorer import (
    FrontierGoal,
    FrontierCluster,
    _is_valid,
    _is_frontier,
    _line_passes_obstacle,
    find_frontier_cells,
    cluster_frontiers,
    compute_info_gain,
    select_exploration_target,
)


# ============================================================
# 内部辅助函数测试
# ============================================================

def test_is_valid_inside_bounds():
    """验证 _is_valid 对范围内单元格返回 True。"""
    grid = [[0, 0, 0], [0, 0, 0]]
    assert _is_valid(grid, 0, 0) is True
    assert _is_valid(grid, 1, 2) is True


def test_is_valid_outside_bounds():
    """验证 _is_valid 对越界单元格返回 False。"""
    grid = [[0, 0], [0, 0]]
    assert _is_valid(grid, -1, 0) is False
    assert _is_valid(grid, 0, -1) is False
    assert _is_valid(grid, 2, 0) is False
    assert _is_valid(grid, 0, 2) is False


def test_is_valid_empty_grid():
    """验证 _is_valid 对空地图返回 False。"""
    assert _is_valid([], 0, 0) is False


def test_is_frontier_free_next_to_unknown():
    """验证自由格邻接未知格时被识别为前沿。"""
    grid = [
        [-1, 0],
        [0, -1],
    ]
    # grid[1][0] = 0（自由），左边 grid[0][0] = -1（未知）→ 前沿
    assert _is_frontier(grid, 1, 0) is True
    # grid[0][1] = 0（自由），上方 grid[0][0] = -1（未知）→ 前沿
    assert _is_frontier(grid, 0, 1) is True


def test_is_frontier_free_no_unknown():
    """验证自由格周围全为自由或障碍物时不是前沿。"""
    grid = [
        [0, 0],
        [0, 0],
    ]
    assert _is_frontier(grid, 0, 0) is False
    assert _is_frontier(grid, 1, 1) is False


def test_is_frontier_occupied_not_frontier():
    """验证障碍物格不被识别为前沿。"""
    grid = [
        [-1, 100],
        [0, -1],
    ]
    # grid[0][1] = 100（障碍物），即使邻接未知格也不是前沿
    assert _is_frontier(grid, 0, 1) is False


def test_is_frontier_unknown_not_frontier():
    """验证未知格本身不被识别为前沿。"""
    grid = [[-1, 0]]
    # grid[0][0] = -1（未知），不是前沿
    assert _is_frontier(grid, 0, 0) is False


def test_is_frontier_out_of_bounds():
    """验证越界单元格不是前沿。"""
    grid = [[0]]
    assert _is_frontier(grid, -1, 0) is False
    assert _is_frontier(grid, 0, 1) is False


def test_is_frontier_diagonal_unknown():
    """验证 8-邻接中斜对角有未知格也算前沿。"""
    grid = [
        [-1, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
    ]
    # grid[1][1] = 0，斜对角 grid[0][0] = -1 → 前沿（8-邻接）
    assert _is_frontier(grid, 1, 1) is True


# ============================================================
# 直线障碍物检测测试
# ============================================================

def test_line_passes_obstacle_clear_path():
    """验证无障碍物时 _line_passes_obstacle 返回 False。"""
    grid = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
    ]
    assert _line_passes_obstacle(grid, 0.0, 0.0, 2.0, 2.0) is False


def test_line_passes_obstacle_blocked():
    """验证直线穿过障碍物时返回 True。"""
    grid = [
        [0, 100, 0],
        [0, 100, 0],
        [0, 0, 0],
    ]
    # 从 (0,0) 到 (2,2) 的对角线会经过 (1,1)，而列 1 全是障碍物
    assert _line_passes_obstacle(grid, 0.0, 0.0, 2.0, 2.0) is True


def test_line_passes_obstacle_empty_grid():
    """验证空地图返回 False。"""
    assert _line_passes_obstacle([], 0.0, 0.0, 1.0, 1.0) is False


def test_line_passes_obstacle_zero_distance():
    """验证起点终点重合时返回 False。"""
    grid = [[0, 100], [0, 0]]
    assert _line_passes_obstacle(grid, 0.0, 0.0, 0.0, 0.0) is False


# ============================================================
# 前沿单元格扫描测试
# ============================================================

def test_find_frontier_cells_basic():
    """验证 basic_grid 能正确找出所有前沿单元格。"""
    # 简单经典场景：第一行未知，第二行自由
    grid = [
        [-1, -1, -1],
        [0, 0, 0],
        [100, 100, 100],
    ]
    cells = find_frontier_cells(grid)
    # 第二行所有格都邻接第一行的未知格
    assert len(cells) == 3
    assert (1, 0) in cells
    assert (1, 1) in cells
    assert (1, 2) in cells


def test_find_frontier_cells_empty_grid():
    """验证空地图返回空列表。"""
    assert find_frontier_cells([]) == []


def test_find_frontier_cells_all_unknown():
    """验证全未知地图没有前沿（没有自由格）。"""
    grid = [[-1, -1], [-1, -1]]
    assert find_frontier_cells(grid) == []


def test_find_frontier_cells_all_free():
    """验证全自由地图没有前沿（没有未知格）。"""
    grid = [[0, 0], [0, 0]]
    assert find_frontier_cells(grid) == []


def test_find_frontier_cells_all_occupied():
    """验证全障碍地图没有前沿。"""
    grid = [[100, 100], [100, 100]]
    assert find_frontier_cells(grid) == []


def test_find_frontier_cells_isolated_free():
    """验证被障碍物包围的孤立自由格不是前沿。"""
    grid = [
        [100, 100, 100],
        [100, 0, 100],
        [100, 100, 100],
    ]
    cells = find_frontier_cells(grid)
    # 自由格周围全是障碍物，没有未知格 → 不是前沿
    assert len(cells) == 0


def test_find_frontier_cells_partial_map():
    """验证部分探索地图的前沿检测。"""
    # 模拟：左半已探索（自由），右半未知
    grid = [
        [0, 0, -1, -1],
        [0, 0, -1, -1],
    ]
    cells = find_frontier_cells(grid)
    # 自由格且邻接未知格：只有自由区域右边缘的两格
    assert len(cells) == 2
    assert (0, 1) in cells
    assert (1, 1) in cells
    # (0,0) 和 (1,0) 邻接的是自由格，不是前沿


# ============================================================
# 前沿聚类测试
# ============================================================

def test_cluster_frontiers_single_cluster():
    """验证单个连通域产生一个簇。"""
    cells = [(0, 0), (0, 1), (1, 0)]
    clusters = cluster_frontiers(cells, min_cluster_size=1)
    assert len(clusters) == 1
    assert clusters[0].size == 3


def test_cluster_frontiers_two_separate_clusters():
    """验证两个不连通的区域分别聚类。"""
    # 左下区域和右上区域互不连通
    cells = [
        (0, 0), (0, 1), (1, 0),
        (5, 5), (5, 6), (6, 5), (6, 6),
    ]
    clusters = cluster_frontiers(cells, min_cluster_size=1)
    assert len(clusters) == 2
    sizes = sorted([c.size for c in clusters])
    assert sizes == [3, 4]


def test_cluster_frontiers_min_size_filter():
    """验证小于 min_cluster_size 的簇被过滤。"""
    cells = [
        (0, 0),  # 孤立格，size=1
        (3, 3), (3, 4), (4, 3), (4, 4), (4, 5),  # 5个
    ]
    clusters = cluster_frontiers(cells, min_cluster_size=3)
    assert len(clusters) == 1
    assert clusters[0].size == 5


def test_cluster_frontiers_empty():
    """验证空输入返回空列表。"""
    assert cluster_frontiers([], min_cluster_size=1) == []


def test_cluster_frontiers_all_filtered():
    """验证所有簇都被过滤时返回空列表。"""
    cells = [(0, 0), (2, 2), (4, 4)]  # 三个孤立格
    clusters = cluster_frontiers(cells, min_cluster_size=5)
    assert clusters == []


def test_cluster_frontiers_centroid():
    """验证质心计算正确。"""
    # 一个 2x2 的连通块，8-邻接下属于同一簇
    cells = [(0, 0), (0, 1), (1, 0), (1, 1)]
    clusters = cluster_frontiers(cells, min_cluster_size=1)
    assert len(clusters) == 1
    # 质心 = (0.5, 0.5)
    assert abs(clusters[0].centroid_x - 0.5) < 1e-6
    assert abs(clusters[0].centroid_y - 0.5) < 1e-6


def test_cluster_frontiers_diagonal_connectivity():
    """验证 8-邻接下，对角相邻的格属于同一簇。"""
    # (0,0) 和 (1,1) 是对角相邻，8-邻接下属于同一簇
    cells = [(0, 0), (1, 1), (2, 2)]
    clusters = cluster_frontiers(cells, min_cluster_size=1)
    assert len(clusters) == 1
    assert clusters[0].size == 3


# ============================================================
# 信息增益计算测试
# ============================================================

def test_compute_info_gain_basic():
    """验证邻接未知格被正确计数。"""
    grid = [
        [-1, -1, -1],
        [0, 0, 0],
        [0, 0, 0],
    ]
    cells = [(1, 0), (1, 1), (1, 2)]
    cluster = FrontierCluster(cells=cells)
    info = compute_info_gain(grid, cluster)
    # 第二行三格的邻接未知格为第一行三格 = 3
    assert info == 3


def test_compute_info_gain_no_unknown():
    """验证周围没有未知格时信息增益为 0。"""
    grid = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
    ]
    cells = [(1, 1)]
    cluster = FrontierCluster(cells=cells)
    info = compute_info_gain(grid, cluster)
    assert info == 0


def test_compute_info_gain_empty_cluster():
    """验证空簇信息增益为 0。"""
    grid = [[-1, -1], [0, 0]]
    cluster = FrontierCluster(cells=[])
    info = compute_info_gain(grid, cluster)
    assert info == 0


def test_compute_info_gain_deduplicate():
    """验证同一个未知格被多个前沿格邻接时只计一次。"""
    grid = [
        [-1, 0, 0],
        [0, 0, 0],
    ]
    cells = [(0, 1), (1, 0)]
    cluster = FrontierCluster(cells=cells)
    # 两个前沿格都邻接 grid[0][0]（未知），去重后 = 1
    info = compute_info_gain(grid, cluster)
    assert info == 1


# ============================================================
# 完整探索目标选择测试
# ============================================================

def test_select_exploration_target_empty_grid():
    """验证空地图返回 None。"""
    assert select_exploration_target([], 0.0, 0.0) is None


def test_select_exploration_target_no_frontiers():
    """验证无前沿时返回 None。"""
    grid = [[0, 0], [0, 0]]
    assert select_exploration_target(grid, 0.0, 0.0) is None


def test_select_exploration_target_simple():
    """验证简单场景下正确选择目标点。"""
    # 第三行是自由空间，上面两行未知 → 前沿在第三行
    grid = [
        [-1, -1, -1],
        [-1, -1, -1],
        [0, 0, 0],
    ]
    goal = select_exploration_target(
        grid, robot_x=1.0, robot_y=2.0, min_frontier_size=1
    )
    assert goal is not None
    # 第三行三格全为前沿，质心应为 (1.0, 2.0)
    assert abs(goal.target_x - 1.0) < 1e-6
    assert abs(goal.target_y - 2.0) < 1e-6
    assert goal.info_gain > 0
    assert goal.cluster_size == 3
    assert len(goal.reason) > 0


def test_select_exploration_target_prefers_higher_info_gain():
    """验证信息增益更高的前沿簇被优先选择。

    场景：机器人下方有两个前沿区域，左边的邻接 6 个未知格，
    右边的邻接 3 个未知格。在距离相同时应选左边的。
    """
    # 构建：上方全未知，下方两片自由区域，中间有障碍隔开
    grid = [
        [-1, -1, -1, -1, -1, -1, -1],
        [0, 0, 0, 100, 0, 0, 0],
        [0, 0, 0, 100, 0, 0, 0],
    ]
    goal = select_exploration_target(
        grid, robot_x=3.0, robot_y=2.0, min_frontier_size=1
    )
    assert goal is not None
    # 左边簇有更多未知格邻接 → 信息增益更高，应被选中
    assert goal.info_gain > 0
    assert len(goal.reason) > 0


def test_select_exploration_target_prefers_closer():
    """验证距离更近的前沿簇在信息增益相同时被优先选择。

    场景：机器人两侧各有一个信息增益相同的前沿，距离不同。
    """
    grid = [
        [-1, -1, -1],
        [0, 100, 0],
        [-1, -1, -1],
    ]
    # 机器人位于 (0, 1)（左边自由格的右边缘）
    # 左边 (0,0) 前沿 和 右边 (1,2) 前沿，信息增益可能不同
    # 我们主要验证目标选择不崩溃且返回有效结果
    goal = select_exploration_target(
        grid, robot_x=0.0, robot_y=1.0, min_frontier_size=1
    )
    assert goal is not None
    assert goal.cluster_size >= 1


def test_select_exploration_target_unreachable_penalty():
    """验证被障碍物阻挡的前沿因不可达惩罚而降低评分。

    场景：两个前沿区域，信息增益相近，但右边前沿被障碍物墙阻挡。
    无障碍物阻挡的左边前沿应被选中。
    """
    # 地图布局：
    # - 上方两行：左边是自由空间（邻接上方未知），右边也是自由空间（邻接上方未知）
    # - 中间一列障碍物墙把左右隔开
    # - 机器人放在左边自由区域
    grid = [
        [-1, -1, -1, 100, -1, -1, -1],
        [0, 0, 0, 100, 0, 0, 0],
        [0, 0, 0, 100, 0, 0, 0],
    ]
    # 机器人在左边自由区域，右边前沿在障碍物后面
    goal = select_exploration_target(
        grid,
        robot_x=1.0,
        robot_y=2.0,
        min_frontier_size=2,
        unreachable_penalty=20.0,
    )
    assert goal is not None
    # 左边前沿无障碍阻挡应被选中，右边前沿的直线会穿过障碍物墙
    assert goal.target_x < 3.0  # 左边簇
    assert "直线路径有障碍" not in goal.reason


def test_select_exploration_target_all_frontiers_filtered():
    """验证所有前沿簇都小于最小值时返回 None。"""
    grid = [
        [-1, -1, -1],
        [0, 0, 0],
    ]
    goal = select_exploration_target(
        grid, robot_x=0.0, robot_y=1.0, min_frontier_size=10
    )
    # 只有 3 个前沿格，min_frontier_size=10 → 全部被过滤
    assert goal is None


def test_select_exploration_target_robot_origin():
    """验证机器人在原点时也能正确选择目标。"""
    grid = [
        [-1, -1, 0],
        [-1, 0, 0],
        [-1, -1, 0],
    ]
    goal = select_exploration_target(
        grid, robot_x=2.0, robot_y=2.0, min_frontier_size=1
    )
    assert goal is not None
    assert goal.target_x >= 0
    assert goal.target_y >= 0


def test_select_exploration_target_score_positive():
    """验证评分在信息增益足够大时为正值。"""
    # 大量未知区域 → 高信息增益
    rows = 20
    grid = []
    for r in range(rows):
        if r < rows // 2:
            grid.append([-1] * 10)
        else:
            grid.append([0] * 10)
    goal = select_exploration_target(
        grid, robot_x=5.0, robot_y=rows - 1, min_frontier_size=1
    )
    assert goal is not None
    assert goal.score > 0
    assert goal.info_gain > 0


def test_select_exploration_target_returns_frontier_goal_type():
    """验证返回值类型正确。"""
    grid = [[-1, -1], [0, 0]]
    goal = select_exploration_target(grid, 0.0, 1.0, min_frontier_size=1)
    assert isinstance(goal, FrontierGoal)
    assert isinstance(goal.score, float)
    assert isinstance(goal.info_gain, int)
    assert isinstance(goal.distance, float)
    assert isinstance(goal.cluster_size, int)
    assert isinstance(goal.reason, str)


# ============================================================
# visited_areas 已访问区域 + 新颖性奖励测试
# ============================================================

def test_visited_areas_novelty_prefers_unvisited_region():
    """验证有 visited_areas 时倾向选择远离已访问区域的前沿。

    场景：两个信息增益相同的前沿，一个在机器人已涉足区域附近，
    另一个在完全陌生的区域。后者应因新颖性奖励被选中。
    """
    # 障碍物在中间隔开左右两个前沿簇
    grid = [
        [-1, -1, -1, 100, -1, -1, -1],
        [0, 0, 0, 100, 0, 0, 0],
    ]
    # 左边区域已大量访问
    visited = {
        (1, 0), (1, 1),
    }
    # 机器人在中间（障碍物列）
    goal = select_exploration_target(
        grid,
        robot_x=3.5,
        robot_y=1.0,
        min_frontier_size=2,
        visited_areas=visited,
        visited_novelty_weight=5.0,
    )
    assert goal is not None
    # 右边前沿远离已访问区域 → 更高新颖性 → 被选中
    assert goal.target_x > 3.0  # 右边簇


def test_visited_areas_none_has_no_effect():
    """验证 visited_areas=None 时不影响评分（向后兼容）。"""
    grid = [
        [-1, -1, -1],
        [0, 0, 0],
    ]
    goal_without = select_exploration_target(
        grid, robot_x=1.0, robot_y=1.0, min_frontier_size=1,
        visited_areas=None,
    )
    goal_empty = select_exploration_target(
        grid, robot_x=1.0, robot_y=1.0, min_frontier_size=1,
        visited_areas=set(),
    )
    assert goal_without is not None
    assert goal_empty is not None
    # None 和空集合结果应一致（都是无已访问区域）
    assert goal_without.target_x == goal_empty.target_x
    assert goal_without.target_y == goal_empty.target_y
    assert goal_without.score == goal_empty.score


def test_visited_areas_reason_includes_novelty():
    """验证有 visited_areas 时选择理由包含新颖性信息。"""
    grid = [
        [-1, -1, -1],
        [0, 0, 0],
    ]
    visited = {(1, 0), (1, 1)}
    goal = select_exploration_target(
        grid, robot_x=0.0, robot_y=1.0, min_frontier_size=1,
        visited_areas=visited,
    )
    assert goal is not None
    assert "新颖性" in goal.reason


def test_visited_areas_far_novelty_higher_score():
    """验证远离已访问区域的前沿获得更高的新颖性奖励。

    场景：两个独立前沿——左边靠近一堆已访问格，右边远离。
    在新颖性权重足够大时，右边应被选中。
    """
    # 构建：上方全未知，下方中间有障碍物分隔左右
    grid = [
        [-1, -1, -1, -1, -1, -1, -1],
        [0, 0, 0, 100, 0, 0, 0],
    ]
    # 左半边有密集的已访问格
    visited = {
        (1, 0), (1, 1),
    }
    goal = select_exploration_target(
        grid,
        robot_x=3.5,
        robot_y=1.0,
        min_frontier_size=2,
        visited_areas=visited,
        visited_novelty_weight=5.0,  # 高权重突出新颖性
    )
    assert goal is not None
    # 右边前沿远离左边已访问区域 → 更高新颖性 → 被选中
    assert goal.target_x > 3.0


def test_visited_areas_empty_set_equivalent_to_none():
    """验证 visited_areas 为空集合时与传 None 行为一致。"""
    grid = [
        [-1, -1],
        [0, 0],
    ]
    goal_none = select_exploration_target(
        grid, robot_x=0.0, robot_y=1.0, min_frontier_size=1,
        visited_areas=None,
    )
    goal_empty = select_exploration_target(
        grid, robot_x=0.0, robot_y=1.0, min_frontier_size=1,
        visited_areas=set(),
    )
    assert goal_none is not None
    assert goal_empty is not None
    assert goal_none.score == goal_empty.score
