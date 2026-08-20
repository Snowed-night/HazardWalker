"""SLAM 监测纯函数库：跳变判定、漂移量、地图质量指标。

所属组：导航组。
文件作用：
- 提供 ROS-independent 的纯函数，供 slam_monitor_node 采集数据后调用。
- 全部逻辑不依赖 ROS，可在离线测试中验证，便于安全地调阈值。

内容：
- detect_pose_jump: 判断一帧位姿位移是否超过物理上限（跳变）。
- drift_magnitude: map→odom 平移量的欧氏距离。
- yaw_from_quaternion: 四元数提取 yaw 角。
- map_occupancy_stats: 占用栅格的比例指标。
"""

from __future__ import annotations

import math

import numpy as np


def detect_pose_jump(
    displacement_m: float,
    elapsed_s: float,
    max_speed_m_s: float = 1.0,
    min_distance_m: float = 0.5,
) -> bool:
    """判断一帧位姿位移是否超过按最大允许速度推算的物理上限。

    Cartographer 在对称/重复走廊存在 scan matching 多解，map→base 会在
    相似位置之间瞬移（实测 7.5~31 m）。正常底盘速度约 0.35 m/s，任意一帧
    位移都远小于 ``max_speed_m_s * elapsed_s + min_distance_m``；而定位瞬移
    会显著超过该上限。用「速度 × 时间 + 固定容差」而非固定距离阈值，可避免
    TF 短暂丢失后恢复（elapsed 变大）时误伤正常运动。异常输入返回 False，
    保持正常行为而不是中断观测。
    """
    try:
        displacement = float(displacement_m)
        elapsed = float(elapsed_s)
        max_speed = float(max_speed_m_s)
        min_distance = float(min_distance_m)
    except (TypeError, ValueError):
        return False
    if not all(math.isfinite(value) for value in (
        displacement,
        elapsed,
        max_speed,
        min_distance,
    )):
        return False
    elapsed = max(0.0, elapsed)
    max_speed = max(0.0, max_speed)
    min_distance = max(0.0, min_distance)
    return displacement > max_speed * elapsed + min_distance


def drift_magnitude(x: float, y: float) -> float:
    """map→odom 平移量的欧氏距离；非法输入返回 0.0。"""
    try:
        return math.hypot(float(x), float(y))
    except (TypeError, ValueError):
        return 0.0


def yaw_from_quaternion(w: float, x: float, y: float, z: float) -> float:
    """从四元数 (w, x, y, z) 提取绕 z 轴的 yaw 角。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def map_occupancy_stats(grid: np.ndarray) -> dict:
    """统计占用栅格的比例指标。

    沿用 nav_recorder.save_map 的阈值约定：自由 [0, 49]、占用 [65, 100]、
    未知 < 0；中间 50~64 的模糊区不计入自由也不计入占用（仍计入 total）。

    Args:
        grid: numpy 数组，OccupancyGrid 语义：-1=未知，0~100=占用概率。
    Returns:
        dict:
            occupied_ratio: 占用 / (占用 + 自由)，即已知区域里墙的占比。
            free_ratio: 自由 / (占用 + 自由)。
            unknown_ratio: 未知 / 总单元数，即探索剩余量。
            known_cells: 占用 + 自由 的单元数。
            total_cells: 总单元数。
    """
    total = int(grid.size)
    if total == 0:
        return {
            'occupied_ratio': 0.0,
            'free_ratio': 0.0,
            'unknown_ratio': 0.0,
            'known_cells': 0,
            'total_cells': 0,
        }
    unknown = int(np.count_nonzero(grid < 0))
    occupied = int(np.count_nonzero(grid >= 65))
    free = int(np.count_nonzero((grid >= 0) & (grid <= 49)))
    known = occupied + free
    return {
        'occupied_ratio': (occupied / known) if known else 0.0,
        'free_ratio': (free / known) if known else 0.0,
        'unknown_ratio': unknown / total,
        'known_cells': known,
        'total_cells': total,
    }
