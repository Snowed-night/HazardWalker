"""三维体素地图的纯算法函数，不依赖 ROS，便于离线回归测试。"""

from __future__ import annotations

import math

import numpy as np


def quaternion_transform_matrix(
        translation_xyz, quaternion_xyzw) -> np.ndarray:
    """由平移和 xyzw 四元数生成 4×4 齐次变换矩阵。"""

    tx, ty, tz = map(float, translation_xyz)
    x, y, z, w = map(float, quaternion_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if (not math.isfinite(norm) or norm < 1e-9
            or not all(math.isfinite(value) for value in (tx, ty, tz))):
        raise ValueError('TF 平移或四元数无效')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w), tx],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w), ty],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y), tz],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=np.float64)
    return matrix


def transform_points(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    """对 N×3 点数组应用刚体变换，拒绝形状或数值异常。"""

    points = np.asarray(points, dtype=np.float64)
    matrix = np.asarray(matrix, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or matrix.shape != (4, 4):
        raise ValueError('点云或变换矩阵形状无效')
    if not np.isfinite(points).all() or not np.isfinite(matrix).all():
        raise ValueError('点云或变换矩阵含 NaN/Inf')
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def voxel_indices(points: np.ndarray, voxel_size_m: float) -> np.ndarray:
    """生成去重后的整数体素坐标。"""

    if not math.isfinite(voxel_size_m) or voxel_size_m <= 0.0:
        raise ValueError('体素边长必须为有限正数')
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('点云无效')
    indices = np.floor(points / voxel_size_m).astype(np.int32)
    return np.unique(indices, axis=0)
