"""把官方深度图中部带转换为水平 LaserScan 的纯函数。

深度图采用光学系（Z 前、X 右），官方 ``real_sense`` TF 采用机体系（X 前、
Y 左、Z 上）。因此每列的水平角为 ``atan2(-(u-cx), fx)``，输出按角度递增。
"""

import math

import numpy as np


def depth_image_to_planar_scan(
        depth_image,
        fx,
        cx,
        row_min_fraction=0.35,
        row_max_fraction=0.65,
        column_stride=2,
        depth_percentile=10.0,
        min_range_m=0.40,
        max_range_m=8.0):
    """从深度图的水平带生成 ``(angle_min, angle_max, increment, ranges)``。

    每列取低分位深度以保留墙体/家具的最近表面，同时用多行统计抑制单像素噪声。
    无有效量测的列输出 ``inf``，交由 Cartographer 处理为有限长度自由射线。
    """

    depth = np.asarray(depth_image, dtype=np.float32)
    if depth.ndim != 2 or depth.shape[0] < 2 or depth.shape[1] < 2:
        raise ValueError('depth_image must be a 2D array with at least 2x2 pixels')
    focal_x = float(fx)
    principal_x = float(cx)
    if not math.isfinite(focal_x) or focal_x <= 0.0:
        raise ValueError('fx must be positive and finite')
    if not math.isfinite(principal_x):
        raise ValueError('cx must be finite')

    height, width = depth.shape
    row_start = max(0, min(height - 1, int(height * float(row_min_fraction))))
    row_stop = max(row_start + 1, min(
        height, int(math.ceil(height * float(row_max_fraction))),
    ))
    stride = max(1, int(column_stride))
    percentile = min(100.0, max(0.0, float(depth_percentile)))
    minimum = float(min_range_m)
    maximum = float(max_range_m)
    if not (math.isfinite(minimum) and math.isfinite(maximum)
            and 0.0 < minimum < maximum):
        raise ValueError('range limits must satisfy 0 < min < max')

    columns = list(range(width - 1, -1, -stride))
    angles = [
        math.atan2(-(column - principal_x), focal_x)
        for column in columns
    ]
    ranges = []
    band = depth[row_start:row_stop, :]
    for column, angle in zip(columns, angles):
        values = band[:, column]
        valid = values[
            np.isfinite(values)
            & (values >= minimum)
            & (values <= maximum)
        ]
        if valid.size == 0:
            ranges.append(float('inf'))
            continue
        forward_depth = float(np.percentile(valid, percentile))
        # 深度图给出光轴 Z；LaserScan 需要水平极坐标斜距。
        ranges.append(forward_depth / max(1e-6, math.cos(angle)))

    if len(angles) < 2:
        raise ValueError('column_stride leaves fewer than two scan samples')
    angle_increment = (angles[-1] - angles[0]) / (len(angles) - 1)
    return angles[0], angles[-1], angle_increment, ranges
