"""SLAM 轨迹与物理控制里程的赛后对齐评估。

物理里程只用于测试验收，绝不反馈给 SLAM 或导航算法。按标准轨迹绝对误差
做一次不允许缩放的全局 SE(2) 对齐；剩余误差直接反映尺度或轨迹漂移。
"""

from __future__ import annotations

import math
from typing import Iterable, Mapping


def evaluate_map_physical_alignment(samples: Iterable[Mapping]) -> dict:
    """以全轨迹最小二乘 SE(2) 对齐，返回绝对轨迹误差统计。

    只允许一个固定旋转和平移，明确禁止尺度拟合。因此坐标原点/朝向差异不会
    被误判成漂移，里程比例错误、局部跳变和随时间累积的分离仍会保留。
    """

    rows = []
    for sample in samples:
        try:
            row = {
                'time': float(sample['ros_sec']),
                'map_x': float(sample['x']),
                'map_y': float(sample['y']),
                'physical_x': float(sample['official_x']),
                'physical_y': float(sample['official_y']),
            }
        except (KeyError, TypeError, ValueError):
            continue
        if all(math.isfinite(value) for value in row.values()):
            rows.append(row)
    if len(rows) < 3:
        raise ValueError('至少需要三条同时含 map 与物理位姿的轨迹样本')

    rows.sort(key=lambda item: item['time'])
    map_center_x = sum(row['map_x'] for row in rows) / len(rows)
    map_center_y = sum(row['map_y'] for row in rows) / len(rows)
    physical_center_x = (
        sum(row['physical_x'] for row in rows) / len(rows))
    physical_center_y = (
        sum(row['physical_y'] for row in rows) / len(rows))
    cross = 0.0
    dot = 0.0
    spread = 0.0
    for row in rows:
        map_x = row['map_x'] - map_center_x
        map_y = row['map_y'] - map_center_y
        physical_x = row['physical_x'] - physical_center_x
        physical_y = row['physical_y'] - physical_center_y
        cross += map_x * physical_y - map_y * physical_x
        dot += map_x * physical_x + map_y * physical_y
        spread += map_x * map_x + map_y * map_y
    if spread < 0.25:
        raise ValueError('校准段位移不足 0.5 m，无法可靠确定坐标系朝向')
    rotation = math.atan2(cross, dot)
    cos_rotation = math.cos(rotation)
    sin_rotation = math.sin(rotation)
    translation_x = (
        physical_center_x
        - cos_rotation * map_center_x
        + sin_rotation * map_center_y)
    translation_y = (
        physical_center_y
        - sin_rotation * map_center_x
        - cos_rotation * map_center_y)

    errors = []
    for row in rows:
        predicted_x = (
            translation_x
            + cos_rotation * row['map_x']
            - sin_rotation * row['map_y'])
        predicted_y = (
            translation_y
            + sin_rotation * row['map_x']
            + cos_rotation * row['map_y'])
        errors.append(math.hypot(
            predicted_x - row['physical_x'],
            predicted_y - row['physical_y']))
    ordered = sorted(errors)
    p95_index = min(len(ordered) - 1, int(math.ceil(0.95 * len(ordered))) - 1)
    return {
        'sample_count': len(rows),
        'alignment_sample_count': len(rows),
        'rotation_deg': math.degrees(rotation),
        'translation_x_m': translation_x,
        'translation_y_m': translation_y,
        'mean_error_m': sum(errors) / len(errors),
        'rms_error_m': math.sqrt(sum(value * value for value in errors) / len(errors)),
        'p95_error_m': ordered[p95_index],
        'max_error_m': ordered[-1],
        'final_error_m': errors[-1],
    }


def alignment_is_acceptable(
        metrics: Mapping, p95_limit_m: float = 1.0,
        max_limit_m: float = 2.0,
) -> bool:
    """正式稳定门：绝大多数误差不超过 1 m，任何时刻不超过 2 m。"""

    try:
        p95 = float(metrics['p95_error_m'])
        maximum = float(metrics['max_error_m'])
    except (KeyError, TypeError, ValueError):
        return False
    return (
        math.isfinite(p95)
        and math.isfinite(maximum)
        and p95 <= float(p95_limit_m)
        and maximum <= float(max_limit_m)
    )
