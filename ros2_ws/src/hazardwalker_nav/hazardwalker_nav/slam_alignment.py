"""SLAM 轨迹与物理控制里程的赛后对齐评估。

物理里程只用于测试验收，绝不反馈给 SLAM 或导航算法。固定前 10 秒的刚体
对齐后，后续误差直接反映 map 轨迹是否与真实运动逐渐分离。
"""

from __future__ import annotations

import math
from statistics import median
from typing import Iterable, Mapping


def evaluate_map_physical_alignment(
        samples: Iterable[Mapping], calibration_duration_s: float = 10.0,
) -> dict:
    """以早期航向差固定 SE(2) 对齐，返回后续位置误差统计。"""

    rows = []
    for sample in samples:
        try:
            row = {
                'time': float(sample['ros_sec']),
                'map_x': float(sample['x']),
                'map_y': float(sample['y']),
                'map_yaw': math.radians(float(sample['yaw_deg'])),
                'physical_x': float(sample['official_x']),
                'physical_y': float(sample['official_y']),
                'physical_yaw': math.radians(
                    float(sample['official_yaw_deg'])),
            }
        except (KeyError, TypeError, ValueError):
            continue
        if all(math.isfinite(value) for value in row.values()):
            rows.append(row)
    if len(rows) < 3:
        raise ValueError('至少需要三条同时含 map 与物理位姿的轨迹样本')

    rows.sort(key=lambda item: item['time'])
    start_time = rows[0]['time']
    calibration = [
        row for row in rows
        if row['time'] <= start_time + max(0.1, calibration_duration_s)
    ]
    if len(calibration) < 3:
        calibration = rows[:min(30, len(rows))]
    sine = sum(math.sin(
        row['physical_yaw'] - row['map_yaw']) for row in calibration)
    cosine = sum(math.cos(
        row['physical_yaw'] - row['map_yaw']) for row in calibration)
    rotation = math.atan2(sine, cosine)
    cos_rotation = math.cos(rotation)
    sin_rotation = math.sin(rotation)

    translations_x = []
    translations_y = []
    for row in calibration:
        rotated_x = (
            cos_rotation * row['map_x'] - sin_rotation * row['map_y'])
        rotated_y = (
            sin_rotation * row['map_x'] + cos_rotation * row['map_y'])
        translations_x.append(row['physical_x'] - rotated_x)
        translations_y.append(row['physical_y'] - rotated_y)
    translation_x = median(translations_x)
    translation_y = median(translations_y)

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
        'calibration_sample_count': len(calibration),
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
