"""根据合法 SLAM 位姿累计人工巡检运动覆盖指标。

该模块只处理公开里程计数值，不读取 Gazebo 真值或场景文件。路径长度采用平面
相邻位姿累计，并拒绝时间倒退和不可能的跳变，避免 SLAM 重定位瞬移把静止录包
伪装成有效巡检。
"""

from __future__ import annotations

import math


class PatrolCoverageTracker:
    """累计样本、平面路程、覆盖跨度和楼层高度变化。"""

    def __init__(self, *, max_planar_step_m=1.5, max_vertical_step_m=3.0):
        self.max_planar_step_m = self._positive(
            max_planar_step_m, 'max_planar_step_m')
        self.max_vertical_step_m = self._positive(
            max_vertical_step_m, 'max_vertical_step_m')
        self.sample_count = 0
        self.accepted_segment_count = 0
        self.rejected_sample_count = 0
        self.planar_path_length_m = 0.0
        self._first = None
        self._last = None
        self._bounds = None

    def update(self, x, y, z, stamp_sec):
        """加入一个按时间递增的位姿；非法样本返回 False。"""

        values = tuple(float(value) for value in (x, y, z, stamp_sec))
        if not all(math.isfinite(value) for value in values):
            self.rejected_sample_count += 1
            return False
        point = values[:3]
        stamp = values[3]
        if self._last is not None:
            previous_point, previous_stamp = self._last
            if stamp <= previous_stamp:
                self.rejected_sample_count += 1
                return False
            planar_step = math.hypot(
                point[0] - previous_point[0], point[1] - previous_point[1])
            vertical_step = abs(point[2] - previous_point[2])
            if (planar_step > self.max_planar_step_m
                    or vertical_step > self.max_vertical_step_m):
                # SLAM 坐标发生不连续跳变后，从新锚点重新累计本轮覆盖。保留
                # 跳变前边界会让下一帧正常运动把坐标修正量伪装成覆盖跨度。
                self.planar_path_length_m = 0.0
                self.accepted_segment_count = 0
                self.sample_count = 1
                self._first = (point, stamp)
                self._last = (point, stamp)
                self._bounds = [
                    point[0], point[0], point[1], point[1],
                    point[2], point[2],
                ]
                self.rejected_sample_count += 1
                return False
            self.planar_path_length_m += planar_step
            self.accepted_segment_count += 1
        else:
            self._first = (point, stamp)
        self._last = (point, stamp)
        self.sample_count += 1
        self._extend_bounds(point)
        return True

    def snapshot(self):
        """返回可直接写入 JSON/ROS String 的覆盖状态。"""

        if self._first is None or self._last is None or self._bounds is None:
            return {
                'sample_count': self.sample_count,
                'accepted_segment_count': self.accepted_segment_count,
                'rejected_sample_count': self.rejected_sample_count,
                'planar_path_length_m': 0.0,
                'planar_span_m': 0.0,
                'start_to_end_displacement_m': 0.0,
                'vertical_span_m': 0.0,
                'elapsed_sec': 0.0,
            }
        first_point, first_stamp = self._first
        last_point, last_stamp = self._last
        min_x, max_x, min_y, max_y, min_z, max_z = self._bounds
        return {
            'sample_count': self.sample_count,
            'accepted_segment_count': self.accepted_segment_count,
            'rejected_sample_count': self.rejected_sample_count,
            'planar_path_length_m': round(self.planar_path_length_m, 6),
            'planar_span_m': round(math.hypot(
                max_x - min_x, max_y - min_y), 6),
            'start_to_end_displacement_m': round(math.hypot(
                last_point[0] - first_point[0],
                last_point[1] - first_point[1]), 6),
            'vertical_span_m': round(max_z - min_z, 6),
            'elapsed_sec': round(max(0.0, last_stamp - first_stamp), 6),
        }

    def _extend_bounds(self, point):
        x, y, z = point
        if self._bounds is None:
            self._bounds = [x, x, y, y, z, z]
            return
        self._bounds[0] = min(self._bounds[0], x)
        self._bounds[1] = max(self._bounds[1], x)
        self._bounds[2] = min(self._bounds[2], y)
        self._bounds[3] = max(self._bounds[3], y)
        self._bounds[4] = min(self._bounds[4], z)
        self._bounds[5] = max(self._bounds[5], z)

    @staticmethod
    def _positive(value, name):
        number = float(value)
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f'{name} 必须是有限正数')
        return number


def validate_patrol_coverage(
        payload, *, minimum_samples=20, minimum_path_length_m=8.0,
        minimum_planar_span_m=3.0):
    """返回正式巡检覆盖门禁的失败原因；空列表表示通过。"""

    if not isinstance(payload, dict):
        return ['巡检覆盖状态不是 JSON 对象']
    errors = []
    requirements = (
        ('sample_count', int(minimum_samples), '样本数'),
        ('planar_path_length_m', float(minimum_path_length_m), '平面路程'),
        ('planar_span_m', float(minimum_planar_span_m), '平面覆盖跨度'),
    )
    for key, minimum, label in requirements:
        try:
            value = float(payload.get(key, -1))
        except (TypeError, ValueError):
            value = -1.0
        if not math.isfinite(value) or value < minimum:
            errors.append(f'{label} {value:g} 小于最低 {minimum:g}')
    return errors
