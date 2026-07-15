"""公开激光与 IMU 增量定位的纯函数回归。"""

import math
import os
import sys


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_perception'))

from hazardwalker_perception.scan_imu_localization import (
    ScanImuLocalizer,
    ScanImuLocalizerConfig,
    quaternion_to_yaw,
    scan_ranges_to_points,
)


def test_scan_ranges_filter_invalid_points_and_respect_stride():
    points = scan_ranges_to_points(
        [float('nan'), 0.05, 1.0, float('inf'), 2.0],
        angle_min=0.0, angle_increment=math.pi / 4,
        min_range_m=0.15, max_range_m=3.0, stride=2,
    )

    assert len(points) == 2
    assert round(points[0][1], 6) == 1.0
    assert round(points[1][1], 6) == 0.0


def test_scan_imu_localizer_recovers_small_translation_without_odometry_input():
    config = ScanImuLocalizerConfig(
        laser_offset_x_m=0.0,
        laser_offset_y_m=0.0,
        search_radius_m=0.4,
        search_step_m=0.05,
        occupancy_resolution_m=0.04,
        min_match_count=3,
    )
    localizer = ScanImuLocalizer(config)
    landmarks_world = [(3.0, 0.0), (0.0, 3.0), (-2.0, 1.0), (1.0, -2.0)]

    initial = localizer.update_points(landmarks_world, imu_yaw_rad=0.0)
    moved_scan = [(x - 0.20, y) for x, y in landmarks_world]
    tracked = localizer.update_points(moved_scan, imu_yaw_rad=0.0)

    assert initial.status == 'initialized'
    assert tracked.status == 'tracking'
    assert abs(tracked.pose.x - 0.20) <= 0.051
    assert abs(tracked.pose.y) <= 0.051


def test_imu_orientation_sets_relative_yaw_without_reading_ground_truth():
    assert round(quaternion_to_yaw(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)), 6) == round(math.pi / 2, 6)

    localizer = ScanImuLocalizer(ScanImuLocalizerConfig(
        laser_offset_x_m=0.0, min_match_count=1,
    ))
    localizer.update_points([(2.0, 0.0)], imu_yaw_rad=1.0)
    result = localizer.update_points([(0.0, -2.0)], imu_yaw_rad=1.0 + math.pi / 2)

    assert abs(result.pose.yaw - math.pi / 2) < 1e-6
