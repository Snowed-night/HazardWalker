"""公开激光与 IMU 增量定位的纯函数回归。"""

import math
import os
import sys
from pathlib import Path


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_perception'))

from hazardwalker_perception.scan_imu_localization import (
    Pose2D,
    ScanImuLocalizer,
    ScanImuLocalizerConfig,
    floor_index_to_elevation,
    point_cloud_xyz_to_base_points,
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


def test_online_localizer_can_publish_odometry_without_competing_tf():
    """Cartographer 融合模式必须允许单独占有 odom→base TF。"""

    source = (
        Path(REPO_ROOT) / 'ros2_ws' / 'src' / 'hazardwalker_perception' /
        'hazardwalker_perception' / 'scan_imu_localizer_node.py'
    ).read_text(encoding='utf-8')
    assert "declare_parameter('publish_tf', True)" in source
    assert 'if self.tf_broadcaster is not None:' in source
    assert "declare_parameter('command_motion_scale', 1.0)" in source
    assert "declare_parameter('min_effective_linear_speed_mps', 0.30)" in source
    assert 'if abs(command_x) < min_effective_speed:' in source


def test_livox_point_cloud_uses_public_pitch_and_height_filter():
    """PointCloud2 必须按 Livox 公开外参转到 base，并排除地面/顶棚端点。"""
    config = ScanImuLocalizerConfig(
        endpoint_stride=1,
        laser_offset_x_m=0.20,
        laser_offset_y_m=-0.10,
        laser_offset_z_m=0.08,
        laser_pitch_rad=math.pi / 2.0,
        min_endpoint_z_m=-0.25,
        max_endpoint_z_m=1.50,
    )
    points = point_cloud_xyz_to_base_points([
        (0.0, 0.5, 1.0),   # 旋转后 base=(1.2, 0.4, 0.08)，应保留。
        (2.0, 0.0, 0.0),   # 旋转后 base_z=-1.92，属于地面方向，应过滤。
        (float('nan'), 0.0, 1.0),
    ], config)

    assert len(points) == 1
    assert abs(points[0][0] - 1.20) < 1e-6
    assert abs(points[0][1] - 0.40) < 1e-6


def test_public_floor_index_uses_official_fixed_floor_height():
    assert floor_index_to_elevation(0) == 0.0
    assert floor_index_to_elevation(1) == 2.6
    assert floor_index_to_elevation(2) == 5.2


def test_public_floor_index_rejects_corrupted_values():
    for value in (-1, 1.5, True, 'floor_2', 32):
        try:
            floor_index_to_elevation(value)
        except ValueError:
            continue
        raise AssertionError('损坏楼层编号必须被拒绝：%r' % (value,))


def test_floor_transition_clears_scan_history_but_preserves_pose():
    localizer = ScanImuLocalizer(ScanImuLocalizerConfig(min_match_count=1))
    localizer.update_base_points([(1.0, 0.0), (0.0, 1.0)], 0.0)
    localizer.pose = Pose2D(1.2, -0.4, 0.3)

    localizer.reset_matching_map()

    assert localizer.pose == Pose2D(1.2, -0.4, 0.3)
    assert localizer._occupancy == set()
    assert localizer._previous_world_points == []


def test_scan_imu_localizer_recovers_small_translation_with_public_command_prior():
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
    tracked = localizer.update_points(
        moved_scan,
        imu_yaw_rad=0.0,
        motion_prior_base=(0.20, 0.0),
    )

    assert initial.status == 'initialized'
    assert tracked.status == 'tracking'
    assert abs(tracked.pose.x - 0.20) <= 0.051
    assert abs(tracked.pose.y) <= 0.051


def test_scan_imu_localizer_does_not_drift_when_stationary_scores_tie():
    """重复静止扫描的同分候选必须保留上一位姿，不能跳到搜索窗口边缘。"""
    localizer = ScanImuLocalizer(ScanImuLocalizerConfig(
        laser_offset_x_m=0.0,
        laser_offset_y_m=0.0,
        search_radius_m=0.6,
        search_step_m=0.05,
        occupancy_resolution_m=0.08,
        min_match_count=1,
    ))
    # 单面走廊端点会产生大量体素邻域同分解，正好覆盖在线出现的退化几何。
    repeated_scan = [(2.0, y * 0.08) for y in range(-10, 11)]
    localizer.update_points(repeated_scan, imu_yaw_rad=0.0)
    result = localizer.update_points(repeated_scan, imu_yaw_rad=0.0)

    assert result.status == 'tracking'
    assert abs(result.pose.x) < 1e-9
    assert abs(result.pose.y) < 1e-9


def test_command_motion_prior_breaks_degenerate_corridor_tie_forward():
    """走廊几何退化时，同分解应沿公开 cmd_vel 先验而非跳到反方向。"""
    localizer = ScanImuLocalizer(ScanImuLocalizerConfig(
        laser_offset_x_m=0.0,
        laser_offset_y_m=0.0,
        search_radius_m=0.2,
        search_step_m=0.05,
        occupancy_resolution_m=0.08,
        min_match_count=1000,
    ))
    repeated_scan = [(2.0, y * 0.08) for y in range(-10, 11)]
    localizer.update_points(repeated_scan, imu_yaw_rad=0.0)
    result = localizer.update_points(
        repeated_scan,
        imu_yaw_rad=0.0,
        motion_prior_base=(0.05, 0.0),
    )

    assert result.status == 'motion_prior_only'
    assert result.pose.x > 0.0
    assert abs(result.pose.y) < 0.05


def test_tracking_match_cannot_reverse_command_prior_in_corridor():
    """即使 ICP 达到匹配阈值，也不得用长走廊多解反转已下发动作方向。"""
    localizer = ScanImuLocalizer(ScanImuLocalizerConfig(
        laser_offset_x_m=0.0,
        laser_offset_y_m=0.0,
        search_radius_m=0.2,
        search_step_m=0.05,
        occupancy_resolution_m=0.08,
        min_match_count=1,
        scan_correction_gain=0.02,
        max_scan_correction_m=0.001,
    ))
    repeated_scan = [(2.0, y * 0.08) for y in range(-10, 11)]
    localizer.update_points(repeated_scan, imu_yaw_rad=0.0)
    result = localizer.update_points(
        repeated_scan,
        imu_yaw_rad=0.0,
        motion_prior_base=(0.05, 0.0),
    )

    assert result.status == 'tracking'
    assert result.pose.x >= 0.049
    assert abs(result.pose.y) <= 0.0011


def test_repeated_corridor_updates_accumulate_forward_without_lateral_runaway():
    localizer = ScanImuLocalizer(ScanImuLocalizerConfig(
        laser_offset_x_m=0.0,
        laser_offset_y_m=0.0,
        min_match_count=1,
        scan_correction_gain=0.02,
        max_scan_correction_m=0.001,
    ))
    repeated_scan = [(2.0, y * 0.08) for y in range(-10, 11)]
    localizer.update_points(repeated_scan, imu_yaw_rad=0.0)
    result = None
    for _ in range(20):
        result = localizer.update_points(
            repeated_scan,
            imu_yaw_rad=0.0,
            motion_prior_base=(0.01, 0.0),
        )

    assert result.pose.x > 0.18
    assert abs(result.pose.y) < 0.025


def test_scan_imu_icp_keeps_translation_stable_during_in_place_rotation():
    """IMU 提供转角后，相邻帧 ICP 不应把纯旋转误记成平移。"""
    config = ScanImuLocalizerConfig(
        laser_offset_x_m=0.0,
        laser_offset_y_m=0.0,
        occupancy_resolution_m=0.04,
        min_match_count=3,
        icp_max_correspondence_m=0.60,
    )
    localizer = ScanImuLocalizer(config)
    world_landmarks = [
        (1.2, 0.4), (2.5, -0.8), (-1.3, 1.9), (-2.2, -0.5), (0.3, 3.1),
    ]
    localizer.update_points(world_landmarks, imu_yaw_rad=0.0)
    yaw = math.radians(35.0)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotated_scan = [
        # 世界点转换到旋转后的 base：R(-yaw) * point。
        (cosine * x + sine * y, -sine * x + cosine * y)
        for x, y in world_landmarks
    ]
    result = localizer.update_points(rotated_scan, imu_yaw_rad=yaw)

    assert result.status == 'tracking'
    assert abs(result.pose.x) < 0.03
    assert abs(result.pose.y) < 0.03


def test_imu_orientation_sets_relative_yaw_without_reading_ground_truth():
    assert round(quaternion_to_yaw(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)), 6) == round(math.pi / 2, 6)

    localizer = ScanImuLocalizer(ScanImuLocalizerConfig(
        laser_offset_x_m=0.0, min_match_count=1,
    ))
    localizer.update_points([(2.0, 0.0)], imu_yaw_rad=1.0)
    result = localizer.update_points([(0.0, -2.0)], imu_yaw_rad=1.0 + math.pi / 2)

    assert abs(result.pose.yaw - math.pi / 2) < 1e-6
