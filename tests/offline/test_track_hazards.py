"""危险源多帧确认和去重离线测试。

所属组：感知组 / 测试组。
文件作用：
验证 `track_hazards.py` 能按三维距离合并观测、确认稳定目标、拒绝长期丢失目标。
不依赖 ROS、Gazebo 或真实相机。
"""
import math
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_perception'))

from hazardwalker_perception.track_hazards import (
    HazardObservation,
    HazardTracker,
    HazardTrackerConfig,
    distance_m,
    track_to_hazard_dict,
)


"""验证三维距离函数使用欧氏距离。"""
def test_distance_m_uses_3d_euclidean_distance():
    assert distance_m((0.0, 0.0, 0.0), (3.0, 4.0, 12.0)) == 13.0


"""验证近距离多帧观测会合并为同一条 confirmed 轨迹。"""
def test_tracker_merges_near_observations_and_confirms_track():
    tracker = HazardTracker(HazardTrackerConfig(confirm_observation_count=3, merge_distance_m=0.5))

    tracker.update([HazardObservation(position=(1.0, 2.0, 0.5), confidence=0.70, stamp_sec=1.0)])
    tracker.update([HazardObservation(position=(1.1, 2.0, 0.5), confidence=0.80, stamp_sec=2.0)])
    tracks = tracker.update([HazardObservation(position=(0.9, 2.1, 0.5), confidence=0.75, stamp_sec=3.0)])

    assert len(tracks) == 1
    assert tracks[0].status == 'confirmed'
    assert tracks[0].observation_count == 3
    assert tracks[0].confidence == 0.80
    assert tracks[0].missed_count == 0


"""验证距离超过 merge_distance_m 的观测会创建新的危险源轨迹。"""
def test_tracker_creates_new_track_for_far_observation():
    tracker = HazardTracker(HazardTrackerConfig(confirm_observation_count=2, merge_distance_m=0.5))

    tracker.update([{'position': [1.0, 0.0, 0.5], 'confidence': 0.8, 'id': 'frame1_box1'}], stamp_sec=1.0)
    tracks = tracker.update([{'position': [3.0, 0.0, 0.5], 'confidence': 0.7, 'id': 'frame2_box1'}], stamp_sec=2.0)

    assert len(tracks) == 2
    assert tracks[0].track_id == 1
    assert tracks[1].track_id == 2
    assert tracks[0].missed_count == 1
    assert tracks[1].missed_count == 0


"""验证没有新观测时，长期丢失的 tentative 轨迹会被 rejected。"""
def test_tracker_rejects_track_after_missed_count_threshold():
    tracker = HazardTracker(HazardTrackerConfig(reject_after_missed_count=2))

    tracker.update([HazardObservation(position=(1.0, 0.0, 0.5), confidence=0.8)], stamp_sec=1.0)
    assert len(tracker.active_tracks()) == 1

    tracker.update([], stamp_sec=2.0)
    tracks = tracker.update([], stamp_sec=3.0)

    assert tracks == []
    assert tracker.tracks[0].status == 'rejected'
    assert tracker.tracks[0].missed_count == 2


def test_time_based_loss_is_independent_of_duplicate_frame_rate():
    """正式链即使高频重复空帧，也只按仿真时间判定轨迹丢失。"""

    tracker = HazardTracker(HazardTrackerConfig(
        reject_after_missed_count=2,
        reject_after_missed_sec=20.0,
    ))
    tracker.update([HazardObservation(
        position=(0.0, 0.0, 0.3), confidence=0.9, stamp_sec=100.0,
    )], stamp_sec=100.0)

    for index in range(1000):
        tracker.update([], stamp_sec=100.0 + index * 0.01)
    assert tracker.tracks[0].status == 'tentative'

    tracker.update([], stamp_sec=120.1)
    assert tracker.tracks[0].status == 'rejected'


"""验证同一帧多个观测不会重复匹配到同一条轨迹。"""
def test_tracker_keeps_two_same_frame_observations_as_separate_tracks():
    tracker = HazardTracker(HazardTrackerConfig(confirm_observation_count=2, merge_distance_m=0.5))

    tracks = tracker.update([
        HazardObservation(position=(1.0, 0.0, 0.5), confidence=0.8),
        HazardObservation(position=(1.2, 0.1, 0.5), confidence=0.7),
    ], stamp_sec=1.0)

    assert len(tracks) == 2
    assert tracks[0].track_id != tracks[1].track_id


"""验证启用多视角门槛后，同一视角重复观测不会直接确认红球。"""
def test_tracker_requires_distinct_views_before_confirmation():
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=2,
        merge_distance_m=0.5,
    ))
    for stamp in (1.0, 2.0, 3.0):
        tracks = tracker.update([
            HazardObservation(
                position=(1.0, 0.0, 0.5), confidence=0.8,
                stamp_sec=stamp, view_id='pos:0.0:0.0:0.0|yaw:0',
            )
        ])
    assert tracks[0].status == 'tentative'
    tracks = tracker.update([
        HazardObservation(
            position=(1.1, 0.0, 0.5), confidence=0.85,
            stamp_sec=4.0, view_id='pos:0.4:0.0:0.0|yaw:30',
        )
    ])
    assert tracks[0].status == 'confirmed'
    assert tracks[0].view_ids == ['pos:0.0:0.0:0.0|yaw:0', 'pos:0.4:0.0:0.0|yaw:30']


def test_official_rgbd_profile_requires_two_spherical_depth_views_before_confirmation():
    """正式模式不得由仅 RGB 圆形或深度未知的三视角确认红色圆柱端面。"""
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=3,
        min_spherical_views_for_confirm=2,
        expected_sphere_diameter_m=0.30,
        max_sphere_diameter_relative_error=0.35,
        merge_distance_m=0.5,
    ))
    for view_id in ('front', 'left', 'right'):
        tracks = tracker.update([HazardObservation(
            position=(1.0, 0.0, 0.5), confidence=0.9,
            view_id=view_id, confirmation_eligible=True,
            depth_shape_status='unknown', apparent_diameter_m=0.30,
            aspect_ratio=0.95, depth_curvature_m=0.06,
        )])

    assert tracks[0].status == 'needs_reobservation'
    assert tracks[0].evidence_status == 'insufficient_multiview_spherical_depth'

    for view_id in ('left_depth', 'right_depth'):
        tracks = tracker.update([HazardObservation(
            position=(1.0, 0.0, 0.5), confidence=0.9,
            view_id=view_id, confirmation_eligible=True,
            depth_shape_status='spherical', apparent_diameter_m=0.30,
            aspect_ratio=0.95, depth_curvature_m=0.06,
        )])

    assert tracks[0].status == 'confirmed'


def test_partial_or_unstable_spherical_observations_do_not_supply_confirmation_evidence():
    """局部可见弧段即使深度看似球面，也只能触发复查而不能补齐正证据。"""
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=3,
        min_spherical_views_for_confirm=2,
        merge_distance_m=0.5,
    ))
    for view_id in ('front', 'left', 'right'):
        tracks = tracker.update([HazardObservation(
            position=(1.0, 0.0, 0.5), confidence=0.9,
            view_id=view_id, confirmation_eligible=True,
            depth_shape_status='unknown', apparent_diameter_m=0.30,
            aspect_ratio=0.95, depth_curvature_m=0.06,
        )])
    for view_id in ('partial_left', 'partial_right'):
        tracks = tracker.update([HazardObservation(
            position=(1.0, 0.0, 0.5), confidence=0.5,
            view_id=view_id, confirmation_eligible=False,
            depth_shape_status='spherical', apparent_diameter_m=0.30,
            aspect_ratio=0.30, depth_curvature_m=0.06,
        )])

    assert tracks[0].status == 'needs_reobservation'
    assert tracks[0].spherical_view_ids == []
    assert tracks[0].evidence_status == 'insufficient_multiview_spherical_depth'


def test_ineligible_partial_observations_cannot_drift_a_confirmed_world_position():
    """纵使上游误传 partial 的表面点，跟踪器也必须保持完整观测的世界中心。"""

    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=1,
        merge_distance_m=0.5,
    ))
    tracker.update([HazardObservation(
        position=(1.0, 2.0, 0.3), confidence=0.95,
        view_id='complete', confirmation_eligible=True,
    )])
    for index in range(30):
        tracker.update([HazardObservation(
            position=(1.45, 2.0, 0.3), confidence=0.4,
            view_id='partial_%d' % index, confirmation_eligible=False,
        )])

    assert tracker.tracks[0].position == (1.0, 2.0, 0.3)
    assert tracker.tracks[0].eligible_observation_count == 1
    assert tracker.tracks[0].status == 'confirmed'


def test_official_size_prior_rechecks_a_spherical_but_wrong_sized_red_object():
    """题目目标尺寸固定为直径 0.30 m，不能把明显更大的红球/圆物直接提交。"""
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=3,
        min_spherical_views_for_confirm=2,
        expected_sphere_diameter_m=0.30,
        max_sphere_diameter_relative_error=0.35,
        merge_distance_m=0.5,
    ))
    for view_id in ('front', 'left', 'right'):
        tracks = tracker.update([HazardObservation(
            position=(1.0, 0.0, 0.5), confidence=0.9,
            view_id=view_id, confirmation_eligible=True,
            depth_shape_status='spherical', apparent_diameter_m=0.55,
            aspect_ratio=0.95, depth_curvature_m=0.08,
        )])

    assert tracks[0].status == 'needs_reobservation'
    assert tracks[0].evidence_status == 'inconsistent_absolute_sphere_diameter'
    assert track_to_hazard_dict(tracks[0])['median_apparent_diameter_m'] == 0.55


"""验证轨迹可以转换为结果 JSON 使用的危险源字段。"""
def test_track_to_hazard_dict_preserves_confirmation_fields():
    tracker = HazardTracker(HazardTrackerConfig(confirm_observation_count=1))
    track = tracker.update([
        HazardObservation(position=(1.23456, 2.0, 0.5), confidence=0.87654, source_id='box_1', stamp_sec=4.0)
    ])[0]

    hazard = track_to_hazard_dict(track)

    assert hazard['id'] == 1
    assert hazard['position'] == [1.2346, 2.0, 0.5]
    assert hazard['position_frame_id'] == 'start'
    assert hazard['confidence'] == 0.8765
    assert hazard['status'] == 'confirmed'
    assert hazard['observation_count'] == 1
    assert hazard['source_ids'] == ['box_1']
    assert hazard['source_observation_count'] == 1


def test_track_payload_bounds_repeated_source_ids_without_losing_count():
    tracker = HazardTracker(HazardTrackerConfig(confirm_observation_count=1))
    track = None
    for index in range(40):
        track = tracker.update([HazardObservation(
            position=(1.0, 2.0, 0.5), confidence=0.9,
            source_id=f'frame-{index}', stamp_sec=float(index + 1),
        )])[0]

    hazard = track_to_hazard_dict(track)
    assert hazard['source_observation_count'] == 40
    assert len(hazard['source_ids']) == 32
    assert hazard['source_ids'][0] == 'frame-0'
    assert hazard['source_ids'][-1] == 'frame-39'


def test_flat_evidence_from_two_views_rejects_non_spherical_track():
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=2,
        min_distinct_views=2,
        merge_distance_m=0.5,
        min_non_spherical_views_to_reject=2,
    ))
    tracker.update([HazardObservation(
        position=(1.0, 0.0, 0.5), confidence=0.8,
        view_id='left', confirmation_eligible=False, depth_shape_status='flat',
    )])
    tracks = tracker.update([HazardObservation(
        position=(1.02, 0.0, 0.5), confidence=0.82,
        view_id='right', confirmation_eligible=False, depth_shape_status='flat',
    )])

    assert tracks == []
    assert tracker.tracks[0].status == 'rejected_non_spherical'
    assert tracker.tracks[0].evidence_status == 'multi_view_flat_or_non_spherical'


def test_anisotropic_depth_evidence_rejects_cylindrical_track():
    """两个视角均呈单轴曲率时，圆柱轨迹必须作为非球体拒绝。"""

    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=3,
        min_non_spherical_views_to_reject=2,
    ))
    tracker.update([HazardObservation(
        position=(1.0, 0.0, 0.2), confidence=0.9,
        view_id='left', confirmation_eligible=False, depth_shape_status='anisotropic',
    )])
    tracker.update([HazardObservation(
        position=(1.02, 0.0, 0.2), confidence=0.9,
        view_id='right', confirmation_eligible=False, depth_shape_status='anisotropic',
    )])

    assert tracker.active_tracks() == []
    assert tracker.tracks[0].status == 'rejected_non_spherical'
    assert tracker.tracks[0].evidence_status == 'multi_view_flat_or_non_spherical'


def test_published_tracks_keeps_rejection_tombstone_for_downstream_revocation():
    """已确认后被多视角否决的 ID 必须继续发布 rejected 状态以撤销旧结果。"""

    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=1,
        min_distinct_views=1,
        min_non_spherical_views_to_reject=2,
    ))
    tracker.update([HazardObservation(
        position=(1.0, 0.0, 0.2), confidence=0.95,
        view_id='front', confirmation_eligible=True,
        depth_shape_status='spherical',
    )])
    track_id = tracker.tracks[0].track_id
    for view_id in ('left', 'right'):
        tracker.update([HazardObservation(
            position=(1.0, 0.0, 0.2), confidence=0.9,
            view_id=view_id, confirmation_eligible=False,
            depth_shape_status='anisotropic',
        )])

    assert tracker.active_tracks() == []
    tombstones = tracker.published_tracks()
    assert len(tombstones) == 1
    assert tombstones[0].track_id == track_id
    assert tombstones[0].status == 'rejected_non_spherical'


def test_rejected_non_spherical_track_cannot_be_revived_by_round_frontal_views():
    """两个独立非球面视角是任务内终局反证，圆柱端面不能靠后续圆形帧复活。"""

    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=3,
        min_non_spherical_views_to_reject=2,
        min_spherical_views_for_confirm=2,
        merge_distance_m=0.5,
    ))
    for view_id in ('non_sphere_left', 'non_sphere_right'):
        tracker.update([HazardObservation(
            position=(1.0, 0.0, 0.3), confidence=0.9,
            view_id=view_id, confirmation_eligible=False,
            depth_shape_status='anisotropic',
        )])

    for index in range(4):
        tracker.update([HazardObservation(
            position=(1.0, 0.0, 0.3), confidence=0.99,
            view_id='round_front_%d' % index, confirmation_eligible=True,
            depth_shape_status='spherical', apparent_diameter_m=0.30,
            aspect_ratio=0.99, depth_curvature_m=0.06,
        )])

    assert tracker.active_tracks() == []
    assert tracker.confirmed_tracks() == []
    assert tracker.tracks[0].status == 'rejected_non_spherical'
    assert tracker.tracks[0].evidence_status == 'multi_view_flat_or_non_spherical'


def test_single_flat_round_view_is_explicitly_marked_for_reobservation():
    """圆柱端面首帧不能当普通 tentative，也不能只凭一帧永久拒绝。"""
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=3,
        min_view_bearing_span_deg=25.0,
    ))

    tracks = tracker.update([HazardObservation(
        position=(1.0, 0.0, 0.2), confidence=0.9, stamp_sec=1.0,
        view_id='front', confirmation_eligible=False, depth_shape_status='flat',
        apparent_diameter_m=0.30, aspect_ratio=1.0, depth_curvature_m=0.0,
        view_bearing_rad=0.0,
    )])

    assert len(tracks) == 1
    assert tracks[0].status == 'needs_reobservation'
    assert tracks[0].evidence_status == 'single_view_flat_or_non_spherical'


def test_inconsistent_apparent_size_stays_needs_reobservation():
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=2,
        min_distinct_views=2,
        merge_distance_m=0.5,
        max_apparent_diameter_cv=0.20,
    ))
    tracker.update([HazardObservation(
        position=(1.0, 0.0, 0.5), confidence=0.8,
        view_id='left', apparent_diameter_m=0.30,
    )])
    tracks = tracker.update([HazardObservation(
        position=(1.01, 0.0, 0.5), confidence=0.85,
        view_id='right', apparent_diameter_m=0.80,
    )])

    assert tracks[0].status == 'needs_reobservation'
    assert tracks[0].evidence_status == 'inconsistent_apparent_size'


def test_positive_views_can_outvote_one_flat_depth_outlier():
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=2,
        merge_distance_m=0.5,
    ))
    tracker.update([HazardObservation(
        position=(1.0, 0.0, 0.5), confidence=0.8,
        view_id='flat_outlier', confirmation_eligible=False, depth_shape_status='flat',
    )])
    for index, view_id in enumerate(('left', 'center', 'right'), start=1):
        tracks = tracker.update([HazardObservation(
            position=(1.0 + index * 0.01, 0.0, 0.5), confidence=0.85,
            view_id=view_id, confirmation_eligible=True,
            depth_shape_status='sphere_like', apparent_diameter_m=0.30 + index * 0.005,
        )])

    assert tracks[0].status == 'confirmed'


def test_forward_only_views_cannot_confirm_without_lateral_parallax():
    """圆柱正面在连续前后帧可近似圆形，必须取得真正侧向视角才可确认。"""
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=3,
        min_view_bearing_span_deg=25.0,
        merge_distance_m=0.5,
    ))
    for index, bearing in enumerate((0.00, 0.03, -0.02), start=1):
        tracks = tracker.update([HazardObservation(
            position=(2.0, 0.0, 0.3), confidence=0.95,
            view_id='front_%d' % index, confirmation_eligible=True,
            apparent_diameter_m=0.30, aspect_ratio=0.98, depth_curvature_m=0.06,
            view_bearing_rad=bearing,
        )])

    assert tracks[0].status == 'needs_reobservation'
    assert tracks[0].evidence_status == 'insufficient_lateral_parallax'

    tracks = tracker.update([HazardObservation(
        position=(2.01, 0.0, 0.3), confidence=0.95,
        view_id='side', confirmation_eligible=True,
        apparent_diameter_m=0.30, aspect_ratio=0.98, depth_curvature_m=0.06,
        view_bearing_rad=0.55,
    )])
    assert tracks[0].status == 'confirmed'
    assert track_to_hazard_dict(tracks[0])['view_bearing_span_deg'] >= 25.0
    assert tracks[0].eligible_observation_count == 4
    assert tracks[0].evidence_status == 'multi_view_sphere_consistent'


def test_two_strong_rgbd_sphere_views_can_use_bounded_parallax_fallback():
    """真实球面深度和官方尺寸先验齐全时，不因遮挡前只取得 7° 而漏报。"""
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=3,
        min_view_bearing_span_deg=25.0,
        expected_sphere_diameter_m=0.30,
        min_spherical_views_for_confirm=2,
        strong_depth_min_distinct_views=2,
        strong_depth_min_view_bearing_span_deg=5.0,
    ))
    observations = (
        ('left', 0.0, 0.046),
        ('left', 0.0, 0.046),
        ('right', math.radians(7.5), 0.045),
    )
    for index, (view_id, bearing, curvature) in enumerate(observations):
        tracks = tracker.update([HazardObservation(
            position=(2.0 + index * 0.01, 0.0, 0.30),
            confidence=0.96,
            view_id=view_id,
            confirmation_eligible=True,
            depth_shape_status='spherical',
            apparent_diameter_m=0.295,
            aspect_ratio=0.99,
            depth_curvature_m=curvature,
            view_bearing_rad=bearing,
        )])

    assert tracks[0].status == 'confirmed'
    assert tracks[0].evidence_status == (
        'strong_rgbd_sphere_geometry_consistent')


def test_strong_rgbd_fallback_rejects_flat_or_unknown_second_view():
    """备用路径不能把圆柱端面或缺深度的圆形轮廓升级为红球。"""
    for second_status in ('flat', 'unknown'):
        tracker = HazardTracker(HazardTrackerConfig(
            confirm_observation_count=2,
            min_distinct_views=3,
            min_view_bearing_span_deg=25.0,
            expected_sphere_diameter_m=0.30,
            min_spherical_views_for_confirm=2,
            strong_depth_min_distinct_views=2,
            strong_depth_min_view_bearing_span_deg=5.0,
        ))
        tracker.update([HazardObservation(
            position=(2.0, 0.0, 0.30), confidence=0.96,
            view_id='front', confirmation_eligible=True,
            depth_shape_status='spherical', apparent_diameter_m=0.30,
            aspect_ratio=0.99, depth_curvature_m=0.045,
            view_bearing_rad=0.0,
        )])
        tracks = tracker.update([HazardObservation(
            position=(2.01, 0.0, 0.30), confidence=0.96,
            view_id='side', confirmation_eligible=(second_status != 'flat'),
            depth_shape_status=second_status, apparent_diameter_m=0.30,
            aspect_ratio=0.99,
            depth_curvature_m=(0.045 if second_status == 'unknown' else 0.0),
            view_bearing_rad=math.radians(7.5),
        )])

        assert tracks[0].status != 'confirmed'


def test_ineligible_occluded_aspect_does_not_poison_later_complete_views():
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=3,
        min_multiview_aspect_ratio=0.88,
    ))
    tracker.update([HazardObservation(
        (1.0, 2.0, 0.3), 0.75, view_id='occluded',
        confirmation_eligible=False, aspect_ratio=0.54,
    )])
    for index, x in enumerate((1.00, 1.02, 0.98), start=1):
        tracks = tracker.update([HazardObservation(
            (x, 2.0, 0.3), 0.95, view_id=f'complete_{index}',
            confirmation_eligible=True, aspect_ratio=0.97,
            apparent_diameter_m=0.30, depth_curvature_m=0.04,
        )])

    assert tracks[0].status == 'confirmed'
    assert tracks[0].eligible_view_ids == ['complete_1', 'complete_2', 'complete_3']
    assert min(tracks[0].aspect_ratios) == 0.97
    assert tracks[0].eligible_observation_count == 3
    assert tracks[0].evidence_status == 'multi_view_sphere_consistent'


def test_elliptical_multiview_shape_never_confirms_as_sphere():
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=2,
        merge_distance_m=0.5,
        min_multiview_aspect_ratio=0.82,
    ))
    for index, view_id in enumerate(('left', 'left', 'right'), start=1):
        tracks = tracker.update([HazardObservation(
            position=(1.0 + index * 0.01, 0.0, 0.5), confidence=0.9,
            view_id=view_id, confirmation_eligible=True,
            depth_shape_status='spherical', apparent_diameter_m=0.35,
            aspect_ratio=0.74,
        )])

    assert tracks[0].status == 'needs_reobservation'
    assert tracks[0].evidence_status == 'inconsistent_multiview_aspect'


def test_unstable_cone_depth_curvature_never_confirms_as_sphere():
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=2,
        merge_distance_m=0.5,
        max_depth_curvature_cv=0.35,
    ))
    curvatures = (0.17, 0.073, 0.074)
    for index, (view_id, curvature) in enumerate(zip(('front', 'left', 'right'), curvatures), start=1):
        tracks = tracker.update([HazardObservation(
            position=(1.0 + index * 0.01, 0.0, 0.5), confidence=0.9,
            view_id=view_id, confirmation_eligible=True, depth_shape_status='spherical',
            apparent_diameter_m=0.30, aspect_ratio=0.9, depth_curvature_m=curvature,
        )])

    assert tracks[0].status == 'needs_reobservation'
    assert tracks[0].evidence_status == 'inconsistent_depth_curvature'


def test_one_nonround_side_view_overrides_two_round_frontal_views():
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=3,
        merge_distance_m=0.5,
        min_multiview_aspect_ratio=0.90,
    ))
    for view_id, aspect in (('front', 0.99), ('near', 0.98), ('side', 0.86)):
        tracks = tracker.update([HazardObservation(
            position=(1.0, 0.0, 0.5), confidence=0.9,
            view_id=view_id, confirmation_eligible=True,
            depth_shape_status='spherical', apparent_diameter_m=0.30,
            aspect_ratio=aspect, depth_curvature_m=0.06,
        )])

    assert tracks[0].status == 'needs_reobservation'
    assert tracks[0].evidence_status == 'inconsistent_multiview_aspect'


def test_near_round_cone_side_view_still_blocks_confirmation():
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=3,
        merge_distance_m=0.5,
        min_multiview_aspect_ratio=0.88,
    ))
    for view_id, aspect in (('front', 0.99), ('near', 0.99), ('side', 0.87)):
        tracks = tracker.update([HazardObservation(
            position=(1.0, 0.0, 0.5), confidence=0.9,
            view_id=view_id, confirmation_eligible=True,
            depth_shape_status='spherical', apparent_diameter_m=0.30,
            aspect_ratio=aspect, depth_curvature_m=0.08,
        )])

    assert tracks[0].status == 'needs_reobservation'
    assert tracks[0].evidence_status == 'inconsistent_multiview_aspect'


def test_cone_face_with_excessive_median_curvature_is_rejected():
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=3,
        merge_distance_m=0.5,
        max_median_normalized_depth_curvature=0.30,
    ))
    for view_id, curvature in (('front', 0.17), ('near', 0.15), ('side', 0.12)):
        tracks = tracker.update([HazardObservation(
            position=(1.0, 0.0, 0.5), confidence=0.9,
            view_id=view_id, confirmation_eligible=True,
            depth_shape_status='spherical', apparent_diameter_m=0.30,
            aspect_ratio=0.96, depth_curvature_m=curvature,
        )])

    assert tracks[0].status == 'needs_reobservation'
    assert tracks[0].evidence_status == 'excessive_normalized_depth_curvature'


def test_flat_round_object_is_rejected_by_low_normalized_curvature():
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=2,
        min_distinct_views=2,
        merge_distance_m=0.5,
        min_normalized_depth_curvature=0.10,
    ))
    for view_id in ('left', 'right'):
        tracks = tracker.update([HazardObservation(
            position=(1.0, 0.0, 0.5), confidence=0.9,
            view_id=view_id, confirmation_eligible=True,
            depth_shape_status='spherical', apparent_diameter_m=0.32,
            aspect_ratio=0.90, depth_curvature_m=0.017,
        )])

    assert tracks[0].status == 'needs_reobservation'
    assert tracks[0].evidence_status == 'insufficient_normalized_depth_curvature'


def test_confirmed_track_survives_short_camera_miss_sequence():
    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=2, min_distinct_views=2,
        reject_after_missed_count=2, merge_distance_m=0.5,
    ))
    tracker.update([HazardObservation(position=(1.0, 0.0, 0.5), confidence=0.9, view_id='left')])
    tracks = tracker.update([HazardObservation(position=(1.0, 0.0, 0.5), confidence=0.9, view_id='right')])
    assert tracks[0].status == 'confirmed'

    tracker.update([])
    tracks = tracker.update([])

    assert tracks[0].status == 'confirmed'
    assert tracks[0].missed_count == 2


def test_rejected_non_sphere_is_not_recreated_as_new_candidate_next_frame():
    """同一圆柱被多视角拒绝后必须继续关联到拒绝记忆，不能逐帧换新 ID。"""

    tracker = HazardTracker(HazardTrackerConfig(
        min_non_spherical_views_to_reject=2,
        merge_distance_m=0.5,
    ))
    for view_id in ('left', 'right'):
        tracker.update([HazardObservation(
            position=(1.0, 0.0, 0.5),
            confidence=0.9,
            view_id=view_id,
            confirmation_eligible=False,
            depth_shape_status='anisotropic',
        )])

    assert len(tracker.tracks) == 1
    assert tracker.tracks[0].status == 'rejected_non_spherical'

    active = tracker.update([HazardObservation(
        position=(1.02, 0.0, 0.5),
        confidence=0.9,
        view_id='third',
        confirmation_eligible=False,
        depth_shape_status='anisotropic',
    )])

    assert active == []
    assert len(tracker.tracks) == 1
    assert tracker.tracks[0].track_id == 1


def test_position_fusion_weights_distinct_views_not_frame_dwell_time():
    """同一视角停留很多帧不能压倒另外两个合法侧视的定位证据。"""

    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=3,
        merge_distance_m=2.0,
    ))
    for _ in range(20):
        tracker.update([HazardObservation(
            position=(0.0, 1.0, 0.3), confidence=0.9, view_id='front',
        )])
    tracker.update([HazardObservation(
        position=(0.4, 1.0, 0.3), confidence=0.9, view_id='middle',
    )])
    tracks = tracker.update([HazardObservation(
        position=(0.8, 1.0, 0.3), confidence=0.9, view_id='side',
    )])

    assert tracks[0].position == (0.4, 1.0, 0.3)
    assert tracks[0].status == 'confirmed'


def test_earliest_view_anchor_resists_later_slam_drift_without_truth():
    """首个完整视角作为静态目标锚点，后续漂移视角只用于确认。"""

    tracker = HazardTracker(HazardTrackerConfig(
        confirm_observation_count=3,
        min_distinct_views=3,
        merge_distance_m=2.0,
        position_fusion_mode='earliest_view_anchor',
    ))
    tracker.update([HazardObservation(
        position=(0.20, 1.0, 0.3), confidence=0.9, view_id='first',
    )])
    tracker.update([HazardObservation(
        position=(0.60, 1.0, 0.3), confidence=0.9, view_id='second',
    )])
    tracks = tracker.update([HazardObservation(
        position=(1.00, 1.0, 0.3), confidence=0.9, view_id='third',
    )])

    assert tracks[0].position == (0.20, 1.0, 0.3)
    assert tracks[0].status == 'confirmed'


def test_projected_track_hint_reacquires_same_target_across_slam_drift():
    """唯一图像投影提示可跨越距离门恢复旧轨迹，但仍保持同一 track ID。"""

    tracker = HazardTracker(HazardTrackerConfig(
        merge_distance_m=0.5,
        min_distinct_views=2,
        confirm_observation_count=2,
    ))
    tracker.update([HazardObservation(
        position=(0.0, 0.0, 0.3), confidence=0.9, view_id='first',
    )])
    tracks = tracker.update([HazardObservation(
        position=(0.7, 0.0, 0.3),
        confidence=0.9,
        view_id='second',
        track_id_hint=1,
    )])

    assert len(tracker.tracks) == 1
    assert tracks[0].track_id == 1
    assert tracks[0].status == 'confirmed'


def test_candidate_track_hint_revives_lost_track_after_active_view_gap():
    """主动横移短暂丢球后，稳定候选提示应复活原 ID 并保留历史视角。"""

    tracker = HazardTracker(HazardTrackerConfig(
        reject_after_missed_count=2,
        merge_distance_m=0.5,
        min_distinct_views=2,
        confirm_observation_count=2,
    ))
    tracker.update([HazardObservation(
        position=(0.0, 0.0, 0.3), confidence=0.9, view_id='first',
    )])
    tracker.update([])
    tracker.update([])
    assert tracker.tracks[0].status == 'rejected'

    tracks = tracker.update([HazardObservation(
        position=(0.8, 0.0, 0.3),
        confidence=0.9,
        view_id='second',
        track_id_hint=1,
    )])

    assert len(tracker.tracks) == 1
    assert tracks[0].track_id == 1
    assert tracks[0].status == 'confirmed'
    assert tracks[0].view_ids == ['first', 'second']


def test_candidate_track_hint_cannot_revive_non_spherical_rejection():
    """明确候选提示也不能复活已有多视角非球体反证的轨迹。"""

    tracker = HazardTracker(HazardTrackerConfig(
        merge_distance_m=0.5,
        min_non_spherical_views_to_reject=1,
    ))
    tracker.update([HazardObservation(
        position=(0.0, 0.0, 0.3),
        confidence=0.9,
        view_id='flat',
        depth_shape_status='flat',
    )])
    assert tracker.tracks[0].status == 'rejected_non_spherical'

    tracker.update([HazardObservation(
        position=(0.8, 0.0, 0.3),
        confidence=0.9,
        view_id='round',
        track_id_hint=1,
    )])

    assert len(tracker.tracks) == 2
    assert tracker.tracks[0].status == 'rejected_non_spherical'
    assert tracker.tracks[1].track_id == 2


def test_single_compatible_spherical_track_can_reacquire_across_slam_drift():
    tracker = HazardTracker(HazardTrackerConfig(
        merge_distance_m=0.5,
        single_track_reacquire_distance_m=1.0,
        single_track_reacquire_diameter_relative_error=0.25,
    ))
    tracker.update([HazardObservation(
        position=(0.0, 0.0, 0.3),
        confidence=0.9,
        view_id='first',
        depth_shape_status='spherical',
        apparent_diameter_m=0.30,
    )])
    tracker.update([])
    tracker.update([HazardObservation(
        position=(0.7, 0.0, 0.3),
        confidence=0.9,
        view_id='second',
        depth_shape_status='spherical',
        apparent_diameter_m=0.31,
    )])

    assert len(tracker.tracks) == 1
    assert tracker.tracks[0].track_id == 1


def test_single_track_reacquire_refuses_ambiguous_two_track_scene():
    tracker = HazardTracker(HazardTrackerConfig(
        merge_distance_m=0.5,
        single_track_reacquire_distance_m=1.0,
        single_track_reacquire_diameter_relative_error=0.25,
    ))
    tracker.update([
        HazardObservation(
            position=(0.0, 0.0, 0.3), confidence=0.9, view_id='left',
            depth_shape_status='spherical', apparent_diameter_m=0.30,
        ),
        HazardObservation(
            position=(2.0, 0.0, 0.3), confidence=0.9, view_id='right',
            depth_shape_status='spherical', apparent_diameter_m=0.30,
        ),
    ])
    tracker.update([])
    tracker.update([HazardObservation(
        position=(1.0, 0.0, 0.3), confidence=0.9, view_id='middle',
        depth_shape_status='spherical', apparent_diameter_m=0.30,
    )])

    assert len(tracker.tracks) == 3


def test_same_frame_duplicate_spherical_observations_create_one_track():
    """同一球的瞬时双轮廓不能绕过一帧一轨约束生成重复危险源。"""

    tracker = HazardTracker(HazardTrackerConfig(
        same_frame_duplicate_distance_m=0.12,
        same_frame_duplicate_diameter_relative_error=0.25,
    ))
    tracks = tracker.update([
        HazardObservation(
            position=(1.0, 2.0, 0.3),
            confidence=0.82,
            view_id='view-00',
            depth_shape_status='spherical',
            apparent_diameter_m=0.30,
        ),
        HazardObservation(
            position=(1.05, 2.0, 0.3),
            confidence=0.91,
            view_id='view-00',
            depth_shape_status='spherical',
            apparent_diameter_m=0.31,
        ),
    ], stamp_sec=1.0)

    assert len(tracks) == 1
    assert tracks[0].observation_count == 1
    assert tracks[0].confidence == 0.91
    assert tracks[0].position == (1.05, 2.0, 0.3)


def test_same_frame_separated_spherical_observations_remain_two_tracks():
    """相距 0.30 m 的两个真实球必须保留为两个目标。"""

    tracker = HazardTracker(HazardTrackerConfig(
        same_frame_duplicate_distance_m=0.12,
        same_frame_duplicate_diameter_relative_error=0.25,
    ))
    tracks = tracker.update([
        HazardObservation(
            position=(1.0, 2.0, 0.3),
            confidence=0.90,
            view_id='view-00',
            depth_shape_status='spherical',
            apparent_diameter_m=0.30,
        ),
        HazardObservation(
            position=(1.30, 2.0, 0.3),
            confidence=0.88,
            view_id='view-00',
            depth_shape_status='spherical',
            apparent_diameter_m=0.30,
        ),
    ], stamp_sec=1.0)

    assert len(tracks) == 2
