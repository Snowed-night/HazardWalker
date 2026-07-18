"""危险源多帧确认和去重离线测试。

所属组：感知组 / 测试组。
文件作用：
验证 `track_hazards.py` 能按三维距离合并观测、确认稳定目标、拒绝长期丢失目标。
不依赖 ROS、Gazebo 或真实相机。
"""
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
