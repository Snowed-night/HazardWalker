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
