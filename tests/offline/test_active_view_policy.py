"""主动重观察策略离线测试。

所属组：感知定位组 / 测试组。
文件作用：验证贴边、密集、小目标、圆度不稳和稳定候选的动作建议。
"""

import os
import sys
from types import SimpleNamespace

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_perception'))

from hazardwalker_perception.active_view_policy import (
    annotate_detections_with_tracks,
    bbox_iou,
    choose_active_view_action,
)


def _detection(identifier='one', x_min=100, y_min=100, x_max=150, y_max=150,
               confidence=0.9, red_pixel_count=1200, circularity=0.9, depth_m=None):
    detection = {
        'id': identifier,
        'bbox': {'x_min': x_min, 'y_min': y_min, 'x_max': x_max, 'y_max': y_max},
        'confidence': confidence,
        'red_pixel_count': red_pixel_count,
        'shape': {'circularity': circularity},
    }
    if depth_m is not None:
        detection['depth_m'] = depth_m
    return detection


def test_empty_frame_keeps_exploring():
    action = choose_active_view_action([], 640, 480)
    assert action.action == 'continue_exploring'


def test_edge_candidate_turns_toward_candidate():
    action = choose_active_view_action([_detection(x_min=2, x_max=40)], 640, 480)
    assert action.action == 'turn_left'
    assert action.priority == 100


def test_overlapping_candidates_request_lateral_motion():
    first = _detection(identifier='first', x_min=200, y_min=100, x_max=280, y_max=180)
    second = _detection(identifier='second', x_min=240, y_min=120, x_max=320, y_max=200)
    action = choose_active_view_action([first, second], 640, 480)
    assert action.action == 'move_left'
    assert action.priority == 90


def test_edge_candidate_outranks_overlapping_candidates():
    edge = _detection(identifier='edge', x_min=1, x_max=80, y_min=100, y_max=180)
    overlap = _detection(identifier='overlap', x_min=30, x_max=110, y_min=110, y_max=190)
    action = choose_active_view_action([edge, overlap], 640, 480)
    assert action.action == 'turn_left'
    assert action.priority == 100


def test_small_candidate_requests_approach():
    action = choose_active_view_action([_detection(x_max=120, y_max=120, red_pixel_count=100)], 640, 480)
    assert action.action == 'move_forward'


def test_unstable_roundness_requests_side_view():
    action = choose_active_view_action([_detection(circularity=0.65)], 640, 480)
    assert action.action == 'move_left'


def test_explicit_merged_candidate_requests_lateral_reobservation():
    candidate = _detection()
    candidate['requires_reobservation'] = True
    action = choose_active_view_action([candidate], 640, 480)
    assert action.action == 'move_left'
    assert action.priority == 92


def test_partial_candidate_is_not_starved_by_higher_confidence_complete_ball():
    """同帧完整球不能凭高置信度阻塞局部球的主动复查。"""

    stable = _detection(
        identifier='stable', x_min=360, x_max=450,
        confidence=0.99, red_pixel_count=3000, circularity=0.98,
    )
    partial = _detection(
        identifier='partial', x_min=120, x_max=145,
        confidence=0.35, red_pixel_count=80, circularity=0.25,
    )
    partial['requires_reobservation'] = True

    action = choose_active_view_action([stable, partial], 640, 480)

    assert action.target_id == 'partial'
    assert action.priority == 92
    assert action.action == 'move_left'


def test_stable_candidate_requests_independent_side_view_before_confirmation():
    action = choose_active_view_action([_detection()], 640, 480)
    assert action.action == 'move_left'
    assert '圆柱' in action.reason


def test_excessive_normalized_depth_curvature_prioritizes_side_view():
    candidate = _detection()
    candidate['apparent_diameter_m'] = 0.30
    candidate['depth_shape'] = {'status': 'spherical', 'curvature_m': 0.15}

    action = choose_active_view_action([candidate], 640, 480)

    assert action.action == 'move_left'
    assert action.priority == 93
    assert '曲率' in action.reason


def test_bbox_iou_returns_overlap_ratio():
    first = {'x_min': 0, 'y_min': 0, 'x_max': 9, 'y_max': 9}
    second = {'x_min': 5, 'y_min': 0, 'x_max': 14, 'y_max': 9}
    assert bbox_iou(first, second) == 50.0 / 150.0


"""深度近似平面的红色圆形候选必须优先侧向复查，避免单视角圆柱误确认。"""
def test_flat_depth_candidate_requests_lateral_shape_recheck():
    candidate = _detection(identifier='cylinder_like', x_min=200, y_min=150, x_max=300, y_max=250)
    candidate['depth_shape'] = {'status': 'flat'}

    action = choose_active_view_action([candidate], 640, 480)

    assert action.action == 'move_left'
    assert '非球体' in action.reason


"""单轴曲率候选疑似圆柱侧面，必须优先获取独立侧视。"""
def test_anisotropic_depth_candidate_requests_lateral_shape_recheck():
    candidate = _detection(
        identifier='cylinder_side', x_min=200, y_min=150, x_max=300, y_max=250,
    )
    candidate['depth_shape'] = {'status': 'anisotropic'}

    action = choose_active_view_action([candidate], 640, 480)

    assert action.action == 'move_left'
    assert action.priority == 94
    assert '单轴曲面' in action.reason


def test_lateral_recheck_chooses_right_for_right_side_candidate():
    candidate = _detection(identifier='right', x_min=430, y_min=150, x_max=530, y_max=250)
    candidate['depth_shape'] = {'status': 'flat'}

    action = choose_active_view_action([candidate], 640, 480)

    assert action.action == 'move_right'
    assert '向右横移' in action.reason


def test_detection_uses_stable_track_id_and_preserves_rejected_status():
    tracks = [
        SimpleNamespace(
            track_id=7,
            position=(1.0, 2.0, 0.3),
            status='rejected_non_spherical',
        ),
    ]
    detections = [{
        'id': 1,
        'localized_position': [1.05, 2.0, 0.3],
        'bbox': {'x_min': 10, 'y_min': 10, 'x_max': 30, 'y_max': 30},
    }]

    annotated = annotate_detections_with_tracks(
        detections, tracks, merge_distance_m=0.5,
    )

    assert annotated[0]['id'] == '7'
    assert annotated[0]['track_id'] == '7'
    assert annotated[0]['track_status'] == 'rejected_non_spherical'


def test_partial_bbox_inherits_track_id_from_world_projection_without_3d_update():
    """完整框变成遮挡窄弧后，仍用旧世界轨迹保持同一复查 ID。"""

    tracks = [
        SimpleNamespace(
            track_id=7,
            position=(1.0, 2.0, 0.3),
            status='confirmed',
        ),
    ]
    detections = [{
        'id': 1,
        'bbox': {'x_min': 118, 'y_min': 80, 'x_max': 124, 'y_max': 120},
        'requires_reobservation': True,
    }]

    annotated = annotate_detections_with_tracks(
        detections,
        tracks,
        merge_distance_m=0.5,
        projected_tracks=[{
            'track_id': '7',
            'center_u': 100.0,
            'center_v': 100.0,
            'radius_px': 25.0,
            'depth_m': 2.0,
        }],
    )

    assert annotated[0]['id'] == '7'
    assert annotated[0]['track_id'] == '7'
    assert annotated[0]['track_association'] == 'image_projection'


def test_track_association_is_one_to_one_with_two_same_frame_candidates():
    tracks = [
        SimpleNamespace(track_id=3, position=(1.0, 0.0, 0.3), status='confirmed'),
    ]
    detections = [
        {
            'id': 'near',
            'localized_position': [1.01, 0.0, 0.3],
            'bbox': {'x_min': 90, 'y_min': 90, 'x_max': 110, 'y_max': 110},
        },
        {
            'id': 'duplicate',
            'localized_position': [1.02, 0.0, 0.3],
            'bbox': {'x_min': 91, 'y_min': 91, 'x_max': 111, 'y_max': 111},
        },
    ]

    annotated = annotate_detections_with_tracks(
        detections, tracks, merge_distance_m=0.5,
    )

    assert sum(item['track_id'] == '3' for item in annotated) == 1
    assert sum(item['track_status'] == 'untracked' for item in annotated) == 1


def test_ambiguous_projected_tracks_fail_closed_without_guessing_id():
    tracks = [
        SimpleNamespace(track_id=1, position=(1.0, 0.0, 0.3), status='confirmed'),
        SimpleNamespace(track_id=2, position=(1.2, 0.0, 0.3), status='confirmed'),
    ]
    detections = [{
        'id': 'partial',
        'bbox': {'x_min': 98, 'y_min': 90, 'x_max': 104, 'y_max': 110},
    }]
    projections = [
        {'track_id': '1', 'center_u': 100.0, 'center_v': 100.0, 'radius_px': 25.0},
        {'track_id': '2', 'center_u': 102.0, 'center_v': 100.0, 'radius_px': 25.0},
    ]

    annotated = annotate_detections_with_tracks(
        detections, tracks, merge_distance_m=0.5, projected_tracks=projections,
    )

    assert annotated[0]['track_status'] == 'untracked'
    assert annotated[0]['track_association'] == 'none'
