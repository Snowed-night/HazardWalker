"""主动重观察策略离线测试。

所属组：感知定位组 / 测试组。
文件作用：验证贴边、密集、小目标、圆度不稳和稳定候选的动作建议。
"""

import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_perception'))

from hazardwalker_perception.active_view_policy import bbox_iou, choose_active_view_action


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
    assert action.action == 'move_laterally'
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
    assert action.action == 'move_laterally'


def test_explicit_merged_candidate_requests_lateral_reobservation():
    candidate = _detection()
    candidate['requires_reobservation'] = True
    action = choose_active_view_action([candidate], 640, 480)
    assert action.action == 'move_laterally'
    assert action.priority == 92


def test_stable_candidate_holds_for_multi_frame_confirmation():
    action = choose_active_view_action([_detection()], 640, 480)
    assert action.action == 'hold_observation'


def test_bbox_iou_returns_overlap_ratio():
    first = {'x_min': 0, 'y_min': 0, 'x_max': 9, 'y_max': 9}
    second = {'x_min': 5, 'y_min': 0, 'x_max': 14, 'y_max': 9}
    assert bbox_iou(first, second) == 50.0 / 150.0
