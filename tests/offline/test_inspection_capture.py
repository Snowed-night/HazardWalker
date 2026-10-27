"""观察点采帧门控的离线行为测试。"""

from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
PERCEPTION_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
if str(PERCEPTION_SRC) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SRC))

from hazardwalker_perception.inspection_capture import (  # noqa: E402
    InspectionCaptureGate,
)


def _observe(gate, frame_index, **overrides):
    values = {
        'frame_index': frame_index,
        'stamp_sec': 10.0 + frame_index,
        'camera_stable': True,
        'depth_synchronized': True,
        'tf_synchronized': True,
        'localization_ready': True,
        'hazard_count': 0,
        'detection_count': 0,
    }
    values.update(overrides)
    return gate.observe_frame(**values)


def test_requires_a_new_frame_after_request():
    gate = InspectionCaptureGate()
    assert gate.accept_request({
        'goal_id': 'g1', 'obstacle_id': 'o1', 'floor': 0,
        'sector': 'far_left',
    }, current_frame_index=8)

    assert _observe(gate, 8) is None
    result = _observe(gate, 9)
    assert result['success'] is True
    assert result['goal_id'] == 'g1'


def test_unstable_or_unsynchronized_frame_never_acknowledges_goal():
    gate = InspectionCaptureGate()
    gate.accept_request({'goal_id': 'g1'}, current_frame_index=2)

    assert _observe(gate, 3, camera_stable=False) is None
    assert _observe(gate, 4, depth_synchronized=False) is None
    assert _observe(gate, 5, tf_synchronized=False) is None
    assert _observe(gate, 6, localization_ready=False) is None
    assert gate.pending.goal_id == 'g1'


def test_idempotent_retry_does_not_move_fresh_frame_boundary():
    gate = InspectionCaptureGate()
    assert gate.accept_request({'goal_id': 'g1'}, current_frame_index=10)
    assert gate.accept_request({'goal_id': 'g1'}, current_frame_index=20)

    result = _observe(gate, 11)
    assert result is not None
    assert result['frame_index'] == 11


def test_completed_or_unknown_goal_cannot_be_replayed():
    gate = InspectionCaptureGate()
    assert not gate.accept_request({'goal_id': ''}, current_frame_index=0)
    gate.accept_request({'goal_id': 'g1'}, current_frame_index=0)
    assert _observe(gate, 1) is not None
    assert not gate.accept_request({'goal_id': 'g1'}, current_frame_index=2)


def test_zero_detections_is_still_a_valid_processed_observation():
    gate = InspectionCaptureGate()
    gate.accept_request({'goal_id': 'empty-view'}, current_frame_index=3)
    result = _observe(
        gate, 4, hazard_count=0, detection_count=0,
    )

    assert result['success'] is True
    assert result['hazard_count'] == 0
    assert result['detection_count'] == 0
