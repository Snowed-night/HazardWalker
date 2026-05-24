import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_decision'))

from hazardwalker_decision.result_builder import build_mission_result


def test_build_mission_result_counts_confirmed_hazards():
    result = build_mission_result(
        mission_id='test_run',
        status='FINISHED',
        hazards=[
            {'id': 1, 'position': [1.0, 2.0, 0.5], 'confidence': 0.9},
            {'id': 2, 'position': [3.0, 2.0, 0.5], 'confidence': 0.7, 'status': 'tentative'},
        ],
        duration_sec=12.5,
        return_success=True,
    )

    assert result['mission_id'] == 'test_run'
    assert result['status'] == 'FINISHED'
    assert len(result['hazards']) == 2
    assert result['hazards'][0]['status'] == 'confirmed'
    assert result['hazards'][1]['status'] == 'tentative'
    assert result['metrics']['num_confirmed_hazards'] == 1
    assert result['metrics']['return_success'] is True
