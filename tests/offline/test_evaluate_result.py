import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'scripts'))

from evaluate_result import evaluate_result


def test_evaluate_result_accepts_valid_result():
    result = {
        'mission_id': 'test_run',
        'status': 'FINISHED',
        'hazards': [
            {'id': 1, 'status': 'confirmed', 'position': [1.0, 0.0, 0.5]},
        ],
        'metrics': {
            'duration_sec': 10.0,
            'return_success': True,
            'num_confirmed_hazards': 1,
        },
    }

    ok, errors, summary = evaluate_result(result)

    assert ok
    assert errors == []
    assert summary['confirmed_hazard_count'] == 1


def test_evaluate_result_rejects_mismatched_confirmed_count():
    result = {
        'mission_id': 'test_run',
        'status': 'FINISHED',
        'hazards': [
            {'id': 1, 'status': 'confirmed', 'position': [1.0, 0.0, 0.5]},
        ],
        'metrics': {
            'duration_sec': 10.0,
            'return_success': True,
            'num_confirmed_hazards': 0,
        },
    }

    ok, errors, _summary = evaluate_result(result)

    assert not ok
    assert errors
