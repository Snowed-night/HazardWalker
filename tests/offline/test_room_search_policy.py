"""房间搜索视角状态机离线回归。"""

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
sys.path.insert(0, str(PACKAGE_ROOT))

from hazardwalker_perception.room_search_policy import (  # noqa: E402
    RoomSearchPolicyConfig,
    choose_fixed_sweep_action,
    choose_room_search_action,
    coverage_ratio,
)


def _candidate(x_min=280, x_max=360, requires_reobservation=False):
    return {
        'id': 'candidate-1',
        'bbox': {'x_min': x_min, 'y_min': 180, 'x_max': x_max, 'y_max': 280},
        'confidence': 0.95,
        'red_pixel_count': 1200,
        'shape': {'circularity': 0.92},
        'requires_reobservation': requires_reobservation,
        'depth_shape': {'status': 'spherical', 'curvature_m': 0.04},
        'apparent_diameter_m': 0.24,
    }


def test_candidate_recheck_preempts_uncovered_sector():
    result = choose_room_search_action(
        [_candidate()], 640, 480, {2}, 2, {'candidate-1': 0},
    )

    assert result.mode == 'candidate_recheck'
    assert result.action == 'move_left'
    assert result.target_id == 'candidate-1'
    assert result.priority > 100


def test_recheck_budget_prevents_distractor_starvation():
    result = choose_room_search_action(
        [_candidate()], 640, 480, {2}, 2, {'candidate-1': 2},
    )

    assert result.mode == 'coverage'
    assert result.target_sector == 1
    assert result.action == 'turn_left'


def test_current_unvisited_sector_must_stabilize_before_turning():
    result = choose_room_search_action([], 640, 480, set(), 2)

    assert result.action == 'hold_observation'
    assert result.target_sector == 2


def test_nearest_unvisited_sector_reduces_turning():
    result = choose_room_search_action([], 640, 480, {1, 2}, 2)

    assert result.target_sector == 3
    assert result.action == 'turn_right'


def test_all_sectors_covered_finishes_search():
    result = choose_room_search_action([], 640, 480, {0, 1, 2, 3, 4}, 4)

    assert result.action == 'search_complete'
    assert result.mode == 'complete'


def test_fixed_sweep_ignores_candidate_order_and_visits_first_remaining():
    result = choose_fixed_sweep_action({0}, 3, order=(0, 1, 2, 3, 4))

    assert result.mode == 'fixed_sweep'
    assert result.target_sector == 1
    assert result.action == 'turn_left'


def test_coverage_ratio_deduplicates_and_ignores_invalid_sectors():
    assert coverage_ratio({-1, 0, 0, 2, 9}, sector_count=5) == 0.4


def test_invalid_sector_count_is_rejected():
    try:
        choose_room_search_action(
            [], 640, 480, set(), 0, config=RoomSearchPolicyConfig(sector_count=0),
        )
    except ValueError as exc:
        assert 'sector_count' in str(exc)
    else:
        raise AssertionError('expected ValueError')
