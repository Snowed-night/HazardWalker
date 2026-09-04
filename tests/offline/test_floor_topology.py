"""楼层拓扑与动态房间计数的纯逻辑测试。"""

import math
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
NAV_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav'
if str(NAV_SRC) not in sys.path:
    sys.path.insert(0, str(NAV_SRC))

from hazardwalker_nav.floor_topology import (  # noqa: E402
    DoorwayObservation,
    RoomCompletionEvidence,
    build_far_to_near_visit_order,
    discover_room_doorways,
    room_completion_is_valid,
)


def _noisy_observations(doors, samples=4):
    result = []
    offsets = (-0.12, -0.04, 0.05, 0.11)
    for x_value, y_value in doors:
        for index in range(samples):
            offset = offsets[index]
            result.append(DoorwayObservation(
                x_value + offset,
                y_value - 0.5 * offset,
                confidence=0.9,
            ))
    return result


def test_discovers_four_rooms_from_noisy_observations_without_fixed_rows():
    observations = _noisy_observations([
        (-2.0, 8.3), (2.1, 8.1),
        (-1.9, 19.7), (2.0, 20.0),
    ])
    rooms = discover_room_doorways(
        observations, entry_origin=(0.0, 0.0), entry_axis=(0.0, 1.0),
        minimum_lateral_m=1.0, longitudinal_cluster_radius_m=0.8,
    )

    assert len(rooms) == 4
    assert {room.side for room in rooms} == {'left', 'right'}
    assert all(room.support_count == 4 for room in rooms)


def test_room_count_adapts_to_six_room_floor_and_does_not_assume_four():
    observations = _noisy_observations([
        (-2.2, 5.0), (2.1, 5.2),
        (-2.0, 14.0), (2.2, 13.8),
        (-2.1, 27.0), (2.0, 27.2),
    ])
    rooms = discover_room_doorways(
        observations, entry_origin=(0.0, 0.0), entry_axis=(0.0, 1.0),
        minimum_lateral_m=1.0,
    )

    assert len(rooms) == 6


def test_missing_opposite_room_is_not_mirrored_or_invented():
    observations = _noisy_observations([
        (-2.0, 8.0), (2.0, 8.0), (-2.0, 18.0),
    ])
    rooms = discover_room_doorways(
        observations, entry_origin=(0.0, 0.0), entry_axis=(0.0, 1.0),
        minimum_lateral_m=1.0,
    )

    assert len(rooms) == 3
    assert sum(room.side == 'right' for room in rooms) == 1


def test_topology_is_invariant_under_translation_and_rotation():
    base_doors = [(-2.0, 7.0), (2.0, 7.0), (-2.0, 18.0), (2.0, 18.0)]
    base = discover_room_doorways(
        _noisy_observations(base_doors),
        entry_origin=(0.0, 0.0), entry_axis=(0.0, 1.0),
        minimum_lateral_m=1.0,
    )
    # 逆时针旋转 90° 后平移 (11, -3)。
    transformed_doors = [(11.0 - y, -3.0 + x) for x, y in base_doors]
    transformed = discover_room_doorways(
        _noisy_observations(transformed_doors),
        entry_origin=(11.0, -3.0), entry_axis=(-1.0, 0.0),
        minimum_lateral_m=1.0,
    )

    assert len(base) == len(transformed) == 4
    assert [round(room.progress_m, 1) for room in base] == [
        round(room.progress_m, 1) for room in transformed]
    assert [room.side for room in base] == [room.side for room in transformed]


def test_visit_order_is_far_to_near_and_accepts_unpaired_rooms():
    observations = _noisy_observations([
        (-2.0, 6.0), (2.0, 6.2),
        (-2.0, 16.0),
        (-2.0, 29.0), (2.0, 29.1),
    ])
    rooms = discover_room_doorways(
        observations, entry_origin=(0.0, 0.0), entry_axis=(0.0, 1.0),
        minimum_lateral_m=1.0,
    )
    order = build_far_to_near_visit_order(rooms)

    assert len(order) == 5
    station_progress = []
    for room in order:
        if not station_progress or abs(station_progress[-1] - room.progress_m) > 1.0:
            station_progress.append(room.progress_m)
    assert station_progress == sorted(station_progress, reverse=True)


def test_room_completion_requires_entry_visibility_and_return_not_waypoint_count():
    valid = RoomCompletionEvidence(
        crossed_door=True,
        maximum_door_depth_m=3.2,
        visibility_coverage_ratio=0.94,
        observation_count=5,
        returned_to_door=True,
    )
    assert room_completion_is_valid(valid, 2.0, 0.90, 3)

    assert not room_completion_is_valid(
        RoomCompletionEvidence(
            crossed_door=False,
            maximum_door_depth_m=0.2,
            visibility_coverage_ratio=0.95,
            observation_count=8,
            returned_to_door=True,
        ),
        2.0, 0.90, 3,
    )
    assert not room_completion_is_valid(
        RoomCompletionEvidence(
            crossed_door=True,
            maximum_door_depth_m=3.0,
            visibility_coverage_ratio=0.35,
            observation_count=1,
            returned_to_door=True,
        ),
        2.0, 0.90, 3,
    )
