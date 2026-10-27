from __future__ import annotations

import unittest

from building_generator_core.constraints import BuildingConstraints
from building_generator_core.generator import generate_layout


class LayoutGeneratorTest(unittest.TestCase):
    def test_generation_is_deterministic_and_reachable(self) -> None:
        constraints = BuildingConstraints.from_dict(
            {
                "seed": 11,
                "floor_count": 3,
                "rooms_per_floor": {"min": 4, "max": 6},
                "room_type_mix": {"office": 2, "storage": 1, "meeting": 1},
                "building_footprint_limit": {"width": 34.0, "length": 72.0},
            }
        )

        layout_a = generate_layout(constraints)
        layout_b = generate_layout(constraints)

        self.assertEqual(layout_a.signature, layout_b.signature)
        self.assertEqual(layout_a.entrance_pose, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertEqual(len(layout_a.floors), 3)

        for floor in layout_a.floors:
            self.assertGreaterEqual(len(floor.rooms), 4)
            self.assertTrue(floor.reachability["stair"])
            self.assertTrue(floor.reachability["elevator"])
            self.assertTrue(all(floor.reachability["rooms"].values()))

    def test_generation_uses_most_of_requested_building_length(self) -> None:
        constraints = BuildingConstraints.from_dict(
            {
                "seed": 77,
                "floor_count": 3,
                "rooms_per_floor": 6,
                "building_footprint_limit": {"width": 30.0, "length": 60.0},
            }
        )

        layout = generate_layout(constraints)

        for floor in layout.floors:
            self.assertGreaterEqual(floor.corridor_bounds.y_max, 57.0)
            self.assertGreaterEqual(max(room.bounds.y_max for room in floor.rooms), 57.0)
            self.assertLessEqual(floor.corridor_bounds.y_max, 60.0)
            self.assertLessEqual(max(room.bounds.y_max for room in floor.rooms), 60.0)

    def test_rooms_start_after_core_zone(self) -> None:
        constraints = BuildingConstraints.from_dict(
            {
                "seed": 21,
                "floor_count": 3,
                "rooms_per_floor": 6,
                "building_footprint_limit": {"width": 30.0, "length": 60.0},
            }
        )

        layout = generate_layout(constraints)

        for floor in layout.floors:
            core_end_y = max(floor.stair_bounds.y_max, floor.elevator_bounds.y_max)
            self.assertGreaterEqual(floor.corridor_bounds.y_min, core_end_y + 1.6)
            self.assertTrue(all(room.bounds.y_min >= floor.corridor_bounds.y_min for room in floor.rooms))

    def test_only_main_and_elevator_doors_are_generated(self) -> None:
        constraints = BuildingConstraints.from_dict(
            {
                "seed": 33,
                "floor_count": 2,
                "rooms_per_floor": 4,
                "building_footprint_limit": {"width": 30.0, "length": 60.0},
            }
        )

        layout = generate_layout(constraints)

        door_kinds = [door.kind for door in layout.door_specs]
        self.assertEqual(door_kinds.count("main_entrance"), 1)
        self.assertEqual(door_kinds.count("elevator"), len(layout.floors))
        self.assertNotIn("stair_fire", door_kinds)


if __name__ == "__main__":
    unittest.main()
