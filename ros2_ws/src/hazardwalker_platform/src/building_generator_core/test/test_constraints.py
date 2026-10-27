from __future__ import annotations

import unittest

from building_generator_core.constraints import BuildingConstraints


class BuildingConstraintsTest(unittest.TestCase):
    def test_supports_exact_and_range_values(self) -> None:
        constraints = BuildingConstraints.from_dict(
            {
                "seed": 7,
                "floor_count": {"min": 2, "max": 4},
                "rooms_per_floor": 6,
                "building_footprint_limit": {"width": 32.0, "length": 60.0},
            }
        )

        self.assertEqual(constraints.seed, 7)
        self.assertEqual(constraints.floor_count.min_value, 2)
        self.assertEqual(constraints.floor_count.max_value, 4)
        self.assertEqual(constraints.rooms_per_floor.exact_value, 6)
        self.assertEqual(constraints.building_footprint_limit["width"], 32.0)
        self.assertTrue(constraints.stair_required)
        self.assertTrue(constraints.elevator_required)
        self.assertEqual(constraints.origin_anchor, "main_entrance_center")


if __name__ == "__main__":
    unittest.main()
