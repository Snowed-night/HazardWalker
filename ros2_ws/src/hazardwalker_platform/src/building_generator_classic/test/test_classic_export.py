from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from building_generator_classic.classic_export import export_classic_bundle
from building_generator_core.constraints import BuildingConstraints
from building_generator_core.generator import generate_layout


class ClassicExportTest(unittest.TestCase):
    def test_export_classic_bundle_writes_manifest(self) -> None:
        layout = generate_layout(
            BuildingConstraints.from_dict(
                {
                    "seed": 31,
                    "floor_count": 2,
                    "rooms_per_floor": 5,
                    "building_footprint_limit": {"width": 30.0, "length": 56.0},
                }
            )
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            manifest_path = export_classic_bundle(layout, tmpdir)
            manifest_text = Path(manifest_path).read_text()

        self.assertIn("world_sdf:", manifest_text)
        self.assertIn("door_config:", manifest_text)
        self.assertIn("validation_report:", manifest_text)


if __name__ == "__main__":
    unittest.main()
