from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

from building_generator_core.constraints import BuildingConstraints
from building_generator_core.exporter import export_sdf
from building_generator_core.generator import generate_layout
import yaml


class ExporterTest(unittest.TestCase):
    def test_export_writes_world_model_and_metadata_files(self) -> None:
        constraints = BuildingConstraints.from_dict(
            {
                "seed": 19,
                "floor_count": 2,
                "rooms_per_floor": 4,
                "building_footprint_limit": {"width": 28.0, "length": 54.0},
            }
        )
        layout = generate_layout(constraints)

        with tempfile.TemporaryDirectory() as tmpdir:
            artifact_paths = export_sdf(layout, target="gazebo_classic", output_dir=tmpdir)

            expected_files = {
                Path(artifact_paths.world_sdf),
                Path(artifact_paths.model_sdf),
                Path(artifact_paths.layout_metadata),
                Path(artifact_paths.elevator_config),
                Path(artifact_paths.door_config),
                Path(artifact_paths.validation_report),
            }
            self.assertTrue(all(path.exists() for path in expected_files))

            world_text = Path(artifact_paths.world_sdf).read_text()
            model_text = Path(artifact_paths.model_sdf).read_text()
            validation_report = json.loads(Path(artifact_paths.validation_report).read_text())
            self.assertIn("<world name=", world_text)
            self.assertIn("generated_building", world_text)
            self.assertIn('<sdf version="1.7">', world_text)
            self.assertIn('<sdf version="1.7">', model_text)
            self.assertEqual(validation_report["status"], "pass")
            self.assertTrue(all(item["status"] == "pass" for item in validation_report["checks"]))

    def test_stair_core_is_open_and_has_no_stair_door(self) -> None:
        constraints = BuildingConstraints.from_dict(
            {
                "seed": 19,
                "floor_count": 2,
                "rooms_per_floor": 4,
                "building_footprint_limit": {"width": 28.0, "length": 54.0},
            }
        )
        layout = generate_layout(constraints)

        with tempfile.TemporaryDirectory() as tmpdir:
            artifact_paths = export_sdf(layout, target="gazebo_classic", output_dir=tmpdir)
            root = ET.parse(artifact_paths.world_sdf).getroot()

            building_model = next(model for model in root.findall(".//model") if model.get("name") == "generated_building")
            link_names = {link.get("name") for link in building_model.findall("link")}

            self.assertNotIn("stair_core_floor_0_east_lower", link_names)
            self.assertNotIn("stair_core_floor_0_east_upper", link_names)
            self.assertNotIn("dynamic_stair_fire_floor_0", Path(artifact_paths.world_sdf).read_text())
            self.assertTrue(all(door.kind != "stair_fire" for door in layout.door_specs))

    def test_second_stair_flight_connects_turn_and_upper_landings(self) -> None:
        constraints = BuildingConstraints.from_dict(
            {
                "seed": 19,
                "floor_count": 3,
                "rooms_per_floor": 4,
                "building_footprint_limit": {"width": 28.0, "length": 54.0},
            }
        )
        layout = generate_layout(constraints)

        with tempfile.TemporaryDirectory() as tmpdir:
            artifact_paths = export_sdf(layout, target="gazebo_classic", output_dir=tmpdir)
            root = ET.parse(artifact_paths.world_sdf).getroot()
            building_model = next(model for model in root.findall(".//model") if model.get("name") == "generated_building")

            def bounds(name: str):
                link = next(link for link in building_model.findall("link") if link.get("name") == name)
                pose = [float(value) for value in link.findtext("pose").split()]
                size = [float(value) for value in link.find("./collision/geometry/box/size").text.split()]
                return (
                    pose[1] - size[1] / 2.0,
                    pose[1] + size[1] / 2.0,
                    pose[2] - size[2] / 2.0,
                    pose[2] + size[2] / 2.0,
                )

            turn_y_min, turn_y_max, turn_z_min, turn_z_max = bounds("stair_turn_landing_floor_0")
            first_y_min, first_y_max, first_z_min, first_z_max = bounds("stair_flight_b_floor_0_step_0")
            last_y_min, last_y_max, last_z_min, last_z_max = bounds("stair_flight_b_floor_0_step_9")
            upper_y_min, upper_y_max, upper_z_min, upper_z_max = bounds("stair_floor_landing_floor_1")

            self.assertAlmostEqual(first_y_max, turn_y_min, places=3)
            self.assertAlmostEqual(first_z_min, turn_z_max, places=3)
            self.assertAlmostEqual(last_y_min, upper_y_max, places=3)
            self.assertAlmostEqual(last_z_max, upper_z_max, places=3)

    def test_stair_floor_landing_reaches_core_opening_edge(self) -> None:
        constraints = BuildingConstraints.from_dict(
            {
                "seed": 19,
                "floor_count": 2,
                "rooms_per_floor": 4,
                "building_footprint_limit": {"width": 28.0, "length": 54.0},
            }
        )
        layout = generate_layout(constraints)

        with tempfile.TemporaryDirectory() as tmpdir:
            artifact_paths = export_sdf(layout, target="gazebo_classic", output_dir=tmpdir)
            root = ET.parse(artifact_paths.world_sdf).getroot()
            building_model = next(model for model in root.findall(".//model") if model.get("name") == "generated_building")

            landing = next(link for link in building_model.findall("link") if link.get("name") == "stair_floor_landing_floor_0")
            pose = [float(value) for value in landing.findtext("pose").split()]
            size = [float(value) for value in landing.find("./collision/geometry/box/size").text.split()]
            landing_x_max = pose[0] + size[0] / 2.0

            self.assertAlmostEqual(landing_x_max, layout.floors[0].stair_bounds.x_max - 0.09, places=3)

    def test_elevator_open_panels_clear_the_doorway(self) -> None:
        constraints = BuildingConstraints.from_dict(
            {
                "seed": 77,
                "floor_count": 2,
                "rooms_per_floor": 4,
                "building_footprint_limit": {"width": 30.0, "length": 60.0},
            }
        )
        layout = generate_layout(constraints)

        with tempfile.TemporaryDirectory() as tmpdir:
            artifact_paths = export_sdf(layout, target="gazebo_classic", output_dir=tmpdir)
            door_config = yaml.safe_load(Path(artifact_paths.door_config).read_text())
            elevator_door = next(item for item in door_config["doors"] if item["kind"] == "elevator")

            left_open = elevator_door["panel_poses"]["left_open"]
            right_open = elevator_door["panel_poses"]["right_open"]
            panel_width = elevator_door["width"] / 2.0
            required_offset = elevator_door["width"] / 2.0 + panel_width / 2.0

            self.assertLess(left_open[0], 0.0)
            self.assertLess(right_open[0], 0.0)
            self.assertGreater(abs(left_open[1]), required_offset)
            self.assertGreater(abs(right_open[1]), required_offset)
            self.assertEqual(elevator_door["motion_duration"], 60.0)

    def test_elevator_car_opening_faces_the_lobby_door_side(self) -> None:
        constraints = BuildingConstraints.from_dict(
            {
                "seed": 77,
                "floor_count": 2,
                "rooms_per_floor": 4,
                "building_footprint_limit": {"width": 30.0, "length": 60.0},
            }
        )
        layout = generate_layout(constraints)

        with tempfile.TemporaryDirectory() as tmpdir:
            artifact_paths = export_sdf(layout, target="gazebo_classic", output_dir=tmpdir)
            root = ET.parse(artifact_paths.world_sdf).getroot()
            elevator_model = next(model for model in root.findall(".//model") if model.get("name") == "dynamic_elevator_main")
            model_pose = [float(value) for value in elevator_model.findtext("pose").split()]
            self.assertAlmostEqual(model_pose[5], -1.5707963267948966, places=4)

            car_link = elevator_model.find("link")
            wall_links = {item.get("name"): item for item in car_link.findall("collision")}
            self.assertNotIn("front_collision", wall_links)
            self.assertIn("back_collision", wall_links)

            floor_collision = next(item for item in car_link.findall("collision") if item.get("name") == "car_floor_collision")
            floor_pose = [float(value) for value in floor_collision.findtext("pose").split()]
            floor_size = [float(value) for value in floor_collision.find("./geometry/box/size").text.split()]
            floor_top_z = model_pose[2] + floor_pose[2] + floor_size[2] / 2.0
            self.assertAlmostEqual(floor_top_z, layout.floors[0].elevation, places=4)

            elevator_door = next(door for door in layout.door_specs if door.id == "elevator_floor_0")
            car_front_x = model_pose[0] - floor_size[1] / 2.0
            self.assertAlmostEqual(car_front_x - elevator_door.pose[0], 0.06, places=4)

    def test_elevator_door_has_floor_infill_between_threshold_and_car(self) -> None:
        constraints = BuildingConstraints.from_dict(
            {
                "seed": 77,
                "floor_count": 2,
                "rooms_per_floor": 4,
                "building_footprint_limit": {"width": 30.0, "length": 60.0},
            }
        )
        layout = generate_layout(constraints)

        with tempfile.TemporaryDirectory() as tmpdir:
            artifact_paths = export_sdf(layout, target="gazebo_classic", output_dir=tmpdir)
            root = ET.parse(artifact_paths.world_sdf).getroot()
            building_model = next(model for model in root.findall(".//model") if model.get("name") == "generated_building")
            infill = next(link for link in building_model.findall("link") if link.get("name") == "elevator_sill_infill_floor_0")
            pose = [float(value) for value in infill.findtext("pose").split()]
            size = [float(value) for value in infill.find("./collision/geometry/box/size").text.split()]

            self.assertGreater(size[0], 0.03)
            self.assertAlmostEqual(pose[2] + size[2] / 2.0, layout.floors[0].elevation, places=4)
            self.assertAlmostEqual(pose[0] - size[0] / 2.0, layout.floors[0].elevator_bounds.x_min, places=4)


if __name__ == "__main__":
    unittest.main()
