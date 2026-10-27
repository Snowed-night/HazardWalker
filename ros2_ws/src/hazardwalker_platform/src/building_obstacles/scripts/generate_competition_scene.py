#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import random
import sys
import xml.dom.minidom
import xml.etree.ElementTree as ET


def _ensure_generator_on_path() -> None:
    try:
        import building_generator_core  # noqa: F401
        return
    except ImportError:
        pass

    scripts_dir = Path(__file__).resolve().parent
    src_dir = scripts_dir.parent.parent
    candidate = src_dir / "building_generator_core"
    if candidate.exists():
        sys.path.insert(0, str(candidate))


_ensure_generator_on_path()

from building_generator_core.constraints import BuildingConstraints
from building_generator_core.exporter import export_sdf
from building_generator_core.generator import generate_layout
from building_generator_core.layout import BuildingLayout, FurnitureSpec, RoomSpec


DANGER_RADIUS = 0.15
DISTRACTOR_SPHERE_RADIUS = 0.15
DISTRACTOR_BOX_SIZE = (0.30, 0.30, 0.30)
ROOM_WALL_CLEARANCE = 0.45
FURNITURE_CLEARANCE = 0.35
SOURCE_CLEARANCE = 0.35
DOOR_KEEP_OUT_DEPTH = 1.55
DOOR_KEEP_OUT_HALF_WIDTH = 1.05
MAX_PLACEMENT_ATTEMPTS = 8000


@dataclass(frozen=True)
class SourceKind:
    kind: str
    shape: str
    color: str
    is_danger: bool
    radius: float | None = None
    size: tuple[float, float, float] | None = None

    @property
    def placement_radius(self) -> float:
        if self.radius is not None:
            return self.radius
        assert self.size is not None
        return math.hypot(self.size[0] / 2.0, self.size[1] / 2.0)

    @property
    def center_z_offset(self) -> float:
        if self.radius is not None:
            return self.radius
        assert self.size is not None
        return self.size[2] / 2.0


DANGER_KIND = SourceKind(
    kind="danger_red_sphere",
    shape="sphere",
    color="red",
    is_danger=True,
    radius=DANGER_RADIUS,
)
DISTRACTOR_KINDS = [
    SourceKind(
        kind="distractor_green_sphere",
        shape="sphere",
        color="green",
        is_danger=False,
        radius=DISTRACTOR_SPHERE_RADIUS,
    ),
    SourceKind(
        kind="distractor_red_box",
        shape="box",
        color="red",
        is_danger=False,
        size=DISTRACTOR_BOX_SIZE,
    ),
]


@dataclass(frozen=True)
class PlacedSource:
    object_id: int
    name: str
    source_kind: SourceKind
    room_id: str
    floor_index: int
    position: tuple[float, float, float]

    @property
    def placement_radius(self) -> float:
        return self.source_kind.placement_radius

    def as_truth_entry(self) -> dict[str, object]:
        entry: dict[str, object] = {
            "id": self.object_id,
            "model_name": self.name,
            "position": [round(value, 3) for value in self.position],
            "floor_index": self.floor_index,
            "room_id": self.room_id,
            "color": self.source_kind.color,
            "shape": self.source_kind.shape,
            "is_danger": self.source_kind.is_danger,
        }
        if self.source_kind.radius is not None:
            entry["radius"] = self.source_kind.radius
        if self.source_kind.size is not None:
            entry["size"] = list(self.source_kind.size)
        return entry


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    seed = args.seed if args.seed is not None else random.SystemRandom().randint(1, 2**31 - 1)
    output_dir = Path(args.output_dir).resolve()
    results_dir = Path(args.results_dir).resolve() if args.results_dir else output_dir.parent / "results"
    output_dir.mkdir(parents=True, exist_ok=True)
    results_dir.mkdir(parents=True, exist_ok=True)

    constraints = BuildingConstraints.from_dict(
        {
            "seed": seed,
            "floor_count": _parse_count_spec(args.floor_count),
            "rooms_per_floor": _parse_count_spec(args.rooms_per_floor),
            "building_footprint_limit": {
                "width": args.width,
                "length": args.length,
            },
        }
    )
    layout = generate_layout(constraints)
    artifact_paths = export_sdf(layout, target="gazebo_classic", output_dir=output_dir)

    obstacle_rng = random.Random(seed ^ 0x5EED5EED)
    danger_count = _sample_count(args.danger_count, obstacle_rng)
    distractor_count = _sample_count(args.distractor_count, obstacle_rng)
    sources = _place_sources(layout, obstacle_rng, danger_count, distractor_count)

    world_path = output_dir / "competition_scene.world"
    _write_world_with_sources(Path(artifact_paths.world_sdf), world_path, sources)
    # Keep world.sdf as the full competition world as well; model.sdf remains the bare building model.
    Path(artifact_paths.world_sdf).write_text(world_path.read_text(encoding="utf-8"), encoding="utf-8")

    truth_data = _build_truth_data(layout, seed, sources)
    truth_path = results_dir / "danger_truth.json"
    output_truth_path = output_dir / "danger_truth.json"
    truth_json = json.dumps(truth_data, indent=2, ensure_ascii=False) + "\n"
    truth_path.write_text(truth_json, encoding="utf-8")
    output_truth_path.write_text(truth_json, encoding="utf-8")

    building_config_path = output_dir / "building_config.json"
    building_config_path.write_text(
        json.dumps(_build_building_config(layout, seed, world_path, sources), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    manifest = {
        "schema": "competition_scene_manifest_v1",
        "seed": seed,
        "world_file": str(world_path),
        "building_config": str(building_config_path),
        "layout_metadata": artifact_paths.layout_metadata,
        "door_config": artifact_paths.door_config,
        "elevator_config": artifact_paths.elevator_config,
        "validation_report": artifact_paths.validation_report,
        "truth_file": str(truth_path),
        "output_truth_file": str(output_truth_path),
        "danger_count": danger_count,
        "distractor_count": distractor_count,
        "source_count": len(sources),
        "robot_start": {
            "x": args.robot_x,
            "y": args.robot_y,
            "z": args.robot_z,
            "yaw": args.robot_yaw,
        },
        "competition_interfaces": {
            "velocity_command_topic": "/cmd_vel",
            "odometry_topic": "/Odometry_gazebo",
            "truth_file_for_referee": str(truth_path),
            "team_detection_file": str(results_dir / "detected_danger.json"),
        },
    }
    manifest_path = output_dir / "scene_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate a competition Gazebo world with one random building and colocated source objects."
    )
    parser.add_argument("--output-dir", required=True, help="Directory for generated world and metadata.")
    parser.add_argument("--results-dir", help="Directory for referee truth and team result files.")
    parser.add_argument("--seed", type=int, help="Scene seed. If omitted, a random seed is selected and recorded.")
    parser.add_argument("--floor-count", default="3", help="Exact value or min:max range.")
    parser.add_argument("--rooms-per-floor", default="4", help="Exact value or min:max range.")
    parser.add_argument("--width", type=float, default=20.0)
    parser.add_argument("--length", type=float, default=36.0)
    parser.add_argument("--danger-count", default="3:6", help="Exact value or min:max range.")
    parser.add_argument("--distractor-count", default="4:8", help="Exact value or min:max range.")
    parser.add_argument("--robot-x", type=float, default=0.0)
    parser.add_argument("--robot-y", type=float, default=-2.2)
    parser.add_argument("--robot-z", type=float, default=0.6)
    parser.add_argument("--robot-yaw", type=float, default=1.5708)
    return parser


def _parse_count_spec(raw_value: str):
    if ":" in str(raw_value):
        min_value, max_value = str(raw_value).split(":", 1)
        return {"min": int(min_value), "max": int(max_value)}
    return int(raw_value)


def _sample_count(raw_value: str, rng: random.Random) -> int:
    spec = str(raw_value)
    if ":" in spec:
        min_value, max_value = spec.split(":", 1)
        return rng.randint(int(min_value), int(max_value))
    return int(spec)


def _place_sources(
    layout: BuildingLayout,
    rng: random.Random,
    danger_count: int,
    distractor_count: int,
) -> list[PlacedSource]:
    rooms = [room for floor in layout.floors for room in floor.rooms]
    if not rooms:
        raise ValueError("cannot place sources: generated building has no rooms")

    source_plan = [DANGER_KIND for _ in range(danger_count)]
    source_plan.extend(rng.choice(DISTRACTOR_KINDS) for _ in range(distractor_count))
    rng.shuffle(source_plan)

    placed: list[PlacedSource] = []
    danger_index = 0
    distractor_index = 0
    for object_id, source_kind in enumerate(source_plan):
        if source_kind.is_danger:
            name = f"danger_red_sphere_{danger_index:02d}"
            danger_index += 1
        else:
            name = f"{source_kind.kind}_{distractor_index:02d}"
            distractor_index += 1
        placed.append(_place_one_source(layout, rooms, rng, object_id, name, source_kind, placed))
    return placed


def _place_one_source(
    layout: BuildingLayout,
    rooms: list[RoomSpec],
    rng: random.Random,
    object_id: int,
    name: str,
    source_kind: SourceKind,
    placed: list[PlacedSource],
) -> PlacedSource:
    radius = source_kind.placement_radius
    margin = ROOM_WALL_CLEARANCE + radius
    for _ in range(MAX_PLACEMENT_ATTEMPTS):
        room = rng.choice(rooms)
        if room.bounds.width <= margin * 2.0 or room.bounds.length <= margin * 2.0:
            continue

        x = rng.uniform(room.bounds.x_min + margin, room.bounds.x_max - margin)
        y = rng.uniform(room.bounds.y_min + margin, room.bounds.y_max - margin)
        if _blocks_room_door(room, x, y, radius):
            continue
        if _overlaps_furniture(room.furniture, x, y, radius):
            continue

        z = float(room.goal_pose[2]) + source_kind.center_z_offset
        candidate = PlacedSource(
            object_id=object_id,
            name=name,
            source_kind=source_kind,
            room_id=room.id,
            floor_index=room.floor_index,
            position=(x, y, z),
        )
        if _overlaps_existing_sources(candidate, placed):
            continue
        return candidate

    raise ValueError(
        f"failed to place source '{name}' without overlap after {MAX_PLACEMENT_ATTEMPTS} attempts"
    )


def _blocks_room_door(room: RoomSpec, x: float, y: float, radius: float) -> bool:
    door_x = float(room.door_pose[0])
    door_y = float(room.door_pose[1])
    half_width = DOOR_KEEP_OUT_HALF_WIDTH + radius
    depth = DOOR_KEEP_OUT_DEPTH + radius
    if abs(y - door_y) > half_width:
        return False
    if room.side == "left":
        return door_x - depth <= x <= door_x
    return door_x <= x <= door_x + depth


def _overlaps_furniture(furniture_items: list[FurnitureSpec], x: float, y: float, radius: float) -> bool:
    clearance = FURNITURE_CLEARANCE + radius
    for item in furniture_items:
        item_x = float(item.pose[0])
        item_y = float(item.pose[1])
        half_x = float(item.size[0]) / 2.0 + clearance
        half_y = float(item.size[1]) / 2.0 + clearance
        if abs(x - item_x) <= half_x and abs(y - item_y) <= half_y:
            return True
    return False


def _overlaps_existing_sources(candidate: PlacedSource, placed: list[PlacedSource]) -> bool:
    for other in placed:
        min_distance = candidate.placement_radius + other.placement_radius + SOURCE_CLEARANCE
        dx = candidate.position[0] - other.position[0]
        dy = candidate.position[1] - other.position[1]
        dz = candidate.position[2] - other.position[2]
        if math.sqrt(dx * dx + dy * dy + dz * dz) < min_distance:
            return True
    return False


def _write_world_with_sources(source_world: Path, destination_world: Path, sources: list[PlacedSource]) -> None:
    root = ET.parse(source_world).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"world element not found in {source_world}")
    for source in sources:
        world.append(_build_source_model(source))
    destination_world.write_text(_to_pretty_xml(root), encoding="utf-8")


def _build_source_model(source: PlacedSource) -> ET.Element:
    model = ET.Element("model", {"name": source.name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = _pose_text((*source.position, 0.0, 0.0, 0.0))
    link = ET.SubElement(model, "link", {"name": "link"})
    _append_source_geometry(link, "collision", source, visual=False)
    _append_source_geometry(link, "visual", source, visual=True)
    return model


def _append_source_geometry(link: ET.Element, tag: str, source: PlacedSource, *, visual: bool) -> None:
    element = ET.SubElement(link, tag, {"name": tag})
    geometry = ET.SubElement(element, "geometry")
    if source.source_kind.shape == "sphere":
        sphere = ET.SubElement(geometry, "sphere")
        ET.SubElement(sphere, "radius").text = f"{source.source_kind.radius:.3f}"
    elif source.source_kind.shape == "box":
        box = ET.SubElement(geometry, "box")
        assert source.source_kind.size is not None
        ET.SubElement(box, "size").text = _vector_text(source.source_kind.size)
    else:
        raise ValueError(f"unsupported source shape: {source.source_kind.shape}")

    if visual:
        material = ET.SubElement(element, "material")
        color = _source_color(source.source_kind.color)
        ET.SubElement(material, "ambient").text = color
        ET.SubElement(material, "diffuse").text = color


def _source_color(color: str) -> str:
    if color == "red":
        return "1.0 0.0 0.0 1.0"
    if color == "green":
        return "0.0 1.0 0.0 1.0"
    raise ValueError(f"unsupported source color: {color}")


def _build_truth_data(layout: BuildingLayout, seed: int, sources: list[PlacedSource]) -> dict[str, object]:
    danger_sources = [source.as_truth_entry() for source in sources if source.source_kind.is_danger]
    distraction_sources = [source.as_truth_entry() for source in sources if not source.source_kind.is_danger]
    return {
        "schema": "competition_danger_truth_v1",
        "seed": seed,
        "building": {
            "model_name": layout.model_name,
            "floor_count": len(layout.floors),
            "floor_heights": [floor.elevation for floor in layout.floors],
            "footprint": layout.footprint,
        },
        "source_rules": {
            "danger": "red sphere",
            "distractors": ["red box", "green sphere"],
            "placement": "room interior only; avoids walls, furniture, source overlap, and room door keep-out areas",
        },
        "danger_sources": danger_sources,
        "distraction_sources": distraction_sources,
    }


def _build_building_config(
    layout: BuildingLayout,
    seed: int,
    world_path: Path,
    sources: list[PlacedSource],
) -> dict[str, object]:
    return {
        "schema": "competition_building_config_v1",
        "seed": seed,
        "model_name": layout.model_name,
        "world_file": str(world_path),
        "num_floors": len(layout.floors),
        "floor_heights": [floor.elevation for floor in layout.floors],
        "building_width": layout.footprint["width"],
        "building_depth": layout.footprint["length"],
        "room_count": sum(len(floor.rooms) for floor in layout.floors),
        "danger_count": sum(1 for source in sources if source.source_kind.is_danger),
        "distractor_count": sum(1 for source in sources if not source.source_kind.is_danger),
        "entrance_pose": list(layout.entrance_pose),
        "target_points": layout.target_points,
    }


def _pose_text(values: tuple[float, float, float, float, float, float]) -> str:
    return " ".join(f"{value:.6f}" for value in values)


def _vector_text(values: tuple[float, ...]) -> str:
    return " ".join(f"{value:.6f}" for value in values)


def _to_pretty_xml(root: ET.Element) -> str:
    rough = ET.tostring(root, encoding="utf-8")
    return xml.dom.minidom.parseString(rough).toprettyxml(indent="  ")


if __name__ == "__main__":
    raise SystemExit(main())
