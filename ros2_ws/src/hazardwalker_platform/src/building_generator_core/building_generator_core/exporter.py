from __future__ import annotations

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from building_generator_core.layout import (
    ArtifactPaths,
    BuildingLayout,
    DoorSpec,
    FloorLayout,
    Rect2D,
    RoomSpec,
)
import yaml

SLAB_THICKNESS = 0.22
WALL_THICKNESS = 0.18
SDF_VERSION = "1.7"
DOOR_CLEARANCE = 0.02
DOOR_PANEL_THICKNESS = 0.06
ELEVATOR_CAR_FLOOR_THICKNESS = 0.12
ELEVATOR_CAR_DOOR_GAP = 0.06
STAIR_STEP_HEIGHT = 0.13
STAIR_STEP_DEPTH = 0.26
STAIR_LANDING_THICKNESS = 0.18
RAILING_HEIGHT = 1.0
STAIR_SIDE_CLEARANCE = 0.12
STAIR_CENTER_GAP = 0.16
STAIR_ENTRY_LANDING_LENGTH = 1.0
STAIR_MID_LANDING_LENGTH = 1.05
STAIR_EXIT_LANDING_LENGTH = 0.9
STAIR_LANDING_OFFSET = 0.2
CORE_ROOM_BUFFER = 1.0
VALIDATION_TOLERANCE = 1e-3


def export_sdf(layout: BuildingLayout, target: str, output_dir: str | Path) -> ArtifactPaths:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    world_sdf_path = output_path / "world.sdf"
    model_sdf_path = output_path / "model.sdf"
    metadata_path = output_path / "layout_metadata.json"
    elevator_config_path = output_path / "elevator_config.yaml"
    door_config_path = output_path / "door_config.yaml"
    validation_report_path = output_path / "generation_checks.json"

    world_sdf_path.write_text(_render_world_sdf(layout), encoding="utf-8")
    model_sdf_path.write_text(_render_model_sdf(layout), encoding="utf-8")
    metadata_path.write_text(
        json.dumps(
            {
                "target": target,
                **layout.as_dict(),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    elevator_config_path.write_text(
        yaml.safe_dump(
            {
                "elevators": [_elevator_config_entry(layout, item.id) for item in layout.elevator_specs],
                "signature": layout.signature,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    door_config_path.write_text(
        yaml.safe_dump(
            {
                "doors": [_door_config_entry(item) for item in layout.door_specs],
                "signature": layout.signature,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    validation_report = _validate_generated_artifacts(
        layout,
        world_sdf_path=world_sdf_path,
        elevator_config_path=elevator_config_path,
        door_config_path=door_config_path,
    )
    validation_report_path.write_text(
        json.dumps(validation_report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    failed_checks = [
        item["name"]
        for item in validation_report["checks"]
        if item["status"] != "pass"
    ]
    if failed_checks:
        raise ValueError(
            "generated artifact validation failed: " + ", ".join(failed_checks)
        )

    return ArtifactPaths(
        world_sdf=str(world_sdf_path),
        model_sdf=str(model_sdf_path),
        layout_metadata=str(metadata_path),
        elevator_config=str(elevator_config_path),
        door_config=str(door_config_path),
        validation_report=str(validation_report_path),
    )


def _render_world_sdf(layout: BuildingLayout) -> str:
    sdf = ET.Element("sdf", {"version": SDF_VERSION})
    world = ET.SubElement(sdf, "world", {"name": "generated_world"})
    include_sun = ET.SubElement(world, "include")
    ET.SubElement(include_sun, "uri").text = "model://sun"
    include_ground = ET.SubElement(world, "include")
    ET.SubElement(include_ground, "uri").text = "model://ground_plane"
    world.append(_build_static_shell_model(layout))
    for door in layout.door_specs:
        world.append(_build_door_model(door))
    for elevator in layout.elevator_specs:
        world.append(_build_elevator_model(layout, elevator.id))
    return _to_pretty_xml(sdf)


def _render_model_sdf(layout: BuildingLayout) -> str:
    sdf = ET.Element("sdf", {"version": SDF_VERSION})
    sdf.append(_build_static_shell_model(layout))
    return _to_pretty_xml(sdf)


def _build_static_shell_model(layout: BuildingLayout) -> ET.Element:
    model = ET.Element("model", {"name": layout.model_name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = "0 0 0 0 0 0"

    _append_foundation(model, layout)
    _append_entrance_apron(model, layout)

    for floor in layout.floors:
        _append_floor_plate(model, layout=layout, floor=floor)
        _append_exterior_shell(model, layout=layout, floor=floor)
        _append_room_shell(model, floor=floor)
        _append_core_shell(model, layout=layout, floor=floor)
        _append_elevator_shaft_details(model, floor=floor)
        for room in floor.rooms:
            for furniture in room.furniture:
                _append_box(
                    model,
                    name=furniture.id,
                    size=furniture.size,
                    pose=furniture.pose,
                    color=_furniture_color(furniture.kind),
                )

    _append_stair_geometry(model, layout=layout)

    roof_z = layout.floors[-1].elevation + layout.wall_height + SLAB_THICKNESS / 2.0
    _append_box(
        model,
        name="roof",
        size=(layout.footprint["width"], layout.footprint["length"], SLAB_THICKNESS),
        pose=(0.0, layout.footprint["length"] / 2.0, roof_z, 0.0, 0.0, 0.0),
        color="0.72 0.74 0.76 1",
    )
    return model


def _append_foundation(model: ET.Element, layout: BuildingLayout) -> None:
    _append_box(
        model,
        name="foundation",
        size=(layout.footprint["width"] + 2.0, layout.footprint["length"] + 2.0, 0.3),
        pose=(0.0, layout.footprint["length"] / 2.0, -0.15, 0.0, 0.0, 0.0),
        color="0.58 0.58 0.60 1",
    )


def _append_entrance_apron(model: ET.Element, layout: BuildingLayout) -> None:
    _append_box(
        model,
        name="entrance_apron",
        size=(4.5, 2.4, 0.08),
        pose=(0.0, -1.2, 0.04, 0.0, 0.0, 0.0),
        color="0.62 0.62 0.64 1",
    )


def _append_floor_plate(model: ET.Element, *, layout: BuildingLayout, floor: FloorLayout) -> None:
    z_floor = floor.elevation - SLAB_THICKNESS / 2.0
    half_width = layout.footprint["width"] / 2.0
    half_corridor = floor.corridor_bounds.width / 2.0
    stair_outer_x = floor.stair_bounds.x_min - WALL_THICKNESS / 2.0
    stair_inner_x = floor.stair_bounds.x_max + WALL_THICKNESS / 2.0
    elevator_inner_x = floor.elevator_bounds.x_min - WALL_THICKNESS / 2.0
    elevator_outer_x = floor.elevator_bounds.x_max + WALL_THICKNESS / 2.0

    slab_sections = [
        Rect2D(
            x_min=-half_width,
            x_max=half_width,
            y_min=floor.corridor_bounds.y_min,
            y_max=layout.footprint["length"],
        ),
        Rect2D(
            x_min=-half_width,
            x_max=stair_outer_x,
            y_min=0.0,
            y_max=floor.lobby_bounds.y_max,
        ),
        Rect2D(
            x_min=stair_inner_x,
            x_max=-half_corridor,
            y_min=0.0,
            y_max=floor.lobby_bounds.y_max,
        ),
        Rect2D(
            x_min=-half_corridor,
            x_max=half_corridor,
            y_min=0.0,
            y_max=floor.lobby_bounds.y_max,
        ),
        Rect2D(
            x_min=half_corridor,
            x_max=elevator_inner_x,
            y_min=0.0,
            y_max=floor.lobby_bounds.y_max,
        ),
        Rect2D(
            x_min=elevator_outer_x,
            x_max=half_width,
            y_min=0.0,
            y_max=floor.lobby_bounds.y_max,
        ),
    ]

    for section_index, rect in enumerate(slab_sections):
        if rect.width <= 0.05 or rect.length <= 0.05:
            continue
        _append_box(
            model,
            name=f"slab_floor_{floor.floor_index}_{section_index}",
            size=(rect.width, rect.length, SLAB_THICKNESS),
            pose=(rect.center[0], rect.center[1], z_floor, 0.0, 0.0, 0.0),
            color="0.76 0.76 0.77 1",
        )

    _append_box(
        model,
        name=f"stair_edge_infill_floor_{floor.floor_index}",
        size=(WALL_THICKNESS, floor.stair_bounds.length, SLAB_THICKNESS),
        pose=(floor.stair_bounds.x_max, floor.stair_bounds.center[1], z_floor, 0.0, 0.0, 0.0),
        color="0.76 0.76 0.77 1",
    )


def _append_exterior_shell(model: ET.Element, *, layout: BuildingLayout, floor: FloorLayout) -> None:
    z_center = floor.elevation + layout.wall_height / 2.0
    half_width = layout.footprint["width"] / 2.0
    total_length = layout.footprint["length"]
    entrance_width = _find_door(floor_index=0, kind="main_entrance", layout=layout).width

    west_x = -half_width + WALL_THICKNESS / 2.0
    east_x = half_width - WALL_THICKNESS / 2.0
    north_y = total_length - WALL_THICKNESS / 2.0
    south_y = WALL_THICKNESS / 2.0

    _append_wall(
        model,
        name=f"exterior_west_floor_{floor.floor_index}",
        center=(west_x, total_length / 2.0, z_center),
        size=(WALL_THICKNESS, total_length, layout.wall_height),
    )
    _append_wall(
        model,
        name=f"exterior_east_floor_{floor.floor_index}",
        center=(east_x, total_length / 2.0, z_center),
        size=(WALL_THICKNESS, total_length, layout.wall_height),
    )
    _append_wall(
        model,
        name=f"exterior_north_floor_{floor.floor_index}",
        center=(0.0, north_y, z_center),
        size=(layout.footprint["width"], WALL_THICKNESS, layout.wall_height),
    )
    if floor.floor_index == 0:
        _append_split_horizontal_wall(
            model,
            base_name="exterior_south_entrance",
            y=south_y,
            x_min=-half_width,
            x_max=half_width,
            opening_center=0.0,
            z_center=z_center,
            wall_height=layout.wall_height,
            opening_width=entrance_width + DOOR_CLEARANCE * 2.0,
        )
    else:
        _append_wall(
            model,
            name=f"exterior_south_floor_{floor.floor_index}",
            center=(0.0, south_y, z_center),
            size=(layout.footprint["width"], WALL_THICKNESS, layout.wall_height),
        )


def _append_room_shell(model: ET.Element, *, floor: FloorLayout) -> None:
    north_end = floor.corridor_bounds.y_max
    wall_height = _floor_wall_height(floor)
    _append_wall(
        model,
        name=f"north_end_wall_floor_{floor.floor_index}",
        center=(0.0, north_end, floor.elevation + wall_height / 2.0),
        size=(floor.corridor_bounds.width, WALL_THICKNESS, wall_height),
    )
    for room in floor.rooms:
        _append_room_walls(model, room=room, floor=floor)


def _append_room_walls(model: ET.Element, *, room: RoomSpec, floor: FloorLayout) -> None:
    wall_height = _floor_wall_height(floor)
    z_center = floor.elevation + wall_height / 2.0
    y_center = room.bounds.center[1]
    x_center = room.bounds.center[0]
    full_y = room.bounds.length
    full_x = room.bounds.width

    if room.side == "left":
        corridor_wall_x = room.bounds.x_max - WALL_THICKNESS / 2.0
        exterior_wall_x = room.bounds.x_min + WALL_THICKNESS / 2.0
        opening_y = room.door_pose[1]
        _append_split_vertical_wall(
            model,
            base_name=f"{room.id}_corridor_wall",
            x=corridor_wall_x,
            y_min=room.bounds.y_min,
            y_max=room.bounds.y_max,
            opening_center=opening_y,
            z_center=z_center,
            wall_height=wall_height,
        )
        _append_wall(
            model,
            name=f"{room.id}_west_wall",
            center=(exterior_wall_x, y_center, z_center),
            size=(WALL_THICKNESS, full_y, wall_height),
        )
    else:
        corridor_wall_x = room.bounds.x_min + WALL_THICKNESS / 2.0
        exterior_wall_x = room.bounds.x_max - WALL_THICKNESS / 2.0
        opening_y = room.door_pose[1]
        _append_split_vertical_wall(
            model,
            base_name=f"{room.id}_corridor_wall",
            x=corridor_wall_x,
            y_min=room.bounds.y_min,
            y_max=room.bounds.y_max,
            opening_center=opening_y,
            z_center=z_center,
            wall_height=wall_height,
        )
        _append_wall(
            model,
            name=f"{room.id}_east_wall",
            center=(exterior_wall_x, y_center, z_center),
            size=(WALL_THICKNESS, full_y, wall_height),
        )

    _append_wall(
        model,
        name=f"{room.id}_south_wall",
        center=(x_center, room.bounds.y_min + WALL_THICKNESS / 2.0, z_center),
        size=(full_x, WALL_THICKNESS, wall_height),
    )
    _append_wall(
        model,
        name=f"{room.id}_north_wall",
        center=(x_center, room.bounds.y_max - WALL_THICKNESS / 2.0, z_center),
        size=(full_x, WALL_THICKNESS, wall_height),
    )


def _append_core_shell(model: ET.Element, *, layout: BuildingLayout, floor: FloorLayout) -> None:
    wall_height = _floor_wall_height(floor)
    z_center = floor.elevation + wall_height / 2.0
    elevator_door = _find_door(floor_index=floor.floor_index, kind="elevator", layout=layout)
    _append_open_core_box(
        model,
        bounds=floor.stair_bounds,
        z_center=z_center,
        wall_height=wall_height,
        prefix=f"stair_core_floor_{floor.floor_index}",
        open_side="east",
    )
    _append_core_box_with_door(
        model,
        bounds=floor.elevator_bounds,
        z_center=z_center,
        wall_height=wall_height,
        door_y=elevator_door.pose[1],
        door_width=1.2,
        prefix=f"elevator_core_floor_{floor.floor_index}",
        door_side="west",
    )


def _append_open_core_box(
    model: ET.Element,
    *,
    bounds: Rect2D,
    z_center: float,
    wall_height: float,
    prefix: str,
    open_side: str,
) -> None:
    x_center = bounds.center[0]
    y_center = bounds.center[1]
    if open_side != "east":
        raise ValueError(f"unsupported open side: {open_side}")

    _append_wall(
        model,
        name=f"{prefix}_west",
        center=(bounds.x_min + WALL_THICKNESS / 2.0, y_center, z_center),
        size=(WALL_THICKNESS, bounds.length, wall_height),
    )
    _append_wall(
        model,
        name=f"{prefix}_south",
        center=(x_center, bounds.y_min + WALL_THICKNESS / 2.0, z_center),
        size=(bounds.width, WALL_THICKNESS, wall_height),
    )
    _append_wall(
        model,
        name=f"{prefix}_north",
        center=(x_center, bounds.y_max - WALL_THICKNESS / 2.0, z_center),
        size=(bounds.width, WALL_THICKNESS, wall_height),
    )


def _append_core_box_with_door(
    model: ET.Element,
    *,
    bounds: Rect2D,
    z_center: float,
    wall_height: float,
    door_y: float,
    door_width: float,
    prefix: str,
    door_side: str,
) -> None:
    x_center = bounds.center[0]
    y_center = bounds.center[1]
    if door_side == "east":
        _append_split_vertical_wall(
            model,
            base_name=f"{prefix}_east",
            x=bounds.x_max - WALL_THICKNESS / 2.0,
            y_min=bounds.y_min,
            y_max=bounds.y_max,
            opening_center=door_y,
            z_center=z_center,
            wall_height=wall_height,
            opening_width=door_width + DOOR_CLEARANCE * 2.0,
        )
        _append_wall(
            model,
            name=f"{prefix}_west",
            center=(bounds.x_min + WALL_THICKNESS / 2.0, y_center, z_center),
            size=(WALL_THICKNESS, bounds.length, wall_height),
        )
    else:
        _append_split_vertical_wall(
            model,
            base_name=f"{prefix}_west",
            x=bounds.x_min + WALL_THICKNESS / 2.0,
            y_min=bounds.y_min,
            y_max=bounds.y_max,
            opening_center=door_y,
            z_center=z_center,
            wall_height=wall_height,
            opening_width=door_width + DOOR_CLEARANCE * 2.0,
        )
        _append_wall(
            model,
            name=f"{prefix}_east",
            center=(bounds.x_max - WALL_THICKNESS / 2.0, y_center, z_center),
            size=(WALL_THICKNESS, bounds.length, wall_height),
        )

    _append_wall(
        model,
        name=f"{prefix}_south",
        center=(x_center, bounds.y_min + WALL_THICKNESS / 2.0, z_center),
        size=(bounds.width, WALL_THICKNESS, wall_height),
    )
    _append_wall(
        model,
        name=f"{prefix}_north",
        center=(x_center, bounds.y_max - WALL_THICKNESS / 2.0, z_center),
        size=(bounds.width, WALL_THICKNESS, wall_height),
    )


def _append_stair_geometry(model: ET.Element, *, layout: BuildingLayout) -> None:
    if not layout.floors:
        return

    stair_bounds = layout.floors[0].stair_bounds
    floor_landing_length = STAIR_ENTRY_LANDING_LENGTH
    mid_landing_length = STAIR_MID_LANDING_LENGTH
    tread_depth = STAIR_STEP_DEPTH
    half_rise = layout.floor_height / 2.0
    flight_steps = max(1, int(round(half_rise / STAIR_STEP_HEIGHT)))
    step_height = half_rise / flight_steps
    flight_run = flight_steps * tread_depth
    required_length = floor_landing_length + flight_run + mid_landing_length
    available_length = stair_bounds.length - STAIR_LANDING_OFFSET * 2.0
    if required_length > available_length:
        scale = available_length / required_length
        tread_depth *= scale
        floor_landing_length *= scale
        mid_landing_length *= scale
        flight_run = flight_steps * tread_depth

    x_center = stair_bounds.center[0]
    inner_width = stair_bounds.width - WALL_THICKNESS
    clear_width = inner_width - STAIR_CENTER_GAP
    half_run_width = max(clear_width / 2.0, 0.9)
    inner_left_x = stair_bounds.x_min + STAIR_SIDE_CLEARANCE + half_run_width / 2.0
    inner_right_x = stair_bounds.x_max - STAIR_SIDE_CLEARANCE - half_run_width / 2.0

    floor_landing_y_min = stair_bounds.y_min + STAIR_LANDING_OFFSET
    floor_landing_y_max = floor_landing_y_min + floor_landing_length
    turn_landing_y_min = floor_landing_y_max + flight_run
    turn_landing_y_max = turn_landing_y_min + mid_landing_length

    for floor in layout.floors:
        _append_box(
            model,
            name=f"stair_floor_landing_floor_{floor.floor_index}",
            size=(inner_width, floor_landing_length, STAIR_LANDING_THICKNESS),
            pose=(
                x_center,
                (floor_landing_y_min + floor_landing_y_max) / 2.0,
                floor.elevation - STAIR_LANDING_THICKNESS / 2.0,
                0.0,
                0.0,
                0.0,
            ),
            color="0.68 0.68 0.70 1",
        )

    for floor in layout.floors[:-1]:
        z_base = floor.elevation
        _append_stair_flight(
            model,
            base_name=f"stair_flight_a_floor_{floor.floor_index}",
            x_center=inner_left_x,
            y_start=floor_landing_y_max,
            z_start=z_base,
            width=half_run_width,
            step_count=flight_steps,
            step_depth=tread_depth,
            step_height=step_height,
            direction=1.0,
        )
        _append_box(
            model,
            name=f"stair_turn_landing_floor_{floor.floor_index}",
            size=(inner_width, mid_landing_length, STAIR_LANDING_THICKNESS),
            pose=(
                x_center,
                (turn_landing_y_min + turn_landing_y_max) / 2.0,
                z_base + half_rise - STAIR_LANDING_THICKNESS / 2.0,
                0.0,
                0.0,
                0.0,
            ),
            color="0.68 0.68 0.70 1",
        )
        _append_stair_flight(
            model,
            base_name=f"stair_flight_b_floor_{floor.floor_index}",
            x_center=inner_right_x,
            y_start=turn_landing_y_min,
            z_start=z_base + half_rise,
            width=half_run_width,
            step_count=flight_steps,
            step_depth=tread_depth,
            step_height=step_height,
            direction=-1.0,
        )


def _append_stair_flight(
    model: ET.Element,
    *,
    base_name: str,
    x_center: float,
    y_start: float,
    z_start: float,
    width: float,
    step_count: int,
    step_depth: float,
    step_height: float,
    direction: float = 1.0,
) -> None:
    for step_index in range(step_count):
        y_center = y_start + direction * step_depth * (step_index + 0.5)
        z_center = z_start + step_height * (step_index + 0.5)
        _append_box(
            model,
            name=f"{base_name}_step_{step_index}",
            size=(width, step_depth, step_height),
            pose=(x_center, y_center, z_center, 0.0, 0.0, 0.0),
            color="0.64 0.64 0.66 1",
        )


def _append_elevator_shaft_details(model: ET.Element, *, floor: FloorLayout) -> None:
    shaft = floor.elevator_bounds
    inner_clear_length = shaft.length - WALL_THICKNESS
    infill_depth = max(ELEVATOR_CAR_DOOR_GAP - 0.01, 0.03)
    _append_box(
        model,
        name=f"elevator_threshold_floor_{floor.floor_index}",
        size=(0.22, 1.50, 0.06),
        pose=(shaft.x_min - 0.11, shaft.center[1], floor.elevation + 0.03, 0.0, 0.0, 0.0),
        color="0.50 0.51 0.54 1",
    )
    _append_box(
        model,
        name=f"elevator_sill_infill_floor_{floor.floor_index}",
        size=(infill_depth, inner_clear_length, SLAB_THICKNESS),
        pose=(shaft.x_min + infill_depth / 2.0, shaft.center[1], floor.elevation - SLAB_THICKNESS / 2.0, 0.0, 0.0, 0.0),
        color="0.72 0.72 0.73 1",
    )
    _append_box(
        model,
        name=f"elevator_header_floor_{floor.floor_index}",
        size=(0.12, 1.60, 0.28),
        pose=(shaft.x_min - 0.06, shaft.center[1], floor.elevation + 2.26, 0.0, 0.0, 0.0),
        color="0.50 0.51 0.54 1",
    )
    _append_box(
        model,
        name=f"elevator_jamb_left_floor_{floor.floor_index}",
        size=(0.12, 0.10, 2.20),
        pose=(shaft.x_min - 0.06, shaft.center[1] - 0.75, floor.elevation + 1.10, 0.0, 0.0, 0.0),
        color="0.50 0.51 0.54 1",
    )
    _append_box(
        model,
        name=f"elevator_jamb_right_floor_{floor.floor_index}",
        size=(0.12, 0.10, 2.20),
        pose=(shaft.x_min - 0.06, shaft.center[1] + 0.75, floor.elevation + 1.10, 0.0, 0.0, 0.0),
        color="0.50 0.51 0.54 1",
    )
    _append_box(
        model,
        name=f"elevator_call_panel_floor_{floor.floor_index}",
        size=(0.08, 0.22, 0.32),
        pose=(shaft.x_min - 0.08, shaft.center[1] + 0.98, floor.elevation + 1.2, 0.0, 0.0, 0.0),
        color="0.20 0.20 0.24 1",
    )


def _build_door_model(door: DoorSpec) -> ET.Element:
    model = ET.Element("model", {"name": f"dynamic_{door.id}"})
    ET.SubElement(model, "static").text = "false"
    ET.SubElement(model, "pose").text = _format_pose(door.pose)
    panel_width = door.width / 2.0
    left_offset = -panel_width / 2.0
    right_offset = panel_width / 2.0
    _append_kinematic_panel(model, "left_panel", (0.0, left_offset, 0.0, 0.0, 0.0, 0.0), (DOOR_PANEL_THICKNESS, panel_width, door.height))
    _append_kinematic_panel(model, "right_panel", (0.0, right_offset, 0.0, 0.0, 0.0, 0.0), (DOOR_PANEL_THICKNESS, panel_width, door.height))
    return model


def _build_elevator_model(layout: BuildingLayout, elevator_id: str) -> ET.Element:
    elevator = next(item for item in layout.elevator_specs if item.id == elevator_id)
    current_floor = layout.floors[elevator.current_floor]
    model = ET.Element("model", {"name": f"dynamic_{elevator.id}"})
    ET.SubElement(model, "static").text = "false"
    ET.SubElement(model, "pose").text = _format_pose(_elevator_car_pose(elevator, current_floor.elevation))
    link = ET.SubElement(model, "link", {"name": "car"})
    ET.SubElement(link, "gravity").text = "false"
    ET.SubElement(link, "kinematic").text = "true"
    ET.SubElement(link, "pose").text = "0 0 0 0 0 0"

    floor_size = (elevator.car_size[0], elevator.car_size[1], ELEVATOR_CAR_FLOOR_THICKNESS)
    for tag in ("collision", "visual"):
        element = ET.SubElement(link, tag, {"name": f"car_floor_{tag}"})
        ET.SubElement(element, "pose").text = _format_pose(
            (0.0, 0.0, -elevator.car_size[2] / 2.0 + ELEVATOR_CAR_FLOOR_THICKNESS / 2.0, 0.0, 0.0, 0.0)
        )
        geometry = ET.SubElement(element, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = _format_vec(floor_size)
        if tag == "visual":
            material = ET.SubElement(element, "material")
            ET.SubElement(material, "ambient").text = "0.32 0.32 0.35 1"
            ET.SubElement(material, "diffuse").text = "0.32 0.32 0.35 1"

    wall_panels = [
        ("back", (elevator.car_size[0], 0.08, elevator.car_size[2]), (0.0, elevator.car_size[1] / 2.0 - 0.04, 0.0, 0.0, 0.0, 0.0)),
        ("left", (0.08, elevator.car_size[1], elevator.car_size[2]), (-elevator.car_size[0] / 2.0 + 0.04, 0.0, 0.0, 0.0, 0.0, 0.0)),
        ("right", (0.08, elevator.car_size[1], elevator.car_size[2]), (elevator.car_size[0] / 2.0 - 0.04, 0.0, 0.0, 0.0, 0.0, 0.0)),
        ("top", (elevator.car_size[0], elevator.car_size[1], 0.06), (0.0, 0.0, elevator.car_size[2] / 2.0 - 0.03, 0.0, 0.0, 0.0)),
    ]
    for panel_name, size, pose in wall_panels:
        for tag in ("collision", "visual"):
            element = ET.SubElement(link, tag, {"name": f"{panel_name}_{tag}"})
            ET.SubElement(element, "pose").text = _format_pose(pose)
            geometry = ET.SubElement(element, "geometry")
            box = ET.SubElement(geometry, "box")
            ET.SubElement(box, "size").text = _format_vec(size)
            if tag == "visual":
                material = ET.SubElement(element, "material")
                ET.SubElement(material, "ambient").text = "0.55 0.56 0.58 1"
                ET.SubElement(material, "diffuse").text = "0.55 0.56 0.58 1"
    return model


def _door_config_entry(door: DoorSpec) -> dict[str, object]:
    payload = door.as_dict()
    payload["model_name"] = f"dynamic_{door.id}"
    panel_offset = door.width / 4.0
    slide_offset = door.width / 4.0 + 0.06
    recess_offset = 0.0
    if door.kind == "elevator":
        slide_offset = door.width / 2.0 + WALL_THICKNESS / 2.0 + DOOR_PANEL_THICKNESS
        recess_offset = -(WALL_THICKNESS / 2.0 + DOOR_PANEL_THICKNESS / 2.0)
        payload["motion_duration"] = 60.0
    payload["panel_poses"] = {
        "left_closed": [0.0, -panel_offset, 0.0, 0.0, 0.0, 0.0],
        "right_closed": [0.0, panel_offset, 0.0, 0.0, 0.0, 0.0],
        "left_open": [recess_offset, -panel_offset - slide_offset, 0.0, 0.0, 0.0, 0.0],
        "right_open": [recess_offset, panel_offset + slide_offset, 0.0, 0.0, 0.0, 0.0],
    }
    payload["closed_pose"] = list(door.pose)
    return payload


def _elevator_config_entry(layout: BuildingLayout, elevator_id: str) -> dict[str, object]:
    elevator = next(item for item in layout.elevator_specs if item.id == elevator_id)
    payload = elevator.as_dict()
    payload["model_name"] = f"dynamic_{elevator.id}"
    payload["floor_poses"] = {
        str(floor_index): list(_elevator_car_pose(elevator, layout.floors[floor_index].elevation))
        for floor_index in elevator.served_floors
    }
    return payload


def _elevator_car_pose(
    elevator,
    floor_elevation: float,
) -> tuple[float, float, float, float, float, float]:
    return (
        elevator.shaft_bounds.x_min + elevator.car_size[1] / 2.0 + ELEVATOR_CAR_DOOR_GAP,
        elevator.shaft_bounds.center[1],
        floor_elevation + elevator.car_size[2] / 2.0 - ELEVATOR_CAR_FLOOR_THICKNESS,
        0.0,
        0.0,
        -math.pi / 2.0,
    )


def _validate_generated_artifacts(
    layout: BuildingLayout,
    *,
    world_sdf_path: Path,
    elevator_config_path: Path,
    door_config_path: Path,
) -> dict[str, object]:
    world_root = ET.parse(world_sdf_path).getroot()
    elevator_config = yaml.safe_load(elevator_config_path.read_text()) or {}
    door_config = yaml.safe_load(door_config_path.read_text()) or {}

    checks = []
    checks.extend(_validate_elevator_floor_poses(layout, elevator_config))
    checks.extend(_validate_elevator_initial_world_pose(layout, world_root))
    checks.extend(_validate_elevator_sill_infill(layout, world_root))
    checks.extend(_validate_elevator_door_config(layout, door_config))
    checks.extend(_validate_dynamic_door_models(layout, world_root, door_config))
    checks.extend(_validate_stair_geometry(layout, world_root))
    checks.extend(_validate_floor_edge_infills(layout, world_root))
    checks.extend(_validate_rooms(layout))

    passed_count = sum(1 for item in checks if item["status"] == "pass")
    failed_count = len(checks) - passed_count
    return {
        "signature": layout.signature,
        "status": "pass" if failed_count == 0 else "fail",
        "summary": {
            "passed": passed_count,
            "failed": failed_count,
        },
        "checks": checks,
    }


def _validate_elevator_floor_poses(
    layout: BuildingLayout,
    elevator_config: dict[str, object],
) -> list[dict[str, object]]:
    checks: list[dict[str, object]] = []
    elevator_specs = {
        str(item["id"]): item
        for item in elevator_config.get("elevators", [])
    }
    for elevator in layout.elevator_specs:
        spec = elevator_specs.get(elevator.id)
        checks.append(
            _validation_result(
                f"{elevator.id}_config_present",
                spec is not None,
                expected=True,
                actual=spec is not None,
            )
        )
        if spec is None:
            continue

        floor_poses = spec.get("floor_poses", {}) or {}
        for floor_index in elevator.served_floors:
            pose = [float(value) for value in floor_poses[str(floor_index)]]
            expected_pose = list(_elevator_car_pose(elevator, layout.floors[floor_index].elevation))
            floor_top_z = pose[2] - elevator.car_size[2] / 2.0 + ELEVATOR_CAR_FLOOR_THICKNESS
            door_gap = pose[0] - elevator.car_size[1] / 2.0 - elevator.shaft_bounds.x_min

            checks.append(
                _validation_result(
                    f"{elevator.id}_floor_{floor_index}_pose",
                    _pose_matches(pose, expected_pose),
                    expected=expected_pose,
                    actual=pose,
                )
            )
            checks.append(
                _validation_result(
                    f"{elevator.id}_floor_{floor_index}_floor_alignment",
                    _is_close(floor_top_z, layout.floors[floor_index].elevation),
                    expected=layout.floors[floor_index].elevation,
                    actual=floor_top_z,
                )
            )
            checks.append(
                _validation_result(
                    f"{elevator.id}_floor_{floor_index}_door_gap",
                    _is_close(door_gap, ELEVATOR_CAR_DOOR_GAP),
                    expected=ELEVATOR_CAR_DOOR_GAP,
                    actual=door_gap,
                )
            )
    return checks


def _validate_elevator_initial_world_pose(
    layout: BuildingLayout,
    world_root: ET.Element,
) -> list[dict[str, object]]:
    checks: list[dict[str, object]] = []
    for elevator in layout.elevator_specs:
        model = _find_model(world_root, f"dynamic_{elevator.id}")
        checks.append(
            _validation_result(
                f"{elevator.id}_world_model_present",
                model is not None,
                expected=True,
                actual=model is not None,
            )
        )
        if model is None:
            continue
        pose = _parse_float_list(model.findtext("pose"))
        expected_pose = list(_elevator_car_pose(elevator, layout.floors[elevator.current_floor].elevation))
        checks.append(
            _validation_result(
                f"{elevator.id}_world_initial_pose",
                _pose_matches(pose, expected_pose),
                expected=expected_pose,
                actual=pose,
            )
        )
    return checks


def _validate_elevator_sill_infill(
    layout: BuildingLayout,
    world_root: ET.Element,
) -> list[dict[str, object]]:
    checks: list[dict[str, object]] = []
    building_model = _find_model(world_root, layout.model_name)
    checks.append(
        _validation_result(
            "generated_building_present",
            building_model is not None,
            expected=True,
            actual=building_model is not None,
        )
    )
    if building_model is None:
        return checks

    links = {
        link.get("name"): link
        for link in building_model.findall("link")
    }
    for floor in layout.floors:
        link_name = f"elevator_sill_infill_floor_{floor.floor_index}"
        link = links.get(link_name)
        checks.append(
            _validation_result(
                f"{link_name}_present",
                link is not None,
                expected=True,
                actual=link is not None,
            )
        )
        if link is None:
            continue
        pose = _parse_float_list(link.findtext("pose"))
        size = _parse_float_list(link.find("./collision/geometry/box/size").text)
        top_z = pose[2] + size[2] / 2.0
        x_min = pose[0] - size[0] / 2.0
        checks.append(
            _validation_result(
                f"{link_name}_top_alignment",
                _is_close(top_z, floor.elevation),
                expected=floor.elevation,
                actual=top_z,
            )
        )
        checks.append(
            _validation_result(
                f"{link_name}_starts_at_door_plane",
                _is_close(x_min, floor.elevator_bounds.x_min),
                expected=floor.elevator_bounds.x_min,
                actual=x_min,
            )
        )
        checks.append(
            _validation_result(
                f"{link_name}_depth_within_gap",
                size[0] <= ELEVATOR_CAR_DOOR_GAP and size[0] >= 0.03,
                expected={"min": 0.03, "max": ELEVATOR_CAR_DOOR_GAP},
                actual=size[0],
            )
        )
    return checks


def _validate_elevator_door_config(
    layout: BuildingLayout,
    door_config: dict[str, object],
) -> list[dict[str, object]]:
    checks: list[dict[str, object]] = []
    door_specs = {
        str(item["id"]): item
        for item in door_config.get("doors", [])
    }
    for door in layout.door_specs:
        if door.kind != "elevator":
            continue
        spec = door_specs.get(door.id)
        checks.append(
            _validation_result(
                f"{door.id}_config_present",
                spec is not None,
                expected=True,
                actual=spec is not None,
            )
        )
        if spec is None:
            continue
        panel_poses = spec.get("panel_poses", {}) or {}
        left_open = panel_poses.get("left_open")
        right_open = panel_poses.get("right_open")
        checks.append(
            _validation_result(
                f"{door.id}_open_panel_poses_present",
                left_open is not None and right_open is not None,
                expected=True,
                actual={
                    "left_open": left_open is not None,
                    "right_open": right_open is not None,
                },
            )
        )
        if left_open is None or right_open is None:
            continue

        required_offset = door.width / 2.0 + door.width / 4.0
        clears_doorway = abs(float(left_open[1])) >= required_offset and abs(float(right_open[1])) >= required_offset
        recessed = float(left_open[0]) < 0.0 and float(right_open[0]) < 0.0
        checks.append(
            _validation_result(
                f"{door.id}_open_panels_clear_doorway",
                clears_doorway,
                expected={"minimum_offset": required_offset},
                actual={
                    "left_open_y": float(left_open[1]),
                    "right_open_y": float(right_open[1]),
                },
            )
        )
        checks.append(
            _validation_result(
                f"{door.id}_open_panels_recessed",
                recessed,
                expected={"negative_x": True},
                actual={
                    "left_open_x": float(left_open[0]),
                    "right_open_x": float(right_open[0]),
                },
            )
        )
    return checks


def _validate_dynamic_door_models(
    layout: BuildingLayout,
    world_root: ET.Element,
    door_config: dict[str, object],
) -> list[dict[str, object]]:
    checks: list[dict[str, object]] = []
    door_specs = {
        str(item["id"]): item
        for item in door_config.get("doors", [])
    }
    for door in layout.door_specs:
        if not door.dynamic:
            continue
        model_name = f"dynamic_{door.id}"
        model = _find_model(world_root, model_name)
        spec = door_specs.get(door.id)
        checks.append(
            _validation_result(
                f"{door.id}_dynamic_model_present",
                model is not None,
                expected=True,
                actual=model is not None,
            )
        )
        checks.append(
            _validation_result(
                f"{door.id}_door_config_present",
                spec is not None,
                expected=True,
                actual=spec is not None,
            )
        )
        if model is None or spec is None:
            continue

        links = {
            link.get("name"): link
            for link in model.findall("link")
        }
        checks.append(
            _validation_result(
                f"{door.id}_door_panels_present",
                "left_panel" in links and "right_panel" in links,
                expected={"left_panel": True, "right_panel": True},
                actual={
                    "left_panel": "left_panel" in links,
                    "right_panel": "right_panel" in links,
                },
            )
        )

        panel_poses = spec.get("panel_poses", {}) or {}
        required_keys = {"left_closed", "right_closed", "left_open", "right_open"}
        checks.append(
            _validation_result(
                f"{door.id}_door_pose_keys_complete",
                required_keys.issubset(panel_poses.keys()),
                expected=sorted(required_keys),
                actual=sorted(panel_poses.keys()),
            )
        )
        if not required_keys.issubset(panel_poses.keys()):
            continue

        left_closed = [float(value) for value in panel_poses["left_closed"]]
        left_open = [float(value) for value in panel_poses["left_open"]]
        right_closed = [float(value) for value in panel_poses["right_closed"]]
        right_open = [float(value) for value in panel_poses["right_open"]]
        checks.append(
            _validation_result(
                f"{door.id}_left_panel_moves_when_open",
                not _pose_matches(left_closed, left_open),
                expected={"different_from_closed": True},
                actual={"closed": left_closed, "open": left_open},
            )
        )
        checks.append(
            _validation_result(
                f"{door.id}_right_panel_moves_when_open",
                not _pose_matches(right_closed, right_open),
                expected={"different_from_closed": True},
                actual={"closed": right_closed, "open": right_open},
            )
        )
    return checks


def _validate_stair_geometry(
    layout: BuildingLayout,
    world_root: ET.Element,
) -> list[dict[str, object]]:
    checks: list[dict[str, object]] = []
    building_model = _find_model(world_root, layout.model_name)
    if building_model is None:
        return checks
    links = {
        link.get("name"): link
        for link in building_model.findall("link")
    }

    for floor in layout.floors:
        landing_name = f"stair_floor_landing_floor_{floor.floor_index}"
        landing_link = links.get(landing_name)
        checks.append(
            _validation_result(
                f"{landing_name}_present",
                landing_link is not None,
                expected=True,
                actual=landing_link is not None,
            )
        )
        if landing_link is not None:
            pose, size = _link_box_pose_and_size(landing_link)
            landing_top = pose[2] + size[2] / 2.0
            landing_x_max = pose[0] + size[0] / 2.0
            checks.append(
                _validation_result(
                    f"{landing_name}_top_alignment",
                    _is_close(landing_top, floor.elevation),
                    expected=floor.elevation,
                    actual=landing_top,
                )
            )
            checks.append(
                _validation_result(
                    f"{landing_name}_reaches_opening_edge",
                    _is_close(landing_x_max, floor.stair_bounds.x_max - WALL_THICKNESS / 2.0),
                    expected=floor.stair_bounds.x_max - WALL_THICKNESS / 2.0,
                    actual=landing_x_max,
                )
            )

    for floor_index in range(len(layout.floors) - 1):
        required_names = [
            f"stair_turn_landing_floor_{floor_index}",
            f"stair_flight_b_floor_{floor_index}_step_0",
            f"stair_flight_b_floor_{floor_index}_step_9",
            f"stair_floor_landing_floor_{floor_index + 1}",
        ]
        presence = all(name in links for name in required_names)
        checks.append(
            _validation_result(
                f"stair_floor_{floor_index}_upper_connection_links_present",
                presence,
                expected=required_names,
                actual={name: name in links for name in required_names},
            )
        )
        if not presence:
            continue

        turn_pose, turn_size = _link_box_pose_and_size(links[required_names[0]])
        first_pose, first_size = _link_box_pose_and_size(links[required_names[1]])
        last_pose, last_size = _link_box_pose_and_size(links[required_names[2]])
        upper_pose, upper_size = _link_box_pose_and_size(links[required_names[3]])

        turn_y_min = turn_pose[1] - turn_size[1] / 2.0
        turn_z_max = turn_pose[2] + turn_size[2] / 2.0
        first_y_max = first_pose[1] + first_size[1] / 2.0
        first_z_min = first_pose[2] - first_size[2] / 2.0
        last_y_min = last_pose[1] - last_size[1] / 2.0
        last_z_max = last_pose[2] + last_size[2] / 2.0
        upper_y_max = upper_pose[1] + upper_size[1] / 2.0
        upper_z_max = upper_pose[2] + upper_size[2] / 2.0

        checks.append(
            _validation_result(
                f"stair_floor_{floor_index}_turn_to_second_flight",
                _is_close(first_y_max, turn_y_min) and _is_close(first_z_min, turn_z_max),
                expected={"first_y_max": turn_y_min, "first_z_min": turn_z_max},
                actual={"first_y_max": first_y_max, "first_z_min": first_z_min},
            )
        )
        checks.append(
            _validation_result(
                f"stair_floor_{floor_index}_second_flight_to_upper_landing",
                _is_close(last_y_min, upper_y_max) and _is_close(last_z_max, upper_z_max),
                expected={"last_y_min": upper_y_max, "last_z_max": upper_z_max},
                actual={"last_y_min": last_y_min, "last_z_max": last_z_max},
            )
        )
    return checks


def _validate_rooms(layout: BuildingLayout) -> list[dict[str, object]]:
    checks: list[dict[str, object]] = []
    half_width = layout.footprint["width"] / 2.0
    for floor in layout.floors:
        checks.append(
            _validation_result(
                f"floor_{floor.floor_index}_room_count_positive",
                len(floor.rooms) > 0,
                expected={">": 0},
                actual=len(floor.rooms),
            )
        )

        overlap_pairs: list[list[str]] = []
        for index, room in enumerate(floor.rooms):
            checks.append(
                _validation_result(
                    f"{room.id}_bounds_positive",
                    room.bounds.width > 0.0 and room.bounds.length > 0.0,
                    expected={"width": ">0", "length": ">0"},
                    actual={"width": room.bounds.width, "length": room.bounds.length},
                )
            )
            checks.append(
                _validation_result(
                    f"{room.id}_within_room_band",
                    (
                        room.bounds.x_min >= -half_width - VALIDATION_TOLERANCE
                        and room.bounds.x_max <= half_width + VALIDATION_TOLERANCE
                        and room.bounds.y_min >= floor.corridor_bounds.y_min - VALIDATION_TOLERANCE
                        and room.bounds.y_max <= floor.corridor_bounds.y_max + VALIDATION_TOLERANCE
                    ),
                    expected={
                        "x_min>=": -half_width,
                        "x_max<=": half_width,
                        "y_min>=": floor.corridor_bounds.y_min,
                        "y_max<=": floor.corridor_bounds.y_max,
                    },
                    actual=room.bounds.as_dict(),
                )
            )
            checks.append(
                _validation_result(
                    f"{room.id}_clears_stair_core",
                    not _rectangles_overlap(room.bounds, floor.stair_bounds),
                    expected=False,
                    actual=_rectangles_overlap(room.bounds, floor.stair_bounds),
                )
            )
            checks.append(
                _validation_result(
                    f"{room.id}_clears_elevator_core",
                    not _rectangles_overlap(room.bounds, floor.elevator_bounds),
                    expected=False,
                    actual=_rectangles_overlap(room.bounds, floor.elevator_bounds),
                )
            )
            checks.append(
                _validation_result(
                    f"{room.id}_door_pose_valid",
                    _pose_matches(list(room.door_pose), _expected_room_door_pose(room, floor)),
                    expected=_expected_room_door_pose(room, floor),
                    actual=list(room.door_pose),
                )
            )
            checks.append(
                _validation_result(
                    f"{room.id}_goal_pose_inside_room",
                    _room_goal_pose_valid(room, floor),
                    expected={"inside_room": True, "z": floor.elevation},
                    actual=list(room.goal_pose),
                )
            )
            checks.append(
                _validation_result(
                    f"{room.id}_furniture_within_room",
                    all(_furniture_within_room(room, furniture) for furniture in room.furniture),
                    expected={"all_inside_room": True},
                    actual={
                        furniture.id: _furniture_bounds(furniture)
                        for furniture in room.furniture
                    },
                )
            )

            for other_room in floor.rooms[index + 1 :]:
                if _rectangles_overlap(room.bounds, other_room.bounds):
                    overlap_pairs.append([room.id, other_room.id])

        checks.append(
            _validation_result(
                f"floor_{floor.floor_index}_rooms_do_not_overlap",
                not overlap_pairs,
                expected=[],
                actual=overlap_pairs,
            )
        )
    return checks


def _validate_floor_edge_infills(
    layout: BuildingLayout,
    world_root: ET.Element,
) -> list[dict[str, object]]:
    checks: list[dict[str, object]] = []
    building_model = _find_model(world_root, layout.model_name)
    if building_model is None:
        return checks
    links = {
        link.get("name"): link
        for link in building_model.findall("link")
    }

    for floor in layout.floors:
        stair_link_name = f"stair_edge_infill_floor_{floor.floor_index}"
        stair_link = links.get(stair_link_name)
        checks.append(
            _validation_result(
                f"{stair_link_name}_present",
                stair_link is not None,
                expected=True,
                actual=stair_link is not None,
            )
        )
        if stair_link is not None:
            pose, size = _link_box_pose_and_size(stair_link)
            checks.append(
                _validation_result(
                    f"{stair_link_name}_top_alignment",
                    _is_close(pose[2] + size[2] / 2.0, floor.elevation),
                    expected=floor.elevation,
                    actual=pose[2] + size[2] / 2.0,
                )
            )
            checks.append(
                _validation_result(
                    f"{stair_link_name}_at_stair_open_edge",
                    _is_close(pose[0], floor.stair_bounds.x_max),
                    expected=floor.stair_bounds.x_max,
                    actual=pose[0],
                )
            )

        elevator_link_name = f"elevator_sill_infill_floor_{floor.floor_index}"
        elevator_link = links.get(elevator_link_name)
        checks.append(
            _validation_result(
                f"{elevator_link_name}_present",
                elevator_link is not None,
                expected=True,
                actual=elevator_link is not None,
            )
        )
        if elevator_link is not None:
            pose, size = _link_box_pose_and_size(elevator_link)
            checks.append(
                _validation_result(
                    f"{elevator_link_name}_full_inner_width",
                    _is_close(size[1], floor.elevator_bounds.length - WALL_THICKNESS),
                    expected=floor.elevator_bounds.length - WALL_THICKNESS,
                    actual=size[1],
                )
            )
            checks.append(
                _validation_result(
                    f"{elevator_link_name}_top_alignment",
                    _is_close(pose[2] + size[2] / 2.0, floor.elevation),
                    expected=floor.elevation,
                    actual=pose[2] + size[2] / 2.0,
                )
            )
    return checks


def _find_model(root: ET.Element, model_name: str) -> ET.Element | None:
    for model in root.findall(".//model"):
        if model.get("name") == model_name:
            return model
    return None


def _parse_float_list(raw_text: str) -> list[float]:
    return [float(value) for value in raw_text.split()]


def _link_box_pose_and_size(link: ET.Element) -> tuple[list[float], list[float]]:
    pose = _parse_float_list(link.findtext("pose"))
    size = _parse_float_list(link.find("./collision/geometry/box/size").text)
    return pose, size


def _rectangles_overlap(a: Rect2D, b: Rect2D) -> bool:
    return (
        min(a.x_max, b.x_max) - max(a.x_min, b.x_min) > VALIDATION_TOLERANCE
        and min(a.y_max, b.y_max) - max(a.y_min, b.y_min) > VALIDATION_TOLERANCE
    )


def _expected_room_door_pose(room: RoomSpec, floor: FloorLayout) -> list[float]:
    if room.side == "left":
        return [
            room.bounds.x_max,
            (room.bounds.y_min + room.bounds.y_max) / 2.0,
            floor.elevation + 1.2,
            0.0,
            0.0,
            0.0,
        ]
    return [
        room.bounds.x_min,
        (room.bounds.y_min + room.bounds.y_max) / 2.0,
        floor.elevation + 1.2,
        0.0,
        0.0,
        math.pi,
    ]


def _room_goal_pose_valid(room: RoomSpec, floor: FloorLayout) -> bool:
    return (
        room.bounds.x_min + VALIDATION_TOLERANCE <= room.goal_pose[0] <= room.bounds.x_max - VALIDATION_TOLERANCE
        and room.bounds.y_min + VALIDATION_TOLERANCE <= room.goal_pose[1] <= room.bounds.y_max - VALIDATION_TOLERANCE
        and _is_close(room.goal_pose[2], floor.elevation)
    )


def _furniture_within_room(room: RoomSpec, furniture) -> bool:
    bounds = _furniture_bounds(furniture)
    return (
        bounds["x_min"] >= room.bounds.x_min - VALIDATION_TOLERANCE
        and bounds["x_max"] <= room.bounds.x_max + VALIDATION_TOLERANCE
        and bounds["y_min"] >= room.bounds.y_min - VALIDATION_TOLERANCE
        and bounds["y_max"] <= room.bounds.y_max + VALIDATION_TOLERANCE
    )


def _furniture_bounds(furniture) -> dict[str, float]:
    x_value, y_value, _, _, _, yaw = furniture.pose
    size_x, size_y, _ = furniture.size
    half_x = abs(math.cos(yaw)) * size_x / 2.0 + abs(math.sin(yaw)) * size_y / 2.0
    half_y = abs(math.sin(yaw)) * size_x / 2.0 + abs(math.cos(yaw)) * size_y / 2.0
    return {
        "x_min": x_value - half_x,
        "x_max": x_value + half_x,
        "y_min": y_value - half_y,
        "y_max": y_value + half_y,
    }


def _pose_matches(actual: list[float], expected: list[float]) -> bool:
    return all(_is_close(actual_value, expected_value) for actual_value, expected_value in zip(actual, expected))


def _is_close(actual: float, expected: float, *, tolerance: float = VALIDATION_TOLERANCE) -> bool:
    return abs(float(actual) - float(expected)) <= tolerance


def _validation_result(
    name: str,
    passed: bool,
    *,
    expected: object,
    actual: object,
) -> dict[str, object]:
    return {
        "name": name,
        "status": "pass" if passed else "fail",
        "expected": expected,
        "actual": actual,
    }


def _find_door(*, floor_index: int, kind: str, layout: BuildingLayout) -> DoorSpec:
    for door in layout.door_specs:
        if door.floor_index == floor_index and door.kind == kind:
            return door
    raise KeyError(f"missing door kind={kind} floor={floor_index}")


def _append_split_vertical_wall(
    model: ET.Element,
    *,
    base_name: str,
    x: float,
    y_min: float,
    y_max: float,
    opening_center: float,
    z_center: float,
    wall_height: float,
    opening_width: float = 1.2,
) -> None:
    lower_y_max = opening_center - opening_width / 2.0
    upper_y_min = opening_center + opening_width / 2.0
    if lower_y_max > y_min:
        _append_wall(
            model,
            name=f"{base_name}_lower",
            center=(x, (y_min + lower_y_max) / 2.0, z_center),
            size=(WALL_THICKNESS, lower_y_max - y_min, wall_height),
        )
    if y_max > upper_y_min:
        _append_wall(
            model,
            name=f"{base_name}_upper",
            center=(x, (upper_y_min + y_max) / 2.0, z_center),
            size=(WALL_THICKNESS, y_max - upper_y_min, wall_height),
        )


def _append_kinematic_panel(
    model: ET.Element,
    name: str,
    pose: tuple[float, float, float, float, float, float],
    size: tuple[float, float, float],
) -> None:
    link = ET.SubElement(model, "link", {"name": name})
    ET.SubElement(link, "gravity").text = "false"
    ET.SubElement(link, "kinematic").text = "true"
    ET.SubElement(link, "pose").text = _format_pose(pose)
    for tag in ("collision", "visual"):
        element = ET.SubElement(link, tag, {"name": f"{name}_{tag}"})
        geometry = ET.SubElement(element, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = _format_vec(size)
        if tag == "visual":
            material = ET.SubElement(element, "material")
            ET.SubElement(material, "ambient").text = "0.18 0.18 0.20 1"
            ET.SubElement(material, "diffuse").text = "0.18 0.18 0.20 1"


def _append_split_horizontal_wall(
    model: ET.Element,
    *,
    base_name: str,
    y: float,
    x_min: float,
    x_max: float,
    opening_center: float,
    z_center: float,
    wall_height: float,
    opening_width: float,
) -> None:
    left_x_max = opening_center - opening_width / 2.0
    right_x_min = opening_center + opening_width / 2.0
    if left_x_max > x_min:
        _append_wall(
            model,
            name=f"{base_name}_left",
            center=((x_min + left_x_max) / 2.0, y, z_center),
            size=(left_x_max - x_min, WALL_THICKNESS, wall_height),
        )
    if x_max > right_x_min:
        _append_wall(
            model,
            name=f"{base_name}_right",
            center=((right_x_min + x_max) / 2.0, y, z_center),
            size=(x_max - right_x_min, WALL_THICKNESS, wall_height),
        )


def _append_wall(
    model: ET.Element,
    *,
    name: str,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
) -> None:
    _append_box(
        model,
        name=name,
        size=size,
        pose=(center[0], center[1], center[2], 0.0, 0.0, 0.0),
        color="0.86 0.86 0.88 1",
    )


def _append_box(
    model: ET.Element,
    *,
    name: str,
    size: tuple[float, float, float],
    pose: tuple[float, float, float, float, float, float],
    color: str,
) -> None:
    link = ET.SubElement(model, "link", {"name": name})
    ET.SubElement(link, "pose").text = _format_pose(pose)
    for tag in ("collision", "visual"):
        element = ET.SubElement(link, tag, {"name": f"{name}_{tag}"})
        geometry = ET.SubElement(element, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = _format_vec(size)
        if tag == "visual":
            material = ET.SubElement(element, "material")
            ET.SubElement(material, "ambient").text = color
            ET.SubElement(material, "diffuse").text = color
    ET.SubElement(link, "gravity").text = "false"


def _furniture_color(kind: str) -> str:
    palette = {
        "desk": "0.54 0.43 0.34 1",
        "chair": "0.30 0.30 0.34 1",
        "cabinet": "0.72 0.72 0.74 1",
        "side_table": "0.52 0.42 0.33 1",
        "planter": "0.28 0.46 0.30 1",
        "storage_rack": "0.56 0.57 0.60 1",
        "pallet": "0.56 0.44 0.28 1",
        "meeting_table": "0.50 0.38 0.28 1",
        "sofa": "0.24 0.36 0.54 1",
        "coffee_table": "0.60 0.48 0.34 1",
        "bookshelf": "0.50 0.40 0.30 1",
        "bench": "0.46 0.36 0.28 1",
        "shelf": "0.68 0.68 0.70 1",
    }
    return palette.get(kind, "0.55 0.45 0.35 1")


def _format_pose(values: tuple[float, float, float, float, float, float]) -> str:
    return " ".join(f"{value:.4f}" for value in values)


def _format_vec(values: tuple[float, float, float]) -> str:
    return " ".join(f"{value:.4f}" for value in values)


def _floor_wall_height(floor: FloorLayout) -> float:
    return 2.45


def _to_pretty_xml(root: ET.Element) -> str:
    xml_text = ET.tostring(root, encoding="unicode")
    try:
        from xml.dom import minidom

        pretty = minidom.parseString(xml_text).toprettyxml(indent="  ")
        return "\n".join(line for line in pretty.splitlines() if line.strip()) + "\n"
    except Exception:
        return xml_text + "\n"
