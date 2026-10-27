from __future__ import annotations

import hashlib
import json
import math
import random
from typing import Any

from building_generator_core.constraints import BuildingConstraints
from building_generator_core.layout import (
    BuildingLayout,
    DoorSpec,
    ElevatorSpec,
    FloorLayout,
    FurnitureSpec,
    Rect2D,
    RoomSpec,
)

CORRIDOR_WIDTH = 2.2
LOBBY_DEPTH = 7.4
FLOOR_HEIGHT = 2.6
WALL_HEIGHT = 2.45
WALL_THICKNESS = 0.18
STAIR_CLEAR_WIDTH = 3.2
ELEVATOR_CLEAR_WIDTH = 2.4
ROOM_DEPTH_MARGIN = 0.5
ROOM_SEGMENT_MIN = 3.6
ROOM_SEGMENT_MAX = 7.5
CORE_ROOM_BUFFER = 1.0
STAIR_LANDING_OFFSET = 0.2
STAIR_ENTRY_LANDING_LENGTH = 1.0


def generate_layout(constraints: BuildingConstraints) -> BuildingLayout:
    rng = random.Random(constraints.seed)
    floor_count = constraints.floor_count.sample(rng)
    room_counts = [constraints.rooms_per_floor.sample(rng) for _ in range(floor_count)]

    footprint_width = constraints.building_footprint_limit["width"]
    footprint_length = constraints.building_footprint_limit["length"]
    half_width = footprint_width / 2.0

    room_depth = max((footprint_width - CORRIDOR_WIDTH) / 2.0 - ROOM_DEPTH_MARGIN, 3.6)
    if room_depth < 3.6:
        raise ValueError("building footprint too narrow for corridor and rooms")

    shaft_bounds = Rect2D(
        x_min=CORRIDOR_WIDTH / 2.0 + 0.55,
        x_max=CORRIDOR_WIDTH / 2.0 + 0.55 + ELEVATOR_CLEAR_WIDTH,
        y_min=1.25,
        y_max=3.95,
    )
    stair_bounds = Rect2D(
        x_min=-CORRIDOR_WIDTH / 2.0 - 0.55 - STAIR_CLEAR_WIDTH,
        x_max=-CORRIDOR_WIDTH / 2.0 - 0.55,
        y_min=0.85,
        y_max=LOBBY_DEPTH - 0.55,
    )
    if stair_bounds.x_min < -half_width + 0.4 or shaft_bounds.x_max > half_width - 0.4:
        raise ValueError("building footprint too narrow for internal stair and elevator cores")

    floors: list[FloorLayout] = []
    doors: list[DoorSpec] = []
    elevator_lobbies: dict[int, tuple[float, float, float, float, float, float]] = {}
    target_points: dict[str, Any] = {
        "main_entrance": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "rooms": {},
        "stairs": {},
        "elevators": {},
    }

    core_zone_end_y = max(stair_bounds.y_max, shaft_bounds.y_max) + CORE_ROOM_BUFFER

    for floor_index, room_count in enumerate(room_counts):
        elevation = floor_index * FLOOR_HEIGHT
        segment_count = math.ceil(room_count / 2.0)
        corridor_end_y = footprint_length - WALL_THICKNESS / 2.0
        usable_room_span = corridor_end_y - core_zone_end_y
        if usable_room_span <= 0:
            raise ValueError("building footprint too short for lobby and corridor")
        segment_length = usable_room_span / max(segment_count, 1)
        if segment_length < ROOM_SEGMENT_MIN:
            raise ValueError("building footprint too short for requested room count")

        lobby_bounds = Rect2D(
            x_min=-half_width,
            x_max=half_width,
            y_min=0.0,
            y_max=core_zone_end_y,
        )
        corridor_bounds = Rect2D(
            x_min=-CORRIDOR_WIDTH / 2.0,
            x_max=CORRIDOR_WIDTH / 2.0,
            y_min=core_zone_end_y,
            y_max=corridor_end_y,
        )

        rooms = _build_rooms(
            floor_index=floor_index,
            elevation=elevation,
            room_count=room_count,
            room_depth=room_depth,
            corridor_bounds=corridor_bounds,
            constraints=constraints,
            rng=rng,
        )

        elevator_door_pose = (
            shaft_bounds.x_min,
            (shaft_bounds.y_min + shaft_bounds.y_max) / 2.0,
            elevation + 1.2,
            0.0,
            0.0,
            math.pi,
        )
        if floor_index == 0:
            doors.append(
                DoorSpec(
                    id="main_entrance",
                    floor_index=0,
                    kind="main_entrance",
                    pose=(0.0, 0.0, 1.2, 0.0, 0.0, math.pi / 2.0),
                    width=2.0,
                    height=2.4,
                    initial_open=True,
                    dynamic=True,
                )
            )
        doors.extend(
            [
                DoorSpec(
                    id=f"elevator_floor_{floor_index}",
                    floor_index=floor_index,
                    kind="elevator",
                    pose=elevator_door_pose,
                    width=1.4,
                    height=2.2,
                    initial_open=(floor_index == 0),
                    dynamic=True,
                ),
            ]
        )
        elevator_lobbies[floor_index] = elevator_door_pose
        target_points["stairs"][str(floor_index)] = [
            stair_bounds.x_max - 0.9,
            stair_bounds.y_min + STAIR_LANDING_OFFSET + STAIR_ENTRY_LANDING_LENGTH / 2.0,
            elevation,
            0.0,
            0.0,
            0.0,
        ]
        target_points["elevators"][str(floor_index)] = list(elevator_door_pose)
        for room in rooms:
            target_points["rooms"][room.id] = list(room.goal_pose)

        floors.append(
            FloorLayout(
                floor_index=floor_index,
                elevation=elevation,
                lobby_bounds=lobby_bounds,
                corridor_bounds=corridor_bounds,
                stair_bounds=stair_bounds,
                elevator_bounds=shaft_bounds,
                rooms=rooms,
                reachability={
                    "stair": True,
                    "elevator": True,
                    "rooms": {room.id: True for room in rooms},
                },
            )
        )

    elevator_specs = [
        ElevatorSpec(
            id="elevator_main",
            shaft_bounds=shaft_bounds,
            served_floors=list(range(floor_count)),
            current_floor=0,
            car_size=(shaft_bounds.width - 0.5, min(shaft_bounds.length - 0.6, 2.6), 2.3),
            lobby_positions=elevator_lobbies,
        )
    ]

    metadata = {
        "seed": constraints.seed,
        "origin_anchor": constraints.origin_anchor,
        "dynamic_doors": constraints.dynamic_doors,
        "dynamic_elevator": constraints.dynamic_elevator,
        "room_type_mix": constraints.room_type_mix,
        "room_counts": room_counts,
        "connectivity": "stairs_and_elevator",
    }

    signature = _build_signature(
        footprint={
            "width": footprint_width,
            "length": footprint_length,
        },
        floors=floors,
        doors=doors,
        elevator_specs=elevator_specs,
        metadata=metadata,
    )

    return BuildingLayout(
        model_name="generated_building",
        footprint={
            "width": footprint_width,
            "length": footprint_length,
        },
        floor_height=FLOOR_HEIGHT,
        wall_height=WALL_HEIGHT,
        entrance_pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        floors=floors,
        door_specs=doors,
        elevator_specs=elevator_specs,
        signature=signature,
        target_points=target_points,
        metadata=metadata,
    )


def _build_rooms(
    *,
    floor_index: int,
    elevation: float,
    room_count: int,
    room_depth: float,
    corridor_bounds: Rect2D,
    constraints: BuildingConstraints,
    rng: random.Random,
) -> list[RoomSpec]:
    rooms: list[RoomSpec] = []
    segment_count = math.ceil(room_count / 2.0)
    segment_length = corridor_bounds.length / max(segment_count, 1)

    room_types = list(constraints.room_type_mix.keys())
    weights = list(constraints.room_type_mix.values())

    for room_index in range(room_count):
        side = "left" if room_index % 2 == 0 else "right"
        segment_index = room_index // 2
        y_min = corridor_bounds.y_min + segment_index * segment_length
        y_max = y_min + segment_length
        if side == "left":
            bounds = Rect2D(
                x_min=-CORRIDOR_WIDTH / 2.0 - room_depth,
                x_max=-CORRIDOR_WIDTH / 2.0,
                y_min=y_min,
                y_max=y_max,
            )
            door_y = (y_min + y_max) / 2.0
            door_pose = (
                bounds.x_max,
                door_y,
                elevation + 1.2,
                0.0,
                0.0,
                0.0,
            )
            goal_pose = (
                bounds.center[0] + 0.5,
                bounds.center[1],
                elevation + 0.0,
                0.0,
                0.0,
                0.0,
            )
        else:
            bounds = Rect2D(
                x_min=CORRIDOR_WIDTH / 2.0,
                x_max=CORRIDOR_WIDTH / 2.0 + room_depth,
                y_min=y_min,
                y_max=y_max,
            )
            door_y = (y_min + y_max) / 2.0
            door_pose = (
                bounds.x_min,
                door_y,
                elevation + 1.2,
                0.0,
                0.0,
                math.pi,
            )
            goal_pose = (
                bounds.center[0] - 0.5,
                bounds.center[1],
                elevation + 0.0,
                0.0,
                0.0,
                0.0,
            )

        room_type = rng.choices(room_types, weights=weights, k=1)[0]
        furniture = _build_furniture(
            room_id=f"floor_{floor_index}_room_{room_index}",
            room_type=room_type,
            bounds=bounds,
            elevation=elevation,
        )
        rooms.append(
            RoomSpec(
                id=f"floor_{floor_index}_room_{room_index}",
                floor_index=floor_index,
                room_type=room_type,
                bounds=bounds,
                side=side,
                door_pose=door_pose,
                goal_pose=goal_pose,
                furniture=furniture,
            )
        )
    return rooms


def _build_furniture(
    *,
    room_id: str,
    room_type: str,
    bounds: Rect2D,
    elevation: float,
) -> list[FurnitureSpec]:
    x_center, y_center = bounds.center
    base_z = elevation + 0.45
    x_span = max(bounds.width - 1.6, 1.2)
    y_span = max(bounds.length - 1.8, 1.4)
    x_left = x_center - x_span * 0.25
    x_right = x_center + x_span * 0.25
    y_south = y_center - y_span * 0.25
    y_north = y_center + y_span * 0.25
    if room_type == "office":
        return [
            FurnitureSpec(
                id=f"{room_id}_desk",
                kind="desk",
                pose=(x_left, y_north, base_z, 0.0, 0.0, 0.0),
                size=(1.5, 0.7, 0.75),
            ),
            FurnitureSpec(
                id=f"{room_id}_chair",
                kind="chair",
                pose=(x_left, y_north - 0.9, elevation + 0.28, 0.0, 0.0, 0.0),
                size=(0.55, 0.55, 0.55),
            ),
            FurnitureSpec(
                id=f"{room_id}_cabinet",
                kind="cabinet",
                pose=(x_right, y_south, elevation + 0.9, 0.0, 0.0, 0.0),
                size=(0.6, 1.0, 1.8),
            ),
            FurnitureSpec(
                id=f"{room_id}_side_table",
                kind="side_table",
                pose=(x_right, y_north, elevation + 0.32, 0.0, 0.0, 0.0),
                size=(0.7, 0.7, 0.45),
            ),
            FurnitureSpec(
                id=f"{room_id}_planter",
                kind="planter",
                pose=(x_center, y_south, elevation + 0.45, 0.0, 0.0, 0.0),
                size=(0.45, 0.45, 0.9),
            ),
        ]
    if room_type == "storage":
        return [
            FurnitureSpec(
                id=f"{room_id}_rack_a",
                kind="storage_rack",
                pose=(x_left, y_north, elevation + 0.9, 0.0, 0.0, 0.0),
                size=(0.8, 1.6, 1.8),
            ),
            FurnitureSpec(
                id=f"{room_id}_rack_b",
                kind="storage_rack",
                pose=(x_right, y_north, elevation + 0.9, 0.0, 0.0, 0.0),
                size=(0.8, 1.6, 1.8),
            ),
            FurnitureSpec(
                id=f"{room_id}_rack_c",
                kind="storage_rack",
                pose=(x_left, y_south, elevation + 0.9, 0.0, 0.0, 0.0),
                size=(0.8, 1.5, 1.8),
            ),
            FurnitureSpec(
                id=f"{room_id}_pallet",
                kind="pallet",
                pose=(x_right, y_south, elevation + 0.12, 0.0, 0.0, 0.0),
                size=(1.0, 1.0, 0.24),
            ),
        ]
    if room_type == "meeting":
        return [
            FurnitureSpec(
                id=f"{room_id}_table",
                kind="meeting_table",
                pose=(x_center, y_center, base_z, 0.0, 0.0, 0.0),
                size=(2.2, 1.0, 0.75),
            ),
            FurnitureSpec(
                id=f"{room_id}_chair_north",
                kind="chair",
                pose=(x_center, y_north, elevation + 0.28, 0.0, 0.0, 0.0),
                size=(0.5, 0.5, 0.55),
            ),
            FurnitureSpec(
                id=f"{room_id}_chair_south",
                kind="chair",
                pose=(x_center, y_south, elevation + 0.28, 0.0, 0.0, 0.0),
                size=(0.5, 0.5, 0.55),
            ),
            FurnitureSpec(
                id=f"{room_id}_chair_west",
                kind="chair",
                pose=(x_left - 0.45, y_center, elevation + 0.28, 0.0, 0.0, 1.57),
                size=(0.5, 0.5, 0.55),
            ),
            FurnitureSpec(
                id=f"{room_id}_chair_east",
                kind="chair",
                pose=(x_right + 0.45, y_center, elevation + 0.28, 0.0, 0.0, 1.57),
                size=(0.5, 0.5, 0.55),
            ),
            FurnitureSpec(
                id=f"{room_id}_credenza",
                kind="cabinet",
                pose=(x_right, y_south, elevation + 0.45, 0.0, 0.0, 0.0),
                size=(1.2, 0.45, 0.9),
            ),
        ]
    if room_type == "lounge":
        return [
            FurnitureSpec(
                id=f"{room_id}_sofa_a",
                kind="sofa",
                pose=(x_left, y_south, base_z, 0.0, 0.0, 0.0),
                size=(1.8, 0.8, 0.8),
            ),
            FurnitureSpec(
                id=f"{room_id}_sofa_b",
                kind="sofa",
                pose=(x_right, y_south, base_z, 0.0, 0.0, 3.14),
                size=(1.8, 0.8, 0.8),
            ),
            FurnitureSpec(
                id=f"{room_id}_coffee_table",
                kind="coffee_table",
                pose=(x_center, y_center, elevation + 0.25, 0.0, 0.0, 0.0),
                size=(0.8, 0.6, 0.4),
            ),
            FurnitureSpec(
                id=f"{room_id}_bookshelf",
                kind="bookshelf",
                pose=(x_right, y_north, elevation + 0.85, 0.0, 0.0, 0.0),
                size=(0.45, 1.2, 1.7),
            ),
            FurnitureSpec(
                id=f"{room_id}_planter",
                kind="planter",
                pose=(x_left, y_north, elevation + 0.45, 0.0, 0.0, 0.0),
                size=(0.45, 0.45, 0.9),
            ),
        ]
    return [
        FurnitureSpec(
            id=f"{room_id}_bench",
            kind="bench",
            pose=(x_center, y_center, base_z, 0.0, 0.0, 0.0),
            size=(1.5, 0.6, 0.75),
        ),
        FurnitureSpec(
            id=f"{room_id}_shelf",
            kind="shelf",
            pose=(x_right, y_north, elevation + 0.9, 0.0, 0.0, 0.0),
            size=(0.45, 1.0, 1.8),
        ),
    ]


def _build_signature(
    *,
    footprint: dict[str, float],
    floors: list[FloorLayout],
    doors: list[DoorSpec],
    elevator_specs: list[ElevatorSpec],
    metadata: dict[str, Any],
) -> str:
    payload = {
        "footprint": footprint,
        "floors": [floor.as_dict() for floor in floors],
        "doors": [door.as_dict() for door in doors],
        "elevators": [elevator.as_dict() for elevator in elevator_specs],
        "metadata": metadata,
    }
    text = json.dumps(payload, sort_keys=True)
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:16]
