from __future__ import annotations

from dataclasses import dataclass
from typing import Any

DEFAULT_ROOM_TYPE_MIX = {
    "office": 3,
    "storage": 1,
    "meeting": 1,
    "lounge": 1,
}

DEFAULT_FOOTPRINT_LIMIT = {
    "width": 36.0,
    "length": 72.0,
}

DEFAULT_DYNAMIC_DOORS = [
    "main_entrance",
    "elevator",
]


@dataclass(frozen=True)
class CountConstraint:
    min_value: int
    max_value: int
    exact_value: int | None = None

    @classmethod
    def from_spec(cls, spec: Any, field_name: str, minimum: int = 1) -> "CountConstraint":
        if isinstance(spec, bool):
            raise ValueError(f"{field_name} cannot be boolean")
        if isinstance(spec, int):
            if spec < minimum:
                raise ValueError(f"{field_name} must be >= {minimum}")
            return cls(min_value=spec, max_value=spec, exact_value=spec)
        if isinstance(spec, (list, tuple)) and len(spec) == 2:
            min_value = int(spec[0])
            max_value = int(spec[1])
            return cls._from_min_max(min_value, max_value, field_name, minimum)
        if isinstance(spec, dict):
            if "exact" in spec:
                exact = int(spec["exact"])
                if exact < minimum:
                    raise ValueError(f"{field_name} exact must be >= {minimum}")
                return cls(min_value=exact, max_value=exact, exact_value=exact)
            min_value = int(spec.get("min", minimum))
            max_value = int(spec.get("max", min_value))
            return cls._from_min_max(min_value, max_value, field_name, minimum)
        raise ValueError(f"Unsupported {field_name} spec: {spec!r}")

    @classmethod
    def _from_min_max(
        cls,
        min_value: int,
        max_value: int,
        field_name: str,
        minimum: int,
    ) -> "CountConstraint":
        if min_value < minimum or max_value < minimum:
            raise ValueError(f"{field_name} range must be >= {minimum}")
        if min_value > max_value:
            raise ValueError(f"{field_name} min cannot exceed max")
        exact_value = min_value if min_value == max_value else None
        return cls(min_value=min_value, max_value=max_value, exact_value=exact_value)

    def sample(self, rng) -> int:
        return rng.randint(self.min_value, self.max_value)


@dataclass(frozen=True)
class BuildingConstraints:
    seed: int
    floor_count: CountConstraint
    rooms_per_floor: CountConstraint
    room_type_mix: dict[str, int]
    building_footprint_limit: dict[str, float]
    stair_required: bool = True
    elevator_required: bool = True
    origin_anchor: str = "main_entrance_center"
    dynamic_doors: list[str] | None = None
    dynamic_elevator: bool = True

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "BuildingConstraints":
        floor_count = CountConstraint.from_spec(data.get("floor_count", 2), "floor_count", minimum=1)
        rooms_per_floor = CountConstraint.from_spec(
            data.get("rooms_per_floor", 4), "rooms_per_floor", minimum=1
        )
        room_type_mix = _parse_room_type_mix(data.get("room_type_mix", DEFAULT_ROOM_TYPE_MIX))
        footprint = _parse_footprint_limit(
            data.get("building_footprint_limit", DEFAULT_FOOTPRINT_LIMIT)
        )
        dynamic_doors = data.get("dynamic_doors", DEFAULT_DYNAMIC_DOORS)
        if dynamic_doors is None:
            dynamic_doors = []
        return cls(
            seed=int(data.get("seed", 0)),
            floor_count=floor_count,
            rooms_per_floor=rooms_per_floor,
            room_type_mix=room_type_mix,
            building_footprint_limit=footprint,
            stair_required=bool(data.get("stair_required", True)),
            elevator_required=bool(data.get("elevator_required", True)),
            origin_anchor=data.get("origin_anchor", "main_entrance_center"),
            dynamic_doors=list(dynamic_doors),
            dynamic_elevator=bool(data.get("dynamic_elevator", True)),
        )


def _parse_room_type_mix(spec: Any) -> dict[str, int]:
    if not isinstance(spec, dict) or not spec:
        raise ValueError("room_type_mix must be a non-empty mapping")
    parsed: dict[str, int] = {}
    for key, value in spec.items():
        parsed[str(key)] = int(value)
        if parsed[str(key)] <= 0:
            raise ValueError("room_type_mix weights must be positive")
    return parsed


def _parse_footprint_limit(spec: Any) -> dict[str, float]:
    if not isinstance(spec, dict):
        raise ValueError("building_footprint_limit must be a mapping")
    width = float(spec.get("width", spec.get("max_width", DEFAULT_FOOTPRINT_LIMIT["width"])))
    length = float(spec.get("length", spec.get("max_length", DEFAULT_FOOTPRINT_LIMIT["length"])))
    if width < 18.0:
        raise ValueError("building footprint width must be >= 18.0")
    if length < 24.0:
        raise ValueError("building footprint length must be >= 24.0")
    return {
        "width": width,
        "length": length,
    }
