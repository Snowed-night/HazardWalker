from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class DoorState:
    door_id: str
    kind: str
    model_name: str
    is_open: bool
    closed_pose: list[float]
    open_pose: list[float]
    panel_poses: dict[str, list[float]]
    motion_duration: float = 0.0


@dataclass
class ElevatorState:
    elevator_id: str
    model_name: str
    current_floor: int
    served_floors: list[int]
    floor_poses: dict[int, list[float]]
    state: str = "idle"
    doors_open: bool = False


class BuildingControlRuntime:
    """In-memory state machine used by the Gazebo Classic control adapter."""

    def __init__(self, *, door_specs: list[dict[str, Any]], elevator_specs: list[dict[str, Any]]):
        self._doors = {
            spec["id"]: DoorState(
                door_id=spec["id"],
                kind=str(spec.get("kind", "")),
                model_name=str(spec.get("model_name", f"dynamic_{spec['id']}")),
                is_open=bool(spec.get("initial_open", False)),
                closed_pose=[float(value) for value in spec.get("closed_pose", spec.get("pose", [0, 0, 0, 0, 0, 0]))],
                open_pose=[float(value) for value in spec.get("open_pose", spec.get("pose", [0, 0, 0, 0, 0, 0]))],
                panel_poses={
                    str(key): [float(value) for value in pose]
                    for key, pose in (spec.get("panel_poses", {}) or {}).items()
                },
                motion_duration=max(0.0, float(spec.get("motion_duration", 0.0))),
            )
            for spec in door_specs
        }
        self._elevators = {
            spec["id"]: ElevatorState(
                elevator_id=spec["id"],
                model_name=str(spec.get("model_name", f"dynamic_{spec['id']}")),
                current_floor=int(spec.get("current_floor", 0)),
                served_floors=[int(value) for value in spec.get("served_floors", [])],
                floor_poses={
                    int(key): [float(value) for value in pose]
                    for key, pose in (spec.get("floor_poses", {}) or {}).items()
                },
            )
            for spec in elevator_specs
        }

    def set_door_state(self, door_id: str, open_state: bool) -> dict[str, Any]:
        if door_id not in self._doors:
            return {
                "accepted": False,
                "state": "missing",
                "message": f"unknown door '{door_id}'",
            }
        state = self._doors[door_id]
        previous_is_open = state.is_open
        state.is_open = bool(open_state)
        start_suffix = "open" if previous_is_open else "closed"
        target_suffix = "open" if state.is_open else "closed"
        return {
            "accepted": True,
            "state": "open" if state.is_open else "closed",
            "message": f"door '{door_id}' set to {'open' if state.is_open else 'closed'}",
            "door_id": state.door_id,
            "kind": state.kind,
            "model_name": state.model_name,
            "target_pose": state.closed_pose,
            "model_pose": state.closed_pose,
            "motion_duration": state.motion_duration,
            "start_panel_poses": {
                "left_panel": state.panel_poses.get(f"left_{start_suffix}"),
                "right_panel": state.panel_poses.get(f"right_{start_suffix}"),
            },
            "panel_poses": {
                "left_panel": state.panel_poses.get(f"left_{target_suffix}"),
                "right_panel": state.panel_poses.get(f"right_{target_suffix}"),
            },
        }

    def call_elevator(self, elevator_id: str, target_floor: int, open_doors: bool) -> dict[str, Any]:
        if elevator_id not in self._elevators:
            return {
                "accepted": False,
                "current_floor": -1,
                "state": "missing",
                "message": f"unknown elevator '{elevator_id}'",
            }
        state = self._elevators[elevator_id]
        if target_floor not in state.served_floors:
            return {
                "accepted": False,
                "current_floor": state.current_floor,
                "state": "invalid_floor",
                "message": f"floor {target_floor} is not served by '{elevator_id}'",
            }

        state.state = "moving" if target_floor != state.current_floor else "idle"
        state.current_floor = target_floor
        state.doors_open = bool(open_doors)
        state.state = "door_open" if state.doors_open else "idle"
        return {
            "accepted": True,
            "current_floor": state.current_floor,
            "state": state.state,
            "message": f"elevator '{elevator_id}' moved to floor {target_floor}",
            "model_name": state.model_name,
            "target_pose": state.floor_poses.get(state.current_floor),
        }
