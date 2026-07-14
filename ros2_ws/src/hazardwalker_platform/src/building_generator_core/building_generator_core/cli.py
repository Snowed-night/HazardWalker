from __future__ import annotations

import argparse
import json
from pathlib import Path

from building_generator_core.constraints import BuildingConstraints
from building_generator_core.exporter import export_sdf
from building_generator_core.generator import generate_layout


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Random building generator for Gazebo")
    subparsers = parser.add_subparsers(dest="command", required=True)

    generate_parser = subparsers.add_parser("generate", help="Generate one building scene")
    _add_common_arguments(generate_parser)

    batch_parser = subparsers.add_parser("batch", help="Generate multiple scenes")
    _add_common_arguments(batch_parser, include_seed=False)
    batch_parser.add_argument("--seed-list", required=True, help="Comma-separated seed list")

    args = parser.parse_args(argv)
    if args.command == "generate":
        artifact_paths = _generate_one(args.seed, args)
        print(json.dumps(artifact_paths.as_dict(), indent=2, sort_keys=True))
        return 0

    manifest = []
    for seed_value in _parse_seed_list(args.seed_list):
        scene_dir = Path(args.output_dir) / f"seed_{seed_value}"
        scene_dir.mkdir(parents=True, exist_ok=True)
        scene_args = argparse.Namespace(**vars(args))
        scene_args.output_dir = str(scene_dir)
        artifact_paths = _generate_one(seed_value, scene_args)
        manifest.append(
            {
                "seed": seed_value,
                **artifact_paths.as_dict(),
            }
        )
    manifest_path = Path(args.output_dir) / "batch_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"manifest": str(manifest_path), "count": len(manifest)}, indent=2))
    return 0


def _add_common_arguments(parser: argparse.ArgumentParser, *, include_seed: bool = True) -> None:
    if include_seed:
        parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--floor-count", default="2:4")
    parser.add_argument("--rooms-per-floor", default="4:8")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--target", default="gazebo_classic", choices=("gazebo_classic",))
    parser.add_argument("--width", type=float, default=36.0)
    parser.add_argument("--length", type=float, default=72.0)


def _generate_one(seed: int, args: argparse.Namespace):
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
    return export_sdf(layout, target=args.target, output_dir=args.output_dir)


def _parse_count_spec(raw_value: str):
    if ":" in raw_value:
        min_value, max_value = raw_value.split(":", 1)
        return {"min": int(min_value), "max": int(max_value)}
    return int(raw_value)


def _parse_seed_list(raw_value: str) -> list[int]:
    return [int(part.strip()) for part in raw_value.split(",") if part.strip()]
