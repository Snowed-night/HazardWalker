from __future__ import annotations

from pathlib import Path

from building_generator_core import BuildingLayout, export_sdf
import yaml


def export_classic_bundle(layout: BuildingLayout, output_dir: str | Path) -> str:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    artifact_paths = export_sdf(layout, target="gazebo_classic", output_dir=output_path)
    manifest_path = output_path / "classic_bundle.yaml"
    manifest_path.write_text(
        yaml.safe_dump(
            {
                "world_sdf": artifact_paths.world_sdf,
                "model_sdf": artifact_paths.model_sdf,
                "door_config": artifact_paths.door_config,
                "elevator_config": artifact_paths.elevator_config,
                "layout_metadata": artifact_paths.layout_metadata,
                "validation_report": artifact_paths.validation_report,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return str(manifest_path)
