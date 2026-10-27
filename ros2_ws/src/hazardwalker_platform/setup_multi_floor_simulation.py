#!/usr/bin/env python3

import os
import sys
import subprocess

def run_command(cmd, description):
    print(f"\n{'='*60}")
    print(f"{description}")
    print(f"{'='*60}")
    print(f"Running: {cmd}")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"Warning: {description} returned non-zero exit code")
    return result.returncode == 0

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    building_obstacles_dir = os.path.join(script_dir, 'src', 'building_obstacles')
    output_dir = os.path.join(script_dir, 'generated_building')
    results_dir = os.path.join(script_dir, 'results')

    print("\n" + "="*60)
    print("Competition Scene Simulation Setup")
    print("="*60)

    print("\nStep 1: Generate competition scene world...")
    gen_script = os.path.join(building_obstacles_dir, 'scripts', 'generate_competition_scene.py')
    cmd = [
        "python3",
        gen_script,
        "--output-dir",
        output_dir,
        "--results-dir",
        results_dir,
    ]
    if not run_command(cmd, "Generating competition scene"):
        print("Failed to generate competition scene, exiting.")
        return 1

    print("\nStep 2: Setup complete!")
    print("="*60)
    print("\nTo run the simulation:")
    print(f"  cd {script_dir}")
    print("  ./auto.sh")
    print("\nGenerated files:")
    print(f"  world: {os.path.join(output_dir, 'competition_scene.world')}")
    print(f"  truth: {os.path.join(results_dir, 'danger_truth.json')}")
    print("="*60)

    return 0

if __name__ == '__main__':
    sys.exit(main())
