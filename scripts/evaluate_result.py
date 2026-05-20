"""Evaluate a HazardWalker result JSON file.

This is a lightweight checker for the first-stage minimal demo. It validates the
result schema and prints a short summary that can be pasted into weekly reports.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


REQUIRED_TOP_LEVEL_KEYS = {'mission_id', 'status', 'hazards', 'metrics'}
REQUIRED_METRIC_KEYS = {'duration_sec', 'return_success', 'num_confirmed_hazards'}


def evaluate_result(result: dict) -> tuple[bool, list[str], dict]:
    """Validate and summarize one result dict."""

    errors = []
    missing_top_keys = REQUIRED_TOP_LEVEL_KEYS - set(result.keys())
    if missing_top_keys:
        errors.append(f'missing top-level keys: {sorted(missing_top_keys)}')

    hazards = result.get('hazards', [])
    if not isinstance(hazards, list):
        errors.append('hazards must be a list')
        hazards = []

    metrics = result.get('metrics', {})
    if not isinstance(metrics, dict):
        errors.append('metrics must be an object')
        metrics = {}

    missing_metric_keys = REQUIRED_METRIC_KEYS - set(metrics.keys())
    if missing_metric_keys:
        errors.append(f'missing metric keys: {sorted(missing_metric_keys)}')

    confirmed = [hazard for hazard in hazards if hazard.get('status') == 'confirmed']
    reported_confirmed = metrics.get('num_confirmed_hazards')
    if reported_confirmed is not None and reported_confirmed != len(confirmed):
        errors.append(
            f'num_confirmed_hazards={reported_confirmed}, but counted {len(confirmed)} confirmed hazards'
        )

    summary = {
        'mission_id': result.get('mission_id', ''),
        'status': result.get('status', ''),
        'duration_sec': metrics.get('duration_sec', None),
        'return_success': metrics.get('return_success', None),
        'hazard_count': len(hazards),
        'confirmed_hazard_count': len(confirmed),
    }
    return len(errors) == 0, errors, summary


def main() -> int:
    parser = argparse.ArgumentParser(description='Evaluate HazardWalker result JSON.')
    parser.add_argument('result_json', help='Path to reports/run_results/<timestamp>_result.json')
    args = parser.parse_args()

    path = Path(args.result_json)
    if not path.exists():
        print(f'Result file not found: {path}', file=sys.stderr)
        return 1

    with path.open('r', encoding='utf-8') as f:
        result = json.load(f)

    ok, errors, summary = evaluate_result(result)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if not ok:
        print('\nErrors:', file=sys.stderr)
        for error in errors:
            print(f'- {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
