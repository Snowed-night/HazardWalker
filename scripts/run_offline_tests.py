"""Run offline tests without requiring pytest.

This script is intentionally small and dependency-free. It lets new members run
the current algorithm tests on a normal Python environment before ROS 2/Gazebo
is available.
"""

from __future__ import annotations

import importlib.util
import sys
import traceback
from pathlib import Path


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    test_dir = repo_root / 'tests' / 'offline'

    if not test_dir.exists():
        print(f'Offline test directory not found: {test_dir}', file=sys.stderr)
        return 1

    passed = 0
    failed = 0
    for path in sorted(test_dir.glob('test_*.py')):
        module = load_module(path)
        for name in sorted(dir(module)):
            if not name.startswith('test_'):
                continue
            test_fn = getattr(module, name)
            if not callable(test_fn):
                continue
            test_name = f'{path.name}::{name}'
            try:
                test_fn()
            except Exception:
                failed += 1
                print(f'FAIL {test_name}')
                traceback.print_exc()
            else:
                passed += 1
                print(f'PASS {test_name}')

    print(f'\nOffline tests: {passed} passed, {failed} failed')
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
