"""离线测试入口。

所属组：测试组。
文件作用：
- 不依赖 pytest，直接扫描 `tests/offline/` 下的测试函数并运行。
- 让成员在没有 ROS 2、没有 Gazebo、没有主力机时也能做算法验证。

当前职责：
- 动态导入离线测试模块。
- 执行以 `test_` 开头的函数。
- 汇总通过/失败结果并返回退出码。

后续扩展方式：
- 如果后面测试量变大，可以在这里增加测试分类、过滤参数和结果导出。
- 目前优先保持零依赖和简单可读，方便新成员直接跑。

验证方式：
- 运行 `python scripts/run_offline_tests.py`。
- 观察每个 `PASS/FAIL` 项和最后的统计结果。
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
