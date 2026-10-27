#!/usr/bin/env python3
"""从已取得的官方SimEnv导入运行资源；先校验整批文件，再复制，不覆盖不同内容。"""
from pathlib import Path, PurePosixPath
import argparse
import hashlib
import json
import os
import shutil

ROOT = Path(__file__).resolve().parents[2]

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def prepare(root=ROOT, source=None):
    assets = json.loads((root / 'reproduction/runtime-assets.json').read_text(encoding='utf-8'))
    platform = root / 'ros2_ws/src/hazardwalker_platform'
    pending = []
    for rel, expected in assets.items():
        parts = PurePosixPath(rel)
        if parts.is_absolute() or '..' in parts.parts:
            raise ValueError('资源清单必须使用安全相对路径')
        destination = platform.joinpath(*parts.parts)
        if not destination.resolve().is_relative_to(platform.resolve()):
            raise ValueError('资源目标越出平台目录')
        if destination.exists():
            if not destination.is_file() or sha(destination) != expected:
                raise ValueError('已有资源内容不同，拒绝覆盖：' + rel)
            continue
        if source is None:
            raise ValueError('缺少运行资源。设置OFFICIAL_SIMENV_SOURCE_ROOT或使用--source指定官方SimEnv根目录。缺项：' + rel)
        candidate = Path(source).expanduser().resolve().joinpath(*parts.parts)
        if not candidate.is_file() or sha(candidate) != expected:
            raise ValueError('官方资源缺失或校验不同：' + rel)
        pending.append((candidate, destination))
    for candidate, destination in pending:
        destination.parent.mkdir(exist_ok=True, parents=True)
        shutil.copy2(candidate, destination)
    return len(pending), len(assets)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=os.environ.get('OFFICIAL_SIMENV_SOURCE_ROOT'))
    args = parser.parse_args()
    copied, count = prepare(source=args.source)
    print(f'运行资源{count}项校验通过，本次导入{copied}项。')

if __name__ == '__main__':
    main()
