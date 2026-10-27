#!/usr/bin/env python3
"""校验旧版冻结算法和参数，并检查 Linux ROS2/Docker 运行条件。"""
from pathlib import Path
import argparse
import ast
import hashlib
import json
import platform
import shutil
import subprocess
import os

ROOT = Path(__file__).resolve().parents[2]

def verify_sources(root=ROOT):
    manifest = json.loads((root / 'reproduction/frozen-files.json').read_text(encoding='utf-8'))
    errors = []
    for rel, expected in manifest['files'].items():
        path = root / rel
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            errors.append('源码校验不符：' + rel)
    for rel, target in manifest['symlinks'].items():
        path = root / rel
        if path.is_symlink():
            if str(path.readlink()) != target:
                errors.append('符号链接目标不符：' + rel)
        elif not path.is_file() or path.read_text(encoding='utf-8').strip() != target:
            errors.append('缺少Catkin链接或内容与声明不符：' + rel)
    return errors

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-only', action='store_true')
    args = parser.parse_args()
    errors = verify_sources()
    info = json.loads((ROOT / 'reproduction/frozen-files.json').read_text(encoding='utf-8'))
    print(f'旧版文件校验：{len(info["files"])}项；基线：final_root_cause_validation_once')
    for path in (ROOT / 'scripts/reproduction').glob('*.py'):
        ast.parse(path.read_text(encoding='utf-8'), str(path))
    if not args.source_only:
        if platform.system() != 'Linux' or platform.machine() not in ('x86_64', 'amd64'):
            errors.append('完整仿真需要Linux x86_64；其它系统可执行--source-only。')
        if not Path('/opt/ros/jazzy/setup.bash').is_file():
            errors.append('缺少ROS2 Jazzy：/opt/ros/jazzy/setup.bash')
        for name in ('docker', 'git', 'colcon', 'ffmpeg'):
            if not shutil.which(name):
                errors.append('缺少命令：' + name)
        if shutil.which('docker'):
            image = os.environ.get('SIMENV_IMAGE', 'simenv_ros1:noetic-focal')
            result = subprocess.run(['docker', 'image', 'inspect', image, '--format', '{{.Id}}'], capture_output=True, text=True)
            if result.returncode:
                errors.append('没有可访问的仿真镜像：' + image + '；按docs/guidebook/environment.md准备。')
            else:
                original = json.loads((ROOT / 'reproduction/baseline.json').read_text(encoding='utf-8'))['original_image_id']
                print('当前镜像ID：' + result.stdout.strip())
                if result.stdout.strip() != original:
                    print('环境差异：镜像ID与历史成功环境不同，本轮应记录此差异。')
            result = subprocess.run(['docker', 'compose', 'version'], capture_output=True)
            if result.returncode:
                errors.append('缺少Docker Compose插件。')
    for message in errors:
        print('ERROR ' + message)
    print('检查通过。' if not errors else '检查未通过。')
    return int(bool(errors))

if __name__ == '__main__':
    raise SystemExit(main())
