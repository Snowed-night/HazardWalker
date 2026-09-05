#!/usr/bin/env python3
"""运行子进程并把输出写入有大小上限的轮转日志。"""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys


def rotate(path: Path, backups: int) -> None:
    if backups <= 0:
        path.unlink(missing_ok=True)
        return
    oldest = path.with_name(path.name + f'.{backups}')
    oldest.unlink(missing_ok=True)
    for index in range(backups - 1, 0, -1):
        source = path.with_name(path.name + f'.{index}')
        if source.exists():
            source.replace(path.with_name(path.name + f'.{index + 1}'))
    if path.exists():
        path.replace(path.with_name(path.name + '.1'))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--log', required=True)
    parser.add_argument('--max-bytes', type=int, default=64 * 1024 * 1024)
    parser.add_argument('--backups', type=int, default=2)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = list(args.command)
    if command[:1] == ['--']:
        command = command[1:]
    if not command:
        parser.error('缺少待运行命令')

    log_path = Path(args.log).expanduser().resolve()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    maximum = max(1024, int(args.max_bytes))
    backups = max(0, int(args.backups))
    child = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    def forward(signum, _frame):
        if child.poll() is None:
            os.killpg(child.pid, signum)

    signal.signal(signal.SIGTERM, forward)
    signal.signal(signal.SIGINT, forward)

    written = 0
    handle = log_path.open('wb')
    try:
        assert child.stdout is not None
        while True:
            chunk = child.stdout.read(64 * 1024)
            if not chunk:
                break
            offset = 0
            while offset < len(chunk):
                if written >= maximum:
                    handle.close()
                    rotate(log_path, backups)
                    handle = log_path.open('wb')
                    written = 0
                size = min(maximum - written, len(chunk) - offset)
                handle.write(chunk[offset:offset + size])
                handle.flush()
                written += size
                offset += size
        return child.wait()
    finally:
        handle.close()
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()


if __name__ == '__main__':
    sys.exit(main())
