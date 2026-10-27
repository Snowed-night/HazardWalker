#!/usr/bin/env python3
"""运行旧版三层任务一次：检查独占、新目录与依赖，结束后核验实际返航及识别结果。"""
from pathlib import Path
import argparse
import fcntl
import json
import os
import re
import signal
import subprocess
import sys
import tempfile
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from reproduction_check_result import evaluate

def validate_name(name):
    return bool(re.fullmatch(r'[A-Za-z0-9_.-]+', name)) and name not in {'.', '..'}

def busy_gazebo():
    result = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if (entry / 'comm').read_text().strip() == 'gzserver' and (entry / 'stat').read_text().split(') ', 1)[1][0] != 'Z':
                result.append(int(entry.name))
        except (OSError, IndexError):
            pass
    return result

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('seed', nargs='?', type=int, default=202609102)
    parser.add_argument('result_name', nargs='?')
    args = parser.parse_args()
    name = args.result_name or f'seed_{args.seed}_{datetime.now(timezone.utc):%Y%m%d_%H%M%S}'
    if args.seed < 0 or not validate_name(name):
        parser.error('种子必须非负；结果名只允许字母数字_.-且不能为.或..')
    output = ROOT / 'reports/nav' / name
    if output.exists():
        parser.error('结果目录已存在，请使用新名称：' + str(output))
    subprocess.run([sys.executable, str(ROOT / 'scripts/reproduction/verify.py')], check=True)
    subprocess.run([sys.executable, str(ROOT / 'scripts/reproduction/prepare_assets.py')], check=True)
    controller = ROOT / 'ros2_ws/src/hazardwalker_platform/.ros1_catkin_ws/devel/lib/unitree_guide/junior_ctrl'
    if not (ROOT / 'install/setup.bash').is_file() or not os.access(controller, os.X_OK):
        parser.error('缺少构建产物，请先执行bash build.sh')
    lock = (Path(tempfile.gettempdir()) / f'hazardwalker-experiment-{os.getuid()}.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('该账号已有HazardWalker实验持有运行锁')
    busy = busy_gazebo()
    if busy:
        parser.error('服务器已有Gazebo仿真，占用PID：' + ', '.join(map(str, busy)))
    platform = ROOT / 'ros2_ws/src/hazardwalker_platform'
    for rel in ('results', 'results/video_capture', 'logs', 'generated_building'):
        path = platform / rel
        path.mkdir(exist_ok=True, parents=True)
        if not os.access(path, os.W_OK):
            parser.error('运行目录不可写：' + str(path))
    env = dict(os.environ, HAZARDWALKER_ROOT=str(ROOT), HAZARDWALKER_SEED=str(args.seed),
               DOCKER_SIMENV_USER=os.environ.get('DOCKER_SIMENV_USER', 'station_cluster'),
               ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID', '43'),
               USE_GPU=os.environ.get('USE_GPU', '0'), HAZARDWALKER_RECORD_SLAM_INPUTS='0')
    env.pop('HAZARDWALKER_ALLOW_DIRTY_DIAGNOSTIC', None)
    process = subprocess.Popen(['bash', 'scripts/run_stable_three_floor_experiment.sh', str(output)],
                               cwd=ROOT, env=env, start_new_session=True)
    def interrupt(signum, frame):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    previous = {s: signal.signal(s, interrupt) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        status = process.wait()
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
    result = evaluate(output)
    result['runner_exit_code'] = status
    check = subprocess.run([sys.executable, str(ROOT / 'scripts/reproduction/verify.py'), '--source-only'])
    result['source_check_passed'] = check.returncode == 0
    result['passed'] = result['passed'] and status == 0 and check.returncode == 0
    if output.is_dir():
        (output / 'reproduction_acceptance.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1

if __name__ == '__main__':
    raise SystemExit(main())
