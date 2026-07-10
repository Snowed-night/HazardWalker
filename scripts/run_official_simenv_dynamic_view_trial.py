#!/usr/bin/env python3
"""用官方 `/hw/cmd_vel` 做一次受控红球真实视角变化试验。

本脚本不移动目标、不读取场景真值：仅临时生成标准红球，连续运行当前感知
节点，在发布有限时长的机器人转向命令前后保存原生检测图与快照。最终记录
`view_id`、轨迹状态和离散视角数，用来证明平台是否真的提供了第二相机视角。
"""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path


def _run(command, env, timeout=30, check=True):
    result = subprocess.run(command, env=env, text=True, capture_output=True, timeout=timeout)
    if check and result.returncode != 0:
        raise RuntimeError(f'command failed: {command}\n{result.stdout}\n{result.stderr}')
    return result


def _resolve_entity_id(model_name, env):
    poses = _run(['gz', 'topic', '-e', '-t', '/world/generated_world/pose/info', '-n', '1'], env, timeout=8)
    match = re.search(rf'name:\s+"{re.escape(model_name)}"\s+id:\s+(\d+)', poses.stdout, re.DOTALL)
    if not match:
        raise RuntimeError(f'cannot resolve entity id for {model_name}')
    return int(match.group(1))


def _spawn(model_name, sdf_path, env):
    response = _run([
        'gz', 'service', '-s', '/world/generated_world/create', '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', f'sdf_filename: "{sdf_path}"',
    ], env, check=False)
    for _ in range(5):
        time.sleep(0.4)
        try:
            return _resolve_entity_id(model_name, env)
        except RuntimeError:
            continue
    raise RuntimeError(f'failed to spawn {model_name}: {response.stdout} {response.stderr}')


def _remove(entity_id, env):
    for _ in range(4):
        _run([
            'gz', 'service', '-s', '/world/generated_world/remove', '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', f'id: {entity_id}',
        ], env, check=False)
        time.sleep(0.4)
        poses = _run(['gz', 'topic', '-e', '-t', '/world/generated_world/pose/info', '-n', '1'], env, timeout=8)
        if not re.search(rf'\bid:\s+{int(entity_id)}\b', poses.stdout):
            return


def _capture(case_id, capture_script, output_dir, topic, env):
    _run([
        sys.executable, str(capture_script), '--case-id', case_id, '--output-dir', str(output_dir),
        '--detection-topic', topic, '--timeout-sec', '15',
    ], env, timeout=25)
    return json.loads((output_dir / f'{case_id}_snapshot.json').read_text(encoding='utf-8'))


def _view_ids(snapshot):
    return sorted({str(item.get('view_id', '')) for item in snapshot.get('detections_2d', []) if item.get('view_id')})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdf-path', required=True, type=Path)
    parser.add_argument('--model-name', required=True)
    parser.add_argument('--capture-script', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--angular-z', type=float, default=0.20)
    parser.add_argument('--duration-sec', type=float, default=2.0)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    topic = '/hw/perception/dynamic_view_trial'
    log_path = args.output_dir / 'hsv_detector.log'
    process = None
    handle = None
    entity_id = None
    try:
        entity_id = _spawn(args.model_name, args.sdf_path.resolve(), env)
        time.sleep(2.0)
        handle = log_path.open('w', encoding='utf-8')
        process = subprocess.Popen([
            'ros2', 'run', 'hazardwalker_perception', 'hsv_detector_node', '--ros-args',
            '-r', f'/hw/perception/hazard_detections:={topic}', '-p', 'output_frame:=world',
            '-p', 'confirm_observation_count:=3', '-p', 'confirm_distinct_views:=2',
        ], env=env, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(2.0)
        before = _capture('before_turn', args.capture_script, args.output_dir, topic, env)
        _run([
            'timeout', str(args.duration_sec), 'ros2', 'topic', 'pub', '-r', '10', '/hw/cmd_vel',
            'geometry_msgs/msg/Twist',
            f'{{linear: {{x: 0.0}}, angular: {{z: {args.angular_z}}}}}',
        ], env=env, check=False, timeout=args.duration_sec + 5)
        time.sleep(1.0)
        after = _capture('after_turn', args.capture_script, args.output_dir, topic, env)
        result = {
            'scenario': 'official_simenv_dynamic_view_trial',
            'official_truth_files_read': False,
            'cmd_vel': {'angular_z': args.angular_z, 'duration_sec': args.duration_sec},
            'before_view_ids': _view_ids(before),
            'after_view_ids': _view_ids(after),
            'distinct_view_count_after': max(
                [int(item.get('distinct_view_count', 0)) for item in after.get('hazards', [])] or [0]
            ),
            'track_status_after': [item.get('status') for item in after.get('hazards', [])],
            'before_snapshot': 'before_turn_snapshot.json',
            'after_snapshot': 'after_turn_snapshot.json',
            'interpretation': (
                '只有前后 view_id 不同且 distinct_view_count >= 2 时，才可声称平台提供了真实第二视角。'
            ),
        }
        (args.output_dir / 'summary.json').write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8',
        )
        print(json.dumps(result, ensure_ascii=False))
    finally:
        if process is not None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        if handle is not None:
            handle.close()
        if entity_id is not None:
            _remove(entity_id, env)


if __name__ == '__main__':
    main()
