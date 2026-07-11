#!/usr/bin/env python3
"""在官方 SimEnv ROS2 环境采集可复跑的 RGB-D 感知完整测试套件。

脚本只通过公开 Gazebo 服务临时生成受控模型、启动当前感知节点并订阅公开
`/hw/*` 话题；不会读取比赛危险源真值或场景布局文件。每例都会产出原始图、
实际节点输出快照、标注图和可供测试组汇总的 CSV/JSON。
"""

import argparse
import csv
import json
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class SuiteCase:
    """受控模型用例的公开评测标签，不依赖官方比赛场景真值。"""

    case_id: str
    model_file: str
    model_name: str
    category: str
    expected_red_ball_count: int
    expected_outcome: str
    note: str


SUITE_CASES = (
    SuiteCase('case_01_near_sphere', 'case_01_near_sphere.sdf', 'hw_suite_near_sphere',
              '完整球体正例', 1, 'confirmable', '近距离完整红球，验证 RGB-D 定位链路。'),
    SuiteCase('case_02_far_sphere', 'case_02_far_sphere.sdf', 'hw_suite_far_sphere',
              '远距离球体正例', 1, 'confirmable', '远距离完整红球，验证球心投影补偿。'),
    SuiteCase('case_03_partial_occlusion', 'case_03_partial_occlusion.sdf', 'hw_suite_partial_occlusion',
              'FOV 边缘局部可见', 1, 'reobserve_only', '约 40% 可见球应保留待复查候选，不能单帧确认。'),
    SuiteCase('case_04_red_cube', 'case_04_red_cube.sdf', 'hw_suite_red_cube',
              '红色立方体干扰', 0, 'reject_or_reobserve', '红色立方体不能成为 confirmed 危险源。'),
    SuiteCase('case_05_red_cylinder_face', 'case_05_red_cylinder_face.sdf', 'hw_suite_red_cylinder_face',
              '圆柱端面干扰', 0, 'reject_or_reobserve', '近圆形圆柱端面须由深度几何抑制确认。'),
    SuiteCase('case_06_red_panel', 'case_06_red_panel.sdf', 'hw_suite_red_panel',
              '红色平板干扰', 0, 'reject_or_reobserve', '红色薄板不能成为 confirmed 危险源。'),
    SuiteCase('case_07_multi_separated', 'case_07_multi_separated.sdf', 'hw_suite_multi_separated',
              '分离多球', 3, 'confirmable', '三个分离红球验证多目标计数。'),
    SuiteCase('case_08_touching_pair', 'case_08_touching_pair.sdf', 'hw_suite_touching_pair',
              '粘连双球', 2, 'reobserve_or_split', '验证 Hough/分水岭分离；未完全分离时应触发复查。'),
)


def _run(command, env, timeout=30, check=True):
    """以参数列表执行命令，避免路径和 SDF 请求遭到 shell 转义破坏。"""

    completed = subprocess.run(command, env=env, text=True, capture_output=True, timeout=timeout)
    if check and completed.returncode != 0:
        raise RuntimeError(
            f'Command failed ({completed.returncode}): {command}\n{completed.stdout}\n{completed.stderr}'
        )
    return completed


def _spawn_model(case, model_dir, env):
    """经公开 EntityFactory 服务生成单个受控模型。"""

    model_path = model_dir / case.model_file
    if not model_path.is_file():
        raise FileNotFoundError(model_path)
    response = _run([
        'gz', 'service', '-s', '/world/generated_world/create',
        '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000', '--req', f'sdf_filename: "{model_path}"',
    ], env=env)
    if 'data: true' not in response.stdout.lower():
        raise RuntimeError(f'Gazebo did not confirm model creation: {response.stdout} {response.stderr}')
    # 复合 SDF 的顶层 model 按名称删除在当前 Harmonic 版本中会返回成功但实体
    # 仍留在世界里，因此从公开 PoseInfo 读取本次 model ID，后续按 ID 精确清理。
    time.sleep(0.4)
    poses = _run([
        'gz', 'topic', '-e', '-t', '/world/generated_world/pose/info', '-n', '1',
    ], env=env, timeout=8)
    match = re.search(
        rf'name:\s+"{re.escape(case.model_name)}"\s+id:\s+(\d+)', poses.stdout, re.DOTALL,
    )
    if not match:
        raise RuntimeError(f'Unable to resolve Gazebo entity ID for {case.model_name}.')
    return int(match.group(1))


def _remove_model(case, entity_id, env):
    """按受控模型名清理；失败会抛出，避免后续案例混入前例目标。"""

    response = _run([
        'gz', 'service', '-s', '/world/generated_world/remove',
        '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000', '--req', f'id: {int(entity_id)}',
    ], env=env, check=False)
    if response.returncode != 0 or 'data: true' not in response.stdout.lower():
        raise RuntimeError(
            f'Unable to remove {case.model_name} (id={entity_id}): {response.stdout} {response.stderr}'
        )


def _launch_detector(case, env, output_topic, log_path):
    """独立节点与独立输出话题避免测试窗口混入平台常驻节点的历史轨迹。"""

    command = [
        'ros2', 'run', 'hazardwalker_perception', 'hsv_detector_node', '--ros-args',
        '-r', f'/hw/perception/hazard_detections:={output_topic}',
        '-p', 'output_frame:=world', '-p', 'confirm_observation_count:=3',
        '-p', 'confirm_distinct_views:=2',
    ]
    handle = log_path.open('w', encoding='utf-8')
    # ros2 run 会再派生实际 Python 节点；单独新建进程组，才能确保用例结束时
    # 包装进程和子节点一起停止，避免后续用例收到旧输出或争抢 CPU。
    process = subprocess.Popen(
        command, env=env, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True,
    )
    return process, handle


def _collect_case(case, args, env):
    """运行一例、读取实际快照、以统一字段返回测试记录。"""

    images_dir = args.output_dir / 'images'
    logs_dir = args.output_dir / 'logs'
    images_dir.mkdir(parents=True, exist_ok=True)
    logs_dir.mkdir(parents=True, exist_ok=True)
    output_topic = f'/hw/perception/rgbd_suite/{case.case_id}'
    log_path = logs_dir / f'{case.case_id}_detector.log'
    process = None
    handle = None
    entity_id = None
    started = time.monotonic()
    try:
        entity_id = _spawn_model(case, args.model_dir, env)
        time.sleep(args.settle_sec)
        process, handle = _launch_detector(case, env, output_topic, log_path)
        time.sleep(args.node_warmup_sec)
        _run([
            sys.executable, str(args.capture_script), '--case-id', case.case_id,
            '--output-dir', str(images_dir), '--detection-topic', output_topic,
            '--timeout-sec', str(args.capture_timeout_sec),
        ], env=env, timeout=args.capture_timeout_sec + 10)
        snapshot_path = images_dir / f'{case.case_id}_snapshot.json'
        snapshot = json.loads(snapshot_path.read_text(encoding='utf-8'))
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
        # 清理失败必须显式暴露；不能带着上一例目标继续跑。
        if entity_id is not None:
            _remove_model(case, entity_id, env)

    detections = list(snapshot.get('detections_2d', []))
    hazards = list(snapshot.get('hazards', []))
    eligible_count = sum(1 for item in detections if item.get('confirmation_eligible'))
    reobserve_count = sum(1 for item in detections if item.get('requires_reobservation'))
    suppressed_flat_count = sum(
        1 for item in detections
        if item.get('depth_shape', {}).get('status') == 'flat'
    )
    confirmed_count = sum(1 for item in hazards if item.get('status') == 'confirmed')
    outcome = _evaluate_outcome(case, eligible_count, reobserve_count, confirmed_count)
    return {
        'case_id': case.case_id,
        'category': case.category,
        'expected_red_ball_count': case.expected_red_ball_count,
        'expected_outcome': case.expected_outcome,
        'candidate_count': len(detections),
        'confirmation_eligible_count': eligible_count,
        'reobservation_candidate_count': reobserve_count,
        'flat_depth_suppressed_count': suppressed_flat_count,
        'confirmed_count': confirmed_count,
        'localization_ready': bool(snapshot.get('localization_ready')),
        'result': outcome,
        'elapsed_sec': round(time.monotonic() - started, 3),
        'annotated_image': f'images/{case.case_id}_annotated.png',
        'raw_image': f'images/{case.case_id}_raw.png',
        'snapshot': f'images/{case.case_id}_snapshot.json',
        'detector_log': f'logs/{case.case_id}_detector.log',
        'note': case.note,
    }


def _evaluate_outcome(case, eligible_count, reobserve_count, confirmed_count):
    """只根据当前实际节点输出评估本例，不把受控标签伪装为平台检测结果。"""

    if case.expected_outcome == 'confirmable':
        return 'pass' if eligible_count >= case.expected_red_ball_count else 'fail'
    if case.expected_outcome == 'reobserve_only':
        return 'pass' if reobserve_count >= 1 and confirmed_count == 0 else 'review'
    if case.expected_outcome == 'reject_or_reobserve':
        return 'pass' if confirmed_count == 0 else 'fail'
    if case.expected_outcome == 'reobserve_or_split':
        return 'pass' if eligible_count >= case.expected_red_ball_count or reobserve_count >= 1 else 'review'
    return 'review'


def _write_records(args, records):
    """将同一份数据写成套件 JSON、CSV 及测试组镜像，避免人工转录。"""

    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / 'cases.json').write_text(
        json.dumps(records, ensure_ascii=False, indent=2) + '\n', encoding='utf-8',
    )
    fields = list(records[0].keys()) if records else []
    for path in (args.output_dir / 'cases.csv', args.test_record_dir / 'testing_record_perception.csv'):
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', encoding='utf-8', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(records)
    summary = {
        'run_id': args.output_dir.name,
        'run_date': time.strftime('%Y-%m-%d'),
        'simulator': 'official SimEnv ROS2 Harmonic deployment',
        'official_truth_files_read': False,
        'case_count': len(records),
        'pass_count': sum(item['result'] == 'pass' for item in records),
        'review_count': sum(item['result'] == 'review' for item in records),
        'fail_count': sum(item['result'] == 'fail' for item in records),
        'records': records,
    }
    (args.output_dir / 'summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8',
    )
    args.test_record_dir.mkdir(parents=True, exist_ok=True)
    (args.test_record_dir / 'testing_record_perception.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8',
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-dir', type=Path, required=True)
    parser.add_argument('--capture-script', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--test-record-dir', type=Path, required=True)
    parser.add_argument('--settle-sec', type=float, default=2.5)
    parser.add_argument('--node-warmup-sec', type=float, default=2.5)
    parser.add_argument('--capture-timeout-sec', type=float, default=15.0)
    args = parser.parse_args()
    args.model_dir = args.model_dir.resolve()
    args.capture_script = args.capture_script.resolve()
    args.output_dir = args.output_dir.resolve()
    args.test_record_dir = args.test_record_dir.resolve()
    if not args.capture_script.is_file():
        raise SystemExit(f'capture script not found: {args.capture_script}')

    env = os.environ.copy()
    records = []
    for case in SUITE_CASES:
        print(f'=== {case.case_id} ===', flush=True)
        record = _collect_case(case, args, env)
        records.append(record)
        print(json.dumps(record, ensure_ascii=False), flush=True)
    _write_records(args, records)
    print(json.dumps({'summary': str(args.output_dir / 'summary.json')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
