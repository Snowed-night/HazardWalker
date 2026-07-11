#!/usr/bin/env python3
"""在官方 SimEnv 原生 RGB-D 链路上运行感知证据矩阵。

覆盖局部可见比例扫描、红色非球体多视角形状、数量递进/粘连球和受控三维
定位。每例临时生成 SDF、采集真实 `/hw/*` 图像与当前感知节点输出，并按
PoseInfo 实体 ID 清除模型，防止相邻用例串扰。不会读取比赛裁判真值文件。
"""

import argparse
import csv
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class Scenario:
    """一个受控原生 Gazebo 场景及其仅用于评测的预期信息。"""

    case_id: str
    suite: str
    description: str
    sdf: str
    expected_count: int
    expected_position: tuple | None = None
    expected_visible_ratio: float | None = None
    metadata: dict | None = None


def _run(command, env, timeout=30, check=True):
    """参数列表执行外部命令，避免 SDF 和路径被 shell 转义破坏。"""

    result = subprocess.run(command, env=env, text=True, capture_output=True, timeout=timeout)
    if check and result.returncode != 0:
        raise RuntimeError(f'command failed: {command}\n{result.stdout}\n{result.stderr}')
    return result


def _red_material():
    return '<material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>'


def _gray_material():
    return '<material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material>'


def _visual_link(name, pose, geometry, material):
    return f'<link name="{name}"><pose>{pose}</pose><visual name="visual"><geometry>{geometry}</geometry>{material}</visual></link>'


def _sphere_link(name, x, y, z, radius=0.15):
    return _visual_link(name, f'{x} {y} {z} 0 0 0', f'<sphere><radius>{radius}</radius></sphere>', _red_material())


def _box_link(name, x, y, z, size, material=None):
    sx, sy, sz = size
    return _visual_link(
        name, f'{x} {y} {z} 0 0 0', f'<box><size>{sx} {sy} {sz}</size></box>',
        material or _red_material(),
    )


def _cylinder_link(name, x, y, z, radius, length, roll=0.0):
    return _visual_link(
        name, f'{x} {y} {z} {roll} 0 0',
        f'<cylinder><radius>{radius}</radius><length>{length}</length></cylinder>', _red_material(),
    )


def _sdf(model_name, links):
    """将可见模型链接封装为能由 EntityFactory 创建的静态 SDF。"""

    return '<?xml version="1.0"?><sdf version="1.9"><model name="%s"><static>true</static>%s</model></sdf>' % (
        model_name, ''.join(links),
    )


def _segment_cut_for_visible_ratio(visible_ratio):
    """求圆被竖直遮挡板截断后右侧剩余面积的归一化切线位置。"""

    target = max(0.001, min(0.999, float(visible_ratio)))
    low, high = -1.0, 1.0
    for _ in range(60):
        value = (low + high) / 2.0
        right_fraction = (math.acos(value) - value * math.sqrt(max(0.0, 1.0 - value * value))) / math.pi
        if right_fraction > target:
            low = value
        else:
            high = value
    return (low + high) / 2.0


def build_partial_visibility_scenarios():
    """构造 1 个基准和左右各 10 档的真实前景遮挡红球场景。"""

    scenarios = [Scenario(
        case_id='native3d_rgbd_occlusion_baseline', suite='partial_visibility',
        description='无遮挡完整红球基准',
        sdf=_sdf('hw_matrix_native3d_rgbd_occlusion_baseline', [_sphere_link('ball_link', 0.0, 0.0, 0.30, 0.12)]),
        expected_count=1, expected_visible_ratio=1.0, metadata={'baseline': True},
    )]
    radius = 0.12
    ball_y = 0.0
    # 相机 y 约为 -1.81、目标 y=0；遮挡板设在 y=-0.90 的中间视线平面，
    # 投影尺度约为目标球的 1/2，因此用等效半径换算切线位置。
    occluder_scale = 0.50
    for side in ('left', 'right'):
        for percent in (5, 10, 15, 25, 35, 45, 55, 65, 75, 85):
            visible = percent / 100.0
            cut = _segment_cut_for_visible_ratio(visible) * radius * occluder_scale
            width = 0.20
            if side == 'left':
                plate_center_x = cut - width / 2.0
            else:
                # 右遮挡等价于保留左半圆，镜像左遮挡的切线位置。
                plate_center_x = -cut + width / 2.0
            links = [
                _sphere_link('ball_link', 0.0, ball_y, 0.30, radius),
                _box_link('foreground_plate', plate_center_x, -0.90, 0.30, (width, 0.02, 0.50), _gray_material()),
            ]
            case_id = f'native3d_rgbd_occlusion_{side}_{percent:02d}pct'
            scenarios.append(Scenario(
                case_id=case_id, suite='partial_visibility',
                description=f'前景板从{side}侧遮挡，设计可见比例 {percent}%',
                sdf=_sdf(f'hw_matrix_{case_id}', links), expected_count=1,
                expected_visible_ratio=visible,
                metadata={'side': side, 'target_visible_ratio': visible, 'plate_width_m': width},
            ))
    return scenarios


def build_shape_scenarios():
    """构造球、方体、平板、圆柱正/侧面及混合场景，保留多视角证据。"""

    specs = [
        ('sphere_control', '红球正例', [_sphere_link('sphere', 0, 0, 0.30)], 1),
        ('cube', '红色立方体', [_box_link('cube', 0, 0, 0.30, (0.30, 0.30, 0.30))], 0),
        ('cuboid', '红色长方体', [_box_link('cuboid', 0, 0, 0.30, (0.50, 0.16, 0.22))], 0),
        ('cylinder_face', '红色圆柱端面正对相机', [_cylinder_link('cylinder', 0, 0, 0.30, 0.15, 0.12, roll=math.pi / 2)], 0),
        ('cylinder_side', '红色圆柱侧面', [_cylinder_link('cylinder', 0, 0, 0.30, 0.15, 0.42)], 0),
        ('flat_panel', '红色平板', [_box_link('panel', 0, 0, 0.30, (0.48, 0.03, 0.24))], 0),
        ('sphere_cylinder_mixed', '红球与圆柱端面混合', [
            _sphere_link('sphere', -0.35, 0, 0.30),
            _cylinder_link('cylinder', 0.35, 0, 0.30, 0.15, 0.12, roll=math.pi / 2),
        ], 1),
        ('sphere_panel_mixed', '红球与平板混合', [
            _sphere_link('sphere', -0.35, 0, 0.30),
            _box_link('panel', 0.38, 0, 0.30, (0.36, 0.03, 0.22)),
        ], 1),
    ]
    return [Scenario(
        case_id=f'native3d_rgbd_shape_{name}', suite='shape_multiview', description=description,
        sdf=_sdf(f'hw_matrix_native3d_rgbd_shape_{name}', links), expected_count=expected,
        metadata={'shape_case': name},
    ) for name, description, links, expected in specs]


def build_multi_count_scenarios():
    """构造从 1 到 10 的受控多球数量/粘连递进实验。"""

    scenarios = []
    for count in (1, 2, 3, 4, 5, 6, 8, 10):
        radius = 0.08
        spacing = 0.18 if count <= 8 else 0.16
        start = -(count - 1) * spacing / 2.0
        links = [_sphere_link(f'ball_{index:02d}', start + index * spacing, 0.0, 0.30, radius)
                 for index in range(count)]
        case_id = f'native3d_rgbd_multi_{count:02d}_red'
        scenarios.append(Scenario(
            case_id=case_id, suite='multi_count', description=f'{count} 个均匀分离的受控红球',
            sdf=_sdf(f'hw_matrix_{case_id}', links), expected_count=count,
            metadata={'target_count': count, 'radius_m': radius, 'spacing_m': spacing},
        ))
    # 两个相切球专门验证 Hough/分水岭而不是数量插值。
    touching_links = [_sphere_link('left', -0.12, -1.40, 0.30, 0.13), _sphere_link('right', 0.12, -1.40, 0.30, 0.13)]
    scenarios.append(Scenario(
        case_id='native3d_rgbd_touching_pair', suite='multi_count', description='两个相切红球',
        sdf=_sdf('hw_matrix_native3d_rgbd_touching_pair', touching_links), expected_count=2,
        metadata={'target_count': 2, 'touching': True},
    ))
    return scenarios


def build_localization_scenarios():
    """构造不同距离、横向偏移的标准半径红球，记录实际世界坐标误差。"""

    positions = (
        (0.0, -0.20, 0.30), (-0.25, -0.20, 0.30), (0.25, -0.20, 0.30),
        (0.0, 0.0, 0.30), (-0.30, 0.0, 0.30), (0.30, 0.0, 0.30),
    )
    scenarios = []
    for index, position in enumerate(positions, start=1):
        x, y, z = position
        case_id = f'native3d_rgbd_localization_{index:02d}'
        scenarios.append(Scenario(
            case_id=case_id, suite='localization',
            description=f'受控球世界坐标 ({x:.2f}, {y:.2f}, {z:.2f}) m',
            sdf=_sdf(f'hw_matrix_{case_id}', [_sphere_link('ball', x, y, z, 0.15)]),
            expected_count=1, expected_position=position,
            metadata={'controlled_world_position_m': position, 'radius_m': 0.15},
        ))
    return scenarios


SCENARIO_BUILDERS = {
    'partial_visibility': build_partial_visibility_scenarios,
    'shape_multiview': build_shape_scenarios,
    'multi_count': build_multi_count_scenarios,
    'localization': build_localization_scenarios,
}


def _spawn_sdf(model_name, sdf_path, env):
    for _attempt in range(3):
        response = _run([
            'gz', 'service', '-s', '/world/generated_world/create', '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', f'sdf_filename: "{sdf_path}"',
        ], env, check=False)
        # Gazebo 在高负载时可能先创建成功、再让 service 客户端超时；无论响应
        # 如何都先从公开 PoseInfo 查实体，避免盲目重试生成重复模型。
        for _ in range(5):
            time.sleep(0.35)
            poses = _run(['gz', 'topic', '-e', '-t', '/world/generated_world/pose/info', '-n', '1'], env, timeout=8)
            match = re.search(rf'name:\s+"{re.escape(model_name)}"\s+id:\s+(\d+)', poses.stdout, re.DOTALL)
            if match:
                return int(match.group(1))
        if 'data: true' not in response.stdout.lower():
            time.sleep(1.0)
    raise RuntimeError(f'Cannot create or resolve entity {model_name}.')


def _remove_entity(entity_id, env):
    for _attempt in range(4):
        response = _run([
            'gz', 'service', '-s', '/world/generated_world/remove', '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', f'id: {entity_id}',
        ], env, check=False)
        # 同样以 PoseInfo 为准：remove service 超时并不代表实体仍存在。
        time.sleep(0.35)
        poses = _run(['gz', 'topic', '-e', '-t', '/world/generated_world/pose/info', '-n', '1'], env, timeout=8)
        if not re.search(rf'\bid:\s+{int(entity_id)}\b', poses.stdout):
            return
        if 'data: true' not in response.stdout.lower():
            time.sleep(1.0)
    raise RuntimeError(f'Unable to remove entity {entity_id} after retries.')


def _start_detector(case_id, env, log_path, sphere_radius_m):
    output_topic = f'/hw/perception/evidence_matrix/{case_id}'
    command = [
        'ros2', 'run', 'hazardwalker_perception', 'hsv_detector_node', '--ros-args',
        '-r', f'/hw/perception/hazard_detections:={output_topic}', '-p', 'output_frame:=world',
        '-p', 'confirm_observation_count:=3', '-p', 'confirm_distinct_views:=2',
        '-p', f'sphere_radius_m:={sphere_radius_m}',
    ]
    handle = log_path.open('w', encoding='utf-8')
    process = subprocess.Popen(command, env=env, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
    return process, handle, output_topic


def _stop_detector(process, handle):
    if process is not None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
    if handle is not None:
        handle.close()


def _red_pixels_from_image(path):
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        return 0
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0, 80, 80], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8)),
        cv2.inRange(hsv, np.array([170, 80, 80], dtype=np.uint8), np.array([180, 255, 255], dtype=np.uint8)),
    )
    return int(cv2.countNonZero(mask))


def _distance(a, b):
    return math.sqrt(sum((float(a[index]) - float(b[index])) ** 2 for index in range(3)))


def _run_scenario(scenario, args, env, baseline_red_pixels):
    """创建、采集、评分并清理一个原生 3D 用例。"""

    suite_dir = args.output_root / scenario.suite
    images_dir = suite_dir / 'images'
    snapshots_dir = suite_dir / 'snapshots'
    logs_dir = suite_dir / 'logs'
    generated_dir = suite_dir / 'generated_sdf'
    for directory in (images_dir, snapshots_dir, logs_dir, generated_dir):
        directory.mkdir(parents=True, exist_ok=True)
    model_name = f'hw_matrix_{scenario.case_id}'
    sdf_path = generated_dir / f'{scenario.case_id}.sdf'
    sdf_path.write_text(scenario.sdf, encoding='utf-8')
    entity_id = None
    process = None
    handle = None
    started = time.monotonic()
    try:
        entity_id = _spawn_sdf(model_name, sdf_path, env)
        time.sleep(args.settle_sec)
        radius = float((scenario.metadata or {}).get('radius_m', 0.15))
        process, handle, output_topic = _start_detector(
            scenario.case_id, env, logs_dir / f'{scenario.case_id}_detector.log', radius,
        )
        time.sleep(args.node_warmup_sec)
        _run([
            sys.executable, str(args.capture_script), '--case-id', scenario.case_id,
            '--output-dir', str(images_dir), '--detection-topic', output_topic,
            '--timeout-sec', str(args.capture_timeout_sec),
        ], env, timeout=args.capture_timeout_sec + 10)
        snapshot_path = images_dir / f'{scenario.case_id}_snapshot.json'
        snapshot = json.loads(snapshot_path.read_text(encoding='utf-8'))
        snapshot_target = snapshots_dir / snapshot_path.name
        snapshot_path.replace(snapshot_target)
        raw_path = images_dir / f'{scenario.case_id}_raw.png'
        red_pixels = _red_pixels_from_image(raw_path)
    finally:
        _stop_detector(process, handle)
        if entity_id is not None:
            _remove_entity(entity_id, env)

    detections = list(snapshot.get('detections_2d', []))
    hazards = list(snapshot.get('hazards', []))
    strict_count = sum(1 for item in detections if item.get('confirmation_eligible'))
    partial_count = sum(1 for item in detections if item.get('requires_reobservation'))
    flat_suppressed = sum(1 for item in detections if item.get('depth_shape', {}).get('status') == 'flat')
    localization_error = None
    if scenario.expected_position and hazards:
        localized_positions = [hazard.get('position') for hazard in hazards
                               if len(hazard.get('position', ())) == 3]
        if localized_positions:
            localization_error = min(
                _distance(position, scenario.expected_position) for position in localized_positions
            )
    actual_ratio = red_pixels / float(max(1, baseline_red_pixels))
    result = _score(
        scenario, strict_count, partial_count, flat_suppressed, localization_error, actual_ratio,
    )
    row = {
        'case_id': scenario.case_id,
        'description': scenario.description,
        'expected_red_ball_count': scenario.expected_count,
        'strict_candidate_count': strict_count,
        'partial_reobservation_count': partial_count,
        'flat_depth_suppressed_count': flat_suppressed,
        'raw_red_pixel_count': red_pixels,
        'baseline_red_pixel_count': baseline_red_pixels,
        'actual_red_pixel_ratio': round(actual_ratio, 4),
        'target_visible_ratio': scenario.expected_visible_ratio,
        'localization_error_m': round(localization_error, 4) if localization_error is not None else '',
        'localization_ready': bool(snapshot.get('localization_ready')),
        'result': result,
        'annotated_image': f'images/{scenario.case_id}_annotated.png',
        'snapshot': f'snapshots/{scenario.case_id}_snapshot.json',
        'note': json.dumps(scenario.metadata or {}, ensure_ascii=False),
        'elapsed_sec': round(time.monotonic() - started, 3),
    }
    return row


def _score(scenario, strict_count, partial_count, flat_suppressed, localization_error, actual_ratio):
    """按不同实验目的给出保守验收结果，保留 review 暴露未解决问题。"""

    if scenario.suite == 'partial_visibility':
        if (scenario.metadata or {}).get('baseline'):
            return 'pass' if strict_count >= 1 else 'fail'
        # 遮挡板的最终投影由实际相机位姿决定，必须按原生帧测得的红像素比例
        # 判定，而不是把 SDF 的设计比例当成实验结论。
        if actual_ratio <= 0.01:
            return 'invalid_no_visible_pixels'
        if actual_ratio < 0.45:
            return 'pass' if partial_count >= 1 and strict_count == 0 else 'review'
        return 'pass' if strict_count >= 1 else 'fail'
    if scenario.suite == 'shape_multiview':
        shape_case = (scenario.metadata or {}).get('shape_case', '')
        if shape_case.startswith('sphere'):
            return 'pass' if strict_count >= scenario.expected_count else 'fail'
        if shape_case == 'cylinder_face':
            return 'pass' if strict_count == 0 and flat_suppressed >= 1 else 'review'
        if shape_case == 'flat_panel':
            # 平板可被 2D extent 提前拒绝，也可被 RGB-D 平面证据抑制；两条路径
            # 都满足“不进入 confirmed”的安全要求，报告中保留实际命中路径。
            return 'pass' if strict_count == 0 else 'review'
        return 'pass' if strict_count == 0 else 'review'
    if scenario.suite == 'multi_count':
        return 'pass' if strict_count == scenario.expected_count else 'review'
    if scenario.suite == 'localization':
        return 'pass' if localization_error is not None and localization_error <= 0.15 else 'review'
    return 'review'


def _write_suite_outputs(suite_dir, rows, args):
    """按既有 7 月 5 日规范写 cases、summary 和测试组镜像。"""

    suite_dir.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys()) if rows else []
    (suite_dir / 'cases.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    with (suite_dir / 'cases.csv').open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        'run_id': suite_dir.name,
        'run_date': time.strftime('%Y-%m-%d'),
        'simulator': 'official SimEnv ROS2 Harmonic deployment',
        'official_truth_files_read': False,
        'case_count': len(rows),
        'pass_count': sum(item['result'] == 'pass' for item in rows),
        'review_count': sum(item['result'] == 'review' for item in rows),
        'fail_count': sum(item['result'] == 'fail' for item in rows),
        'invalid_count': sum(item['result'].startswith('invalid_') for item in rows),
        'records': rows,
    }
    (suite_dir / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    record_dir = args.test_record_root / suite_dir.name
    record_dir.mkdir(parents=True, exist_ok=True)
    with (record_dir / 'testing_record_perception.csv').open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    (record_dir / 'testing_record_perception.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8',
    )


def _capture_baseline(args, env):
    """采集无受控模型的底图，用于记录受控红像素相对比例。"""

    baseline_dir = args.output_root / '_baseline'
    baseline_dir.mkdir(parents=True, exist_ok=True)
    # 不启动检测节点，只取一帧原生图；capture 工具要求检测 payload，因此使用
    # 临时 Image 订阅脚本输出的 raw 基准由单独 ROS 节点完成。
    command = [sys.executable, str(args.baseline_capture_script), str(baseline_dir / 'baseline_raw.png')]
    _run(command, env, timeout=args.capture_timeout_sec + 10)
    return _red_pixels_from_image(baseline_dir / 'baseline_raw.png')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', choices=tuple(SCENARIO_BUILDERS) + ('all',), default='all')
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--test-record-root', required=True, type=Path)
    parser.add_argument('--capture-script', required=True, type=Path)
    parser.add_argument('--baseline-capture-script', required=True, type=Path)
    parser.add_argument('--settle-sec', type=float, default=2.5)
    parser.add_argument('--node-warmup-sec', type=float, default=2.5)
    parser.add_argument('--capture-timeout-sec', type=float, default=15.0)
    parser.add_argument('--case-limit', type=int, default=0,
                        help='仅运行每个套件的前 N 例，用于平台诊断；0 表示全部。')
    args = parser.parse_args()
    args.output_root = args.output_root.resolve()
    args.test_record_root = args.test_record_root.resolve()
    args.capture_script = args.capture_script.resolve()
    args.baseline_capture_script = args.baseline_capture_script.resolve()
    env = os.environ.copy()
    selected = list(SCENARIO_BUILDERS) if args.suite == 'all' else [args.suite]
    baseline_red_pixels = _capture_baseline(args, env)
    for suite in selected:
        suite_dir = args.output_root / f'official_simenv_20260710_rgbd_{suite}'
        rows = []
        controlled_red_baseline = None
        scenarios = SCENARIO_BUILDERS[suite]()
        if args.case_limit > 0:
            scenarios = scenarios[:args.case_limit]
        for scenario in scenarios:
            print(f'=== {scenario.case_id} ===', flush=True)
            ratio_baseline = controlled_red_baseline or baseline_red_pixels
            row = _run_scenario(scenario, args, env, ratio_baseline)
            rows.append(row)
            if suite == 'partial_visibility' and (scenario.metadata or {}).get('baseline'):
                controlled_red_baseline = row['raw_red_pixel_count']
            print(json.dumps(row, ensure_ascii=False), flush=True)
        _write_suite_outputs(suite_dir, rows, args)


if __name__ == '__main__':
    main()
