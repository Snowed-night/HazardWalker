#!/usr/bin/env python3
"""在隔离的官方 SimEnv ROS1/Gazebo Classic 环境运行五类红球证据实验。

所属组：感知定位组 / 测试组。

本执行器刻意不兼容 Gazebo Harmonic，也不会在共享 ``simenv_run`` 中写入模型或速度命令。
它只允许指定独立容器，临时通过 ROS1 Gazebo 服务生成 SDF，随后从 ROS2 ``/hw/*``
订阅真实 RGB-D/TF 感知结果。受控 SDF 的真值仅在所有运行期快照保存后参与评估，
不会传给检测器、视角策略或机器人控制。

使用前条件：
1. 隔离的官方 SimEnv 容器已启动，且容器内 ROS1 Gazebo 服务可用；
2. 对应 ROS_DOMAIN_ID 已有唯一的 ROS2 适配器和 HSV 检测节点；
3. 多视角模式还必须显式加 ``--allow-control``，并以里程计证明每次真实横移。

该脚本把失败如实写入 cases.csv/summary.json，绝不会因为模型成功生成而宣称实验通过。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np

from official_simenv_classic_evidence_cases import BUILDERS, build_suite


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CAPTURE_SCRIPT = PROJECT_ROOT / 'scripts' / 'capture_official_simenv_rgbd_case.py'
SHARED_CONTAINER_NAMES = {'simenv_run', 'simenv'}
SUITE_ARCHIVE_DIRS = {
    'multi_ball_clutter': 'official_simenv_20260710_rgbd_multi_ball_clutter',
    'partial_visibility': 'official_simenv_20260710_rgbd_partial_visibility',
    'red_objects': 'official_simenv_20260710_rgbd_red_objects',
    'active_multiview': 'official_simenv_20260710_rgbd_active_multiview',
    'complex_localization': 'official_simenv_20260710_rgbd_complex_localization',
}
RUN_ID_PATTERN = re.compile(r'^[0-9]{8}_[A-Za-z0-9][A-Za-z0-9._-]*$')


def _run(command: list[str], env: dict[str, str], *, timeout: float = 30.0, check: bool = True) -> subprocess.CompletedProcess:
    """运行外部命令，出错时保留 stdout/stderr，避免测试失败被吞掉。"""

    result = subprocess.run(command, text=True, env=env, capture_output=True, timeout=timeout)
    if check and result.returncode != 0:
        raise RuntimeError(
            f'命令失败：{shlex.join(command)}\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}'
        )
    return result


def _validate_isolated_container(container: str, env: dict[str, str]) -> None:
    """硬拒绝共享官方容器，避免感知实验抢占团队正在使用的机器人。"""

    normalized = str(container).strip()
    if not normalized or normalized in SHARED_CONTAINER_NAMES or 'isolated' not in normalized:
        raise ValueError('--isolated-container 必须是名称包含 isolated 的独立容器，禁止操作 simenv_run。')
    result = _run(['docker', 'inspect', '--format', '{{.State.Running}}', normalized], env, check=False)
    if result.returncode != 0 or result.stdout.strip().lower() != 'true':
        raise RuntimeError(f'隔离容器未运行：{normalized}')


def _suite_output_dir(output_root: Path, suite: str, run_id: str = '') -> Path:
    """把有效复跑结果固定放回五类历史目录下，空 run_id 保持旧临时布局兼容。"""

    if suite not in SUITE_ARCHIVE_DIRS:
        raise ValueError(f'未知五类实验：{suite}')
    normalized_run_id = str(run_id or '').strip()
    if not normalized_run_id:
        return Path(output_root) / f'official_simenv_{time.strftime("%Y%m%d")}_{suite}'
    if RUN_ID_PATTERN.fullmatch(normalized_run_id) is None:
        raise ValueError('--run-id 必须符合 YYYYMMDD_<seed或批次标识>。')
    return (
        Path(output_root)
        / SUITE_ARCHIVE_DIRS[suite]
        / 'reruns'
        / normalized_run_id
    )


def _test_record_output_dir(test_record_root: Path, suite: str, run_id: str) -> Path:
    """测试表与效果图使用同一 run_id，防止跨批次错配。"""

    normalized_run_id = str(run_id or '').strip()
    if RUN_ID_PATTERN.fullmatch(normalized_run_id) is None:
        raise ValueError('生成测试记录时必须提供合法 --run-id。')
    return (
        Path(test_record_root)
        / SUITE_ARCHIVE_DIRS[suite]
        / 'reruns'
        / normalized_run_id
    )


def _ros1_bash(container: str, command: str, env: dict[str, str], *, timeout: float = 30.0, check: bool = True):
    setup = (
        'source /opt/ros/noetic/setup.bash; '
        'source /home/ros/Guoyulun/Competition/SimEnv/devel/setup.bash; '
    )
    return _run(['docker', 'exec', container, 'bash', '-lc', setup + command], env, timeout=timeout, check=check)


def _project_camera_forward_center(translation: tuple[float, float, float],
                                   quaternion: tuple[float, float, float, float],
                                   forward_distance_m: float, target_z: float) -> tuple[float, float, float]:
    """把测试夹具投到相机前方；仅用于 Gazebo 临时模型生成，绝不传给运行期算法。"""

    tx, ty, _ = translation
    qx, qy, qz, qw = quaternion
    # 将相机坐标系的 +X（SimEnv real_sense link 的前方）旋转到世界系。
    forward_x = 1.0 - 2.0 * (qy * qy + qz * qz)
    forward_y = 2.0 * (qx * qy + qw * qz)
    horizontal_norm = math.hypot(forward_x, forward_y)
    if horizontal_norm < 1e-6:
        raise ValueError('相机前向在水平面上的投影为零，不能安全生成可见测试夹具。')
    return (
        tx + forward_distance_m * forward_x / horizontal_norm,
        ty + forward_distance_m * forward_y / horizontal_norm,
        target_z,
    )


def _fixture_center_from_camera(container: str, world_frame: str, camera_frame: str,
                                forward_distance_m: float, target_z: float,
                                env: dict[str, str]) -> tuple[float, float, float]:
    """读取 ROS1 TF 后计算夹具中心；TF 只在测试建模阶段使用。"""

    result = _ros1_bash(
        container,
        # tf_echo 在非交互管道下会缓冲 stdout；无缓冲保证 timeout 结束前已经拿到至少一帧 TF。
        f'PYTHONUNBUFFERED=1 timeout -s INT 7 rosrun tf tf_echo '
        f'{shlex.quote(world_frame)} {shlex.quote(camera_frame)}',
        env,
        timeout=11.0,
        check=False,
    )
    output = f'{result.stdout}\n{result.stderr}'
    translations = re.findall(
        r'Translation:\s*\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\]', output,
    )
    rotations = re.findall(
        r'Rotation:\s*in Quaternion\s*\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\]', output,
    )
    if not translations or not rotations:
        raise RuntimeError(
            f'无法读取 {world_frame}->{camera_frame} 的 TF，拒绝使用固定坐标生成不可见案例。\n{output[-1200:]}'
        )
    return _project_camera_forward_center(
        tuple(float(value) for value in translations[-1]),
        tuple(float(value) for value in rotations[-1]),
        forward_distance_m,
        target_z,
    )


def _spawn_case(container: str, case, work_dir: Path, env: dict[str, str]) -> str:
    """在独立世界生成一个临时模型，模型名包含 case_id 便于严格清理。"""

    model_name = f'hw_evidence_{case.case_id}'
    sdf_path = work_dir / f'{model_name}.sdf'
    sdf_path.write_text(case.sdf, encoding='utf-8')
    remote_path = f'/tmp/{model_name}.sdf'
    _run(['docker', 'cp', str(sdf_path), f'{container}:{remote_path}'], env)
    _ros1_bash(
        container,
        f'rosrun gazebo_ros spawn_model -sdf -file {shlex.quote(remote_path)} '
        f'-model {shlex.quote(model_name)} -x 0 -y 0 -z 0',
        env,
    )
    return model_name


def _delete_case(container: str, model_name: str, env: dict[str, str]) -> bool:
    """清理临时模型；无法确认清理成功时阻止后续案例被残留物污染。"""

    for attempt in range(1, 4):
        try:
            result = _ros1_bash(
                container,
                f'rosservice call /gazebo/delete_model "model_name: {model_name}"',
                env,
                timeout=30.0,
                check=False,
            )
            if result.returncode == 0 and 'success: True' in result.stdout:
                return True
        except subprocess.TimeoutExpired:
            pass
        if attempt < 3:
            time.sleep(2.0)
    return False


def _capture_snapshot(case_id: str, view_index: int, output_dir: Path, detection_topic: str,
                      env: dict[str, str], timeout_sec: float) -> dict[str, Any]:
    """调用独立订阅器保存真实 RGB 与感知 JSON，禁止用命令行截断后的 echo 作证据。"""

    image_dir = output_dir / 'images'
    image_dir.mkdir(parents=True, exist_ok=True)
    snapshot_id = f'{case_id}_view{view_index:02d}'
    _run(
        [sys.executable, str(CAPTURE_SCRIPT), '--case-id', snapshot_id,
         '--output-dir', str(image_dir), '--detection-topic', detection_topic,
         '--timeout-sec', str(timeout_sec)],
        env,
        timeout=timeout_sec + 12.0,
    )
    snapshot_path = image_dir / f'{snapshot_id}_snapshot.json'
    return json.loads(snapshot_path.read_text(encoding='utf-8'))


def _background_edge_ratio(snapshot: dict[str, Any], image_dir: Path) -> float:
    """估计背景结构复杂度，排除红色夹具边缘，避免近墙空画面混入正式证据。"""

    image_path = image_dir / str(snapshot['raw_image'])
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f'无法读取真实 RGB 截图以验证环境复杂度：{image_path}')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red_mask = cv2.bitwise_or(
        cv2.inRange(hsv, (0, 70, 45), (12, 255, 255)),
        cv2.inRange(hsv, (170, 70, 45), (179, 255, 255)),
    )
    # 膨胀后去除球轮廓本身，度量主要由门、墙体、家具等真实场景结构贡献。
    red_mask = cv2.dilate(red_mask, np.ones((9, 9), dtype=np.uint8))
    edges = cv2.Canny(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 60, 150)
    structured_edges = cv2.bitwise_and(edges, cv2.bitwise_not(red_mask))
    return float(np.count_nonzero(structured_edges)) / float(structured_edges.size)


def _read_legal_motion(topic: str, env: dict[str, str]) -> dict[str, float] | None:
    """从导航组的合法 SLAM 位姿读取 xy，禁止读取 Gazebo 真值里程计。"""

    normalized = str(topic or '').strip()
    forbidden = {'/hw/odom', '/Odometry_gazebo'}
    if not normalized or normalized in forbidden or 'ground_truth' in normalized:
        return None

    result = _run(
        ['timeout', '8', 'ros2', 'topic', 'echo', normalized, '--once'],
        env,
        timeout=12.0,
        check=False,
    )
    match_x = re.search(r'\n\s*x:\s*([-+0-9.eE]+)', result.stdout)
    match_y = re.search(r'\n\s*y:\s*([-+0-9.eE]+)', result.stdout)
    if not match_x or not match_y:
        return None
    return {'x': float(match_x.group(1)), 'y': float(match_y.group(1))}


def _move_laterally(command_y: float, duration_sec: float, legal_motion_topic: str,
                    env: dict[str, str]) -> dict[str, Any]:
    """发送横移，并仅以显式提供的合法 SLAM 位姿作为真实位移证据。"""

    before = _read_legal_motion(legal_motion_topic, env)
    payload = f'{{linear: {{x: 0.0, y: {float(command_y):.3f}}}, angular: {{z: 0.0}}}}'
    try:
        _run(
            ['timeout', str(duration_sec), 'ros2', 'topic', 'pub', '--wait-matching-subscriptions', '1',
             '--rate', '10', '/hw/cmd_vel', 'geometry_msgs/msg/Twist', payload],
            env,
            timeout=duration_sec + 8.0,
            check=False,
        )
    finally:
        _run(
            ['ros2', 'topic', 'pub', '--once', '/hw/cmd_vel', 'geometry_msgs/msg/Twist',
             '{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}'],
            env,
            timeout=8.0,
            check=False,
        )
    time.sleep(1.0)
    after = _read_legal_motion(legal_motion_topic, env)
    translation = 0.0 if not before or not after else math.hypot(after['x'] - before['x'], after['y'] - before['y'])
    return {'command_y': command_y, 'duration_sec': duration_sec, 'before': before, 'after': after,
            'translation_m': round(translation, 4),
            'motion_evidence_topic': str(legal_motion_topic or ''),
            'motion_evidence_status': 'legal_slam' if before and after else 'unverified'}


def _start_case_detector(command: str, log_path: Path, env: dict[str, str]) -> Optional[subprocess.Popen]:
    """可选地为每个案例启动独立检测器，杜绝上一个模型的轨迹串入当前案例。"""

    if not command.strip():
        return None
    log_path.parent.mkdir(parents=True, exist_ok=True)
    handle = log_path.open('w', encoding='utf-8')
    try:
        process = subprocess.Popen(
            shlex.split(command), env=env, stdout=handle, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except Exception:
        handle.close()
        raise
    # 句柄交给进程持有，停止时由 _stop_case_detector 关闭，避免日志截断。
    process._hazardwalker_log_handle = handle  # type: ignore[attr-defined]
    return process


def _stop_case_detector(process: Optional[subprocess.Popen]) -> None:
    """停止本执行器自己启动的检测器；绝不按名称杀掉其他成员的业务节点。"""

    if process is None:
        return
    try:
        process.terminate()
        process.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=8.0)
    finally:
        handle = getattr(process, '_hazardwalker_log_handle', None)
        if handle is not None:
            handle.close()


def _strict_count(snapshot: dict[str, Any]) -> int:
    return sum(not bool(item.get('requires_reobservation')) for item in snapshot.get('detections_2d', []))


def _partial_count(snapshot: dict[str, Any]) -> int:
    return sum(bool(item.get('requires_reobservation')) for item in snapshot.get('detections_2d', []))


def _confirmed_count(snapshot: dict[str, Any]) -> int:
    return sum(item.get('status') == 'confirmed' for item in snapshot.get('hazards', []))


def _red_pixel_count(snapshot: dict[str, Any], image_dir: Path) -> int:
    """从实际 RGB 原图统计高饱和红像素，用于复核遮挡几何的真实可见比例。"""

    raw_name = str(snapshot.get('raw_image', '')).strip()
    if not raw_name:
        return 0
    try:
        import cv2
        import numpy as np
    except ImportError:
        return 0
    image = cv2.imread(str(image_dir / raw_name), cv2.IMREAD_COLOR)
    if image is None:
        return 0
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    low = cv2.inRange(hsv, np.array((0, 80, 80), dtype=np.uint8), np.array((10, 255, 255), dtype=np.uint8))
    high = cv2.inRange(hsv, np.array((170, 80, 80), dtype=np.uint8), np.array((179, 255, 255), dtype=np.uint8))
    return int(cv2.countNonZero(cv2.bitwise_or(low, high)))


def _localization_errors(snapshot: dict[str, Any], truth_positions: tuple) -> list[float]:
    """运行结束后才将预测位置和本用例 SDF 真值匹配，产生透明定位误差。"""

    predictions = [tuple(item.get('position', ())) for item in snapshot.get('hazards', [])]
    predictions = [value for value in predictions if len(value) == 3]
    unmatched = set(range(len(predictions)))
    errors: list[float] = []
    for truth in truth_positions:
        candidates = [
            (math.sqrt(sum((float(predictions[index][axis]) - float(truth[axis])) ** 2 for axis in range(3))), index)
            for index in unmatched
        ]
        if not candidates:
            break
        error, index = min(candidates)
        unmatched.remove(index)
        errors.append(error)
    return errors


def _evaluate_case(case, snapshots: list[dict[str, Any]], motions: list[dict[str, Any]], min_translation_m: float,
                   *, image_dir: Optional[Path] = None, baseline_red_pixel_count: int = 0) -> dict[str, Any]:
    """按不同类别给出严格、可复核的结果；不把候选数伪装成确认数。"""

    initial = snapshots[0]
    final = snapshots[-1]
    strict_by_view = [_strict_count(item) for item in snapshots]
    partial_by_view = [_partial_count(item) for item in snapshots]
    confirmed = _confirmed_count(final)
    errors = _localization_errors(final, case.expected_sphere_positions)
    actual_moves = sum(item['translation_m'] >= min_translation_m for item in motions)
    is_target = bool(case.expected_sphere_positions)
    actual_red_pixel_count = _red_pixel_count(initial, image_dir) if image_dir else 0
    actual_visible_ratio = (
        round(actual_red_pixel_count / float(baseline_red_pixel_count), 4)
        if baseline_red_pixel_count > 0 else ''
    )

    if case.suite == 'partial_visibility':
        has_candidate = bool(sum(strict_by_view) + sum(partial_by_view) >= 1)
        design_ratio = case.metadata.get('visible_ratio_design')
        # 深度前景板相对球体有视差；允许 15 个百分点的投影误差，但超出即说明
        # 遮挡几何本身没有实现所标注的梯度，必须重做而非把文件名当证据。
        geometry_ok = (design_ratio is None or actual_visible_ratio == ''
                       or abs(float(actual_visible_ratio) - float(design_ratio)) <= 0.15)
        passed = has_candidate and geometry_ok
        criterion = '任一视角必须产生严格或 reobserve 候选；实际红像素可见比例须接近设计值；候选不是确认。'
    elif case.suite == 'red_objects':
        # 此套件故意是单视角实验：它验证红球能进入严格候选、红色非球体不会
        # 被单帧提升为最终危险源。圆柱端面出现候选是合理的，不得把它伪装成球。
        passed = (_strict_count(initial) >= 1) if is_target else confirmed == 0
        criterion = '单视角红球应产生严格候选；所有红色非球体不得被确认，形状结论交给多视角套件。'
    elif case.suite == 'active_multiview':
        moves_ok = actual_moves >= 2
        passed = moves_ok and (confirmed >= 1 if is_target else confirmed == 0)
        criterion = '两次里程计横移均达阈值；红球确认、非球体不得确认。'
    elif case.suite == 'multi_ball_clutter':
        # 粘连拆分的单帧圆数只是一组待复查假设，不能以“候选数 >= 真值数”掩盖
        # 重复圆、红箱或圆柱端面的虚警。必须在两个合法侧视后仅保留恰好对应
        # 受控红球数的 confirmed 轨迹，才可证明没有重复计数。
        moves_ok = actual_moves >= 2
        passed = moves_ok and confirmed == len(case.expected_sphere_positions)
        criterion = '两次合法横移后，confirmed 轨迹数必须与受控红球数完全相等；候选框数量不作为计数成功。'
    elif case.suite == 'complex_localization':
        passed = len(errors) == len(case.expected_sphere_positions)
        criterion = '每个受控红球都要获得一个可匹配的三维预测，误差另行报告。'
    else:
        passed = max(strict_by_view, default=0) + max(partial_by_view, default=0) >= len(case.expected_sphere_positions)
        criterion = '各视角可见候选数必须覆盖受控红球数；最终确认另行统计。'

    return {
        'case_id': case.case_id,
        'suite': case.suite,
        'description': case.description,
        'expected_red_ball_count': len(case.expected_sphere_positions),
        'initial_strict_count': _strict_count(initial),
        'initial_partial_count': _partial_count(initial),
        'initial_red_pixel_count': actual_red_pixel_count,
        'actual_visible_ratio': actual_visible_ratio,
        'strict_counts_by_view': strict_by_view,
        'partial_counts_by_view': partial_by_view,
        'final_confirmed_count': confirmed,
        'actual_lateral_move_count': actual_moves,
        'motions': motions,
        'localized_truth_count': len(errors),
        'mean_localization_error_m': round(sum(errors) / len(errors), 4) if errors else '',
        'max_localization_error_m': round(max(errors), 4) if errors else '',
        'criterion': criterion,
        'result': 'pass' if passed else 'fail',
        'metadata': case.metadata,
    }


def _write_suite_report(suite_dir: Path, suite: str, rows: list[dict[str, Any]], args) -> None:
    """写入五类目录统一的 summary/CSV/README，图片由 capture 脚本保存在 images/。"""

    suite_dir.mkdir(parents=True, exist_ok=True)
    serialized_rows = []
    for row in rows:
        serialized = dict(row)
        for key in ('strict_counts_by_view', 'partial_counts_by_view', 'background_edge_ratios', 'motions', 'metadata'):
            if key in serialized:
                serialized[key] = json.dumps(serialized[key], ensure_ascii=False)
        serialized_rows.append(serialized)
    if serialized_rows:
        # 某一案例可能在生成模型、抓帧或控制阶段失败；失败行与成功行字段不同，
        # CSV 仍必须完整保留错误和成功指标，不能因为字段不齐悄悄丢信息。
        fieldnames = []
        for item in serialized_rows:
            for key in item:
                if key not in fieldnames:
                    fieldnames.append(key)
        with (suite_dir / 'cases.csv').open('w', encoding='utf-8', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(serialized_rows)
    (suite_dir / 'cases.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    summary = {
        'schema': 'hazardwalker_official_simenv_classic_evidence_v1',
        'run_id': args.run_id or suite_dir.name,
        'code_version': args.code_version or 'unrecorded',
        'evidence_class': 'internal_regression',
        'official_score_eligible': False,
        'suite': suite,
        'case_count': len(rows),
        'pass_count': sum(item['result'] == 'pass' for item in rows),
        'fail_count': sum(item['result'] == 'fail' for item in rows),
        'container': args.isolated_container,
        'ros_domain_id': args.ros_domain_id,
        'camera_topic': '/hw/camera/image_raw',
        'detection_topic': args.detection_topic,
        'control_enabled': bool(args.allow_control),
        'fixture_center_world': list(args.resolved_fixture_center),
        'fixture_center_source': args.fixture_center_source,
        'min_background_edge_ratio': args.min_background_edge_ratio,
        'truth_usage': '仅在快照保存后由本脚本匹配；运行期检测器和运动策略不读取真值。',
    }
    (suite_dir / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    (suite_dir / 'README.md').write_text(
        f'# 官方 SimEnv Gazebo Classic：{suite}\n\n'
        '> 证据类别：内部回归；人工生成受控物体，不属于官方随机场景全流程成绩。\n\n'
        '本目录的每张原图和标注图都由当前官方 ROS1/Gazebo Classic 世界的 `/hw/*` 话题采集。'
        '模型只在隔离容器中临时生成；案例结束后删除。\n\n'
        f'- 案例数：{summary["case_count"]}\n'
        f'- 通过/失败：{summary["pass_count"]}/{summary["fail_count"]}\n'
        f'- ROS_DOMAIN_ID：{args.ros_domain_id}\n'
        f'- 临时夹具中心：{summary["fixture_center_world"]}（{summary["fixture_center_source"]}）\n'
        '- 失败案例保留在 `cases.csv` 和 `images/`，不得删除或改写为成功。\n',
        encoding='utf-8',
    )
    if args.test_record_root is not None:
        record_dir = _test_record_output_dir(args.test_record_root, suite, args.run_id)
        record_dir.mkdir(parents=True, exist_ok=True)
        source_csv = suite_dir / 'cases.csv'
        if source_csv.exists():
            shutil.copyfile(source_csv, record_dir / 'testing_record_perception.csv')
        record_payload = dict(summary)
        record_payload['records'] = rows
        (record_dir / 'testing_record_perception.json').write_text(
            json.dumps(record_payload, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', choices=tuple(BUILDERS) + ('all',), required=True)
    parser.add_argument('--isolated-container', required=True)
    parser.add_argument('--center', nargs=3, type=float, required=True,
                        help='本批受控模型的世界坐标中心；必须由隔离场景当前实际布局确定。')
    parser.add_argument('--fixture-center-from-camera-forward-m', type=float, default=0.0, help=(
        '大于 0 时，读取 ROS1 TF 并在相机前方该距离生成临时受控模型；'
        '只用于测试夹具放置，绝不作为检测、控制或定位输入。'
    ))
    # 官方当前桥接发布 map->odom；Gazebo 的生成坐标与 map 在该隔离基线中对齐。
    # `world` 不是该官方 ROS1 TF 树的已发布帧，故默认使用 map。
    parser.add_argument('--fixture-world-frame', default='map')
    parser.add_argument('--fixture-camera-frame', default='real_sense')
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--run-id', default='', help=(
        '归档批次标识，格式 YYYYMMDD_<seed或批次>。提供后自动写入五类既有目录的 '
        'reruns/<run-id>/；省略时仅使用旧临时输出布局。'
    ))
    parser.add_argument('--code-version', default='', help='本轮 Git commit；归档模式必须显式提供。')
    parser.add_argument('--test-record-root', type=Path, default=None, help=(
        '例如 reports/perception/test_records；归档模式必须提供，并使用相同 run_id 写测试表。'
    ))
    parser.add_argument('--detection-topic', default='/hw/perception/hazard_detections')
    parser.add_argument('--ros-domain-id', default=os.environ.get('ROS_DOMAIN_ID', '0'))
    parser.add_argument('--capture-timeout-sec', type=float, default=20.0)
    parser.add_argument('--settle-sec', type=float, default=2.0)
    parser.add_argument('--min-background-edge-ratio', type=float, default=0.006, help=(
        '正式证据每张真实 RGB 图去除红色目标轮廓后的最小 Canny 边缘比例；'
        '低于该值通常是近墙/空白视野，会如实记为失败。'
    ))
    parser.add_argument('--allow-control', action='store_true')
    parser.add_argument('--lateral-speed-y', type=float, default=-0.70)
    parser.add_argument('--lateral-duration-sec', type=float, default=4.0)
    parser.add_argument('--min-lateral-translation-m', type=float, default=0.20)
    parser.add_argument('--legal-motion-topic', default='', help=(
        '导航组发布的合法 SLAM 位姿 topic；禁止使用 /hw/odom、/Odometry_gazebo 或 ground_truth。'
    ))
    parser.add_argument('--case-limit', type=int, default=0, help='仅用于先验收执行链路，0 表示所有案例。')
    parser.add_argument('--case-id-regex', default='', help='仅运行匹配案例；部分可见套件会自动保留 baseline 供像素比例归一化。')
    parser.add_argument('--detector-command', default='',
                        help='可选：每个案例独立启动的检测节点命令。启用后不能同时保留外部同话题检测器。')
    parser.add_argument('--detector-warmup-sec', type=float, default=2.0)
    args = parser.parse_args()
    if args.run_id:
        if RUN_ID_PATTERN.fullmatch(args.run_id.strip()) is None:
            parser.error('--run-id 必须符合 YYYYMMDD_<seed或批次标识>。')
        if not args.code_version.strip():
            parser.error('归档模式必须显式提供 --code-version。')
        if args.test_record_root is None:
            parser.error('归档模式必须显式提供 --test-record-root。')
    elif args.test_record_root is not None:
        parser.error('--test-record-root 只能与 --run-id 一起使用，防止无批次测试表。')

    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
    _validate_isolated_container(args.isolated_container, env)
    selected = tuple(BUILDERS) if args.suite == 'all' else (args.suite,)
    fixture_center = tuple(args.center)
    args.fixture_center_source = 'explicit_center'
    if args.fixture_center_from_camera_forward_m > 0.0:
        fixture_center = _fixture_center_from_camera(
            args.isolated_container,
            args.fixture_world_frame,
            args.fixture_camera_frame,
            args.fixture_center_from_camera_forward_m,
            args.center[2],
            env,
        )
        args.fixture_center_source = (
            f'{args.fixture_world_frame}->{args.fixture_camera_frame} camera_forward '
            f'{args.fixture_center_from_camera_forward_m:.3f}m (test fixture only)'
        )
        print(
            'fixture center from camera forward (test fixture only): '
            + ', '.join(f'{value:.3f}' for value in fixture_center),
            file=sys.stderr,
        )
    args.resolved_fixture_center = fixture_center
    with tempfile.TemporaryDirectory(prefix='hazardwalker_classic_evidence_') as temporary:
        work_dir = Path(temporary)
        for suite in selected:
            cases = list(build_suite(suite, fixture_center))
            if args.case_id_regex:
                pattern = re.compile(args.case_id_regex)
                selected_cases = [item for item in cases if pattern.search(item.case_id)]
                if suite == 'partial_visibility':
                    baselines = [item for item in cases if item.metadata.get('visible_ratio_design') == 1.0]
                    cases = baselines + [item for item in selected_cases if item not in baselines]
                else:
                    cases = selected_cases
            if args.case_limit > 0:
                cases = cases[:args.case_limit]
            suite_dir = _suite_output_dir(args.output_root, suite, args.run_id)
            rows = []
            baseline_red_pixel_count = 0
            for case in cases:
                cleanup_failed = False
                model_name = ''
                detector_process = None
                snapshots: list[dict[str, Any]] = []
                motions: list[dict[str, Any]] = []
                try:
                    model_name = _spawn_case(args.isolated_container, case, work_dir, env)
                    time.sleep(args.settle_sec)
                    detector_process = _start_case_detector(
                        args.detector_command, suite_dir / 'logs' / f'{case.case_id}_detector.log', env,
                    )
                    if detector_process is not None:
                        time.sleep(args.detector_warmup_sec)
                    snapshots.append(_capture_snapshot(case.case_id, 0, suite_dir, args.detection_topic, env, args.capture_timeout_sec))
                    if suite in ('active_multiview', 'multi_ball_clutter'):
                        if not args.allow_control:
                            raise RuntimeError('多视角/多球实验必须加 --allow-control 并在独立容器内执行。')
                        for view_index, command_y in enumerate((args.lateral_speed_y, -2.0 * args.lateral_speed_y), start=1):
                            motions.append(_move_laterally(
                                command_y, args.lateral_duration_sec, args.legal_motion_topic, env,
                            ))
                            snapshots.append(_capture_snapshot(case.case_id, view_index, suite_dir, args.detection_topic, env, args.capture_timeout_sec))
                    if case.suite == 'partial_visibility' and case.metadata.get('visible_ratio_design') == 1.0:
                        baseline_red_pixel_count = _red_pixel_count(snapshots[0], suite_dir / 'images')
                    rows.append(_evaluate_case(
                        case, snapshots, motions, args.min_lateral_translation_m,
                        image_dir=suite_dir / 'images', baseline_red_pixel_count=baseline_red_pixel_count,
                    ))
                    edge_ratios = [_background_edge_ratio(item, suite_dir / 'images') for item in snapshots]
                    rows[-1]['background_edge_ratios'] = edge_ratios
                    if any(value < args.min_background_edge_ratio for value in edge_ratios):
                        rows[-1]['result'] = 'fail'
                        rows[-1]['criterion'] += (
                            f'；背景结构边缘比例必须 >= {args.min_background_edge_ratio:.4f}，'
                            '防止近墙/空白画面充当复杂环境证据。'
                        )
                except Exception as exc:
                    rows.append({
                        'case_id': case.case_id, 'suite': case.suite, 'description': case.description,
                        'expected_red_ball_count': len(case.expected_sphere_positions), 'result': 'fail',
                        'error': str(exc), 'metadata': case.metadata,
                    })
                finally:
                    _stop_case_detector(detector_process)
                    if model_name:
                        cleanup_failed = not _delete_case(args.isolated_container, model_name, env)
                        if cleanup_failed:
                            rows[-1]['result'] = 'fail'
                            previous_error = str(rows[-1].get('error', '')).strip()
                            rows[-1]['error'] = (
                                f'{previous_error}; ' if previous_error else ''
                            ) + 'Gazebo 临时模型清理失败；为防止残留物污染，已中止该套件。'
                if cleanup_failed:
                    break
            _write_suite_report(suite_dir, suite, rows, args)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
