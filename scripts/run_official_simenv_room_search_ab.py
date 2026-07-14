"""在官方复杂房间中比较固定巡检与候选驱动主动视角。

两种策略使用相同受控目标布局和相同起点。每个策略开始前允许用 Gazebo 服务
复位一次机器人；策略运行期间只通过 /hw/cmd_vel 运动，并按实际世界位姿记录
平移、偏航、覆盖率、发现时间和确认结果。输出归入现有 active_multiview 目录
的 search_ab 子目录，不创建第六类感知证据。
"""

import argparse
import csv
import json
import math
import os
import shutil
import time
from pathlib import Path

import cv2
import numpy as np

import run_official_simenv_complex_perception_matrix as matrix
from hazardwalker_perception.active_view_policy import choose_active_view_action
from hazardwalker_perception.room_search_policy import (
    choose_fixed_sweep_action,
    choose_room_search_action,
    coverage_ratio,
)


LANE_OFFSETS_M = (-0.60, -0.30, 0.0, 0.30, 0.60)
BASE_X_M = matrix.ROOM_A_ROBOT[0]
MATCH_DISTANCE_M = 0.55


LAYOUTS = (
    {
        'layout_id': 'room_search_01_left_and_center',
        'spheres': ((2.75, 10.00, 3.30), (3.45, 10.05, 3.32)),
        'distractors': (
            ('cylinder_face', (4.15, 9.90, 3.30)),
            ('flat_panel', (2.95, 10.75, 3.32)),
        ),
    },
    {
        'layout_id': 'room_search_02_right_and_depth',
        'spheres': ((3.00, 9.90, 3.30), (3.80, 10.20, 3.34)),
        'distractors': (
            ('cone_face', (2.45, 10.30, 3.30)),
            ('ellipsoid_flat', (4.20, 10.40, 3.32)),
        ),
    },
    {
        'layout_id': 'room_search_03_sparse_two',
        'spheres': (
            (2.80, 10.00, 3.28),
            (3.70, 10.15, 3.32),
        ),
        'distractors': (
            ('cube', (2.35, 10.55, 3.32)),
            ('cylinder_vertical', (4.20, 10.60, 3.34)),
        ),
    },
)


def _normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _yaw(pose):
    if not pose:
        return 0.0
    return math.atan2(
        2.0 * (pose['qw'] * pose['qz'] + pose['qx'] * pose['qy']),
        1.0 - 2.0 * (pose['qy'] ** 2 + pose['qz'] ** 2),
    )


def _lane_x(sector):
    return BASE_X_M + LANE_OFFSETS_M[int(sector)]


def _nearest_sector(pose):
    return min(
        range(len(LANE_OFFSETS_M)),
        key=lambda sector: abs(_lane_x(sector) - float(pose['x'])),
    )


def _execute_twist(env, duration_sec, linear_x=0.0, linear_y=0.0, angular_z=0.0):
    before = matrix._robot_pose(env)
    started = time.monotonic()
    publish_count = max(3, int(round(float(duration_sec) * 10.0)))
    try:
        matrix._run([
            'ros2', 'topic', 'pub', '--rate', '10', '--times', str(publish_count),
            '/hw/cmd_vel', 'geometry_msgs/msg/Twist',
            (
                f'{{linear: {{x: {linear_x}, y: {linear_y}}}, '
                f'angular: {{z: {angular_z}}}}}'
            ),
        ], env, timeout=duration_sec + 12, check=False)
    finally:
        matrix._stop_robot(env)
    after = matrix._robot_pose(env)
    return {
        'duration_sec': round(time.monotonic() - started, 3),
        'command': {
            'linear_x': linear_x,
            'linear_y': linear_y,
            'angular_z': angular_z,
        },
        'before_pose': before,
        'after_pose': after,
        'translation_m': round(matrix._pose_change_m(before, after), 4),
        'yaw_change_deg': round(matrix._pose_yaw_change_deg(before, after), 3),
    }


def _move_to_sector(env, current_sector, target_sector):
    """按视角带间距执行一次真实横移，并用实际水平位移验收。"""

    delta = int(target_sector) - int(current_sector)
    if delta == 0:
        return [], True, 0.0
    action = 'move_right' if delta > 0 else 'move_left'
    required_horizontal_m = 0.16 * abs(delta)
    motions = []
    duration = min(1.8, max(0.6, abs(delta) * 0.6))
    motion = _translate(env, action, duration)
    motion['target_sector'] = int(target_sector)
    motions.append(motion)
    horizontal_m = float(motion.get('horizontal_translation_m', 0.0))
    # 真实占位底盘的速度响应会随支撑接触、地面摩擦略有波动。以反馈位姿为准
    # 做有限次补偿，而不是把一次速度指令的理想位移误当成已经到达视角带。
    retry_count = 0
    while horizontal_m < required_horizontal_m and retry_count < 3:
        remaining_m = required_horizontal_m - horizontal_m
        retry = _translate(env, action, min(1.5, max(0.7, remaining_m / 0.22)))
        retry['target_sector'] = int(target_sector)
        retry['action_reason'] = 'feedback_shortfall_compensation'
        motions.append(retry)
        horizontal_m += float(retry.get('horizontal_translation_m', 0.0))
        retry_count += 1
    shortfall_m = max(0.0, required_horizontal_m - horizontal_m)
    return motions, horizontal_m >= required_horizontal_m, round(shortfall_m, 4)


def _translate(env, action, duration_sec):
    world_commands = {
        'move_left': (-0.35, 0.0),
        'move_right': (0.35, 0.0),
        'move_forward': (0.0, 0.45),
        'move_backward': (0.0, -0.45),
    }
    world_x, world_y = world_commands.get(action, (-0.45, 0.0))
    pose = matrix._robot_pose(env)
    current_yaw = _yaw(pose)
    # VelocityControl 接受机体坐标速度；先把期望世界平移旋回机体系，避免轻微
    # 偏航后“横移”逐渐变成向家具前冲。
    linear_x = math.cos(current_yaw) * world_x + math.sin(current_yaw) * world_y
    linear_y = -math.sin(current_yaw) * world_x + math.cos(current_yaw) * world_y
    motion = _execute_twist(
        env,
        duration_sec,
        linear_x=linear_x,
        linear_y=linear_y,
    )
    motion['action'] = action
    before = motion.get('before_pose') or {}
    after = motion.get('after_pose') or {}
    if before and after:
        motion['horizontal_translation_m'] = round(math.hypot(
            float(after['x']) - float(before['x']),
            float(after['y']) - float(before['y']),
        ), 4)
    else:
        motion['horizontal_translation_m'] = 0.0
    return motion


def _layout_sdf(layout):
    links = []
    for index, position in enumerate(layout['spheres'], start=1):
        links.append(matrix._sphere(f'sphere_{index:02d}', *position, radius=0.12))
    for index, (shape_name, position) in enumerate(layout['distractors'], start=1):
        shape_links = matrix._shape_links(shape_name, position)
        for link_index, link in enumerate(shape_links, start=1):
            links.append(link.replace(
                'name="',
                f'name="distractor_{index:02d}_{link_index:02d}_',
                1,
            ))
    return matrix._sdf(f'hw_{layout["layout_id"]}', links)


def _distance(a, b):
    return math.sqrt(sum((float(a[index]) - float(b[index])) ** 2 for index in range(3)))


def _hazard_positions(snapshot, confirmed_only=False):
    result = []
    for item in snapshot.get('hazards', []):
        if confirmed_only and item.get('status') != 'confirmed':
            continue
        position = item.get('position')
        if isinstance(position, list) and len(position) >= 3:
            result.append((tuple(float(value) for value in position[:3]), item))
    return result


def _match_positions(snapshot, truths, confirmed_only=False, excluded_items=()):
    hazards = _hazard_positions(snapshot, confirmed_only=confirmed_only)
    excluded_ids = {id(item) for item in excluded_items}
    used = set()
    matches = []
    for truth in truths:
        best = None
        for index, (position, item) in enumerate(hazards):
            if index in used or id(item) in excluded_ids:
                continue
            error = _distance(position, truth)
            if error <= MATCH_DISTANCE_M and (best is None or error < best[0]):
                best = (error, index, item)
        if best is not None:
            used.add(best[1])
            matches.append((truth, best[0], best[2]))
    return matches


def _candidate_cluster_key(snapshot):
    for position, item in _hazard_positions(snapshot):
        if item.get('status') == 'confirmed':
            continue
        return ':'.join(str(round(value / 0.40)) for value in position)
    detections = snapshot.get('detections_2d', [])
    if detections:
        return f'image:{detections[0].get("id", 1)}'
    return ''


def _pending_memory_tracks(snapshot, excluded_keys):
    """选择已有正证据但尚未确认的三维候选，供覆盖结束后定点复查。"""

    candidates = []
    for item in snapshot.get('hazards', []):
        if item.get('status') == 'confirmed':
            continue
        position = item.get('position')
        if not isinstance(position, list) or len(position) < 3:
            continue
        key = ':'.join(str(round(float(value) / 0.40)) for value in position[:3])
        if key in excluded_keys or int(item.get('eligible_observation_count', 0)) <= 0:
            continue
        curvature = item.get('median_normalized_depth_curvature')
        if curvature is not None and not 0.08 <= float(curvature) <= 0.34:
            continue
        candidates.append((
            int(item.get('eligible_view_count', item.get('distinct_view_count', 0))),
            float(item.get('confidence', 0.0)),
            key,
            item,
        ))
    candidates.sort(key=lambda entry: (entry[0], entry[1]), reverse=True)
    return [entry[3] for entry in candidates]


def _capture(
    label,
    args,
    topic,
    env,
    images_dir,
    snapshots_dir,
    started,
    policy,
    sector,
):
    # 轨迹节点要求连续稳定帧才向多视角证据累积。真实 cmd_vel 停止后先等待
    # 传感器/TF 都跨过稳定窗口，避免把运动过渡帧误当作一个有效观察视角。
    time.sleep(0.8)
    snapshot = matrix._capture(
        label,
        args,
        topic,
        env,
        images_dir,
        snapshots_dir,
    )
    snapshot['benchmark'] = {
        'elapsed_sec': round(time.monotonic() - started, 3),
        'policy': policy,
        'sector': int(sector),
        'robot_pose': matrix._robot_pose(env),
    }
    path = snapshots_dir / f'{label}_snapshot.json'
    path.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return snapshot


def _run_fixed(layout, args, env, images_dir, snapshots_dir, logs_dir):
    policy = 'fixed_sweep'
    run_id = f'{layout["layout_id"]}_{policy}'
    print(f'[room-search] {run_id}: reset robot', flush=True)
    matrix._set_robot_pose(matrix.ROOM_A_ROBOT, env)
    process = handle = None
    started = time.monotonic()
    snapshots = []
    motions = []
    visited = set()
    try:
        process, handle, topic = matrix._start_detector(
            run_id,
            env,
            logs_dir / f'{run_id}.log',
            radius=0.12,
        )
        print(f'[room-search] {run_id}: detector ready', flush=True)
        time.sleep(2.5)
        current_sector = _nearest_sector(matrix._robot_pose(env))
        while len(visited) < len(LANE_OFFSETS_M):
            recommendation = choose_fixed_sweep_action(
                visited,
                current_sector,
                order=tuple(range(len(LANE_OFFSETS_M))),
            )
            target_sector = recommendation.target_sector
            lane_motions, reached, error_m = _move_to_sector(
                env, current_sector, target_sector,
            )
            motions.extend(lane_motions)
            if not reached:
                raise RuntimeError(
                    f'fixed sweep cannot reach sector {target_sector}: error_m={error_m}'
                )
            current_sector = int(target_sector)
            label = f'{run_id}_sector{current_sector:02d}_center'
            snapshots.append(_capture(
                label, args, topic, env, images_dir, snapshots_dir,
                started, policy, current_sector,
            ))
            visited.add(current_sector)

            motions.append(_translate(env, 'move_left', 1.4))
            snapshots.append(_capture(
                f'{run_id}_sector{current_sector:02d}_left',
                args, topic, env, images_dir, snapshots_dir,
                started, policy, current_sector,
            ))
            motions.append(_translate(env, 'move_right', 2.8))
            snapshots.append(_capture(
                f'{run_id}_sector{current_sector:02d}_right',
                args, topic, env, images_dir, snapshots_dir,
                started, policy, current_sector,
            ))
            # 回到该视角带中心附近但不额外截图，形成固定三视角模板。
            motions.append(_translate(env, 'move_left', 1.4))
    finally:
        matrix._stop_detector(process, handle)
        matrix._stop_robot(env)
    return _summarize_run(layout, policy, snapshots, motions, visited, started)


def _run_active(layout, args, env, images_dir, snapshots_dir, logs_dir):
    policy = 'active_coverage'
    run_id = f'{layout["layout_id"]}_{policy}'
    print(f'[room-search] {run_id}: reset robot', flush=True)
    matrix._set_robot_pose(matrix.ROOM_A_ROBOT, env)
    process = handle = None
    started = time.monotonic()
    snapshots = []
    motions = []
    visited = set()
    rechecks = {}
    try:
        process, handle, topic = matrix._start_detector(
            run_id,
            env,
            logs_dir / f'{run_id}.log',
            radius=0.12,
        )
        print(f'[room-search] {run_id}: detector ready', flush=True)
        time.sleep(2.5)
        current_sector = _nearest_sector(matrix._robot_pose(env))
        last_snapshot = None
        while len(visited) < len(LANE_OFFSETS_M):
            if current_sector not in visited:
                lane_motions, reached, error_m = _move_to_sector(
                    env, current_sector, current_sector,
                )
                motions.extend(lane_motions)
                if not reached:
                    raise RuntimeError(
                        f'active coverage cannot stabilize sector {current_sector}: '
                        f'error_m={error_m}'
                    )
                last_snapshot = _capture(
                    f'{run_id}_sector{current_sector:02d}_coverage',
                    args, topic, env, images_dir, snapshots_dir,
                    started, policy, current_sector,
                )
                snapshots.append(last_snapshot)
                visited.add(current_sector)

            cluster_key = _candidate_cluster_key(last_snapshot or {})
            sector_recheck_count = 0
            while (
                last_snapshot
                and last_snapshot.get('detections_2d')
                and rechecks.get(cluster_key, 0) < 2
                and sector_recheck_count < 4
            ):
                active = choose_active_view_action(
                    last_snapshot.get('detections_2d', []),
                    640,
                    480,
                )
                count = rechecks.get(cluster_key, 0)
                recommendation = choose_room_search_action(
                    last_snapshot.get('detections_2d', []),
                    640,
                    480,
                    visited,
                    current_sector,
                    {str(active.target_id): count},
                )
                if recommendation.mode != 'candidate_recheck':
                    break
                # 固定使用“中心 -> 左侧 -> 右侧”的大基线括号观察；策略只决定
                # 哪些候选值得付出这两次移动，避免根据瞬时框位置连续向一侧漂移。
                action = 'move_left' if count == 0 else 'move_right'
                duration = 1.4 if count == 0 else 2.8
                motions.append(_translate(env, action, duration))
                rechecks[cluster_key] = count + 1
                sector_recheck_count += 1
                last_snapshot = _capture(
                    f'{run_id}_sector{current_sector:02d}_recheck{count + 1:02d}',
                    args, topic, env, images_dir, snapshots_dir,
                    started, policy, current_sector,
                )
                snapshots.append(last_snapshot)
                if count == 1:
                    # 回到括号中心附近，后续覆盖动作从可比位置继续。
                    motions.append(_translate(env, 'move_left', 1.4))
                new_key = _candidate_cluster_key(last_snapshot)
                if new_key and new_key != cluster_key and new_key not in rechecks:
                    cluster_key = new_key

            recommendation = choose_room_search_action(
                [],
                640,
                480,
                visited,
                current_sector,
                {},
            )
            if recommendation.action == 'search_complete':
                break
            next_sector = int(recommendation.target_sector)
            lane_motions, reached, shortfall_m = _move_to_sector(
                env, current_sector, next_sector,
            )
            motions.extend(lane_motions)
            if not reached:
                raise RuntimeError(
                    f'active coverage cannot move from sector {current_sector} '
                    f'to {next_sector}: shortfall_m={shortfall_m}'
                )
            current_sector = next_sector
            last_snapshot = None

        revisited_memory_keys = set()
        for revisit_index in range(2):
            pending = _pending_memory_tracks(last_snapshot or snapshots[-1], revisited_memory_keys)
            if not pending:
                break
            target = pending[0]
            position = tuple(float(value) for value in target['position'][:3])
            memory_key = ':'.join(str(round(value / 0.40)) for value in position)
            revisited_memory_keys.add(memory_key)
            pose = matrix._robot_pose(env)
            delta_x = position[0] - float(pose['x'])
            if abs(delta_x) >= 0.18:
                action = 'move_right' if delta_x > 0 else 'move_left'
                duration = min(1.5, max(0.7, abs(delta_x) / 0.35))
                motion = _translate(env, action, duration)
                motion['action_reason'] = 'pending_3d_track_alignment'
                motions.append(motion)
            last_snapshot = _capture(
                f'{run_id}_memory{revisit_index + 1:02d}_center',
                args, topic, env, images_dir, snapshots_dir,
                started, policy, current_sector,
            )
            snapshots.append(last_snapshot)
            motions.append(_translate(env, 'move_left', 1.4))
            last_snapshot = _capture(
                f'{run_id}_memory{revisit_index + 1:02d}_left',
                args, topic, env, images_dir, snapshots_dir,
                started, policy, current_sector,
            )
            snapshots.append(last_snapshot)
            motions.append(_translate(env, 'move_right', 2.8))
            last_snapshot = _capture(
                f'{run_id}_memory{revisit_index + 1:02d}_right',
                args, topic, env, images_dir, snapshots_dir,
                started, policy, current_sector,
            )
            snapshots.append(last_snapshot)
            motions.append(_translate(env, 'move_left', 1.4))
    finally:
        matrix._stop_detector(process, handle)
        matrix._stop_robot(env)
    return _summarize_run(layout, policy, snapshots, motions, visited, started)


def _summarize_run(layout, policy, snapshots, motions, visited, started):
    sphere_truths = tuple(layout['spheres'])
    distractor_truths = tuple(position for _name, position in layout['distractors'])
    discovered_at = {}
    confirmed_at = {}
    for index, snapshot in enumerate(snapshots):
        elapsed = float(snapshot.get('benchmark', {}).get('elapsed_sec', 0.0))
        for truth, _error, _item in _match_positions(snapshot, sphere_truths):
            discovered_at.setdefault(str(truth), elapsed)
        for truth, _error, _item in _match_positions(snapshot, sphere_truths, confirmed_only=True):
            confirmed_at.setdefault(str(truth), elapsed)
    final = snapshots[-1] if snapshots else {}
    sphere_matches = _match_positions(final, sphere_truths, confirmed_only=True)
    sphere_confirmed = len(sphere_matches)
    distractor_confirmed = len(_match_positions(
        final,
        distractor_truths,
        confirmed_only=True,
        excluded_items=[match[2] for match in sphere_matches],
    ))
    total_translation = sum(float(item.get('translation_m', 0.0)) for item in motions)
    total_yaw = sum(float(item.get('yaw_change_deg', 0.0)) for item in motions)
    yaw_deviations = [
        abs(math.degrees(_normalize_angle(_yaw(item['after_pose']) - matrix.ROOM_A_ROBOT[3])))
        for item in motions
        if item.get('after_pose')
    ]
    max_yaw_deviation = max(yaw_deviations, default=0.0)
    elapsed_sec = round(time.monotonic() - started, 3)
    return {
        'layout_id': layout['layout_id'],
        'policy': policy,
        'expected_sphere_count': len(sphere_truths),
        'known_distractor_count': len(distractor_truths),
        'sphere_discovered_count': len(discovered_at),
        'sphere_confirmed_count': sphere_confirmed,
        'known_distractor_confirmed_count': distractor_confirmed,
        'coverage_ratio': round(coverage_ratio(visited, len(LANE_OFFSETS_M)), 3),
        'visited_sectors': sorted(visited),
        'capture_count': len(snapshots),
        'motion_count': len(motions),
        'total_translation_m': round(total_translation, 4),
        'total_abs_yaw_deg': round(total_yaw, 3),
        'max_yaw_deviation_deg': round(max_yaw_deviation, 3),
        'elapsed_sec': elapsed_sec,
        'mean_discovery_time_sec': (
            round(sum(discovered_at.values()) / len(discovered_at), 3)
            if discovered_at else ''
        ),
        'mean_confirmation_time_sec': (
            round(sum(confirmed_at.values()) / len(confirmed_at), 3)
            if confirmed_at else ''
        ),
        'discovered_at_sec': discovered_at,
        'confirmed_at_sec': confirmed_at,
        'motions': motions,
        'path': [
            item.get('after_pose')
            for item in motions
            if item.get('after_pose')
        ],
        'result': (
            'pass'
            if (
                sphere_confirmed == len(sphere_truths)
                and distractor_confirmed == 0
                and coverage_ratio(visited, len(LANE_OFFSETS_M)) == 1.0
                and max_yaw_deviation <= 12.0
            )
            else 'review'
        ),
    }


def _write_collage(images_dir, output_path):
    paths = sorted(images_dir.glob('*_annotated.png'))
    if not paths:
        return
    thumbs = []
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        thumb = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
        cv2.putText(
            thumb,
            path.stem.replace('_annotated', '')[-52:],
            (6, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.36,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            thumb,
            path.stem.replace('_annotated', '')[-52:],
            (6, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.36,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )
        thumbs.append(thumb)
    columns = 4
    rows = math.ceil(len(thumbs) / columns)
    canvas = np.full((rows * 240, columns * 320, 3), 30, dtype=np.uint8)
    for index, thumb in enumerate(thumbs):
        row, column = divmod(index, columns)
        canvas[row * 240:(row + 1) * 240, column * 320:(column + 1) * 320] = thumb
    cv2.imwrite(str(output_path), canvas)


def _write_path_plot(rows, output_path):
    canvas = np.full((700, 900, 3), 245, dtype=np.uint8)
    all_points = [
        (float(pose['x']), float(pose['y']))
        for row in rows
        for pose in row.get('path', [])
    ]
    if not all_points:
        return
    min_x = min(point[0] for point in all_points) - 0.3
    max_x = max(point[0] for point in all_points) + 0.3
    min_y = min(point[1] for point in all_points) - 0.3
    max_y = max(point[1] for point in all_points) + 0.3

    def pixel(point):
        x = int(60 + (point[0] - min_x) / max(max_x - min_x, 0.1) * 780)
        y = int(640 - (point[1] - min_y) / max(max_y - min_y, 0.1) * 580)
        return x, y

    colors = {'fixed_sweep': (30, 80, 220), 'active_coverage': (40, 170, 40)}
    for row in rows:
        points = [
            pixel((float(pose['x']), float(pose['y'])))
            for pose in row.get('path', [])
        ]
        color = colors.get(row['policy'], (80, 80, 80))
        for first, second in zip(points, points[1:]):
            cv2.line(canvas, first, second, color, 2, cv2.LINE_AA)
        for point in points:
            cv2.circle(canvas, point, 3, color, -1, cv2.LINE_AA)
    cv2.putText(
        canvas,
        'fixed sweep',
        (70, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        colors['fixed_sweep'],
        2,
    )
    cv2.putText(
        canvas,
        'active coverage',
        (260, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        colors['active_coverage'],
        2,
    )
    cv2.imwrite(str(output_path), canvas)


def _comparison(rows):
    result = {}
    for policy in ('fixed_sweep', 'active_coverage'):
        selected = [row for row in rows if row['policy'] == policy]
        result[policy] = {
            'run_count': len(selected),
            'pass_count': sum(row['result'] == 'pass' for row in selected),
            'mean_elapsed_sec': round(
                sum(float(row['elapsed_sec']) for row in selected) / max(len(selected), 1),
                3,
            ),
            'mean_capture_count': round(
                sum(int(row['capture_count']) for row in selected) / max(len(selected), 1),
                3,
            ),
            'mean_translation_m': round(
                sum(float(row['total_translation_m']) for row in selected) / max(len(selected), 1),
                4,
            ),
            'mean_abs_yaw_deg': round(
                sum(float(row['total_abs_yaw_deg']) for row in selected) / max(len(selected), 1),
                3,
            ),
            'sphere_confirmed_count': sum(int(row['sphere_confirmed_count']) for row in selected),
            'known_distractor_confirmed_count': sum(
                int(row['known_distractor_confirmed_count']) for row in selected
            ),
        }
    fixed = result['fixed_sweep']
    active = result['active_coverage']
    comparable = fixed['run_count'] > 0 and active['run_count'] > 0
    result['active_elapsed_reduction_ratio'] = (
        round(
            (fixed['mean_elapsed_sec'] - active['mean_elapsed_sec'])
            / max(fixed['mean_elapsed_sec'], 0.001),
            4,
        ) if comparable else ''
    )
    result['active_capture_reduction_ratio'] = (
        round(
            (fixed['mean_capture_count'] - active['mean_capture_count'])
            / max(fixed['mean_capture_count'], 0.001),
            4,
        ) if comparable else ''
    )
    if not comparable:
        result['status'] = 'not_comparable_single_policy_smoke'
    else:
        result['status'] = (
            'pass'
            if (
                active['sphere_confirmed_count'] >= fixed['sphere_confirmed_count']
                and active['known_distractor_confirmed_count']
                <= fixed['known_distractor_confirmed_count']
                and result['active_elapsed_reduction_ratio'] > 0.0
            )
            else 'review'
        )
    return result


def _write_outputs(output_dir, test_record_dir, rows):
    fields = [
        key
        for key in rows[0]
        if key not in ('motions', 'path', 'discovered_at_sec', 'confirmed_at_sec')
    ]
    (output_dir / 'room_search_ab_trials.json').write_text(
        json.dumps(rows, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    with (output_dir / 'room_search_ab_trials.csv').open(
        'w',
        encoding='utf-8',
        newline='',
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    comparison = _comparison(rows)
    (output_dir / 'room_search_ab_summary.json').write_text(
        json.dumps(comparison, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    reduction = comparison['active_elapsed_reduction_ratio']
    reduction_text = (
        f'{float(reduction) * 100:.2f}%'
        if reduction != '' else '单策略试跑，不可比较'
    )
    readme = (
        '# 复杂房间搜索策略 A/B\n\n'
        '固定巡检与候选驱动主动视角使用相同布局、相同起点和真实 /hw/cmd_vel 运动。\n'
        '运行期间禁止 set_pose；只有策略切换前允许复位一次。\n\n'
        f'- 布局数：{len({row["layout_id"] for row in rows})}\n'
        f'- 固定巡检平均耗时：{comparison["fixed_sweep"]["mean_elapsed_sec"]} s\n'
        f'- 主动策略平均耗时：{comparison["active_coverage"]["mean_elapsed_sec"]} s\n'
        f'- 主动策略耗时下降：{reduction_text}\n'
        f'- 结论：{comparison["status"]}\n'
    )
    (output_dir / 'README.md').write_text(readme, encoding='utf-8')

    test_record_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(
        output_dir / 'room_search_ab_trials.csv',
        test_record_dir / 'room_search_ab_testing_record.csv',
    )
    (test_record_dir / 'room_search_ab_testing_record.json').write_text(
        json.dumps({
            'run_id': 'official_simenv_20260710_rgbd_active_multiview_room_search_ab',
            'environment': 'official SimEnv complex room with stabilized placeholder base',
            'case_count': len(rows),
            'pass_count': sum(row['result'] == 'pass' for row in rows),
            'comparison': comparison,
            'records': rows,
        }, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--suite-dir', type=Path, required=True)
    parser.add_argument('--test-record-dir', type=Path, required=True)
    parser.add_argument('--runtime-root', type=Path, required=True)
    parser.add_argument('--capture-script', type=Path, required=True)
    parser.add_argument('--capture-timeout-sec', type=float, default=16.0)
    parser.add_argument('--layout-limit', type=int, default=len(LAYOUTS))
    parser.add_argument('--layout-start', type=int, default=0)
    parser.add_argument(
        '--policy',
        choices=('both', 'fixed', 'active'),
        default='both',
    )
    args = parser.parse_args()

    output_dir = args.suite_dir / 'search_ab'
    if output_dir.exists():
        shutil.rmtree(output_dir)
    images_dir = output_dir / 'images'
    snapshots_dir = output_dir / 'snapshots'
    logs_dir = output_dir / 'logs'
    for directory in (images_dir, snapshots_dir, logs_dir, args.runtime_root):
        directory.mkdir(parents=True, exist_ok=True)

    env = os.environ.copy()
    rows = []
    layout_start = min(max(int(args.layout_start), 0), len(LAYOUTS) - 1)
    layout_stop = min(layout_start + max(1, int(args.layout_limit)), len(LAYOUTS))
    layouts = LAYOUTS[layout_start:layout_stop]
    for layout in layouts:
        sdf_path = args.runtime_root / f'{layout["layout_id"]}.sdf'
        sdf_path.write_text(_layout_sdf(layout), encoding='utf-8')
        entity_id = matrix._spawn_sdf(f'hw_{layout["layout_id"]}', sdf_path, env)
        try:
            if args.policy in ('both', 'fixed'):
                rows.append(_run_fixed(
                    layout,
                    args,
                    env,
                    images_dir,
                    snapshots_dir,
                    logs_dir,
                ))
                print(json.dumps(rows[-1], ensure_ascii=False), flush=True)
            if args.policy in ('both', 'active'):
                rows.append(_run_active(
                    layout,
                    args,
                    env,
                    images_dir,
                    snapshots_dir,
                    logs_dir,
                ))
                print(json.dumps(rows[-1], ensure_ascii=False), flush=True)
        finally:
            matrix._remove_entity(entity_id, env, f'hw_{layout["layout_id"]}')

    _write_collage(images_dir, output_dir / 'room_search_ab_collage.png')
    _write_path_plot(rows, output_dir / 'room_search_ab_paths.png')
    _write_outputs(output_dir, args.test_record_dir, rows)
    print(json.dumps(_comparison(rows), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
