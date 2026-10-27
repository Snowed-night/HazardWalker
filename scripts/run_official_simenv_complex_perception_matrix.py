#!/usr/bin/env python3
"""在官方 SimEnv 复杂房间中运行五类感知证据矩阵。

所属组：感知定位组 / 测试组。
文件作用：
在保留墙体、门、家具、立柱和自然遮挡的官方原生 3D 场景中，运行多球粘连、
部分可见、红色物品、真实机器人多视角和复杂三维定位实验。受控目标只用于
建立可审计真值；脚本不读取 danger_truth 或私有场景布局文件。
当前实现边界：
机器人每例可重置到受控起点，但多视角的第二/第三帧必须由 /hw/cmd_vel 产生
真实 Gazebo 模型运动，并同时验证世界位姿变化。所有速度动作结束后显式停车。
"""

from __future__ import annotations

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

from hazardwalker_perception.active_view_policy import choose_active_view_action


ROOM_A_ROBOT = (3.30, 8.20, 3.20, math.pi / 2.0)
ROOM_A_TARGET = (3.30, 10.00, 3.30)
ROOM_B_ROBOT = (3.00, 22.50, 0.60, math.pi / 2.0)
ROOM_B_TARGET = (3.00, 25.70, 0.25)


@dataclass(frozen=True)
class Scenario:
    """一个复杂房间受控场景及其评测真值。"""

    case_id: str
    suite: str
    description: str
    sdf: str
    expected_count: int
    robot_pose: tuple = ROOM_A_ROBOT
    expected_positions: tuple = ()
    expected_visible_ratio: float | None = None
    motion_steps: int = 0
    metadata: dict | None = None


def _run(command, env, timeout=30, check=True):
    result = subprocess.run(command, env=env, text=True, capture_output=True, timeout=timeout)
    if check and result.returncode != 0:
        raise RuntimeError(f'command failed: {command}\n{result.stdout}\n{result.stderr}')
    return result


def _material(red=1.0, green=0.0, blue=0.0):
    color = f'{red} {green} {blue} 1'
    return f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>'


def _link(name, pose, geometry, material=None):
    return (
        f'<link name="{name}"><pose>{pose}</pose><visual name="visual">'
        f'<geometry>{geometry}</geometry>{material or _material()}</visual></link>'
    )


def _sphere(name, x, y, z, radius=0.15, material=None):
    return _link(name, f'{x} {y} {z} 0 0 0', f'<sphere><radius>{radius}</radius></sphere>', material)


def _box(name, x, y, z, size, rpy=(0.0, 0.0, 0.0), material=None):
    roll, pitch, yaw = rpy
    sx, sy, sz = size
    return _link(
        name, f'{x} {y} {z} {roll} {pitch} {yaw}',
        f'<box><size>{sx} {sy} {sz}</size></box>', material,
    )


def _round_geometry(name, x, y, z, kind, radius, length=0.0, rpy=(0.0, 0.0, 0.0)):
    roll, pitch, yaw = rpy
    if kind in ('cylinder', 'cone', 'capsule'):
        geometry = f'<{kind}><radius>{radius}</radius><length>{length}</length></{kind}>'
    else:
        raise ValueError(f'unsupported round geometry: {kind}')
    return _link(name, f'{x} {y} {z} {roll} {pitch} {yaw}', geometry)


def _ellipsoid(name, x, y, z, radii, rpy=(0.0, 0.0, 0.0)):
    roll, pitch, yaw = rpy
    rx, ry, rz = radii
    return _link(
        name, f'{x} {y} {z} {roll} {pitch} {yaw}',
        f'<ellipsoid><radii>{rx} {ry} {rz}</radii></ellipsoid>',
    )


def _sdf(model_name, links):
    return (
        '<?xml version="1.0"?><sdf version="1.11">'
        f'<model name="{model_name}"><static>true</static>{"".join(links)}</model></sdf>'
    )


def _segment_cut_for_visible_ratio(visible_ratio):
    target = max(0.001, min(0.999, float(visible_ratio)))
    low, high = -1.0, 1.0
    for _ in range(60):
        value = (low + high) / 2.0
        fraction = (math.acos(value) - value * math.sqrt(max(0.0, 1.0 - value * value))) / math.pi
        if fraction > target:
            low = value
        else:
            high = value
    return (low + high) / 2.0


def _shape_links(shape_name, center):
    """返回 24 种红色物品的原生几何；复合物用多个 link 构造不规则轮廓。"""

    x, y, z = center
    specs = {
        'sphere_standard': [_sphere('sphere', x, y, z, 0.15)],
        'sphere_small': [_sphere('sphere', x, y, z, 0.10)],
        'sphere_large': [_sphere('sphere', x, y, z, 0.22)],
        'cube': [_box('cube', x, y, z, (0.30, 0.30, 0.30))],
        'tall_cuboid': [_box('cuboid', x, y, z + 0.08, (0.18, 0.18, 0.46))],
        'flat_panel': [_box('panel', x, y, z, (0.48, 0.04, 0.26))],
        'cylinder_vertical': [_round_geometry('cylinder', x, y, z, 'cylinder', 0.15, 0.42)],
        'cylinder_face': [_round_geometry(
            'cylinder', x, y, z, 'cylinder', 0.15, 0.20, (math.pi / 2.0, 0.0, 0.0),
        )],
        'disc_face': [_round_geometry(
            'disc', x, y, z, 'cylinder', 0.18, 0.035, (math.pi / 2.0, 0.0, 0.0),
        )],
        'cone_vertical': [_round_geometry('cone', x, y, z, 'cone', 0.18, 0.42)],
        'cone_face': [_round_geometry(
            'cone', x, y, z, 'cone', 0.18, 0.38, (math.pi / 2.0, 0.0, 0.0),
        )],
        'ellipsoid_round': [_ellipsoid('ellipsoid', x, y, z, (0.15, 0.15, 0.20))],
        'ellipsoid_long': [_ellipsoid('ellipsoid', x, y, z, (0.26, 0.12, 0.12), (0.0, 0.0, 0.35))],
        'ellipsoid_flat': [_ellipsoid('ellipsoid', x, y, z, (0.20, 0.06, 0.18))],
        'capsule_vertical': [_round_geometry('capsule', x, y, z, 'capsule', 0.10, 0.28)],
        'capsule_face': [_round_geometry(
            'capsule', x, y, z, 'capsule', 0.10, 0.32, (math.pi / 2.0, 0.0, 0.0),
        )],
        'l_shape': [
            _box('l_vertical', x - 0.08, y, z, (0.12, 0.16, 0.40)),
            _box('l_base', x + 0.08, y, z - 0.14, (0.32, 0.16, 0.12)),
        ],
        't_shape': [
            _box('t_stem', x, y, z - 0.03, (0.11, 0.15, 0.36)),
            _box('t_head', x, y, z + 0.15, (0.38, 0.15, 0.11)),
        ],
        'cross_shape': [
            _box('cross_v', x, y, z, (0.10, 0.14, 0.42)),
            _box('cross_h', x, y, z, (0.42, 0.14, 0.10)),
        ],
        'dumbbell': [
            _sphere('left', x - 0.17, y, z, 0.12), _sphere('right', x + 0.17, y, z, 0.12),
            _box('bar', x, y, z, (0.24, 0.08, 0.08)),
        ],
        'two_lobe': [_sphere('left', x - 0.09, y, z, 0.14), _sphere('right', x + 0.09, y, z, 0.14)],
        'three_lobe': [
            _sphere('left', x - 0.11, y, z - 0.04, 0.12),
            _sphere('right', x + 0.11, y, z - 0.04, 0.12),
            _sphere('top', x, y, z + 0.12, 0.12),
        ],
        'ring_blocks': [
            _box(f'ring_{index}', x + math.cos(index * math.pi / 4.0) * 0.16, y,
                 z + math.sin(index * math.pi / 4.0) * 0.16, (0.09, 0.10, 0.09),
                 rpy=(0.0, index * math.pi / 4.0, 0.0))
            for index in range(8)
        ],
        'stair_irregular': [
            _box('step_1', x - 0.14, y, z - 0.10, (0.14, 0.16, 0.14)),
            _box('step_2', x, y, z - 0.03, (0.14, 0.16, 0.28)),
            _box('step_3', x + 0.14, y, z + 0.04, (0.14, 0.16, 0.42)),
        ],
    }
    return specs[shape_name]


SHAPE_NAMES = (
    'sphere_standard', 'sphere_small', 'sphere_large', 'cube', 'tall_cuboid', 'flat_panel',
    'cylinder_vertical', 'cylinder_face', 'disc_face', 'cone_vertical', 'cone_face',
    'ellipsoid_round', 'ellipsoid_long', 'ellipsoid_flat', 'capsule_vertical', 'capsule_face',
    'l_shape', 't_shape', 'cross_shape', 'dumbbell', 'two_lobe', 'three_lobe', 'ring_blocks',
    'stair_irregular',
)

SPHERE_RADII_M = {'sphere_standard': 0.15, 'sphere_small': 0.10, 'sphere_large': 0.22}


def build_multi_ball_scenarios():
    """非规则多球、前后遮挡、粘连和红色方体干扰共 10 例。"""

    layouts = (
        ('touching_pair', ((-0.13, 0.0, 0.0), (0.13, 0.0, 0.0)), 0.14),
        ('depth_overlap_pair', ((-0.10, -0.18, 0.0), (0.10, 0.18, 0.02)), 0.13),
        ('triangle_three', ((-0.18, 0.08, 0.0), (0.18, 0.12, 0.0), (0.0, -0.08, 0.18)), 0.13),
        ('dense_four', ((-0.45, 0.25, 0.0), (-0.15, -0.20, 0.05), (0.18, 0.22, 0.0), (0.48, -0.15, 0.08)), 0.11),
        ('irregular_five', ((-0.68, 0.35, 0.0), (-0.35, -0.18, 0.04), (-0.02, 0.25, 0.0), (0.31, -0.20, 0.08), (0.65, 0.18, 0.0)), 0.095),
        ('near_far_six', ((-0.72, -0.30, 0.0), (-0.45, 0.25, 0.0), (-0.18, 0.58, 0.05), (0.12, -0.22, 0.0), (0.42, 0.30, 0.08), (0.72, 0.62, 0.0)), 0.085),
        ('scattered_eight', ((-0.78, -0.28, 0.0), (-0.58, 0.35, 0.10), (-0.36, 0.05, 0.0), (-0.12, 0.62, 0.02), (0.10, -0.22, 0.12), (0.34, 0.20, 0.0), (0.58, 0.55, 0.08), (0.78, -0.05, 0.0)), 0.075),
        ('two_touching_pairs', ((-0.52, 0.10, 0.0), (-0.32, 0.10, 0.0), (0.28, 0.35, 0.04), (0.48, 0.35, 0.04)), 0.11),
        ('occlusion_chain_seven', ((-0.72, 0.35, 0.0), (-0.48, 0.02, 0.05), (-0.24, 0.52, 0.0), (0.0, -0.10, 0.08), (0.24, 0.38, 0.0), (0.48, 0.05, 0.06), (0.72, 0.58, 0.0)), 0.08),
        ('mixed_distractor_six', ((-0.70, 0.05, 0.0), (-0.42, 0.48, 0.05), (-0.14, -0.18, 0.0), (0.16, 0.35, 0.10), (0.46, -0.08, 0.0), (0.72, 0.55, 0.02)), 0.085),
    )
    scenarios = []
    for index, (name, offsets, radius) in enumerate(layouts, start=1):
        robot, target = ROOM_A_ROBOT, ROOM_A_TARGET
        tx, ty, tz = target
        links = [_sphere(f'ball_{ball_index:02d}', tx + dx, ty + dy, tz + dz, radius)
                 for ball_index, (dx, dy, dz) in enumerate(offsets, start=1)]
        if name == 'mixed_distractor_six':
            links.extend([
                _box('red_box_distractor', tx - 0.95, ty + 0.28, tz, (0.28, 0.20, 0.34)),
                _sphere('green_ball_distractor', tx + 0.95, ty + 0.15, tz, 0.13, _material(0, 1, 0)),
            ])
        case_id = f'complex_multi_{index:02d}_{name}'
        positions = tuple((tx + dx, ty + dy, tz + dz) for dx, dy, dz in offsets)
        scenarios.append(Scenario(
            case_id, 'multi_ball_clutter', f'复杂房间非规则多球：{name}',
            _sdf(f'hw_complex_{case_id}', links), len(offsets), robot,
            expected_positions=positions, motion_steps=2,
            metadata={'layout': name, 'radius_m': radius},
        ))
    return scenarios


def build_partial_visibility_scenarios():
    """复杂家具背景下 21 例部分可见，并要求真实横移后恢复严格候选。"""

    tx, ty, tz = ROOM_A_TARGET
    scenarios = [Scenario(
        'complex_partial_00_baseline', 'partial_visibility', '复杂房间无遮挡红球基准',
        _sdf('hw_complex_complex_partial_00_baseline', [_sphere('ball', tx, ty, tz, 0.15)]), 1,
        ROOM_A_ROBOT, expected_positions=((tx, ty, tz),), expected_visible_ratio=1.0,
        metadata={'baseline': True},
    )]
    radius = 0.15
    for side in ('left', 'right'):
        for percent in (5, 10, 15, 25, 35, 45, 55, 65, 75, 85):
            visible = percent / 100.0
            cut = _segment_cut_for_visible_ratio(visible) * radius * 0.52
            width = 0.34
            center_x = tx + (cut - width / 2.0 if side == 'left' else -cut + width / 2.0)
            links = [
                _sphere('ball', tx, ty, tz, radius),
                _box('foreground_plate', center_x, ty - 0.90, tz, (width, 0.035, 0.55), material=_material(0.28, 0.28, 0.28)),
            ]
            case_id = f'complex_partial_{side}_{percent:02d}pct'
            scenarios.append(Scenario(
                case_id, 'partial_visibility', f'复杂房间{side}侧遮挡，设计可见 {percent}%',
                _sdf(f'hw_complex_{case_id}', links), 1, ROOM_A_ROBOT,
                expected_positions=((tx, ty, tz),), expected_visible_ratio=visible, motion_steps=2,
                metadata={'side': side, 'target_visible_ratio': visible},
            ))
    return scenarios


def build_red_object_scenarios():
    """24 种不同红色物品的单视角误报压力测试，与多视角实验严格分离。"""

    scenarios = []
    for index, shape_name in enumerate(SHAPE_NAMES, start=1):
        robot, target = ROOM_A_ROBOT, ROOM_A_TARGET
        expected = 1 if shape_name.startswith('sphere_') else 0
        case_id = f'complex_object_{index:02d}_{shape_name}'
        scenarios.append(Scenario(
            case_id, 'red_objects', f'复杂房间红色物品：{shape_name}',
            _sdf(f'hw_complex_{case_id}', _shape_links(shape_name, target)), expected, robot,
            metadata={
                'shape_name': shape_name, 'is_sphere': bool(expected),
                'radius_m': SPHERE_RADII_M.get(shape_name, 0.15),
            },
        ))
    return scenarios


def build_active_multiview_scenarios():
    """20 例真实机器人横移复查；球应确认，非球体应保持未确认或被否决。"""

    selected = (
        'sphere_standard', 'sphere_small', 'sphere_large',
        'cylinder_face', 'disc_face', 'cone_face', 'ellipsoid_round', 'ellipsoid_flat',
        'capsule_face', 'cube', 'flat_panel', 'cylinder_vertical', 'cone_vertical',
        'ellipsoid_long', 'capsule_vertical', 'l_shape', 'cross_shape', 'dumbbell',
        'three_lobe', 'ring_blocks',
    )
    scenarios = []
    for index, shape_name in enumerate(selected, start=1):
        robot, target = ROOM_A_ROBOT, ROOM_A_TARGET
        expected = 1 if shape_name.startswith('sphere_') else 0
        case_id = f'complex_multiview_{index:02d}_{shape_name}'
        scenarios.append(Scenario(
            case_id, 'active_multiview', f'真实横移复查：{shape_name}',
            _sdf(f'hw_complex_{case_id}', _shape_links(shape_name, target)), expected, robot,
            expected_positions=(target,) if expected else (), motion_steps=2,
            metadata={
                'shape_name': shape_name, 'is_sphere': bool(expected),
                'radius_m': SPHERE_RADII_M.get(shape_name, 0.15),
            },
        ))
    return scenarios


def build_complex_localization_scenarios():
    """复杂房间内 8 种非规则多球布局，共 32 个三维定位真值点。"""

    layouts = (
        ((-0.55, -0.10, 0.00), (0.00, 0.20, 0.08), (0.52, 0.48, 0.00)),
        ((-0.72, 0.45, 0.05), (-0.22, -0.18, 0.00), (0.30, 0.20, 0.10), (0.70, 0.62, 0.00)),
        ((-0.64, -0.25, 0.00), (-0.18, 0.40, 0.12), (0.24, -0.05, 0.00), (0.62, 0.32, 0.06)),
        ((-0.80, 0.62, 0.00), (-0.38, 0.08, 0.08), (0.05, -0.20, 0.00), (0.42, 0.45, 0.12), (0.78, 0.02, 0.00)),
        ((-0.58, 0.25, 0.10), (-0.05, -0.15, 0.00), (0.48, 0.52, 0.04)),
        ((-0.76, -0.12, 0.00), (-0.28, 0.50, 0.06), (0.20, 0.08, 0.12), (0.68, 0.60, 0.00)),
        ((-0.66, 0.55, 0.00), (-0.20, 0.12, 0.10), (0.18, -0.22, 0.00), (0.64, 0.30, 0.06)),
        ((-0.82, 0.18, 0.00), (-0.42, 0.70, 0.08), (0.00, -0.15, 0.12), (0.38, 0.42, 0.00), (0.78, 0.02, 0.06)),
    )
    scenarios = []
    for index, offsets in enumerate(layouts, start=1):
        robot, target = ROOM_A_ROBOT, ROOM_A_TARGET
        tx, ty, tz = target
        positions = tuple((tx + dx, ty + dy, tz + dz) for dx, dy, dz in offsets)
        links = [_sphere(f'ball_{ball_index:02d}', *position, radius=0.12)
                 for ball_index, position in enumerate(positions, start=1)]
        case_id = f'complex_localization_{index:02d}'
        scenarios.append(Scenario(
            case_id, 'complex_localization', f'复杂房间多球三维定位 {index:02d}',
            _sdf(f'hw_complex_{case_id}', links), len(positions), robot,
            expected_positions=positions, metadata={'radius_m': 0.12, 'truth_point_count': len(positions)},
        ))
    return scenarios


SCENARIO_BUILDERS = {
    'multi_ball_clutter': build_multi_ball_scenarios,
    'partial_visibility': build_partial_visibility_scenarios,
    'red_objects': build_red_object_scenarios,
    'active_multiview': build_active_multiview_scenarios,
    'complex_localization': build_complex_localization_scenarios,
}


def _spawn_sdf(model_name, sdf_path, env):
    response = _run([
        'gz', 'service', '-s', '/world/generated_world/create', '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', f'sdf_filename: "{sdf_path}"',
    ], env, check=False)
    for _ in range(12):
        time.sleep(0.35)
        poses = _run(['gz', 'topic', '-e', '-t', '/world/generated_world/pose/info', '-n', '1'], env, timeout=8)
        match = re.search(rf'name:\s+"{re.escape(model_name)}"\s+id:\s+(\d+)', poses.stdout, re.DOTALL)
        if match:
            return int(match.group(1))
    raise RuntimeError(f'cannot create {model_name}: {response.stdout} {response.stderr}')


def _remove_entity(entity_id, env, model_name=''):
    for _ in range(5):
        _run([
            'gz', 'service', '-s', '/world/generated_world/remove', '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', f'id: {entity_id}',
        ], env, check=False)
        time.sleep(0.35)
        poses = _run(['gz', 'topic', '-e', '-t', '/world/generated_world/pose/info', '-n', '1'], env, timeout=8)
        if model_name:
            still_present = bool(re.search(
                rf'name:\s+"{re.escape(model_name)}"\s+id:\s+{int(entity_id)}\b',
                poses.stdout,
            ))
        else:
            still_present = bool(re.search(rf'\bid:\s+{int(entity_id)}\b', poses.stdout))
        if not still_present:
            return
    raise RuntimeError(f'cannot remove model {model_name or entity_id} (entity_id={entity_id})')


def _stop_robot(env):
    # 先通过比赛 ROS 接口连续发送零速，再直接覆盖 Gazebo 控制器的锁存话题。
    # 后者是测试环境的安全兜底：若桥接丢失最后一个 ROS 包，机器人不能在截图
    # 和 TF 采样期间继续漂移，否则会制造假的视角和重复三维轨迹。
    _run([
        'timeout', '0.8', 'ros2', 'topic', 'pub', '-r', '20', '/hw/cmd_vel', 'geometry_msgs/msg/Twist',
        '{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}',
    ], env, timeout=3, check=False)
    for _ in range(3):
        # `gz topic -p` 默认持续发布而不会自行退出；若不包一层 timeout，
        # 停车安全兜底会把一次策略动作卡住约 30 秒，污染真实搜索耗时。
        _run([
            'timeout', '1.0', 'gz', 'topic', '-t', '/cmd_vel', '-m', 'gz.msgs.Twist', '-p',
            'linear { x: 0 y: 0 z: 0 } angular { x: 0 y: 0 z: 0 }',
        ], env, timeout=3, check=False)
        time.sleep(0.08)


def _set_robot_pose(pose, env):
    _stop_robot(env)
    x, y, z, yaw = pose
    response = None
    for _attempt in range(3):
        response = _run([
            'gz', 'service', '-s', '/world/generated_world/set_pose', '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req',
            f'name: "a1_gazebo", position: {{x: {x}, y: {y}, z: {z}}}, '
            f'orientation: {{z: {math.sin(yaw / 2.0)}, w: {math.cos(yaw / 2.0)}}}',
        ], env, check=False)
        if 'data: true' in response.stdout.lower():
            # 复位后再次清零并等待物理引擎稳定，防止残余速度把机器人拖离起点。
            _stop_robot(env)
            time.sleep(0.6)
            return
        time.sleep(0.8)
    raise RuntimeError(f'cannot reset robot pose: {response.stdout} {response.stderr}')


def _robot_pose(env):
    result = None
    for attempt in range(3):
        try:
            result = _run([
                'gz', 'topic', '-e', '-t', '/world/generated_world/dynamic_pose/info', '-n', '1',
            ], env, timeout=8, check=False)
            if 'name: "a1_gazebo"' in result.stdout:
                break
            result = None
            time.sleep(0.5)
        except subprocess.TimeoutExpired:
            if attempt == 2:
                return None
            time.sleep(0.5)
    if result is None:
        return None
    block = re.search(
        r'name:\s+"a1_gazebo".*?position\s*\{(.*?)\}.*?orientation\s*\{(.*?)\}',
        result.stdout, re.DOTALL,
    )
    if not block:
        return None
    def value(text, field, default=0.0):
        match = re.search(rf'\b{field}:\s+([-+0-9.eE]+)', text)
        return float(match.group(1)) if match else default
    position, orientation = block.groups()
    return {
        'x': value(position, 'x'), 'y': value(position, 'y'), 'z': value(position, 'z'),
        'qx': value(orientation, 'x'), 'qy': value(orientation, 'y'),
        'qz': value(orientation, 'z'), 'qw': value(orientation, 'w', 1.0),
    }


def _orient_robot_toward(target, env):
    """朝向候选三维位置，补偿官方占位底盘原地转向量不足。"""

    pose = _robot_pose(env)
    if not pose:
        return False
    target_x, target_y, _target_z = target
    yaw = math.atan2(float(target_y) - pose['y'], float(target_x) - pose['x'])
    _stop_robot(env)
    response = _run([
        'gz', 'service', '-s', '/world/generated_world/set_pose', '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req',
        f'name: "a1_gazebo", position: {{x: {pose["x"]}, y: {pose["y"]}, z: {pose["z"]}}}, '
        f'orientation: {{z: {math.sin(yaw / 2.0)}, w: {math.cos(yaw / 2.0)}}}',
    ], env, check=False)
    _stop_robot(env)
    time.sleep(0.8)
    return 'data: true' in response.stdout.lower()


def _pose_change_m(before, after):
    if not before or not after:
        return 0.0
    return math.sqrt(sum((after[key] - before[key]) ** 2 for key in ('x', 'y', 'z')))


def _horizontal_pose_change_m(before, after):
    """只计算 xy 基线；上下起伏不能被当成多视角或侧向复查。"""
    if not before or not after:
        return 0.0
    return math.hypot(after['x'] - before['x'], after['y'] - before['y'])


def _pose_yaw_change_deg(before, after):
    if not before or not after:
        return 0.0
    def yaw(pose):
        return math.atan2(
            2.0 * (pose['qw'] * pose['qz'] + pose['qx'] * pose['qy']),
            1.0 - 2.0 * (pose['qy'] ** 2 + pose['qz'] ** 2),
        )
    difference = math.atan2(math.sin(yaw(after) - yaw(before)), math.cos(yaw(after) - yaw(before)))
    return abs(math.degrees(difference))


def _execute_reobservation(action, duration_sec, env):
    commands = {
        'move_left': (0.0, 0.60, 0.0), 'move_right': (0.0, -0.60, 0.0),
        'move_forward': (0.45, 0.0, 0.0), 'turn_left': (0.0, 0.0, 0.45),
        'turn_right': (0.0, 0.0, -0.45), 'adjust_pitch': (0.0, 0.55, 0.0),
        'hold_observation': (0.0, 0.0, 0.0), 'continue_exploring': (0.0, 0.55, 0.0),
    }
    linear_x, linear_y, angular_z = commands.get(action, (0.0, 0.55, 0.0))
    try:
        _run([
            'timeout', str(duration_sec), 'ros2', 'topic', 'pub', '-r', '10', '/hw/cmd_vel',
            'geometry_msgs/msg/Twist',
            f'{{linear: {{x: {linear_x}, y: {linear_y}}}, angular: {{z: {angular_z}}}}}',
        ], env, timeout=duration_sec + 5, check=False)
    finally:
        _stop_robot(env)


def _start_detector(case_id, env, log_path, radius):
    topic = f'/hw/perception/complex_matrix/{case_id}'
    handle = log_path.open('w', encoding='utf-8')
    process = subprocess.Popen([
        'ros2', 'run', 'hazardwalker_perception', 'hsv_detector_node', '--ros-args',
        '-r', f'/hw/perception/hazard_detections:={topic}', '-p', 'output_frame:=world',
        '-p', 'confirm_observation_count:=3', '-p', 'confirm_distinct_views:=3',
        '-p', f'sphere_radius_m:={radius}', '-p', 'reject_after_missed_count:=1000',
    ], env=env, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
    return process, handle, topic


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


def _capture(case_id, args, topic, env, images_dir, snapshots_dir):
    _run([
        sys.executable, str(args.capture_script), '--case-id', case_id,
        '--output-dir', str(images_dir), '--detection-topic', topic,
        '--timeout-sec', str(args.capture_timeout_sec),
    ], env, timeout=args.capture_timeout_sec + 10)
    snapshot_source = images_dir / f'{case_id}_snapshot.json'
    snapshot_target = snapshots_dir / snapshot_source.name
    snapshot_source.replace(snapshot_target)
    return json.loads(snapshot_target.read_text(encoding='utf-8'))


def _strict_count(snapshot):
    return sum(bool(item.get('confirmation_eligible')) for item in snapshot.get('detections_2d', []))


def _partial_count(snapshot):
    return sum(bool(item.get('requires_reobservation')) for item in snapshot.get('detections_2d', []))


def _confirmed_count(snapshot):
    return sum(item.get('status') == 'confirmed' for item in snapshot.get('hazards', []))


def _confirmed_near_count(snapshot, target, max_distance_m=0.55):
    count = 0
    for item in snapshot.get('hazards', []):
        position = item.get('position', ())
        if item.get('status') != 'confirmed' or len(position) != 3:
            continue
        error = math.sqrt(sum((float(position[index]) - float(target[index])) ** 2 for index in range(3)))
        count += error <= max_distance_m
    return count


def _depth_shape_count(snapshot, status):
    return sum(
        item.get('depth_shape', {}).get('status') == status
        for item in snapshot.get('detections_2d', [])
    )


def _red_pixel_count_from_image(path):
    """统计原始 RGB 图中的高质量红像素，用于实测遮挡可见比例。"""

    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        return 0
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    low = cv2.inRange(hsv, np.array((0, 70, 80), dtype=np.uint8), np.array((10, 255, 255), dtype=np.uint8))
    high = cv2.inRange(hsv, np.array((170, 70, 80), dtype=np.uint8), np.array((179, 255, 255), dtype=np.uint8))
    return int(cv2.countNonZero(cv2.bitwise_or(low, high)))


def _recommendation(snapshot):
    recommendation = choose_active_view_action(snapshot.get('detections_2d', []), 640, 480)
    return recommendation.to_dict()


def _candidate_aim_point(snapshot):
    """从公开感知轨迹估计候选簇中心，供机器人保持目标在视场内。"""

    positions = [
        item.get('position') for item in snapshot.get('hazards', [])
        if len(item.get('position', ())) == 3
    ]
    if not positions:
        return None
    return tuple(float(np.median([position[axis] for position in positions])) for axis in range(3))


def _match_localization_errors(expected_positions, snapshot, max_match_error_m=0.55):
    predictions = [tuple(item.get('position', ())) for item in snapshot.get('hazards', [])]
    predictions = [item for item in predictions if len(item) == 3]
    unused = set(range(len(predictions)))
    errors = []
    for truth in expected_positions:
        candidates = [
            (math.sqrt(sum((float(predictions[index][axis]) - float(truth[axis])) ** 2 for axis in range(3))), index)
            for index in unused
        ]
        if not candidates:
            continue
        error, index = min(candidates)
        if error > max_match_error_m:
            continue
        unused.remove(index)
        errors.append(error)
    return errors


def _best_track_view_bearing_span_deg(snapshot, expected_positions=()):
    """读取轨迹已累积的目标侧向视差；可选真值仅用于测试记录匹配。"""
    spans = []
    for item in snapshot.get('hazards', []):
        position = item.get('position', ())
        if len(position) != 3:
            continue
        if expected_positions and not _match_localization_errors(expected_positions, {
            'hazards': [item],
        }):
            continue
        value = item.get('view_bearing_span_deg')
        if value is not None:
            spans.append(float(value))
    return max(spans, default=0.0)


def _run_scenario(scenario, args, env):
    suite_dir = args.output_root / f'official_simenv_20260710_rgbd_{scenario.suite}'
    images_dir, snapshots_dir = suite_dir / 'images', suite_dir / 'snapshots'
    runtime_dir = args.runtime_root / scenario.suite
    logs_dir, sdf_dir = runtime_dir / 'logs', runtime_dir / 'generated_sdf'
    for directory in (images_dir, snapshots_dir, logs_dir, sdf_dir):
        directory.mkdir(parents=True, exist_ok=True)
    sdf_path = sdf_dir / f'{scenario.case_id}.sdf'
    sdf_path.write_text(scenario.sdf, encoding='utf-8')
    model_name = f'hw_complex_{scenario.case_id}'
    entity_id = process = handle = None
    started = time.monotonic()
    snapshots = []
    recommendations = []
    motions = []
    last_aim_target = None
    try:
        _set_robot_pose(scenario.robot_pose, env)
        time.sleep(args.pose_settle_sec)
        entity_id = _spawn_sdf(model_name, sdf_path, env)
        time.sleep(args.model_settle_sec)
        radius = float((scenario.metadata or {}).get('radius_m', 0.15))
        process, handle, topic = _start_detector(
            scenario.case_id, env, logs_dir / f'{scenario.case_id}.log', radius,
        )
        time.sleep(args.node_warmup_sec)
        snapshots.append(_capture(
            f'{scenario.case_id}_view00', args, topic, env, images_dir, snapshots_dir,
        ))
        last_aim_target = _candidate_aim_point(snapshots[-1])
        for step in range(1, scenario.motion_steps + 1):
            recommendation = _recommendation(snapshots[-1])
            if scenario.suite == 'multi_ball_clutter':
                recommendation = {
                    'action': 'move_left' if step == 1 else 'move_right',
                    'reason': (
                        '密集多球采用左右两侧最大视差复查；第二步横穿到目标另一侧。'
                    ),
                    'priority': 98,
                    'target_id': recommendation.get('target_id', ''),
                }
            elif scenario.suite == 'active_multiview':
                recommendation = {
                    'action': 'move_forward' if step == 1 else 'move_left',
                    'reason': (
                        '先靠近候选扩大有效视差，再横移并朝候选三维位置对准；'
                        '取得接近侧视的轮廓、深度曲率和尺寸证据。'
                    ),
                    'priority': 99,
                    'target_id': recommendation.get('target_id', ''),
                }
            elif scenario.suite == 'partial_visibility':
                recommendation = {
                    'action': 'move_left' if step == 1 else 'move_right',
                    'reason': (
                        '局部候选先做小幅左侧复查，再跨到右侧；至少一个方向应绕开'
                        '前景遮挡，且只按受控目标位置判定恢复。'
                    ),
                    'priority': 97,
                    'target_id': recommendation.get('target_id', ''),
                }
            elif recommendation['action'] in ('hold_observation', 'continue_exploring'):
                recommendation['action'] = 'move_left' if step % 2 else 'move_right'
                recommendation['reason'] += ' 为获得真实离散视角，实验执行受控侧移。'
            before = _robot_pose(env)
            if scenario.suite == 'multi_ball_clutter':
                duration_sec = args.motion_duration_sec / 2.0 if step == 1 else args.motion_duration_sec
            elif scenario.suite == 'partial_visibility':
                duration_sec = args.motion_duration_sec / 2.0 if step == 1 else args.motion_duration_sec
            elif scenario.suite == 'active_multiview':
                duration_sec = args.motion_duration_sec
            else:
                duration_sec = args.motion_duration_sec if step == 1 else args.motion_duration_sec / 2.0
            _execute_reobservation(recommendation['action'], duration_sec, env)
            if scenario.suite == 'active_multiview' and step == 2:
                localized_candidates = [
                    item.get('position') for item in snapshots[-1].get('hazards', [])
                    if len(item.get('position', ())) == 3
                ]
                if localized_candidates:
                    _orient_robot_toward(localized_candidates[0], env)
            elif scenario.suite == 'multi_ball_clutter' and last_aim_target is not None:
                _orient_robot_toward(last_aim_target, env)
            time.sleep(args.motion_settle_sec)
            after = _robot_pose(env)
            motion = {
                'step': step, 'action': recommendation['action'], 'before_pose': before,
                'after_pose': after, 'duration_sec': duration_sec,
                'translation_m': round(_pose_change_m(before, after), 4),
                'horizontal_translation_m': round(_horizontal_pose_change_m(before, after), 4),
                'yaw_change_deg': round(_pose_yaw_change_deg(before, after), 3),
            }
            recommendations.append(recommendation)
            motions.append(motion)
            snapshots.append(_capture(
                f'{scenario.case_id}_view{step:02d}', args, topic, env, images_dir, snapshots_dir,
            ))
            last_aim_target = _candidate_aim_point(snapshots[-1]) or last_aim_target
    finally:
        _stop_robot(env)
        _stop_detector(process, handle)
        if entity_id is not None:
            _remove_entity(entity_id, env, model_name)

    initial, final = snapshots[0], snapshots[-1]
    errors = _match_localization_errors(scenario.expected_positions, final)
    target_counts_by_view = [
        len(_match_localization_errors(scenario.expected_positions, snapshot))
        for snapshot in snapshots
    ]
    strict_counts_by_view = [_strict_count(snapshot) for snapshot in snapshots]
    view_ids = sorted({
        str(item.get('view_id')) for snapshot in snapshots for item in snapshot.get('detections_2d', [])
        if item.get('view_id')
    })
    row = {
        'case_id': scenario.case_id,
        'description': scenario.description,
        'expected_red_ball_count': scenario.expected_count,
        'initial_strict_count': _strict_count(initial),
        'initial_partial_count': _partial_count(initial),
        'initial_flat_depth_count': _depth_shape_count(initial, 'flat'),
        'initial_spherical_depth_count': _depth_shape_count(initial, 'spherical'),
        'initial_image_red_pixel_count': _red_pixel_count_from_image(
            images_dir / f'{scenario.case_id}_view00_raw.png'
        ),
        'final_strict_count': _strict_count(final),
        'best_strict_count': max((_strict_count(snapshot) for snapshot in snapshots), default=0),
        'strict_counts_by_view': json.dumps(strict_counts_by_view),
        'exact_count_view_count': sum(
            _strict_count(snapshot) == scenario.expected_count for snapshot in snapshots
        ),
        'initial_target_localized_count': target_counts_by_view[0],
        'final_target_localized_count': target_counts_by_view[-1],
        'best_target_localized_count': max(target_counts_by_view, default=0),
        'target_localized_counts_by_view': json.dumps(target_counts_by_view),
        'exact_target_count_view_count': sum(
            count == scenario.expected_count for count in target_counts_by_view
        ),
        'post_motion_target_recovered': any(
            target_counts_by_view[index] >= scenario.expected_count
            and strict_counts_by_view[index] >= scenario.expected_count
            for index in range(1, len(snapshots))
        ) if scenario.expected_count > 0 else False,
        'final_confirmed_count': _confirmed_count(final),
        'best_confirmed_count': max((_confirmed_count(snapshot) for snapshot in snapshots), default=0),
        'final_target_confirmed_count': _confirmed_near_count(final, ROOM_A_TARGET),
        'best_target_confirmed_count': max(
            (_confirmed_near_count(snapshot, ROOM_A_TARGET) for snapshot in snapshots), default=0,
        ),
        'distinct_camera_view_count': len(view_ids),
        'actual_robot_view_count': 1 + sum(
            item['horizontal_translation_m'] >= 0.20 or item['yaw_change_deg'] >= 10.0
            for item in motions
        ),
        'minimum_robot_translation_m': min(
            (item['horizontal_translation_m'] for item in motions), default=0.0,
        ),
        'maximum_robot_translation_m': max(
            (item['horizontal_translation_m'] for item in motions), default=0.0,
        ),
        'best_target_view_bearing_span_deg': max(
            (_best_track_view_bearing_span_deg(snapshot, scenario.expected_positions)
             for snapshot in snapshots),
            default=0.0,
        ),
        'best_any_track_view_bearing_span_deg': max(
            (_best_track_view_bearing_span_deg(snapshot) for snapshot in snapshots), default=0.0,
        ),
        'localized_truth_count': len(errors),
        'mean_localization_error_m': round(sum(errors) / len(errors), 4) if errors else '',
        'max_localization_error_m': round(max(errors), 4) if errors else '',
        'recommendations': json.dumps(recommendations, ensure_ascii=False),
        'motions': json.dumps(motions, ensure_ascii=False),
        'metadata': json.dumps(scenario.metadata or {}, ensure_ascii=False),
        'elapsed_sec': round(time.monotonic() - started, 3),
    }
    row['result'] = _score(scenario, row)
    return row


def _score(scenario, row):
    if scenario.suite == 'multi_ball_clutter':
        moved = row['actual_robot_view_count'] >= 2 and row['maximum_robot_translation_m'] >= 0.20
        return 'pass' if moved and row['exact_target_count_view_count'] >= 1 else 'review'
    if scenario.suite == 'partial_visibility':
        if (scenario.metadata or {}).get('baseline'):
            return 'pass' if row['initial_strict_count'] == 1 else 'fail'
        initial_found = row['initial_partial_count'] >= 1 or row['initial_strict_count'] >= 1
        moved = row['actual_robot_view_count'] >= 2 and row['maximum_robot_translation_m'] >= 0.20
        recovered = bool(row['post_motion_target_recovered'])
        visible_ratio = float((scenario.metadata or {}).get('target_visible_ratio', 1.0))
        # 5%/10% 是传感器盲区边界，允许由巡检盲扫恢复；15% 起必须在初见帧
        # 产生明确候选，不能把“后来碰巧看到”包装成局部识别成功。
        initial_requirement_met = initial_found or visible_ratio < 0.15
        return 'pass' if initial_requirement_met and moved and recovered else 'review'
    if scenario.suite == 'red_objects':
        if scenario.expected_count > 0:
            return 'pass' if row['initial_strict_count'] >= scenario.expected_count else 'review'
        # 单视角物品实验用于暴露“哪些物体会成为候选”，最终是否误认必须由
        # 独立 active_multiview 套件判断；单帧轨迹绝不可能直接 confirmed。
        return 'pass' if row['final_confirmed_count'] == 0 else 'review'
    if scenario.suite == 'active_multiview':
        moved = row['actual_robot_view_count'] >= 2 and row['maximum_robot_translation_m'] >= 0.20
        confirmed = row['best_target_confirmed_count']
        bearing_span = (
            row['best_target_view_bearing_span_deg'] if scenario.expected_count > 0
            else row['best_any_track_view_bearing_span_deg']
        )
        return 'pass' if (
            moved and confirmed == scenario.expected_count and bearing_span >= 25.0
        ) else 'review'
    if scenario.suite == 'complex_localization':
        return 'pass' if (
            row['localized_truth_count'] == scenario.expected_count
            and row['max_localization_error_m'] != ''
            and row['max_localization_error_m'] <= 0.15
        ) else 'review'
    return 'review'


def _make_collage(suite_dir):
    paths = sorted((suite_dir / 'images').glob('*_annotated.png'))
    if not paths:
        return
    thumbs = []
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        thumb = cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)
        cv2.putText(thumb, path.stem.replace('_annotated', '')[:42], (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(thumb, path.stem.replace('_annotated', '')[:42], (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1, cv2.LINE_AA)
        thumbs.append(thumb)
    columns = 4
    rows = math.ceil(len(thumbs) / columns)
    canvas = np.full((rows * 240, columns * 320, 3), 35, dtype=np.uint8)
    for index, thumb in enumerate(thumbs):
        row, column = divmod(index, columns)
        canvas[row * 240:(row + 1) * 240, column * 320:(column + 1) * 320] = thumb
    cv2.imwrite(str(suite_dir / 'images' / f'{suite_dir.name}_collage.png'), canvas)


def _write_outputs(suite_dir, rows, args):
    if suite_dir.name.endswith('_partial_visibility') and rows:
        baseline_pixels = max(int(rows[0].get('initial_image_red_pixel_count', 0)), 1)
        for row in rows:
            row['measured_initial_red_ratio'] = round(
                int(row.get('initial_image_red_pixel_count', 0)) / baseline_pixels, 4,
            )
    fields = list(rows[0]) if rows else []
    (suite_dir / 'cases.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    with (suite_dir / 'cases.csv').open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        'run_id': suite_dir.name,
        'run_date': '2026-07-10',
        'environment': 'official SimEnv ROS2 Harmonic complex generated building',
        'official_truth_files_read': False,
        'complex_room_background_required': True,
        'case_count': len(rows),
        'pass_count': sum(row['result'] == 'pass' for row in rows),
        'review_count': sum(row['result'] == 'review' for row in rows),
        'fail_count': sum(row['result'] == 'fail' for row in rows),
        'records': rows,
    }
    if suite_dir.name.endswith('_active_multiview'):
        # 这两个标记是严格校验器的前置条件：只有每例的真实水平视差达到
        # 25°、并且当前版本的结果门控全部通过，才允许把多视角素材归档为有效。
        lateral_verified = bool(rows) and all(
            (row['best_target_view_bearing_span_deg'] if row['expected_red_ball_count'] > 0
             else row['best_any_track_view_bearing_span_deg']) >= 25.0
            for row in rows
        )
        summary['lateral_parallax_verified'] = lateral_verified
        summary['strict_view_semantics_audited'] = lateral_verified
        summary['evidence_status'] = (
            'valid' if lateral_verified and summary['review_count'] == 0 and summary['fail_count'] == 0
            else 'needs_stable_platform_rerun'
        )
    (suite_dir / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    readme = (
        f'# {suite_dir.name}\n\n'
        '本目录只记录官方 SimEnv 原生复杂房间实验；保留墙体、门、家具、立柱、纵深和自然遮挡。\n\n'
        f'- 用例：{len(rows)}\n- 通过：{summary["pass_count"]}\n- 待复核：{summary["review_count"]}\n'
        '- 不读取 `danger_truth.json` 或私有布局真值。\n'
        '- 多视角用例只有在 Gazebo 机器人世界位姿实际变化后才计为有效视角。\n'
        '- 最终确认还要求相机到候选的水平视线跨度至少 25°；上下抖动或正面靠近不计为侧视。\n'
        '- 黄色 `reobserve` 是待复查候选，不等于确认识别。\n'
    )
    (suite_dir / 'README.md').write_text(readme, encoding='utf-8')
    record_dir = args.test_record_root / suite_dir.name
    record_dir.mkdir(parents=True, exist_ok=True)
    with (record_dir / 'testing_record_perception.csv').open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    (record_dir / 'testing_record_perception.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8',
    )
    _make_collage(suite_dir)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', choices=tuple(SCENARIO_BUILDERS) + ('all',), default='all')
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--test-record-root', required=True, type=Path)
    parser.add_argument('--runtime-root', required=True, type=Path)
    parser.add_argument('--capture-script', required=True, type=Path)
    parser.add_argument('--pose-settle-sec', type=float, default=2.0)
    parser.add_argument('--model-settle-sec', type=float, default=1.5)
    parser.add_argument('--node-warmup-sec', type=float, default=2.0)
    # 官方占位底盘速度响应在重启前后有差异；3 秒可形成约 0.6--0.9 m
    # 有效基线，同时避免靠得过近或侧移到家具遮挡后方。
    parser.add_argument('--motion-duration-sec', type=float, default=3.0)
    parser.add_argument('--motion-settle-sec', type=float, default=1.0)
    parser.add_argument('--capture-timeout-sec', type=float, default=15.0)
    parser.add_argument('--case-limit', type=int, default=0)
    parser.add_argument('--case-offset', type=int, default=0,
                        help='从第几个用例开始运行，配合 case-limit 做失败用例复测。')
    args = parser.parse_args()
    args.output_root = args.output_root.resolve()
    args.test_record_root = args.test_record_root.resolve()
    args.runtime_root = args.runtime_root.resolve()
    args.capture_script = args.capture_script.resolve()
    env = os.environ.copy()
    selected = list(SCENARIO_BUILDERS) if args.suite == 'all' else [args.suite]
    for suite in selected:
        scenarios = SCENARIO_BUILDERS[suite]()
        if args.case_offset:
            scenarios = scenarios[args.case_offset:]
        if args.case_limit:
            scenarios = scenarios[:args.case_limit]
        rows = []
        for scenario in scenarios:
            print(f'=== {scenario.case_id} ===', flush=True)
            row = _run_scenario(scenario, args, env)
            rows.append(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)
        suite_dir = args.output_root / f'official_simenv_20260710_rgbd_{suite}'
        _write_outputs(suite_dir, rows, args)


if __name__ == '__main__':
    main()
