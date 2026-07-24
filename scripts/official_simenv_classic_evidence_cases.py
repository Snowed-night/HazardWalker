#!/usr/bin/env python3
"""生成官方 SimEnv（ROS1 + Gazebo Classic）五类受控感知案例的 SDF。

所属组：感知定位组。
本文件只定义可审计的受控物体和测试真值，不能连接 ROS、不能读取
``danger_truth.json``，更不能向机器人发送速度。运行期的检测、复查与定位仅能使用
RGB、深度、内参、TF 和里程计；测试结束后才可用这里的受控配置计算指标。

与旧的 Gazebo Harmonic 脚本不同，输出固定为 Gazebo Classic 可加载的 SDF 1.6，
只使用官方镜像已验证的 sphere、box、cylinder、cone 和它们的组合。正式执行器必须
显式要求隔离容器，禁止向共享 ``simenv_run`` 写入模型或控制命令。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Dict, Iterable, Sequence, Tuple


Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class ClassicCase:
    """单个受控案例；SDF 仅含临时测试物体，复杂楼宇本体由官方世界提供。"""

    case_id: str
    suite: str
    description: str
    expected_sphere_positions: Tuple[Point3, ...]
    sdf: str
    metadata: Dict[str, object]


def _material(red: float = 0.95, green: float = 0.0, blue: float = 0.0) -> str:
    color = f'{red:.3f} {green:.3f} {blue:.3f} 1'
    return f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>'


def _link(name: str, pose: Point3, geometry: str, *, rpy=(0.0, 0.0, 0.0), material=None) -> str:
    x, y, z = pose
    roll, pitch, yaw = rpy
    rendered_material = _material() if material is None else material
    return (
        f'<link name="{name}"><pose>{x:.4f} {y:.4f} {z:.4f} '
        f'{roll:.8f} {pitch:.8f} {yaw:.8f}</pose>'
        # 临时物体仅作相机视觉证据，不建立 collision，避免干扰官方 A1 控制器动力学。
        f'<visual name="visual"><geometry>{geometry}</geometry>{rendered_material}</visual></link>'
    )


def _sphere(name: str, point: Point3, radius: float, *, material=None) -> str:
    return _link(
        name,
        point,
        f'<sphere><radius>{radius:.4f}</radius></sphere>',
        material=material,
    )


def _box(name: str, point: Point3, size: Sequence[float], *, rpy=(0.0, 0.0, 0.0), material=None) -> str:
    sx, sy, sz = (float(value) for value in size)
    return _link(name, point, f'<box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>', rpy=rpy, material=material)


def _cylinder(name: str, point: Point3, radius: float, length: float, *, rpy=(0.0, 0.0, 0.0)) -> str:
    return _link(
        name, point, f'<cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder>', rpy=rpy,
    )


def _cone(name: str, point: Point3, radius: float, length: float, *, rpy=(0.0, 0.0, 0.0)) -> str:
    return _link(name, point, f'<cone><radius>{radius:.4f}</radius><length>{length:.4f}</length></cone>', rpy=rpy)


def _sdf(model_name: str, links: Iterable[str]) -> str:
    return (
        '<?xml version="1.0"?>\n<sdf version="1.6">\n'
        f'  <model name="{model_name}"><static>true</static>\n'
        f'    {"".join(links)}\n'
        '  </model>\n</sdf>\n'
    )


def _at(center: Point3, dx=0.0, dy=0.0, dz=0.0) -> Point3:
    return (center[0] + dx, center[1] + dy, center[2] + dz)


def _right_circle_segment_fraction(cut_ratio: float) -> float:
    """返回单位圆在 ``x >= cut_ratio`` 一侧的面积比例。"""

    value = max(-1.0, min(1.0, float(cut_ratio)))
    return (math.acos(value) - value * math.sqrt(max(0.0, 1.0 - value * value))) / math.pi


def _cut_for_right_circle_segment_fraction(visible_ratio: float) -> float:
    """用二分反解圆盘右侧可见面积，供遮挡板精确覆盖不同球面比例。"""

    target = max(0.001, min(0.999, float(visible_ratio)))
    low, high = -1.0, 1.0
    for _ in range(60):
        middle = (low + high) / 2.0
        # cut 越向右，保留的右侧圆弧越少。
        if _right_circle_segment_fraction(middle) > target:
            low = middle
        else:
            high = middle
    return (low + high) / 2.0


def _official_a1_partial_precalibration(side: str, target_visible_ratio: float) -> float:
    """补偿官方 A1 起点相机的前景板透视，返回应写入 SDF 的面积切分目标。

    遮挡板位于球体前方约 0.28m，不能把球面面积分的理论切线直接放到球所在深度：
    透视会让左右两侧的实际红像素比例显著偏移。下表来自一次独立的真实 RGB 预标定，
    后续正式批次仍会在 ``cases.csv`` 中再次量测，超过 0.15 的偏差一律失败。
    """

    # 运行器会先把整个夹具的 +Y 观察轴旋到真实相机前向，因此左右两侧在
    # 相机坐标系中必须互为镜像。旧表是在夹具没有随相机旋转时分别测得，
    # 其中右侧数值实际混入了斜视投影；继续使用会把 15% 右侧案例完全遮住。
    # 这里只补偿遮挡板比球更靠近相机产生的共同透视放大，左右共用一组值。
    calibrated = {
        0.05: 0.314, 0.10: 0.369, 0.15: 0.426, 0.25: 0.515, 0.35: 0.600,
        0.45: 0.683, 0.55: 0.759, 0.65: 0.833, 0.75: 0.910, 0.85: 0.980,
    }
    if side not in {'left', 'right'}:
        raise ValueError(f'不支持的遮挡方向：{side}')
    key = min(calibrated, key=lambda value: abs(value - float(target_visible_ratio)))
    return float(calibrated[key])


def _object_links(shape: str, center: Point3) -> Tuple[str, ...]:
    """24 种真实可加载的红色物品；仅前三种球体是真值目标。"""

    red = None
    neutral = _material(0.30, 0.30, 0.30)
    definitions = {
        'sphere_standard': (_sphere('sphere', center, 0.15),),
        'sphere_small': (_sphere('sphere', center, 0.10),),
        'sphere_large': (_sphere('sphere', center, 0.22),),
        'cube': (_box('cube', center, (0.30, 0.30, 0.30)),),
        'tall_cuboid': (_box('cuboid', _at(center, dz=0.08), (0.18, 0.18, 0.46)),),
        'flat_panel': (_box('panel', center, (0.48, 0.04, 0.26)),),
        'cylinder_vertical': (_cylinder('cylinder', center, 0.15, 0.42),),
        'cylinder_face': (_cylinder('cylinder', center, 0.15, 0.32, rpy=(math.pi / 2.0, 0.0, 0.0)),),
        'disc_face': (_cylinder('disc', center, 0.18, 0.035, rpy=(math.pi / 2.0, 0.0, 0.0)),),
        'cone_vertical': (_cone('cone', center, 0.18, 0.42),),
        'cone_face': (_cone('cone', center, 0.18, 0.38, rpy=(math.pi / 2.0, 0.0, 0.0)),),
        # 椭球、胶囊在 Classic 基础几何中不可移植，改为可解释的复合曲面/杆件干扰物。
        'oval_three_spheres': (
            _sphere('oval_left', _at(center, dx=-0.13), 0.11),
            _sphere('oval_mid', center, 0.13),
            _sphere('oval_right', _at(center, dx=0.13), 0.11),
        ),
        'flat_three_spheres': (
            _sphere('flat_left', _at(center, dx=-0.14), 0.10),
            _sphere('flat_right', _at(center, dx=0.14), 0.10),
            _box('flat_bar', center, (0.26, 0.07, 0.08)),
        ),
        'tilted_barrel': (_cylinder('barrel', center, 0.12, 0.42, rpy=(0.0, math.pi / 2.0, 0.30)),),
        'rod_with_caps': (
            _cylinder('rod', center, 0.075, 0.36, rpy=(math.pi / 2.0, 0.0, 0.0)),
            _sphere('cap_left', _at(center, dx=-0.16), 0.09),
            _sphere('cap_right', _at(center, dx=0.16), 0.09),
        ),
        'l_shape': (
            _box('l_vertical', _at(center, dx=-0.08), (0.12, 0.16, 0.40)),
            _box('l_base', _at(center, dx=0.08, dz=-0.14), (0.32, 0.16, 0.12)),
        ),
        't_shape': (
            _box('t_stem', _at(center, dz=-0.03), (0.11, 0.15, 0.36)),
            _box('t_head', _at(center, dz=0.15), (0.38, 0.15, 0.11)),
        ),
        'cross_shape': (
            _box('cross_v', center, (0.10, 0.14, 0.42)),
            _box('cross_h', center, (0.42, 0.14, 0.10)),
        ),
        'dumbbell': (
            _sphere('left', _at(center, dx=-0.17), 0.12),
            _sphere('right', _at(center, dx=0.17), 0.12),
            _box('bar', center, (0.24, 0.08, 0.08)),
        ),
        'two_lobe': (_sphere('left', _at(center, dx=-0.09), 0.14), _sphere('right', _at(center, dx=0.09), 0.14)),
        'three_lobe': (
            _sphere('left', _at(center, dx=-0.11, dz=-0.04), 0.12),
            _sphere('right', _at(center, dx=0.11, dz=-0.04), 0.12),
            _sphere('top', _at(center, dz=0.12), 0.12),
        ),
        'ring_blocks': tuple(
            _box(f'ring_{index}', _at(center, dx=math.cos(index * math.pi / 4.0) * 0.16,
                                        dz=math.sin(index * math.pi / 4.0) * 0.16),
                 (0.09, 0.10, 0.09), rpy=(0.0, index * math.pi / 4.0, 0.0))
            for index in range(8)
        ),
        'stair_irregular': (
            _box('step_1', _at(center, dx=-0.14, dz=-0.10), (0.14, 0.16, 0.14)),
            _box('step_2', _at(center, dz=-0.03), (0.14, 0.16, 0.28)),
            _box('step_3', _at(center, dx=0.14, dz=0.04), (0.14, 0.16, 0.42)),
        ),
        'red_frame': (
            _box('frame_top', _at(center, dz=0.18), (0.42, 0.10, 0.08)),
            _box('frame_bottom', _at(center, dz=-0.18), (0.42, 0.10, 0.08)),
            _box('frame_left', _at(center, dx=-0.17), (0.08, 0.10, 0.36)),
            _box('frame_right', _at(center, dx=0.17), (0.08, 0.10, 0.36)),
        ),
        'red_box_with_gray_core': (
            _box('red_shell', center, (0.38, 0.24, 0.38)),
            _box('gray_core', _at(center, dy=-0.125), (0.18, 0.012, 0.18), material=neutral),
        ),
    }
    if shape not in definitions:
        raise KeyError(f'unknown shape: {shape}')
    return definitions[shape]


RED_OBJECTS = tuple(
    (
        'sphere_standard', 'sphere_small', 'sphere_large', 'cube', 'tall_cuboid', 'flat_panel',
        'cylinder_vertical', 'cylinder_face', 'disc_face', 'cone_vertical', 'cone_face',
        'oval_three_spheres', 'flat_three_spheres', 'tilted_barrel', 'rod_with_caps',
        'l_shape', 't_shape', 'cross_shape', 'dumbbell', 'two_lobe', 'three_lobe',
        'ring_blocks', 'stair_irregular', 'red_frame',
    )
)

MULTIVIEW_OBJECTS = (
    'sphere_standard', 'sphere_small', 'sphere_large', 'cylinder_face', 'disc_face',
    'cone_face', 'cube', 'flat_panel', 'cylinder_vertical', 'cone_vertical',
    'oval_three_spheres', 'flat_three_spheres', 'tilted_barrel', 'rod_with_caps',
    'l_shape', 'cross_shape', 'dumbbell', 'three_lobe', 'ring_blocks', 'red_frame',
)


def build_red_objects(center: Point3) -> Tuple[ClassicCase, ...]:
    """返回 24 例红色物品单视角压力测试，且只把球体标为任务目标。"""

    cases = []
    for index, shape in enumerate(RED_OBJECTS, start=1):
        expected = (center,) if shape.startswith('sphere_') else ()
        case_id = f'official_classic_object_{index:02d}_{shape}'
        cases.append(ClassicCase(
            case_id, 'red_objects', f'官方复杂楼宇红色物品：{shape}', expected,
            _sdf(case_id, _object_links(shape, center)),
            {'shape_name': shape, 'is_red_sphere_target': bool(expected)},
        ))
    return tuple(cases)


def build_active_multiview(center: Point3) -> Tuple[ClassicCase, ...]:
    """返回 20 例独立主动复查案例；球确认、所有其他红物体不得确认。"""

    cases = []
    for index, shape in enumerate(MULTIVIEW_OBJECTS, start=1):
        expected = (center,) if shape.startswith('sphere_') else ()
        case_id = f'official_classic_multiview_{index:02d}_{shape}'
        cases.append(ClassicCase(
            case_id, 'active_multiview', f'官方复杂楼宇真实横移复查：{shape}', expected,
            _sdf(case_id, _object_links(shape, center)),
            {'shape_name': shape, 'is_red_sphere_target': bool(expected), 'required_real_views': 3},
        ))
    return tuple(cases)


def build_multi_ball(center: Point3) -> Tuple[ClassicCase, ...]:
    """返回 10 例非规则分布、粘连和遮挡的多红球案例。"""

    layouts = (
        ((-0.13, 0.00, 0.00), (0.13, 0.00, 0.00)),
        ((-0.10, -0.18, 0.00), (0.10, 0.18, 0.02)),
        ((-0.18, 0.08, 0.00), (0.18, 0.12, 0.00), (0.00, -0.08, 0.18)),
        ((-0.45, 0.25, 0.00), (-0.15, -0.20, 0.05), (0.18, 0.22, 0.00), (0.48, -0.15, 0.08)),
        ((-0.68, 0.35, 0.00), (-0.35, -0.18, 0.04), (-0.02, 0.25, 0.00), (0.31, -0.20, 0.08), (0.65, 0.18, 0.00)),
        ((-0.72, -0.30, 0.00), (-0.45, 0.25, 0.00), (-0.18, 0.58, 0.05), (0.12, -0.22, 0.00), (0.42, 0.30, 0.08), (0.72, 0.62, 0.00)),
        ((-0.78, -0.28, 0.00), (-0.58, 0.35, 0.10), (-0.36, 0.05, 0.00), (-0.12, 0.62, 0.02), (0.10, -0.22, 0.12), (0.34, 0.20, 0.00), (0.58, 0.55, 0.08), (0.78, -0.05, 0.00)),
        ((-0.52, 0.10, 0.00), (-0.32, 0.10, 0.00), (0.28, 0.35, 0.04), (0.48, 0.35, 0.04)),
        ((-0.72, 0.35, 0.00), (-0.48, 0.02, 0.05), (-0.24, 0.52, 0.00), (0.00, -0.10, 0.08), (0.24, 0.38, 0.00), (0.48, 0.05, 0.06), (0.72, 0.58, 0.00)),
        ((-0.70, 0.05, 0.00), (-0.42, 0.48, 0.05), (-0.14, -0.18, 0.00), (0.16, 0.35, 0.10), (0.46, -0.08, 0.00), (0.72, 0.55, 0.02)),
    )
    cases = []
    for index, offsets in enumerate(layouts, start=1):
        positions = tuple(_at(center, *offset) for offset in offsets)
        links = [_sphere(f'ball_{item:02d}', point, 0.12) for item, point in enumerate(positions, start=1)]
        if index == 10:
            links.append(_box('red_box_distractor', _at(center, dx=-0.95, dy=0.28), (0.28, 0.20, 0.34)))
        case_id = f'official_classic_multi_{index:02d}'
        cases.append(ClassicCase(
            case_id, 'multi_ball_clutter', f'官方复杂楼宇非规则多球 {index:02d}', positions,
            _sdf(case_id, links), {'sphere_count': len(positions), 'requires_lateral_recheck': True},
        ))
    return tuple(cases)


def build_partial_visibility(center: Point3) -> Tuple[ClassicCase, ...]:
    """返回无遮挡基准和左右 5--85% 部分可见球，共 21 例。"""

    cases = [ClassicCase(
        'official_classic_partial_00_baseline', 'partial_visibility', '官方复杂楼宇无遮挡红球基准', (center,),
        _sdf('official_classic_partial_00_baseline', [_sphere('ball', center, 0.15)]),
        {'visible_ratio_design': 1.0},
    )]
    for side in ('left', 'right'):
        for percent in (5, 10, 15, 25, 35, 45, 55, 65, 75, 85):
            visible = percent / 100.0
            radius = 0.15
            plate_width = 0.62
            # 相机从 -y 观察时世界 x 近似对应画面水平轴。left 表示遮住球的图像左侧，
            # 因而保留右侧圆面积；right 则反向。切线位置由圆面积分反解，不能再用固定
            # 0.11m 偏移冒充 5%--85% 的遮挡梯度。
            placement_visible = _official_a1_partial_precalibration(side, visible)
            right_fraction = placement_visible if side == 'left' else 1.0 - placement_visible
            cut_ratio = _cut_for_right_circle_segment_fraction(right_fraction)
            cut_x = center[0] + cut_ratio * radius
            center_x = cut_x - plate_width / 2.0 if side == 'left' else cut_x + plate_width / 2.0
            links = [_sphere('ball', center, 0.15), _box(
                'foreground_occluder', (center_x, center[1] - 0.28, center[2]),
                (plate_width, 0.035, 0.52), material=_material(0.28, 0.28, 0.28),
            )]
            case_id = f'official_classic_partial_{side}_{percent:02d}pct'
            cases.append(ClassicCase(
                case_id, 'partial_visibility', f'官方复杂楼宇{side}侧约 {percent}% 可见红球', (center,),
                _sdf(case_id, links), {
                    'visible_ratio_design': visible,
                    'visible_ratio_precalibration_input': placement_visible,
                    'occlusion_side': side,
                    'occluder_center_x': round(center_x, 5),
                    'circle_cut_x': round(cut_x, 5),
                    'requires_reobserve': True,
                },
            ))
    return tuple(cases)


def build_active_partial_reobservation(center: Point3) -> Tuple[ClassicCase, ...]:
    """返回 B 阶段局部可见红球主动复查案例。

    复用已经按圆面积标定的 15%/25%/35% 左右遮挡夹具，但赋予独立 suite，
    使执行器必须从首帧黄色候选开始，根据实时策略移动并最终确认，而不能把
    7 月 10 日的静态遮挡截图当作 B 阶段闭环证据。
    """

    selected_ratios = {0.15, 0.25, 0.35}
    cases = []
    for source in build_partial_visibility(center):
        visible_ratio = source.metadata.get('visible_ratio_design')
        if visible_ratio not in selected_ratios:
            continue
        side = str(source.metadata.get('occlusion_side', 'unknown'))
        percent = int(round(float(visible_ratio) * 100.0))
        case_id = f'official_b_reobserve_{side}_{percent:02d}pct'
        metadata = dict(source.metadata)
        metadata.update({
            'delivery_stage': '20260730',
            'required_initial_state': 'partial_candidate',
            'required_final_state': 'single_confirmed_red_ball',
            'runtime_policy_must_not_read_fixture_metadata': True,
        })
        cases.append(ClassicCase(
            case_id,
            'active_partial_reobservation',
            f'B阶段真实移动主动复查：{side}侧约 {percent}% 可见红球',
            source.expected_sphere_positions,
            _sdf(case_id, _object_links_from_sdf(source.sdf)),
            metadata,
        ))
    return tuple(cases)


def _object_links_from_sdf(sdf: str) -> Tuple[str, ...]:
    """提取受控 SDF 中的 link 片段，供同几何不同 suite 复用。

    案例生成仅处理本文件自身产生的固定 SDF，不接受外部场景文件；若结构异常
    直接报错，避免静默生成空模型。
    """

    import re

    links = tuple(re.findall(r'(<link name="[^"]+">.*?</link>)', sdf))
    if not links:
        raise ValueError('受控部分可见案例 SDF 不含 link，不能生成 B 阶段夹具。')
    return links


def build_complex_localization(center: Point3) -> Tuple[ClassicCase, ...]:
    """返回 8 个非规则多球布局，定位真值只用于运行后误差统计。"""

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
    cases = []
    for index, offsets in enumerate(layouts, start=1):
        positions = tuple(_at(center, *offset) for offset in offsets)
        links = [_sphere(f'ball_{item:02d}', point, 0.12) for item, point in enumerate(positions, start=1)]
        case_id = f'official_classic_localization_{index:02d}'
        cases.append(ClassicCase(
            case_id, 'complex_localization', f'官方复杂楼宇多球三维定位 {index:02d}', positions,
            _sdf(case_id, links), {'truth_point_count': len(positions)},
        ))
    return tuple(cases)


def build_red_ball_3d_localization(center: Point3) -> Tuple[ClassicCase, ...]:
    """构造 A 阶段五个官方规格红球局部三维定位案例。

    每个正例严格使用半径 0.15 m 的红球；真值只在截图完成后转换到检测输出
    坐标系并计算误差，不得进入检测节点。
    """

    offsets = (
        (0.00, 0.00, 0.00),
        (-0.12, 0.00, 0.00),
        (0.12, 0.00, 0.00),
        (0.00, -0.20, 0.00),
        (0.00, 0.20, 0.00),
    )
    cases = []
    for index, offset in enumerate(offsets, start=1):
        position = _at(center, *offset)
        case_id = f'official_a_localization_{index:02d}'
        cases.append(ClassicCase(
            case_id,
            'red_ball_3d_localization',
            f'A阶段官方0.15 m红球局部三维定位 {index:02d}',
            (position,),
            _sdf(case_id, [_sphere('official_red_ball', position, 0.15)]),
            {
                'target_type': 'red_sphere',
                'target_radius_m': 0.15,
                'official_target_spec': True,
                'truth_usage': 'post_capture_evaluation_only',
            },
        ))
    return tuple(cases)


def build_official_distractor_rejection(center: Point3) -> Tuple[ClassicCase, ...]:
    """构造 A 阶段五个官方干扰源组合，目标仍仅为0.15 m红球。"""

    red_ball = _at(center, dx=-0.32)
    red_cube = _at(center, dx=0.32)
    green_ball = center
    green_material = _material(0.0, 0.85, 0.05)
    definitions = (
        (
            'red_cube_only',
            (),
            (_box('official_red_cube', center, (0.30, 0.30, 0.30)),),
        ),
        (
            'green_sphere_only',
            (),
            (_sphere('official_green_sphere', center, 0.15, material=green_material),),
        ),
        (
            'red_ball_with_red_cube',
            (red_ball,),
            (
                _sphere('official_red_ball', red_ball, 0.15),
                _box('official_red_cube', red_cube, (0.30, 0.30, 0.30)),
            ),
        ),
        (
            'red_ball_with_green_sphere',
            (red_ball,),
            (
                _sphere('official_red_ball', red_ball, 0.15),
                _sphere(
                    'official_green_sphere',
                    green_ball,
                    0.15,
                    material=green_material,
                ),
            ),
        ),
        (
            'red_ball_with_both_distractors',
            (red_ball,),
            (
                _sphere('official_red_ball', red_ball, 0.15),
                _box('official_red_cube', red_cube, (0.30, 0.30, 0.30)),
                _sphere(
                    'official_green_sphere',
                    green_ball,
                    0.15,
                    material=green_material,
                ),
            ),
        ),
    )
    cases = []
    for index, (scene, targets, links) in enumerate(definitions, start=1):
        case_id = f'official_a_distractor_{index:02d}_{scene}'
        cases.append(ClassicCase(
            case_id,
            'official_distractor_rejection',
            f'A阶段官方干扰源：{scene}',
            tuple(targets),
            _sdf(case_id, links),
            {
                'scene': scene,
                'official_target_count': len(targets),
                'target_radius_m': 0.15,
                'official_distractors': [
                    item for item in ('red_cube', 'green_sphere')
                    if item in scene or scene == 'red_ball_with_both_distractors'
                ],
            },
        ))
    return tuple(cases)


BUILDERS = {
    'multi_ball_clutter': build_multi_ball,
    'partial_visibility': build_partial_visibility,
    'red_objects': build_red_objects,
    'active_multiview': build_active_multiview,
    'active_partial_reobservation': build_active_partial_reobservation,
    'complex_localization': build_complex_localization,
    'red_ball_3d_localization': build_red_ball_3d_localization,
    'official_distractor_rejection': build_official_distractor_rejection,
}


def build_suite(suite: str, center: Point3) -> Tuple[ClassicCase, ...]:
    """构造单一类别；调用者应独立启动/停止检测器，禁止跨案例累积轨迹。"""

    try:
        return BUILDERS[suite](center)
    except KeyError as exc:
        raise ValueError(f'unknown suite: {suite}; choices: {", ".join(BUILDERS)}') from exc


def manifest(suite: str, center: Point3 = (0.0, 0.0, 0.15)) -> dict:
    """输出可提交到测试组的清单，不写文件，便于离线校验。"""

    cases = build_suite(suite, center)
    return {
        'suite': suite,
        'case_count': len(cases),
        'cases': [{
            'case_id': item.case_id,
            'description': item.description,
            'expected_sphere_positions': item.expected_sphere_positions,
            'metadata': item.metadata,
        } for item in cases],
    }


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suite', choices=tuple(BUILDERS))
    parser.add_argument('--center', nargs=3, type=float, default=(0.0, 0.0, 0.15))
    args = parser.parse_args()
    print(json.dumps(manifest(args.suite, tuple(args.center)), ensure_ascii=False, indent=2))
