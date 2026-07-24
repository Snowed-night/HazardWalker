"""官方 Gazebo Classic 历史与 A 阶段感知案例的离线校验。"""

import importlib.util
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'official_simenv_classic_evidence_cases.py'
SPEC = importlib.util.spec_from_file_location('official_simenv_classic_evidence_cases', SCRIPT_PATH)
CASES = importlib.util.module_from_spec(SPEC)
# dataclass 在解析 postponed annotation 时会回查 sys.modules；离线入口以文件方式加载测试，
# 因而必须先登记这个纯脚本模块。
sys.modules[SPEC.name] = CASES
SPEC.loader.exec_module(CASES)


def test_historical_suites_keep_required_case_counts():
    center = (1.0, 2.0, 0.15)
    expected_counts = {
        'multi_ball_clutter': 10,
        'partial_visibility': 21,
        'red_objects': 24,
        'active_multiview': 20,
        'complex_localization': 8,
        'red_ball_3d_localization': 5,
        'official_distractor_rejection': 5,
        'active_partial_reobservation': 6,
    }
    for suite, expected_count in expected_counts.items():
        cases = CASES.build_suite(suite, center)
        assert len(cases) == expected_count
        assert all('<sdf version="1.6">' in item.sdf for item in cases)
        assert all('danger_truth' not in item.sdf for item in cases)

    for item in CASES.build_red_objects(center):
        is_sphere = item.metadata['shape_name'].startswith('sphere_')
        assert bool(item.expected_sphere_positions) is is_sphere
        assert item.metadata['is_red_sphere_target'] is is_sphere


def test_stage_a_targets_are_only_official_radius_red_spheres():
    """A阶段不能再把0.10/0.12/0.22 m红球当作官方正例。"""

    center = (1.0, 2.0, 0.15)
    localization = CASES.build_suite('red_ball_3d_localization', center)
    distractors = CASES.build_suite('official_distractor_rejection', center)

    assert len(localization) == 5
    assert all(item.metadata['target_radius_m'] == 0.15 for item in localization)
    assert all('<radius>0.1500</radius>' in item.sdf for item in localization)
    assert all(
        position[2] >= 0.15
        for item in localization
        for position in item.expected_sphere_positions
    )
    assert sum(len(item.expected_sphere_positions) for item in distractors) == 3
    assert all(item.metadata['target_radius_m'] == 0.15 for item in distractors)
    assert '0.1500' in distractors[1].sdf
    assert any('official_red_cube' in item.sdf for item in distractors)
    assert any('official_green_sphere' in item.sdf for item in distractors)
    assert all(
        '<ambient>0.000 0.850 0.050 1</ambient>' in item.sdf
        for item in distractors
        if 'official_green_sphere' in item.sdf
    )


def test_multiview_manifest_requires_real_views_for_every_shape_case():
    cases = CASES.build_active_multiview((0.0, 0.0, 0.15))
    assert len({item.metadata['shape_name'] for item in cases}) == 20
    assert all(item.metadata['required_real_views'] == 3 for item in cases)
    assert all(
        bool(item.expected_sphere_positions) is item.metadata['shape_name'].startswith('sphere_')
        for item in cases
    )


def test_partial_visibility_cases_use_distinct_area_integral_occluder_positions():
    """5%--85% 必须是实际不同的遮挡几何，不能只改文件名。"""
    cases = CASES.build_partial_visibility((0.0, 0.0, 0.15))
    left = [item for item in cases if item.metadata.get('occlusion_side') == 'left']
    right = [item for item in cases if item.metadata.get('occlusion_side') == 'right']

    assert len(left) == 10
    assert len(right) == 10
    assert len({item.metadata['circle_cut_x'] for item in left}) == 10
    assert len({item.metadata['occluder_center_x'] for item in right}) == 10
    # 左遮挡的可见比例越高，遮挡板右缘越向左退，留下更多右侧圆面。
    assert [item.metadata['circle_cut_x'] for item in left] == sorted(
        (item.metadata['circle_cut_x'] for item in left), reverse=True,
    )


def test_stage_b_reobservation_cases_start_from_sub_45_percent_visibility():
    """B阶段案例必须从真正局部可见开始，不能复用无遮挡完整球。"""

    cases = CASES.build_suite(
        'active_partial_reobservation', (0.0, 0.0, 0.15),
    )

    assert len(cases) == 6
    assert {
        item.metadata['visible_ratio_design'] for item in cases
    } == {0.15, 0.25, 0.35}
    assert {
        item.metadata['occlusion_side'] for item in cases
    } == {'left', 'right'}
    assert all(item.metadata['delivery_stage'] == '20260730' for item in cases)
    assert all(item.metadata['required_initial_state'] == 'partial_candidate'
               for item in cases)


def test_partial_visibility_precalibration_is_camera_mirrored():
    """夹具正对相机后左右遮挡必须镜像，不能沿用斜视历史偏置。"""

    cases = CASES.build_suite('active_partial_reobservation', (0.0, 0.0, 0.15))
    by_ratio_and_side = {
        (item.metadata['visible_ratio_design'], item.metadata['occlusion_side']): item
        for item in cases
    }
    for ratio in (0.15, 0.25, 0.35):
        left = by_ratio_and_side[(ratio, 'left')].metadata
        right = by_ratio_and_side[(ratio, 'right')].metadata
        assert left['visible_ratio_precalibration_input'] == right[
            'visible_ratio_precalibration_input'
        ]
        assert abs(left['circle_cut_x'] + right['circle_cut_x']) < 1e-4
