"""官方 Gazebo Classic 五类感知证据案例的离线校验。"""

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


def test_five_suites_have_required_case_counts_and_only_spheres_are_targets():
    center = (1.0, 2.0, 0.15)
    expected_counts = {
        'multi_ball_clutter': 10,
        'partial_visibility': 21,
        'red_objects': 24,
        'active_multiview': 20,
        'complex_localization': 8,
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
