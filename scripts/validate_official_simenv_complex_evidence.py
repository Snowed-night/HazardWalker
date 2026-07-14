"""校验官方复杂环境五套感知证据是否完整且达到归档门槛。"""

import argparse
import json
from pathlib import Path


SUITES = {
    'active_multiview': 20,
    'multi_ball_clutter': 10,
    'partial_visibility': 21,
    'red_objects': 24,
    'complex_localization': 8,
}


def _suite_dir(root: Path, suite: str) -> Path:
    return root / f'official_simenv_20260710_rgbd_{suite}'


def _validate_artifacts(root: Path, suite: str, expected_case_count: int) -> None:
    """同时校验展示素材和结构化记录，避免只凭 cases.json 宣称完成。"""
    suite_dir = _suite_dir(root, suite)
    required = (
        suite_dir / 'README.md',
        suite_dir / 'cases.csv',
        suite_dir / 'cases.json',
        suite_dir / 'summary.json',
        suite_dir / 'images' / f'{suite_dir.name}_collage.png',
    )
    missing = [str(path) for path in required if not path.is_file() or path.stat().st_size == 0]
    assert not missing, f'{suite}: 缺少或为空的规范素材 {missing}'

    annotated_count = len(list((suite_dir / 'images').glob('*_annotated.png')))
    snapshot_count = len(list((suite_dir / 'snapshots').glob('*_snapshot.json')))
    assert annotated_count >= expected_case_count, (
        f'{suite}: 标注图 {annotated_count} 少于用例数 {expected_case_count}'
    )
    assert snapshot_count >= expected_case_count, (
        f'{suite}: 快照 {snapshot_count} 少于用例数 {expected_case_count}'
    )


def _load_rows(root: Path, suite: str):
    _validate_artifacts(root, suite, SUITES[suite])
    path = _suite_dir(root, suite) / 'cases.json'
    if not path.exists():
        raise AssertionError(f'缺少 {path}')
    rows = json.loads(path.read_text(encoding='utf-8'))
    assert len(rows) == SUITES[suite], f'{suite}: 用例数 {len(rows)} != {SUITES[suite]}'
    failed = [(row['case_id'], row.get('result')) for row in rows if row.get('result') != 'pass']
    assert not failed, f'{suite}: 未通过用例 {failed}'
    return rows


def _validate_test_records(test_record_root: Path) -> None:
    """校验测试组 CSV/JSON 是否与五套正式证据一一对应。"""
    for suite, expected_case_count in SUITES.items():
        run_id = f'official_simenv_20260710_rgbd_{suite}'
        record_dir = test_record_root / run_id
        csv_path = record_dir / 'testing_record_perception.csv'
        json_path = record_dir / 'testing_record_perception.json'
        missing = [
            str(path)
            for path in (csv_path, json_path)
            if not path.is_file() or path.stat().st_size == 0
        ]
        assert not missing, f'{suite}: 缺少或为空的测试组表格 {missing}'
        payload = json.loads(json_path.read_text(encoding='utf-8'))
        assert payload.get('case_count') == expected_case_count, (
            f'{suite}: 测试组 JSON 用例数不一致'
        )
        assert len(payload.get('records', [])) == expected_case_count, (
            f'{suite}: 测试组 JSON 记录数不一致'
        )


def _validate_active_view_audit(root: Path) -> None:
    """确认多视角结果来自语义正确的独立水平视角，而非旧版抖动标签。"""

    summary_path = _suite_dir(root, 'active_multiview') / 'summary.json'
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    assert summary.get('strict_view_semantics_audited') is True, (
        '真实运动多视角尚未通过严格视角语义审计：必须证明只按水平基线/朝向计数，'
        '且不能由竖直抖动累计 distinct view。请在稳定平台上重跑该套实验。'
    )
    assert summary.get('lateral_parallax_verified') is True, (
        '真实运动多视角尚未证明目标相对相机的水平视线跨度达到 25°；'
        '正面前后移动不足以排除圆柱、圆锥或圆盘端面。'
    )
    assert summary.get('evidence_status') == 'valid', (
        f'真实运动多视角当前不是有效证据：{summary.get("evidence_status")!r}'
    )
def validate(root: Path, test_record_root: Path | None = None):
    expected_dirs = {_suite_dir(root, suite).name for suite in SUITES}
    actual_dirs = {
        path.name
        for path in root.glob('official_simenv_20260710_*')
        if path.is_dir()
    }
    assert actual_dirs == expected_dirs, (
        '2026-07-10 结果目录必须且只能保留五类有效证据；'
        f'缺少={sorted(expected_dirs - actual_dirs)}，多余={sorted(actual_dirs - expected_dirs)}'
    )

    rows = {suite: _load_rows(root, suite) for suite in SUITES}

    _validate_active_view_audit(root)

    active = rows['active_multiview']
    active_by_shape = {
        json.loads(row['metadata'])['shape_name']: row
        for row in active
    }
    assert all(
        row['best_target_confirmed_count'] == (1 if json.loads(row['metadata'])['is_sphere'] else 0)
        for row in active
    ), '真实运动多视角的球体确认或非球体拒绝不符合真值'
    assert all(row['actual_robot_view_count'] >= 3 for row in active), '存在伪多视角或视角数量不足'
    assert active_by_shape['cylinder_face']['initial_flat_depth_count'] >= 1, '圆柱正面未暴露平面深度证据'
    assert active_by_shape['cone_face']['initial_strict_count'] >= 1, '圆锥正面未覆盖单视角易混淆条件'
    assert active_by_shape['dumbbell']['initial_partial_count'] >= 2, '哑铃未覆盖同连通域多圆候选条件'

    multi = rows['multi_ball_clutter']
    assert all(row['exact_target_count_view_count'] >= 1 for row in multi), '多球场景没有目标位置精确计数视角'
    assert all(row['actual_robot_view_count'] >= 2 for row in multi), '多球场景缺少真实运动视角'

    partial = rows['partial_visibility']
    assert partial[0]['initial_strict_count'] == 1, '无遮挡基准未检出'
    assert all(
        row['initial_partial_count'] >= 1 or row['initial_strict_count'] >= 1
        for row in partial[1:]
        if float(json.loads(row['metadata']).get('target_visible_ratio', 1.0)) >= 0.15
    ), '15% 及以上局部球存在初见静默漏检'
    assert all(row['post_motion_target_recovered'] for row in partial[1:]), '存在扫描后未恢复目标球的用例'

    objects = rows['red_objects']
    objects_by_shape = {
        json.loads(row['metadata'])['shape_name']: row
        for row in objects
    }
    assert all(
        objects_by_shape[name]['initial_strict_count'] >= 1
        for name in ('sphere_standard', 'sphere_small', 'sphere_large')
    ), '三种球体物品基准未检出'
    assert all(
        objects_by_shape[name]['initial_strict_count'] == 0
        for name in ('cube', 'tall_cuboid', 'flat_panel', 'cylinder_vertical')
    ), '明显非球体仍进入严格候选'
    assert objects_by_shape['dumbbell']['initial_partial_count'] >= 2, '哑铃端部未标为多圆待复查候选'
    assert objects_by_shape['two_lobe']['initial_partial_count'] >= 2, '双叶物未标为多圆待复查候选'
    assert objects_by_shape['three_lobe']['initial_partial_count'] >= 3, '三叶物未标为多圆待复查候选'
    assert all(row['final_confirmed_count'] == 0 for row in objects[3:]), '单视角物品压力集出现越权确认'

    localization = rows['complex_localization']
    assert sum(row['localized_truth_count'] for row in localization) == 32, '三维定位真值点未达到 32 个'
    assert max(float(row['max_localization_error_m']) for row in localization) <= 0.15, '定位最大误差超过 15 cm'

    comparison_path = _suite_dir(root, 'active_multiview') / 'strategy_comparison.json'
    assert comparison_path.is_file(), '缺少主动多视角与单视角的策略对比结果'
    comparison = json.loads(comparison_path.read_text(encoding='utf-8'))
    assert comparison['active_multiview_confirmation']['fp'] == 0, '主动多视角仍有非球体误确认'
    assert comparison['active_multiview_confirmation']['fn'] == 0, '主动多视角仍有球体漏确认'

    if test_record_root is not None:
        _validate_test_records(test_record_root)

    return {
        'suite_count': len(SUITES),
        'case_count': sum(len(items) for items in rows.values()),
        'localization_truth_points': 32,
        'artifact_structure': 'complete',
        'testing_record_structure': 'complete' if test_record_root is not None else 'not_checked',
        'retained_20260710_directories': sorted(expected_dirs),
        'status': 'pass',
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path, help='包含五个 official_simenv_* 目录的根目录')
    parser.add_argument('--test-record-root', type=Path, help='可选：测试组 CSV/JSON 根目录')
    args = parser.parse_args()
    print(json.dumps(validate(args.root, args.test_record_root), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
