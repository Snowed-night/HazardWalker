#!/usr/bin/env python3
"""用 2026-07-05 官方复杂场景原始帧回归当前红球检测器。

本脚本不重写或伪造 7 月 5 日素材，而是从每套实验随附的 raw_frames.zip
读取 PPM 原图，以当前 HSV/OpenCV 实现重新检测并产出可审计对比。它覆盖：

* 21 张左右递进遮挡帧：区分盲区、黄色待复查候选与严格候选；
* 多球/复杂房间帧：记录严格框、粘连拆分候选和原始框数量，防止把候选当计数；
* 红色异形物帧：明确圆柱端面只能是“需多视角复查”，不作为单帧球体结论。

输出应放在 2026-07-10 五类正式目录的 reference_20260705_regression 子目录，
因此不会额外创建第六种效果目录。该回归只能证明静态原始帧上的候选语义；
连续多视角 confirmed 和三维定位仍必须由稳定平台上的 RGB-D/TF 实验验收。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import sys
import zipfile
from pathlib import Path

import cv2


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PERCEPTION_SOURCE = PROJECT_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
if str(PERCEPTION_SOURCE) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SOURCE))

from hazardwalker_perception.red_ball_detector import detect_red_balls_rgb_bytes  # noqa: E402


REFERENCE_SUITES = {
    'occlusion': {
        'source_dir': 'official_simenv_20260705_partial_visibility',
        'target_dir': 'official_simenv_20260710_partial_visibility',
    },
    'shapes': {
        'source_dir': 'official_simenv_20260705_extended_red_object_stress',
        'target_dir': 'official_simenv_20260710_extended_red_object_stress',
    },
    'multi_ball': {
        'source_dir': 'official_simenv_20260705_red_ball_detection',
        'target_dir': 'official_simenv_20260710_multi_ball_clutter',
    },
}


def _read_cases(source_dir: Path) -> list[dict]:
    return json.loads((source_dir / 'cases.json').read_text(encoding='utf-8'))


def _extract_raw_frames(source_dir: Path, work_dir: Path) -> Path:
    """按需要解压仓库中已有原始帧，不把 PPM 复制到 Git 结果目录。"""

    archive = source_dir / 'raw_frames.zip'
    destination = work_dir / source_dir.name
    marker = destination / '.complete'
    if marker.exists():
        return destination
    if destination.exists():
        shutil.rmtree(destination)
    destination.mkdir(parents=True)
    with zipfile.ZipFile(archive) as handle:
        handle.extractall(destination)
    marker.write_text('extracted from repository reference archive\n', encoding='utf-8')
    return destination


def _find_raw_frame(extracted_root: Path, case: dict) -> Path:
    raw_value = str(case.get('raw_image', ''))
    basename = Path(raw_value).name
    candidates = list(extracted_root.rglob(basename))
    if len(candidates) != 1:
        raise RuntimeError(f'cannot uniquely find raw frame {raw_value}: {candidates}')
    return candidates[0]


def _detect(image):
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return detect_red_balls_rgb_bytes(
        rgb.tobytes(), image.shape[1], image.shape[0],
        include_partial_candidates=True,
        min_area_px=200,
        min_circularity=0.65,
        # 与线上 ROS 节点一致：近矩形端面不作为严格正证据，转为黄色复查候选。
        max_extent=0.82,
        # 与 ROS 节点默认候选策略保持一致：低可见帧只作为导航重观察线索。
        partial_min_area_px=20,
        partial_min_circularity=0.18,
        partial_min_aspect_ratio=0.12,
        partial_min_value=50,
    )


def _semantic_counts(detections) -> dict:
    strict = [item for item in detections if not item.requires_reobservation]
    reobserve = [item for item in detections if item.requires_reobservation]
    return {
        'raw_detection_count': len(detections),
        'strict_candidate_count': len(strict),
        'reobserve_candidate_count': len(reobserve),
        # 同一粘连连通域内拆出的候选与普通红色杂物分开统计；前者仅能作为
        # 多球计数/主动横移线索，不能当作已确认球体。
        'merged_split_candidate_count': sum(
            bool(item.from_merged_split) for item in detections
        ),
        'max_confidence': round(max((item.confidence for item in detections), default=0.0), 4),
    }


def _occlusion_expected(case: dict) -> str:
    ratio = float(case.get('target_visible_ratio', 0.0))
    if ratio <= 0.05:
        return 'reobserve_candidate_required'
    if ratio < 0.45:
        return 'reobserve_candidate_required'
    return 'strict_candidate_required'


def _classify_occlusion(case: dict, counts: dict) -> tuple[str, str]:
    expected = _occlusion_expected(case)
    if expected == 'reobserve_candidate_required':
        result = 'pass' if counts['reobserve_candidate_count'] >= 1 else 'fail'
    else:
        result = 'pass' if counts['strict_candidate_count'] >= 1 else 'fail'
    return expected, result


def _classify_shape(case: dict, counts: dict) -> tuple[str, str]:
    shape = str(case.get('shape', ''))
    if shape == 'red_sphere_control':
        expected = 'strict_candidate_required'
        return expected, 'pass' if counts['strict_candidate_count'] == 1 else 'fail'
    if shape == 'red_cylinder':
        # 单帧圆柱端面从几何上无法与球体严格区分；严禁把它写成单帧识别失败。
        expected = 'must_trigger_multiview_shape_recheck'
        # 端面如果被降级为黄色候选，正是预期行为；是否为球由后续侧视决定。
        return expected, 'review' if counts['raw_detection_count'] >= 1 else 'fail'
    if shape == 'mixed_red_shapes':
        expected = 'contains_sphere_and_shape_recheck'
        return expected, 'review'
    expected = 'no_strict_candidate_required'
    return expected, 'pass' if counts['strict_candidate_count'] == 0 else 'fail'


def _classify_multi_ball(case: dict, counts: dict) -> tuple[str, str]:
    case_id = str(case.get('case_id', ''))
    if 'spawned_10_red' in case_id:
        expected = '10_merged_sphere_candidates_required_before_multiview_confirmation'
        # 该原始帧的生成记录明确为 10 个红球；红方块、远处红色杂物不计入
        # 同连通域分离数。这里证明 2D 能正确提供 10 个待复查球候选，最终
        # 是否分别确认仍由稳定 RGB-D 多视角轨迹验收。
        result = 'pass' if counts['merged_split_candidate_count'] == 10 else 'fail'
        return expected, result
    elif 'oblique' in case_id:
        expected = 'oblique_multi_ball_count_needs_multiview_truth_matching'
    else:
        expected = 'complex_room_single_target_presence_only'
    # 旧帧包含官方环境既有红物、遮挡和未知真值，不能从 2D 原图诚实推出精确计数。
    return expected, 'recorded_not_scored'


def _annotate(image, detections, title: str):
    canvas = image.copy()
    for index, item in enumerate(detections, start=1):
        color = (0, 215, 255) if item.requires_reobservation else (0, 0, 255)
        cv2.rectangle(canvas, (item.x_min, item.y_min), (item.x_max, item.y_max), color, 2)
        # 密集球群中逐框叠加长文字会遮住目标本身；框内只留短编号，完整语义
        # 交给顶部汇总、CSV 与 JSON，答辩图仍能一眼看清分离是否合理。
        cv2.putText(canvas, f'#{index}', (item.x_min + 2, min(item.y_max - 4, item.y_min + 16)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
    cv2.putText(canvas, title, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, title, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                (20, 20, 20), 1, cv2.LINE_AA)
    strict_count = sum(not item.requires_reobservation for item in detections)
    reobserve_count = len(detections) - strict_count
    legend = f'strict={strict_count} (red)  reobserve={reobserve_count} (yellow)'
    cv2.putText(canvas, legend, (12, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.46,
                (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, legend, (12, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.46,
                (20, 20, 20), 1, cv2.LINE_AA)
    return canvas


def _write_collage(images: list[Path], output_path: Path, max_items=12):
    selected = images[:max_items]
    if not selected:
        return
    tiles = []
    for image_path in selected:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        tiles.append(cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA))
    if not tiles:
        return
    rows = []
    for index in range(0, len(tiles), 3):
        row = tiles[index:index + 3]
        while len(row) < 3:
            row.append(cv2.cvtColor(cv2.UMat(240, 320, cv2.CV_8UC1).get(), cv2.COLOR_GRAY2BGR))
        rows.append(cv2.hconcat(row))
    cv2.imwrite(str(output_path), cv2.vconcat(rows))


def _evaluate_suite(kind: str, source_root: Path, output_root: Path, work_dir: Path) -> dict:
    spec = REFERENCE_SUITES[kind]
    source_dir = source_root / spec['source_dir']
    target_root = output_root / spec['target_dir'] / 'reference_20260705_regression'
    images_dir = target_root / 'images'
    target_root.mkdir(parents=True, exist_ok=True)
    images_dir.mkdir(exist_ok=True)
    extracted = _extract_raw_frames(source_dir, work_dir)
    records = []
    annotated_paths = []
    for case in _read_cases(source_dir):
        raw = _find_raw_frame(extracted, case)
        image = cv2.imread(str(raw), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f'cannot decode {raw}')
        detections = _detect(image)
        counts = _semantic_counts(detections)
        if kind == 'occlusion':
            expected, result = _classify_occlusion(case, counts)
        elif kind == 'shapes':
            expected, result = _classify_shape(case, counts)
        else:
            expected, result = _classify_multi_ball(case, counts)
        case_id = str(case['case_id'])
        annotation = images_dir / f'{case_id}_current_detector.png'
        cv2.imwrite(str(annotation), _annotate(image, detections, f'current detector | {result}'))
        annotated_paths.append(annotation)
        records.append({
            'case_id': case_id,
            'reference_suite': spec['source_dir'],
            'expected_semantics': expected,
            'result': result,
            **counts,
            'annotation': str(annotation.relative_to(target_root)).replace('\\', '/'),
        })
    fields = list(records[0]) if records else []
    with (target_root / 'cases.csv').open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)
    (target_root / 'cases.json').write_text(
        json.dumps(records, ensure_ascii=False, indent=2) + '\n', encoding='utf-8',
    )
    summary = {
        'reference_suite': spec['source_dir'],
        'case_count': len(records),
        'pass_count': sum(item['result'] == 'pass' for item in records),
        'review_count': sum(item['result'] == 'review' for item in records),
        'fail_count': sum(item['result'] == 'fail' for item in records),
        'scoring_boundary': (
            '2D source-frame regression only; it does not certify 3D confirmed tracks, '
            'multi-view shape rejection, or localization accuracy.'
        ),
    }
    (target_root / 'summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8',
    )
    _write_collage(annotated_paths, target_root / 'current_detector_collage.png')
    (target_root / 'README.md').write_text(
        '# 2026-07-05 复杂场景原始帧回归\n\n'
        '本目录由当前检测器重新处理仓库中已有的 2026-07-05 `raw_frames.zip`。'
        '红框表示严格二维候选，黄框表示仅供导航换视角的 `reobserve` 候选。\n\n'
        f'- 参考套件：`{spec["source_dir"]}`\n'
        f'- 案例数：{summary["case_count"]}\n'
        f'- 通过/待复核/失败：{summary["pass_count"]}/{summary["review_count"]}/{summary["fail_count"]}\n'
        '- 圆柱端面和多球密集帧不能由单帧二维图给出最终球体/精确计数结论；'
        '它们保留为多视角和三维真值重跑的回归输入。\n',
        encoding='utf-8',
    )
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--reference-root', type=Path,
        default=PROJECT_ROOT / 'reports' / 'perception' / 'simulation' / '3d_native',
    )
    parser.add_argument(
        '--output-root', type=Path,
        default=PROJECT_ROOT / 'reports' / 'perception' / 'simulation' / '3d_native',
    )
    parser.add_argument('--work-dir', type=Path, default=PROJECT_ROOT / 'work' / 'reference_20260705')
    parser.add_argument('--suite', choices=tuple(REFERENCE_SUITES) + ('all',), default='all')
    args = parser.parse_args()
    selected = REFERENCE_SUITES if args.suite == 'all' else {args.suite: REFERENCE_SUITES[args.suite]}
    summaries = {
        kind: _evaluate_suite(kind, args.reference_root, args.output_root, args.work_dir)
        for kind in selected
    }
    print(json.dumps(summaries, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
