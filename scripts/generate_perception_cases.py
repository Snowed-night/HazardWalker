"""生成感知组红球检测可视化实验案例。

所属组：感知组 / 测试组。
文件作用：
生成可控的红球、遮挡、多红球、光照背景干扰和红色非球体图像。
调用 `detect_red_balls_rgb_bytes` 得到一个或多个检测结果和形状指标。
输出原图、标注图、summary.csv、summary.json 和汇报用拼图。
同步输出 detection metrics 汇总，用于汇报 precision、recall、F1、top1 error 和 AP50。
额外输出 testing_record_perception.csv，字段对齐测试组感知专项指标。

当前实现边界：
只生成离线 RGB 合成案例，不依赖 ROS、Gazebo 或真实相机。
图片是 RGB 合成图，主要用于算法回归、汇报展示和参数解释。
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
PERCEPTION_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
sys.path.insert(0, str(PERCEPTION_SRC))

from hazardwalker_perception.red_ball_detector import detect_red_balls_rgb_bytes
from hazardwalker_perception.detection_metrics import (
    DetectionSample,
    bbox_iou,
    evaluate_detection_samples,
)


RED = (230, 20, 20)
BRIGHT_RED = (255, 35, 35)
DARK_RED = (70, 5, 5)
LOW_SATURATION_RED = (150, 105, 105)
BACKGROUND = (30, 30, 30)
WIDTH = 160
HEIGHT = 160


@dataclass
class CaseSpec:
    """单个生成案例的参数定义。"""

    case_id: str
    case_type: str
    shape: str
    color_type: str
    expected_detected: bool
    occlusion_ratio: float = 0.0
    note: str = ''
    objects: tuple = ()


"""当前合成球体的理论 bbox。遮挡案例使用可见球体区域作为真值 bbox。"""
def expected_sphere_bbox(occlusion_ratio=0.0):
    center_x = WIDTH // 2
    center_y = HEIGHT // 2
    radius = 42
    x_min = center_x - radius
    x_max = center_x + radius
    if occlusion_ratio > 0.0:
        occlusion_width = int((radius * 2 + 1) * occlusion_ratio)
        x_start = center_x + radius - occlusion_width + 1
        x_max = min(x_max, x_start - 1)
    return [x_min, center_y - radius, x_max, center_y + radius]


"""生成指定中心和半径的圆形真值 bbox。"""
def sphere_bbox(center_x, center_y, radius, occlusion_ratio=0.0):
    x_min = int(center_x - radius)
    x_max = int(center_x + radius)
    if occlusion_ratio > 0.0:
        occlusion_width = int((radius * 2 + 1) * occlusion_ratio)
        x_start = center_x + radius - occlusion_width + 1
        x_max = min(x_max, int(x_start - 1))
    return [x_min, int(center_y - radius), x_max, int(center_y + radius)]


"""生成渐变红色圆形，让二维图像更接近球体外观。"""
def draw_sphere(image, color=RED, occlusion_ratio=0.0, highlight=True, center=None, radius=42):
    if center is None:
        center = (WIDTH // 2, HEIGHT // 2)
    base = np.array(color, dtype=np.float32)
    for y in range(center[1] - radius, center[1] + radius + 1):
        for x in range(center[0] - radius, center[0] + radius + 1):
            dx = x - center[0]
            dy = y - center[1]
            distance = (dx * dx + dy * dy) ** 0.5
            if distance <= radius:
                light = 0.65 + 0.35 * max(0.0, 1.0 - distance / radius)
                highlight_value = 0.20 if highlight and dx < -radius * 0.25 and dy < -radius * 0.25 else 0.0
                pixel = np.clip(base * light + 255.0 * highlight_value, 0, 255)
                image[y, x] = pixel.astype(np.uint8)

    if occlusion_ratio > 0.0:
        occlusion_width = int((radius * 2 + 1) * occlusion_ratio)
        x_start = center[0] + radius - occlusion_width + 1
        cv2.rectangle(
            image,
            (x_start, center[1] - radius - 2),
            (center[0] + radius + 2, center[1] + radius + 2),
            BACKGROUND,
            thickness=-1,
        )


"""生成红色方块，用作红色立方体在图像中的投影对照。"""
def draw_cube_projection(image, color=RED):
    cv2.rectangle(image, (50, 50), (110, 110), color, thickness=-1)
    cv2.line(image, (50, 50), (110, 50), (255, 80, 80), thickness=3)
    cv2.line(image, (50, 50), (50, 110), (170, 10, 10), thickness=3)


"""生成红色不规则物体，kind 用于区分三角形、细长形和碎片形。"""
def draw_irregular(image, kind, color=RED):
    if kind == 'triangle':
        points = np.array([[38, 116], [82, 34], [123, 110]], dtype=np.int32)
        cv2.fillPoly(image, [points], color)
    elif kind == 'elongated':
        points = np.array([[25, 70], [132, 52], [140, 80], [35, 104]], dtype=np.int32)
        cv2.fillPoly(image, [points], color)
    elif kind == 'fragments':
        cv2.rectangle(image, (34, 46), (54, 52), color, thickness=-1)
        cv2.rectangle(image, (76, 86), (112, 92), color, thickness=-1)
        cv2.rectangle(image, (118, 44), (142, 50), color, thickness=-1)
        cv2.rectangle(image, (46, 112), (70, 118), color, thickness=-1)
    else:
        raise ValueError(f'Unknown irregular kind: {kind}')


"""生成复杂背景，背景包含非红色线条和灰色块，用于展示算法抗背景干扰。"""
def make_background(complex_background=False, illumination='normal'):
    image = np.full((HEIGHT, WIDTH, 3), BACKGROUND, dtype=np.uint8)
    if illumination == 'shadow':
        gradient = np.linspace(0.45, 1.0, WIDTH, dtype=np.float32)
        image = np.clip(image.astype(np.float32) * gradient.reshape(1, WIDTH, 1), 0, 255).astype(np.uint8)
    elif illumination == 'bright':
        image = np.full((HEIGHT, WIDTH, 3), (70, 70, 70), dtype=np.uint8)
    if not complex_background:
        return image

    rng = np.random.default_rng(20260609)
    for _index in range(14):
        x1, y1 = rng.integers(0, WIDTH, size=2)
        x2, y2 = rng.integers(0, WIDTH, size=2)
        color = tuple(int(v) for v in rng.integers(45, 115, size=3))
        cv2.line(image, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness=2)
    cv2.rectangle(image, (8, 112), (52, 146), (65, 90, 80), thickness=-1)
    cv2.circle(image, (130, 32), 14, (80, 70, 95), thickness=-1)
    return image


"""根据案例参数生成 RGB 图像。"""
def render_case(spec):
    complex_background = spec.case_type == 'complex_background'
    illumination = 'normal'
    if spec.case_type == 'illumination_shadow':
        illumination = 'shadow'
    elif spec.case_type == 'illumination_bright':
        illumination = 'bright'
    image = make_background(complex_background=complex_background, illumination=illumination)
    color = {
        'normal_red': RED,
        'bright_red': BRIGHT_RED,
        'dark_red': DARK_RED,
        'low_saturation_red': LOW_SATURATION_RED,
    }.get(spec.color_type, RED)

    if spec.objects:
        for obj in spec.objects:
            draw_sphere(
                image,
                color=color,
                center=obj['center'],
                radius=obj['radius'],
                occlusion_ratio=float(obj.get('occlusion_ratio', 0.0)),
                highlight=True,
            )
    elif spec.shape == 'sphere':
        use_highlight = spec.color_type in ('normal_red', 'bright_red')
        draw_sphere(image, color=color, occlusion_ratio=spec.occlusion_ratio, highlight=use_highlight)
    elif spec.shape == 'cube':
        draw_cube_projection(image, color=color)
    elif spec.shape in ('triangle', 'elongated', 'fragments'):
        draw_irregular(image, spec.shape, color=color)
    else:
        raise ValueError(f'Unknown shape: {spec.shape}')
    return image


"""执行检测并整理结果字典。"""
def evaluate_case(spec, image):
    data = bytearray(image.tobytes())
    detections = detect_red_balls_rgb_bytes(data, WIDTH, HEIGHT, min_area_px=80)
    gt_bboxes = expected_bboxes_for_case(spec)
    matches = match_detections_to_ground_truth(detections, gt_bboxes, iou_threshold=0.35)
    actual_detected = len(detections) > 0
    expected_count = len(gt_bboxes)
    true_positive = len(matches)
    false_positive = max(0, len(detections) - true_positive)
    false_negative = max(0, expected_count - true_positive)
    passed = false_positive == 0 and false_negative == 0

    result = {
        'case_id': spec.case_id,
        'case_type': spec.case_type,
        'shape': spec.shape,
        'color_type': spec.color_type,
        'occlusion_ratio': spec.occlusion_ratio,
        'expected_detected': spec.expected_detected,
        'actual_detected': actual_detected,
        'expected_count': expected_count,
        'detection_count': len(detections),
        'true_positive': true_positive,
        'false_positive': false_positive,
        'false_negative': false_negative,
        'pass': passed,
        'confidence': None,
        'circularity': None,
        'aspect_ratio': None,
        'extent': None,
        'red_pixel_count': 0,
        'bbox': None,
        'bboxes': [],
        'gt_bbox': gt_bboxes[0] if len(gt_bboxes) == 1 else None,
        'gt_bboxes': gt_bboxes,
        'iou': None,
        'note': spec.note,
    }
    if detections:
        detection = detections[0]
        pred_bbox = [detection.x_min, detection.y_min, detection.x_max, detection.y_max]
        result.update({
            'confidence': round(detection.confidence, 4),
            'circularity': round(detection.circularity, 4),
            'aspect_ratio': round(detection.aspect_ratio, 4),
            'extent': round(detection.extent, 4),
            'red_pixel_count': sum(item.red_pixel_count for item in detections),
            'bbox': pred_bbox,
            'bboxes': [
                [item.x_min, item.y_min, item.x_max, item.y_max]
                for item in detections
            ],
            'iou': round(best_iou_for_detection(detection, gt_bboxes), 4) if gt_bboxes else None,
        })
    return result, detections


"""在图像上绘制 bbox、通过状态和关键指标。"""
def annotate_image(image, spec, result, detections):
    annotated = image.copy()
    status_color = (30, 210, 60) if result['pass'] else (255, 60, 60)
    for index, detection in enumerate(detections, start=1):
        cv2.rectangle(
            annotated,
            (detection.x_min, detection.y_min),
            (detection.x_max, detection.y_max),
            status_color,
            thickness=3,
        )
        cv2.putText(annotated, str(index), (detection.x_min + 4, detection.y_min + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, status_color, 1)

    expected = 'T' if spec.expected_detected else 'F'
    actual = 'T' if result['actual_detected'] else 'F'
    lines = [
        spec.case_id,
        f'E:{expected} A:{actual} N:{result["detection_count"]}/{result["expected_count"]}',
        f"C:{_format_metric(result['confidence'])} R:{_format_metric(result['circularity'])}",
    ]
    y = 14
    for line in lines:
        cv2.putText(annotated, line, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (245, 245, 245), 2)
        cv2.putText(annotated, line, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.34, status_color, 1)
        y += 14
    return annotated


"""把 None 和浮点数统一格式化，便于写在图上。"""
def _format_metric(value):
    if value is None:
        return 'none'
    return f'{value:.2f}'


"""生成本轮论文效果图使用的可控案例集合。"""
def build_case_specs():
    return [
        CaseSpec('normal_sphere', 'color', 'sphere', 'normal_red', True, note='正常红球'),
        CaseSpec('bright_sphere', 'color', 'sphere', 'bright_red', True, note='亮红球'),
        CaseSpec('dark_sphere', 'color', 'sphere', 'dark_red', False, note='低亮度暗红'),
        CaseSpec('low_sat_sphere', 'color', 'sphere', 'low_saturation_red', False, note='低饱和度红'),
        CaseSpec('occ_10', 'occlusion', 'sphere', 'normal_red', True, 0.10, '10%遮挡'),
        CaseSpec('occ_20', 'occlusion', 'sphere', 'normal_red', True, 0.20, '20%遮挡'),
        CaseSpec('occ_30', 'occlusion', 'sphere', 'normal_red', True, 0.30, '30%遮挡'),
        CaseSpec('occ_50', 'occlusion', 'sphere', 'normal_red', True, 0.50, '50%遮挡'),
        CaseSpec('occ_70', 'occlusion', 'sphere', 'normal_red', False, 0.70, '70%遮挡保守拒绝'),
        CaseSpec('red_cube', 'false_positive', 'cube', 'normal_red', False, note='红色立方体投影'),
        CaseSpec('red_triangle', 'false_positive', 'triangle', 'normal_red', False, note='三角形不规则物体'),
        CaseSpec('red_elongated', 'false_positive', 'elongated', 'normal_red', False, note='细长不规则物体'),
        CaseSpec('red_fragments', 'false_positive', 'fragments', 'normal_red', False, note='碎片状红色干扰'),
        CaseSpec('complex_bg_sphere', 'background', 'sphere', 'normal_red', True, note='复杂背景红球'),
        CaseSpec(
            'multi_three_separate',
            'multi',
            'multi_sphere',
            'normal_red',
            True,
            note='三个分离红球',
            objects=(
                {'center': (38, 48), 'radius': 20},
                {'center': (104, 46), 'radius': 18},
                {'center': (78, 112), 'radius': 22},
            ),
        ),
        CaseSpec(
            'multi_partial_mix',
            'multi',
            'multi_sphere',
            'normal_red',
            True,
            note='多红球和局部遮挡混合',
            objects=(
                {'center': (42, 52), 'radius': 24, 'occlusion_ratio': 0.25},
                {'center': (116, 92), 'radius': 28},
            ),
        ),
        CaseSpec(
            'multi_touching_pair',
            'multi_touching',
            'multi_sphere',
            'normal_red',
            True,
            note='两个相互粘连红球，测试 watershed 分裂',
            objects=(
                {'center': (66, 82), 'radius': 32},
                {'center': (98, 82), 'radius': 32},
            ),
        ),
        CaseSpec('shadow_sphere', 'illumination_shadow', 'sphere', 'normal_red', True, note='阴影渐变环境'),
        CaseSpec('bright_bg_sphere', 'illumination_bright', 'sphere', 'normal_red', True, note='高亮背景环境'),
    ]


"""保存原图、标注图、summary 表、拼图和检测指标汇总。"""
def generate_outputs(output_root):
    cases_dir = output_root / 'cases'
    annotated_dir = output_root / 'annotated'
    figures_dir = output_root / 'figures'
    for path in (cases_dir, annotated_dir, figures_dir):
        path.mkdir(parents=True, exist_ok=True)

    rows = []
    annotated_images = []
    for spec in build_case_specs():
        image = render_case(spec)
        result, detections = evaluate_case(spec, image)
        annotated = annotate_image(image, spec, result, detections)
        rows.append(result)
        annotated_images.append(annotated)

        _write_rgb_png(cases_dir / f'{spec.case_id}.png', image)
        _write_rgb_png(annotated_dir / f'{spec.case_id}.png', annotated)

    _write_summary_csv(output_root / 'summary.csv', rows)
    _write_summary_json(output_root / 'summary.json', rows)
    metrics = write_detection_metrics(output_root, rows)
    testing_record = write_testing_group_perception_record(output_root, rows)
    test_records_dir = (
        REPO_ROOT
        / 'reports'
        / 'perception'
        / 'test_records'
        / output_root.name.replace('synthetic_', '')
    )
    write_testing_group_perception_record(
        test_records_dir,
        rows,
    )
    collage = build_collage(annotated_images)
    collage_path = figures_dir / 'perception_cases_collage.png'
    _write_rgb_png(collage_path, collage)
    return rows, collage_path, metrics, testing_record, test_records_dir


"""返回案例中所有红球真值 bbox。"""
def expected_bboxes_for_case(spec):
    if spec.objects:
        return [
            sphere_bbox(
                obj['center'][0],
                obj['center'][1],
                obj['radius'],
                float(obj.get('occlusion_ratio', 0.0)),
            )
            for obj in spec.objects
        ]
    if spec.expected_detected:
        return [expected_sphere_bbox(spec.occlusion_ratio)]
    return []


"""贪心匹配预测框和真值框，用于多目标案例的 TP/FP/FN 统计。"""
def match_detections_to_ground_truth(detections, gt_bboxes, iou_threshold=0.5):
    matches = []
    used_gt = set()
    candidates = []
    for det_index, detection in enumerate(detections):
        pred_bbox = [detection.x_min, detection.y_min, detection.x_max, detection.y_max]
        for gt_index, gt_bbox in enumerate(gt_bboxes):
            candidates.append((bbox_iou(gt_bbox, pred_bbox), det_index, gt_index))
    candidates.sort(reverse=True)
    used_det = set()
    for iou, det_index, gt_index in candidates:
        if iou < iou_threshold or det_index in used_det or gt_index in used_gt:
            continue
        matches.append((det_index, gt_index, iou))
        used_det.add(det_index)
        used_gt.add(gt_index)
    return matches


"""返回单个预测框对所有真值框的最大 IoU。"""
def best_iou_for_detection(detection, gt_bboxes):
    pred_bbox = [detection.x_min, detection.y_min, detection.x_max, detection.y_max]
    if not gt_bboxes:
        return 0.0
    return max(bbox_iou(gt_bbox, pred_bbox) for gt_bbox in gt_bboxes)


"""把 RGB 图像写成 PNG。"""
def _write_rgb_png(path, image):
    bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(path), bgr)


"""写出 CSV，方便 Excel 或论文表格整理。"""
def _write_summary_csv(path, rows):
    fieldnames = [
        'case_id',
        'case_type',
        'shape',
        'color_type',
        'occlusion_ratio',
        'expected_detected',
        'actual_detected',
        'expected_count',
        'detection_count',
        'true_positive',
        'false_positive',
        'false_negative',
        'pass',
        'confidence',
        'circularity',
        'aspect_ratio',
        'extent',
        'red_pixel_count',
        'bbox',
        'bboxes',
        'gt_bbox',
        'gt_bboxes',
        'iou',
        'note',
    ]
    with path.open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


"""写出 JSON，方便后续脚本继续读取实验结果。"""
def _write_summary_json(path, rows):
    with path.open('w', encoding='utf-8') as f:
        json.dump(rows, f, ensure_ascii=False, indent=2)


"""把合成案例整理为标准目标检测指标。"""
def write_detection_metrics(output_root, rows):
    samples = []
    for row in rows:
        # 旧指标按案例 top1 汇总，便于和前期单目标报告保持可比性。
        samples.append(DetectionSample(
            sample_id=row['case_id'],
            has_ground_truth=row['expected_count'] > 0,
            ground_truth_bbox=row['gt_bbox'] or (row['gt_bboxes'][0] if row['gt_bboxes'] else None),
            predicted_bbox=row['bbox'],
            confidence=0.0 if row['confidence'] is None else float(row['confidence']),
        ))

    metrics = evaluate_detection_samples(samples, iou_threshold=0.5)
    metrics_row = {
        'num_samples': metrics.num_samples,
        'num_ground_truth': metrics.num_ground_truth,
        'true_positive': metrics.true_positive,
        'false_positive': metrics.false_positive,
        'false_negative': metrics.false_negative,
        'true_negative': metrics.true_negative,
        'precision': round(metrics.precision, 4),
        'recall': round(metrics.recall, 4),
        'f1_score': round(metrics.f1_score, 4),
        'accuracy': round(metrics.accuracy, 4),
        'false_positive_rate': round(metrics.false_positive_rate, 4),
        'miss_rate': round(metrics.miss_rate, 4),
        'top1_error': round(metrics.top1_error, 4),
        'ap50': round(metrics.ap50, 4),
    }
    fieldnames = list(metrics_row.keys())
    with (output_root / 'metrics_summary.csv').open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow(metrics_row)
    with (output_root / 'metrics_summary.json').open('w', encoding='utf-8') as f:
        json.dump(metrics_row, f, ensure_ascii=False, indent=2)
    return metrics_row


"""按测试组感知专项指标输出一行记录，便于直接填表。"""
def write_testing_group_perception_record(output_root, rows):
    output_root.mkdir(parents=True, exist_ok=True)
    tp = sum(row['true_positive'] for row in rows)
    fp = sum(row['false_positive'] for row in rows)
    fn = sum(row['false_negative'] for row in rows)
    total_detections = tp + fp
    total_gt = tp + fn
    detection_accuracy = 0.0 if total_gt == 0 else tp / total_gt * 100.0
    false_alarm_rate = 0.0 if total_detections == 0 else fp / total_detections * 100.0
    record = {
        '检测准确率(%)': round(detection_accuracy, 2),
        '虚警率(%)': round(false_alarm_rate, 2),
        '识别数量(个)': tp,
        '虚警数量(个)': fp,
        '漏检数量(个)': fn,
        '定位误差(m)': '未接入仿真深度/TF',
        '平均检测耗时(ms)': '待 Gazebo/主力机计时',
        '检测帧率(FPS)': '待 Gazebo/主力机计时',
        '多帧确认率(%)': '待连续帧测试',
    }
    with (output_root / 'testing_record_perception.csv').open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(record.keys()))
        writer.writeheader()
        writer.writerow(record)
    with (output_root / 'testing_record_perception.json').open('w', encoding='utf-8') as f:
        json.dump(record, f, ensure_ascii=False, indent=2)
    return record


"""把多张标注图拼成论文或汇报可用的大图。"""
def build_collage(images, columns=7, tile_size=160):
    resized = [cv2.resize(image, (tile_size, tile_size), interpolation=cv2.INTER_AREA) for image in images]
    rows = int(np.ceil(len(resized) / columns))
    canvas = np.full((rows * tile_size, columns * tile_size, 3), 245, dtype=np.uint8)
    for index, image in enumerate(resized):
        row = index // columns
        col = index % columns
        y0 = row * tile_size
        x0 = col * tile_size
        canvas[y0:y0 + tile_size, x0:x0 + tile_size] = image
    return canvas


"""命令行入口，默认输出到 reports/perception/2d_detection/synthetic_<timestamp>。"""
def main():
    parser = argparse.ArgumentParser(description='Generate perception red-ball visual cases.')
    parser.add_argument(
        '--output-dir',
        default=None,
        help='Directory for generated perception result images and summaries.',
    )
    args = parser.parse_args()

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_root = (
        Path(args.output_dir)
        if args.output_dir
        else REPO_ROOT / 'reports' / 'perception' / '2d_detection' / f'synthetic_{timestamp}'
    )
    rows, collage_path, metrics, testing_record, test_records_dir = generate_outputs(output_root)
    total = len(rows)
    passed = sum(1 for row in rows if row['pass'])
    print(f'Generated {total} perception cases, {passed}/{total} passed.')
    print(
        'Metrics: '
        f"precision={metrics['precision']:.4f}, "
        f"recall={metrics['recall']:.4f}, "
        f"f1={metrics['f1_score']:.4f}, "
        f"top1_error={metrics['top1_error']:.4f}, "
        f"ap50={metrics['ap50']:.4f}"
    )
    print(
        'Testing record: '
        f"accuracy={testing_record['检测准确率(%)']}%, "
        f"false_alarm_rate={testing_record['虚警率(%)']}%, "
        f"tp={testing_record['识别数量(个)']}, "
        f"fp={testing_record['虚警数量(个)']}, "
        f"fn={testing_record['漏检数量(个)']}"
    )
    print(f'Testing record dir: {test_records_dir}')
    print(f'Collage: {collage_path}')
    return 0 if passed == total else 1


if __name__ == '__main__':
    raise SystemExit(main())
