"""生成感知组红球检测可视化实验案例。

所属组：感知组 / 测试组。
文件作用：
生成可控的红球、遮挡、低亮度、低饱和度和红色非球体图像。
调用 `detect_red_ball_rgb_bytes` 得到检测结果和形状指标。
输出原图、标注图、summary.csv、summary.json、汇报用拼图和遮挡比例指标折线图。
同步输出 detection metrics 汇总，用于汇报 precision、recall、F1、top1 error 和 AP50。

当前实现边界：
只生成单目标离线案例，不依赖 ROS、Gazebo 或真实相机。
图片是 RGB 合成图，主要用于算法回归、汇报展示和参数解释。
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
PERCEPTION_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
sys.path.insert(0, str(PERCEPTION_SRC))

from hazardwalker_perception.red_ball_detector import detect_red_ball_rgb_bytes
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


"""生成渐变红色圆形，让二维图像更接近球体外观。"""
def draw_sphere(image, color=RED, occlusion_ratio=0.0, highlight=True):
    center = (WIDTH // 2, HEIGHT // 2)
    radius = 42
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
def make_background(complex_background=False):
    image = np.full((HEIGHT, WIDTH, 3), BACKGROUND, dtype=np.uint8)
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
    image = make_background(complex_background=complex_background)
    color = {
        'normal_red': RED,
        'bright_red': BRIGHT_RED,
        'dark_red': DARK_RED,
        'low_saturation_red': LOW_SATURATION_RED,
    }.get(spec.color_type, RED)

    if spec.shape == 'sphere':
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
    detection = detect_red_ball_rgb_bytes(data, WIDTH, HEIGHT, min_area_px=80)
    actual_detected = detection is not None
    passed = actual_detected == spec.expected_detected

    result = {
        'case_id': spec.case_id,
        'case_type': spec.case_type,
        'shape': spec.shape,
        'color_type': spec.color_type,
        'occlusion_ratio': spec.occlusion_ratio,
        'expected_detected': spec.expected_detected,
        'actual_detected': actual_detected,
        'pass': passed,
        'confidence': None,
        'circularity': None,
        'aspect_ratio': None,
        'extent': None,
        'red_pixel_count': 0,
        'bbox': None,
        'gt_bbox': expected_sphere_bbox(spec.occlusion_ratio) if spec.expected_detected else None,
        'iou': None,
        'note': spec.note,
    }
    if detection is not None:
        pred_bbox = [detection.x_min, detection.y_min, detection.x_max, detection.y_max]
        result.update({
            'confidence': round(detection.confidence, 4),
            'circularity': round(detection.circularity, 4),
            'aspect_ratio': round(detection.aspect_ratio, 4),
            'extent': round(detection.extent, 4),
            'red_pixel_count': detection.red_pixel_count,
            'bbox': pred_bbox,
            'iou': round(bbox_iou(result['gt_bbox'], pred_bbox), 4) if result['gt_bbox'] else None,
        })
    return result, detection


"""在图像上绘制 bbox、通过状态和关键指标。"""
def annotate_image(image, spec, result, detection):
    annotated = image.copy()
    status_color = (30, 210, 60) if result['pass'] else (255, 60, 60)
    if detection is not None:
        cv2.rectangle(
            annotated,
            (detection.x_min, detection.y_min),
            (detection.x_max, detection.y_max),
            status_color,
            thickness=3,
        )

    expected = 'T' if spec.expected_detected else 'F'
    actual = 'T' if result['actual_detected'] else 'F'
    lines = [
        spec.case_id,
        f'E:{expected} A:{actual}',
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
    ]


"""保存原图、标注图、summary 表、拼图、遮挡指标图和检测指标汇总。"""
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
        result, detection = evaluate_case(spec, image)
        annotated = annotate_image(image, spec, result, detection)
        rows.append(result)
        annotated_images.append(annotated)

        _write_rgb_png(cases_dir / f'{spec.case_id}.png', image)
        _write_rgb_png(annotated_dir / f'{spec.case_id}.png', annotated)

    _write_summary_csv(output_root / 'summary.csv', rows)
    _write_summary_json(output_root / 'summary.json', rows)
    metrics = write_detection_metrics(output_root, rows)
    collage = build_collage(annotated_images)
    collage_path = figures_dir / 'perception_cases_collage.png'
    _write_rgb_png(collage_path, collage)
    write_occlusion_charts(figures_dir, rows)
    return rows, collage_path, metrics


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
        'pass',
        'confidence',
        'circularity',
        'aspect_ratio',
        'extent',
        'red_pixel_count',
        'bbox',
        'gt_bbox',
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
        samples.append(DetectionSample(
            sample_id=row['case_id'],
            has_ground_truth=row['expected_detected'],
            ground_truth_bbox=row['gt_bbox'],
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


"""从合成案例结果中提取遮挡比例和指定指标，用于解释遮挡鲁棒性边界。"""
def _extract_occlusion_metric(rows, metric_name):
    pairs = []
    for row in rows:
        if row['case_type'] != 'occlusion':
            continue
        value = row[metric_name]
        pairs.append((float(row['occlusion_ratio']), 0.0 if value is None else float(value)))
    pairs.sort(key=lambda item: item[0])
    return pairs


"""绘制遮挡比例到 confidence / circularity 的折线图。"""
def write_occlusion_charts(figures_dir, rows):
    chart_specs = [
        ('confidence', 'Occlusion ratio - confidence', 'occlusion_confidence.png'),
        ('circularity', 'Occlusion ratio - circularity', 'occlusion_circularity.png'),
    ]
    for metric_name, title, filename in chart_specs:
        pairs = _extract_occlusion_metric(rows, metric_name)
        if pairs:
            chart = _draw_line_chart(pairs, title, y_max=1.0)
            _write_rgb_png(figures_dir / filename, chart)


"""用 OpenCV 绘制简洁折线图，避免为汇报图额外引入 matplotlib 依赖。"""
def _draw_line_chart(pairs, title, y_max):
    width = 900
    height = 460
    margin_left = 90
    margin_right = 40
    margin_top = 70
    margin_bottom = 80
    chart = np.full((height, width, 3), 255, dtype=np.uint8)

    plot_left = margin_left
    plot_right = width - margin_right
    plot_top = margin_top
    plot_bottom = height - margin_bottom
    cv2.putText(chart, title, (margin_left, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.86, (30, 30, 30), 2)
    cv2.line(chart, (plot_left, plot_top), (plot_left, plot_bottom), (85, 85, 85), 2)
    cv2.line(chart, (plot_left, plot_bottom), (plot_right, plot_bottom), (85, 85, 85), 2)

    for tick in range(0, 6):
        value = tick / 5.0
        y = int(plot_bottom - (plot_bottom - plot_top) * value)
        cv2.line(chart, (plot_left - 5, y), (plot_left, y), (85, 85, 85), 1)
        cv2.putText(chart, f'{value:.1f}', (18, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (45, 45, 45), 1)

    x_values = [ratio for ratio, _value in pairs]
    x_min = min(x_values)
    x_max = max(x_values)
    x_span = max(x_max - x_min, 1e-6)
    points = []
    for ratio, value in pairs:
        x = int(plot_left + (ratio - x_min) / x_span * (plot_right - plot_left))
        y = int(plot_bottom - min(max(value, 0.0), y_max) / y_max * (plot_bottom - plot_top))
        points.append((x, y))

    for start, end in zip(points, points[1:]):
        cv2.line(chart, start, end, (35, 110, 215), 3)

    for (ratio, value), point in zip(pairs, points):
        cv2.circle(chart, point, 6, (35, 110, 215), thickness=-1)
        cv2.putText(chart, f'{value:.2f}', (point[0] - 18, point[1] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (25, 25, 25), 1)
        label = f'{int(round(ratio * 100))}%'
        cv2.putText(chart, label, (point[0] - 14, plot_bottom + 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (25, 25, 25), 1)

    cv2.putText(chart, 'Occlusion ratio', (width // 2 - 78, height - 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (35, 35, 35), 1)
    cv2.putText(chart, 'Metric value', (14, margin_top - 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (35, 35, 35), 1)
    return chart


"""命令行入口，默认输出到 reports/perception/result。"""
def main():
    parser = argparse.ArgumentParser(description='Generate perception red-ball visual cases.')
    parser.add_argument(
        '--output-dir',
        default=str(REPO_ROOT / 'reports' / 'perception' / 'result'),
        help='Directory for generated perception result images and summaries.',
    )
    args = parser.parse_args()

    output_root = Path(args.output_dir)
    rows, collage_path, metrics = generate_outputs(output_root)
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
    print(f'Collage: {collage_path}')
    return 0 if passed == total else 1


if __name__ == '__main__':
    raise SystemExit(main())
