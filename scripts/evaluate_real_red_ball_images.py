"""评估本地实物红球图片。

所属组：感知组 / 测试组。
文件作用：
读取本地 `red_ball` 文件夹中的实物图片。
按 `real_001`、`real_002` 等编号生成统一命名的标注图和结果表。
调用多目标红球检测接口，输出每张图片的检测数量、bbox、confidence 和形状指标。

当前实现边界：
原始网上图片不复制进仓库，只在本地读取。
输出结果默认放到 `reports/perception/2d_detection/real_images_<timestamp>`，批量图片和表格默认不提交 Git。
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
PERCEPTION_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
sys.path.insert(0, str(PERCEPTION_SRC))

from hazardwalker_perception.red_ball_detector import detect_red_balls_rgb_bytes


IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}


"""返回默认实物图片目录。"""
def default_input_dir():
    candidates = [
        Path.home() / 'OneDrive' / 'Desktop' / 'red_ball',
        Path.home() / 'Desktop' / 'red_ball',
    ]
    for path in candidates:
        if path.exists():
            return path
    return candidates[0]


"""读取图片路径并按文件名稳定排序，再统一分配 real_001 这类编号。"""
def list_image_cases(input_dir):
    paths = [
        path for path in sorted(input_dir.iterdir())
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    ]
    return [(f'real_{index:03d}', path) for index, path in enumerate(paths, start=1)]


"""运行多目标检测。"""
def detect_image(image_bgr, min_area_px, min_area_ratio, max_detections):
    height, width = image_bgr.shape[:2]
    area_threshold = max(min_area_px, int(width * height * min_area_ratio))
    return detect_red_balls_rgb_bytes(
        data=bytearray(image_bgr.tobytes()),
        width=width,
        height=height,
        encoding='bgr8',
        min_area_px=area_threshold,
        max_detections=max_detections,
    )


"""在实物图上标注所有检测框和关键指标。"""
def annotate_image(image_bgr, case_id, detections):
    annotated = image_bgr.copy()
    for index, detection in enumerate(detections, start=1):
        color = (40, 220, 40)
        cv2.rectangle(
            annotated,
            (detection.x_min, detection.y_min),
            (detection.x_max, detection.y_max),
            color,
            thickness=max(2, annotated.shape[1] // 700),
        )
        label = f'{index}: c={detection.confidence:.2f} r={detection.circularity:.2f}'
        y = max(24, detection.y_min - 8)
        cv2.putText(
            annotated,
            label,
            (detection.x_min, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            max(0.55, annotated.shape[1] / 2400.0),
            (0, 0, 0),
            thickness=3,
        )
        cv2.putText(
            annotated,
            label,
            (detection.x_min, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            max(0.55, annotated.shape[1] / 2400.0),
            color,
            thickness=1,
        )

    header = f'{case_id} detections={len(detections)}'
    cv2.putText(annotated, header, (18, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4)
    cv2.putText(annotated, header, (18, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (40, 220, 40), 2)
    return annotated


"""将图片缩放到统一宽度，便于拼图。"""
def resize_for_collage(image_bgr, tile_width=360, tile_height=260):
    height, width = image_bgr.shape[:2]
    scale = min(tile_width / width, tile_height / height)
    resized = cv2.resize(image_bgr, (int(width * scale), int(height * scale)), interpolation=cv2.INTER_AREA)
    canvas = np.full((tile_height, tile_width, 3), 245, dtype=np.uint8)
    y0 = (tile_height - resized.shape[0]) // 2
    x0 = (tile_width - resized.shape[1]) // 2
    canvas[y0:y0 + resized.shape[0], x0:x0 + resized.shape[1]] = resized
    return canvas


"""将多张实物检测图拼成一张汇报图。"""
def build_collage(images_bgr, columns=5):
    tiles = [resize_for_collage(image) for image in images_bgr]
    if not tiles:
        return None
    tile_height, tile_width = tiles[0].shape[:2]
    rows = int(np.ceil(len(tiles) / columns))
    canvas = np.full((rows * tile_height, columns * tile_width, 3), 245, dtype=np.uint8)
    for index, tile in enumerate(tiles):
        row = index // columns
        col = index % columns
        y0 = row * tile_height
        x0 = col * tile_width
        canvas[y0:y0 + tile_height, x0:x0 + tile_width] = tile
    return canvas


"""整理单张图片的 summary 行。"""
def build_summary_row(case_id, source_file, image_bgr, detections):
    height, width = image_bgr.shape[:2]
    top = detections[0] if detections else None
    return {
        'case_id': case_id,
        'source_file': source_file.name,
        'width': width,
        'height': height,
        'detection_count': len(detections),
        'top_confidence': None if top is None else round(top.confidence, 4),
        'top_circularity': None if top is None else round(top.circularity, 4),
        'top_aspect_ratio': None if top is None else round(top.aspect_ratio, 4),
        'top_extent': None if top is None else round(top.extent, 4),
        'top_bbox': None if top is None else [top.x_min, top.y_min, top.x_max, top.y_max],
    }


"""整理每个检测框的明细行。"""
def build_detection_rows(case_id, source_file, detections):
    rows = []
    for index, detection in enumerate(detections, start=1):
        rows.append({
            'case_id': case_id,
            'source_file': source_file.name,
            'detection_index': index,
            'confidence': round(detection.confidence, 4),
            'circularity': round(detection.circularity, 4),
            'aspect_ratio': round(detection.aspect_ratio, 4),
            'extent': round(detection.extent, 4),
            'red_pixel_count': detection.red_pixel_count,
            'bbox': [detection.x_min, detection.y_min, detection.x_max, detection.y_max],
        })
    return rows


"""保存 CSV 文件。"""
def write_csv(path, rows, fieldnames):
    with path.open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


"""绘制参数柱状图，展示每张实物图的检测数量和最高置信度。"""
def write_metric_charts(output_dir, summary_rows):
    figures_dir = output_dir / 'figures'
    figures_dir.mkdir(parents=True, exist_ok=True)

    labels = [row['case_id'] for row in summary_rows]
    counts = [row['detection_count'] for row in summary_rows]
    confidences = [0.0 if row['top_confidence'] is None else row['top_confidence'] for row in summary_rows]

    count_chart = _draw_bar_chart(labels, counts, 'Detection count', max(counts + [1]))
    confidence_chart = _draw_bar_chart(labels, confidences, 'Top confidence', 1.0)
    cv2.imwrite(str(figures_dir / 'real_detection_count.png'), count_chart)
    cv2.imwrite(str(figures_dir / 'real_top_confidence.png'), confidence_chart)


"""用 OpenCV 绘制简单柱状图，避免额外引入 matplotlib 依赖。"""
def _draw_bar_chart(labels, values, title, y_max):
    width = 1100
    height = 420
    margin_left = 80
    margin_bottom = 70
    margin_top = 60
    chart = np.full((height, width, 3), 255, dtype=np.uint8)
    cv2.putText(chart, title, (margin_left, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (30, 30, 30), 2)
    cv2.line(chart, (margin_left, margin_top), (margin_left, height - margin_bottom), (80, 80, 80), 2)
    cv2.line(chart, (margin_left, height - margin_bottom), (width - 30, height - margin_bottom), (80, 80, 80), 2)

    bar_area_width = width - margin_left - 60
    step = bar_area_width / max(1, len(values))
    bar_width = max(18, int(step * 0.55))
    for index, value in enumerate(values):
        x_center = int(margin_left + step * index + step / 2)
        normalized = 0.0 if y_max <= 0 else min(1.0, value / y_max)
        bar_height = int((height - margin_bottom - margin_top - 15) * normalized)
        x1 = x_center - bar_width // 2
        x2 = x_center + bar_width // 2
        y1 = height - margin_bottom - bar_height
        y2 = height - margin_bottom
        cv2.rectangle(chart, (x1, y1), (x2, y2), (45, 130, 220), thickness=-1)
        cv2.putText(chart, f'{value:.2g}', (x1, max(25, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 20, 20), 1)
        cv2.putText(chart, labels[index], (x1 - 10, height - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 20, 20), 1)
    return chart


"""主流程：编号、检测、保存标注图、表格、拼图和参数图。"""
def evaluate_real_images(input_dir, output_dir, min_area_px, min_area_ratio, max_detections):
    output_dir.mkdir(parents=True, exist_ok=True)
    annotated_dir = output_dir / 'annotated'
    figures_dir = output_dir / 'figures'
    annotated_dir.mkdir(parents=True, exist_ok=True)
    figures_dir.mkdir(parents=True, exist_ok=True)

    summary_rows = []
    detection_rows = []
    annotated_images = []
    for case_id, image_path in list_image_cases(input_dir):
        image_bgr = cv2.imread(str(image_path))
        if image_bgr is None:
            continue
        detections = detect_image(image_bgr, min_area_px, min_area_ratio, max_detections)
        annotated = annotate_image(image_bgr, case_id, detections)
        annotated_path = annotated_dir / f'{case_id}_annotated.png'
        cv2.imwrite(str(annotated_path), annotated)
        annotated_images.append(annotated)
        summary_rows.append(build_summary_row(case_id, image_path, image_bgr, detections))
        detection_rows.extend(build_detection_rows(case_id, image_path, detections))

    write_csv(output_dir / 'summary.csv', summary_rows, [
        'case_id',
        'source_file',
        'width',
        'height',
        'detection_count',
        'top_confidence',
        'top_circularity',
        'top_aspect_ratio',
        'top_extent',
        'top_bbox',
    ])
    write_csv(output_dir / 'detections.csv', detection_rows, [
        'case_id',
        'source_file',
        'detection_index',
        'confidence',
        'circularity',
        'aspect_ratio',
        'extent',
        'red_pixel_count',
        'bbox',
    ])
    with (output_dir / 'summary.json').open('w', encoding='utf-8') as f:
        json.dump(summary_rows, f, ensure_ascii=False, indent=2)

    collage = build_collage(annotated_images)
    if collage is not None:
        cv2.imwrite(str(figures_dir / 'real_red_ball_collage.png'), collage)
    write_metric_charts(output_dir, summary_rows)
    return summary_rows


"""命令行入口。"""
def main():
    parser = argparse.ArgumentParser(description='Evaluate real red-ball images with numbered outputs.')
    parser.add_argument('--input-dir', default=str(default_input_dir()), help='Folder containing real red-ball images.')
    parser.add_argument(
        '--output-dir',
        default=None,
        help='Output directory for annotated images and summaries.',
    )
    parser.add_argument('--min-area-px', type=int, default=80, help='Absolute minimum contour area.')
    parser.add_argument('--min-area-ratio', type=float, default=0.0002, help='Minimum contour area ratio per image.')
    parser.add_argument('--max-detections', type=int, default=30, help='Maximum detections per image.')
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    if not input_dir.exists():
        print(f'Input directory not found: {input_dir}', file=sys.stderr)
        return 1

    rows = evaluate_real_images(
        input_dir=input_dir,
        output_dir=(
            Path(args.output_dir)
            if args.output_dir
            else REPO_ROOT / 'reports' / 'perception' / '2d_detection' / f'real_images_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        ),
        min_area_px=args.min_area_px,
        min_area_ratio=args.min_area_ratio,
        max_detections=args.max_detections,
    )
    total_detections = sum(row['detection_count'] for row in rows)
    print(f'Evaluated {len(rows)} real images, total detections={total_detections}.')
    print(f'Output: {args.output_dir}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
