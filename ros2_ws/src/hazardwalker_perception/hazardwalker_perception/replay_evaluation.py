"""人工标注回放数据的多目标检测与三维定位指标。

文件作用：
- 对固定 SEED rosbag 回放得到的 ``frames.jsonl`` 做一对一 IoU 匹配；
- 分别报告二维候选与最终确认输出，避免把黄色复查框当成危险源虚警；
- 仅接受人工图像标注或赛后公开参考，不允许运行时读取 Gazebo 真值。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable

from .dynamic_detection_records import build_dynamic_summary


ALLOWED_ANNOTATION_PROVENANCE = {
    'manual_image_annotation',
    'public_postrun_reference',
}


@dataclass(frozen=True)
class MatchCounts:
    true_positive: int
    false_positive: int
    false_negative: int
    true_negative_frames: int


def evaluate_labeled_replay(
    records: list[dict[str, Any]],
    annotation_document: dict[str, Any],
    *,
    iou_threshold: float = 0.5,
) -> dict[str, Any]:
    """评估已人工标注的帧；未标注帧不参与真值指标。"""

    provenance = str(
        annotation_document.get('annotation_provenance', '')).strip()
    if provenance not in ALLOWED_ANNOTATION_PROVENANCE:
        raise ValueError('标注来源必须是人工图像标注或赛后公开参考')
    if not 0.0 < float(iou_threshold) <= 1.0:
        raise ValueError('IoU 阈值必须在 0 到 1 之间')
    annotations = annotation_document.get('frames')
    if not isinstance(annotations, list) or not annotations:
        raise ValueError('标注文档必须包含非空 frames 列表')

    candidate_counts = MatchCounts(0, 0, 0, 0)
    confirmed_counts = MatchCounts(0, 0, 0, 0)
    localization_errors = []
    localization_reference_count = 0
    localization_prediction_count = 0
    used_indices = set()
    for annotation in annotations:
        record_index = int(annotation.get('record_index', -1))
        if record_index < 0 or record_index >= len(records):
            raise ValueError(f'标注 record_index 越界：{record_index}')
        if record_index in used_indices:
            raise ValueError(f'重复标注 record_index：{record_index}')
        used_indices.add(record_index)
        record = records[record_index]
        objects = annotation.get('objects', [])
        if not isinstance(objects, list):
            raise ValueError('objects 必须是列表')
        ground_truth_boxes = [_object_bbox(item) for item in objects]
        candidate_boxes = [
            _detection_bbox(item)
            for item in record.get('detections_2d', [])
            if isinstance(item, dict)
        ]
        confirmed_boxes = [
            _detection_bbox(item)
            for item in record.get('detections_2d', [])
            if isinstance(item, dict)
            and str(item.get('track_status', '')) == 'confirmed'
        ]
        candidate_counts = _add_counts(
            candidate_counts,
            _match_boxes(ground_truth_boxes, candidate_boxes, iou_threshold),
        )
        confirmed_counts = _add_counts(
            confirmed_counts,
            _match_boxes(ground_truth_boxes, confirmed_boxes, iou_threshold),
        )
        frame_errors, reference_count, prediction_count = (
            _frame_localization_errors(record, annotation)
        )
        localization_errors.extend(frame_errors)
        localization_reference_count += reference_count
        localization_prediction_count += prediction_count

    dynamic_summary = build_dynamic_summary(records)
    return {
        'schema_version': 1,
        'annotation_provenance': provenance,
        'truth_inputs_used_at_runtime': False,
        'labeled_frame_count': len(annotations),
        'iou_threshold': float(iou_threshold),
        'candidate_metrics': _metrics(candidate_counts),
        'confirmed_output_metrics': _metrics(confirmed_counts),
        'localization_error_m': _error_summary(
            localization_errors,
            reference_count=localization_reference_count,
            prediction_count=localization_prediction_count,
        ),
        'candidate_to_confirmation_latency_sec': dynamic_summary[
            'candidate_to_confirmation_latency_sec'
        ],
        'processing_time_ms': dynamic_summary['processing_time_ms'],
        'observed_output_rate_hz': dynamic_summary[
            'observed_output_rate_hz'
        ],
        'limitations': [
            '真值指标仅覆盖人工标注帧，未标注帧不计入分母。',
            '三维误差只使用标注明确提供的 start 坐标系赛后参考位置。',
        ],
    }


def _match_boxes(ground_truth, predictions, threshold):
    pairs = []
    for gt_index, gt_box in enumerate(ground_truth):
        for pred_index, pred_box in enumerate(predictions):
            overlap = bbox_iou(gt_box, pred_box)
            if overlap >= threshold:
                pairs.append((overlap, gt_index, pred_index))
    matched_gt = set()
    matched_pred = set()
    for _overlap, gt_index, pred_index in sorted(pairs, reverse=True):
        if gt_index in matched_gt or pred_index in matched_pred:
            continue
        matched_gt.add(gt_index)
        matched_pred.add(pred_index)
    true_positive = len(matched_gt)
    return MatchCounts(
        true_positive=true_positive,
        false_positive=len(predictions) - true_positive,
        false_negative=len(ground_truth) - true_positive,
        true_negative_frames=int(not ground_truth and not predictions),
    )


def bbox_iou(box_a, box_b):
    """计算 ``[x_min, y_min, x_max, y_max]`` 的 IoU。"""

    ax0, ay0, ax1, ay1 = box_a
    bx0, by0, bx1, by1 = box_b
    intersection = max(0.0, min(ax1, bx1) - max(ax0, bx0)) * max(
        0.0, min(ay1, by1) - max(ay0, by0))
    area_a = max(0.0, ax1 - ax0) * max(0.0, ay1 - ay0)
    area_b = max(0.0, bx1 - bx0) * max(0.0, by1 - by0)
    union = area_a + area_b - intersection
    return intersection / union if union > 0.0 else 0.0


def _object_bbox(item):
    if not isinstance(item, dict):
        raise ValueError('标注对象必须是字典')
    return _normalize_bbox(item.get('bbox'))


def _detection_bbox(item):
    return _normalize_bbox(item.get('bbox'))


def _normalize_bbox(value):
    if isinstance(value, dict):
        value = [
            value.get('x_min'), value.get('y_min'),
            value.get('x_max'), value.get('y_max'),
        ]
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError('bbox 必须包含四个坐标')
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError('bbox 坐标必须是有限数值')
    if result[2] <= result[0] or result[3] <= result[1]:
        raise ValueError('bbox 宽高必须为正数')
    return result


def _frame_localization_errors(record, annotation):
    references = []
    for item in annotation.get('objects', []):
        if not isinstance(item, dict) or item.get('world_position') is None:
            continue
        if str(item.get('position_frame_id', 'start')) != 'start':
            raise ValueError('三维参考位置必须使用 start 坐标系')
        references.append(_point(item['world_position']))
    predictions = [
        _point(item['position'])
        for item in record.get('hazards', [])
        if isinstance(item, dict)
        and item.get('status') == 'confirmed'
        and item.get('position') is not None
        and str(item.get('position_frame_id', '')) == 'start'
    ]
    pairs = []
    for ref_index, reference in enumerate(references):
        for pred_index, prediction in enumerate(predictions):
            pairs.append((
                _distance(reference, prediction), ref_index, pred_index))
    used_ref = set()
    used_pred = set()
    errors = []
    for error, ref_index, pred_index in sorted(pairs):
        if ref_index in used_ref or pred_index in used_pred:
            continue
        used_ref.add(ref_index)
        used_pred.add(pred_index)
        errors.append(error)
    return errors, len(references), len(predictions)


def _point(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError('三维位置必须包含三个坐标')
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError('三维位置必须是有限数值')
    return result


def _distance(a, b):
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(a, b)))


def _add_counts(a, b):
    return MatchCounts(
        true_positive=a.true_positive + b.true_positive,
        false_positive=a.false_positive + b.false_positive,
        false_negative=a.false_negative + b.false_negative,
        true_negative_frames=(
            a.true_negative_frames + b.true_negative_frames),
    )


def _metrics(counts):
    precision = _safe_divide(
        counts.true_positive, counts.true_positive + counts.false_positive)
    recall = _safe_divide(
        counts.true_positive, counts.true_positive + counts.false_negative)
    return {
        'true_positive': counts.true_positive,
        'false_positive': counts.false_positive,
        'false_negative': counts.false_negative,
        'true_negative_frames': counts.true_negative_frames,
        'precision': round(precision, 6),
        'recall': round(recall, 6),
        'f1_score': round(_safe_divide(2 * precision * recall, precision + recall), 6),
    }


def _error_summary(
    errors: Iterable[float], *, reference_count: int, prediction_count: int,
):
    values = sorted(float(value) for value in errors)
    if not values:
        return {
            'count': 0,
            'reference_count': int(reference_count),
            'prediction_count': int(prediction_count),
            'coverage': 0.0,
            'mean': None,
            'median': None,
            'max': None,
        }
    middle = len(values) // 2
    median = (
        values[middle] if len(values) % 2
        else (values[middle - 1] + values[middle]) * 0.5
    )
    return {
        'count': len(values),
        'reference_count': int(reference_count),
        'prediction_count': int(prediction_count),
        'coverage': round(
            _safe_divide(len(values), int(reference_count)), 6),
        'mean': round(sum(values) / len(values), 6),
        'median': round(median, 6),
        'max': round(max(values), 6),
    }


def _safe_divide(numerator, denominator):
    return float(numerator) / float(denominator) if denominator else 0.0
