"""目标检测指标计算纯函数。

所属组：感知组 / 测试组。
文件作用：
根据每张图像的真值 bbox、预测 bbox 和置信度，计算 precision、recall、F1、top1 error、AP50 等指标。
当前实现边界：
面向当前红球检测的一图一主目标评估，适合合成案例和小规模仿真实验汇总。
多目标严格 AP 评估后续可在保留 IoU 与 AP 基础函数的前提下扩展。
验证方式：
使用 tests/offline/test_detection_metrics.py 构造真阳性、假阳性、漏检和低 IoU 样例。
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class DetectionSample:
    """单张图像的检测评估输入。"""

    sample_id: str
    has_ground_truth: bool
    ground_truth_bbox: list = None
    predicted_bbox: list = None
    confidence: float = 0.0


@dataclass(frozen=True)
class DetectionMetrics:
    """一组检测样本的汇总指标。"""

    num_samples: int
    num_ground_truth: int
    true_positive: int
    false_positive: int
    false_negative: int
    true_negative: int
    precision: float
    recall: float
    f1_score: float
    accuracy: float
    false_positive_rate: float
    miss_rate: float
    top1_error: float
    ap50: float


"""计算两个 bbox 的 IoU，bbox 格式为 [x_min, y_min, x_max, y_max]。"""
def bbox_iou(box_a, box_b):
    if box_a is None or box_b is None:
        return 0.0

    ax0, ay0, ax1, ay1 = [float(value) for value in box_a]
    bx0, by0, bx1, by1 = [float(value) for value in box_b]
    inter_x0 = max(ax0, bx0)
    inter_y0 = max(ay0, by0)
    inter_x1 = min(ax1, bx1)
    inter_y1 = min(ay1, by1)

    inter_width = max(0.0, inter_x1 - inter_x0 + 1.0)
    inter_height = max(0.0, inter_y1 - inter_y0 + 1.0)
    intersection = inter_width * inter_height

    area_a = max(0.0, ax1 - ax0 + 1.0) * max(0.0, ay1 - ay0 + 1.0)
    area_b = max(0.0, bx1 - bx0 + 1.0) * max(0.0, by1 - by0 + 1.0)
    union = area_a + area_b - intersection
    if union <= 0.0:
        return 0.0
    return intersection / union


"""根据 IoU 阈值计算论文汇报常用检测指标。"""
def evaluate_detection_samples(samples, iou_threshold=0.5):
    true_positive = 0
    false_positive = 0
    false_negative = 0
    true_negative = 0

    for sample in samples:
        has_prediction = sample.predicted_bbox is not None
        matched = (
            sample.has_ground_truth and
            has_prediction and
            bbox_iou(sample.ground_truth_bbox, sample.predicted_bbox) >= iou_threshold
        )

        if matched:
            true_positive += 1
        elif sample.has_ground_truth and has_prediction:
            false_positive += 1
            false_negative += 1
        elif sample.has_ground_truth:
            false_negative += 1
        elif has_prediction:
            false_positive += 1
        else:
            true_negative += 1

    precision = _safe_divide(true_positive, true_positive + false_positive)
    recall = _safe_divide(true_positive, true_positive + false_negative)
    f1_score = _safe_divide(2.0 * precision * recall, precision + recall)
    accuracy = _safe_divide(true_positive + true_negative, len(samples))
    false_positive_rate = _safe_divide(false_positive, false_positive + true_negative)
    miss_rate = 1.0 - recall if true_positive + false_negative > 0 else 0.0
    top1_error = 1.0 - accuracy
    ap50 = average_precision(samples, iou_threshold=iou_threshold)

    return DetectionMetrics(
        num_samples=len(samples),
        num_ground_truth=sum(1 for sample in samples if sample.has_ground_truth),
        true_positive=true_positive,
        false_positive=false_positive,
        false_negative=false_negative,
        true_negative=true_negative,
        precision=precision,
        recall=recall,
        f1_score=f1_score,
        accuracy=accuracy,
        false_positive_rate=false_positive_rate,
        miss_rate=miss_rate,
        top1_error=top1_error,
        ap50=ap50,
    )


"""计算单类别 AP，当前用于 AP50。"""
def average_precision(samples, iou_threshold=0.5):
    num_ground_truth = sum(1 for sample in samples if sample.has_ground_truth)
    if num_ground_truth == 0:
        return 0.0

    predictions = [
        sample for sample in samples
        if sample.predicted_bbox is not None
    ]
    predictions.sort(key=lambda sample: float(sample.confidence), reverse=True)
    if not predictions:
        return 0.0

    precisions = []
    recalls = []
    true_positive = 0
    false_positive = 0
    for sample in predictions:
        matched = (
            sample.has_ground_truth and
            bbox_iou(sample.ground_truth_bbox, sample.predicted_bbox) >= iou_threshold
        )
        if matched:
            true_positive += 1
        else:
            false_positive += 1
        precisions.append(_safe_divide(true_positive, true_positive + false_positive))
        recalls.append(_safe_divide(true_positive, num_ground_truth))

    return _precision_recall_ap(precisions, recalls)


"""用 precision envelope 计算 PR 曲线面积。"""
def _precision_recall_ap(precisions, recalls):
    padded_precisions = [0.0] + list(precisions) + [0.0]
    padded_recalls = [0.0] + list(recalls) + [1.0]

    for index in range(len(padded_precisions) - 2, -1, -1):
        padded_precisions[index] = max(padded_precisions[index], padded_precisions[index + 1])

    ap = 0.0
    for index in range(1, len(padded_recalls)):
        recall_delta = padded_recalls[index] - padded_recalls[index - 1]
        if recall_delta > 0.0:
            ap += recall_delta * padded_precisions[index]
    return ap


"""安全除法，分母为 0 时返回 0。"""
def _safe_divide(numerator, denominator):
    if denominator == 0:
        return 0.0
    return float(numerator) / float(denominator)
