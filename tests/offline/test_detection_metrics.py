"""红球检测指标离线测试。

所属组：感知组 / 测试组。
文件作用：
验证 detection_metrics.py 中 IoU、precision、recall、top1 error 和 AP50 的计算逻辑。
不依赖 ROS、Gazebo 或真实相机。
"""
import math
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_perception'))

from hazardwalker_perception.detection_metrics import (
    DetectionSample,
    average_precision,
    bbox_iou,
    evaluate_detection_samples,
)


"""验证两个 bbox 的 IoU 使用包含端点的像素面积。"""
def test_bbox_iou_computes_overlap_ratio():
    iou = bbox_iou([0, 0, 9, 9], [5, 5, 14, 14])

    assert math.isclose(iou, 25.0 / 175.0)


"""验证完整命中的简单集合能得到满分指标。"""
def test_evaluate_detection_samples_scores_perfect_predictions():
    samples = [
        DetectionSample('positive', True, [0, 0, 9, 9], [0, 0, 9, 9], 0.9),
        DetectionSample('negative', False, None, None, 0.0),
    ]

    metrics = evaluate_detection_samples(samples)

    assert metrics.true_positive == 1
    assert metrics.true_negative == 1
    assert metrics.false_positive == 0
    assert metrics.false_negative == 0
    assert metrics.precision == 1.0
    assert metrics.recall == 1.0
    assert metrics.f1_score == 1.0
    assert metrics.top1_error == 0.0
    assert metrics.ap50 == 1.0


"""验证低 IoU 预测同时记为假阳性和漏检。"""
def test_low_iou_prediction_counts_as_fp_and_fn():
    samples = [
        DetectionSample('bad_box', True, [0, 0, 9, 9], [30, 30, 39, 39], 0.8),
    ]

    metrics = evaluate_detection_samples(samples, iou_threshold=0.5)

    assert metrics.true_positive == 0
    assert metrics.false_positive == 1
    assert metrics.false_negative == 1
    assert metrics.precision == 0.0
    assert metrics.recall == 0.0
    assert metrics.top1_error == 1.0


"""验证假阳性会降低 precision 和 top1 准确率。"""
def test_false_positive_reduces_precision_and_accuracy():
    samples = [
        DetectionSample('positive', True, [0, 0, 9, 9], [0, 0, 9, 9], 0.9),
        DetectionSample('negative_fp', False, None, [0, 0, 5, 5], 0.7),
        DetectionSample('negative_tn', False, None, None, 0.0),
    ]

    metrics = evaluate_detection_samples(samples)

    assert metrics.true_positive == 1
    assert metrics.false_positive == 1
    assert metrics.true_negative == 1
    assert math.isclose(metrics.precision, 0.5)
    assert math.isclose(metrics.recall, 1.0)
    assert math.isclose(metrics.top1_error, 1.0 / 3.0)


"""验证 AP50 会受高置信假阳性排序影响。"""
def test_average_precision_uses_confidence_ranking():
    samples = [
        DetectionSample('fp_high', False, None, [0, 0, 5, 5], 0.95),
        DetectionSample('tp_mid', True, [0, 0, 9, 9], [0, 0, 9, 9], 0.80),
        DetectionSample('tp_low', True, [20, 20, 29, 29], [20, 20, 29, 29], 0.60),
    ]

    ap = average_precision(samples)

    assert math.isclose(ap, 2.0 / 3.0)
