"""红球检测离线测试。

所属组：感知组 / 测试组。
文件作用：
用人工构造的 RGB 图像验证 `red_ball_detector.py`。
不依赖 ROS、Gazebo 或真实相机，OpenCV 可用时验证形状筛选。
覆盖颜色阈值、面积过滤、形状过滤、BGR 输入和遮挡边界。
"""
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_perception'))

from hazardwalker_perception.red_ball_detector import (
    detect_red_ball_rgb_bytes,
    detect_red_balls_rgb_bytes,
    rgb_to_hsv_pixel,
)

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


RED = (230, 20, 20)
DARK_RED = (70, 5, 5)
LOW_SATURATION_RED = (150, 105, 105)
BACKGROUND = (30, 30, 30)

"""生成一张纯 RGB 字节图像，用作离线测试输入。"""
def make_rgb_image(width, height, background=(30, 30, 30)):
    data = bytearray(width * height * 3)
    for i in range(0, len(data), 3):
        data[i] = background[0]
        data[i + 1] = background[1]
        data[i + 2] = background[2]
    return data

"""在图像字节中画红色矩形，模拟相机看到红色目标。"""
def draw_red_square(data, width, x_min, y_min, x_max, y_max):
    for y in range(y_min, y_max + 1):
        for x in range(x_min, x_max + 1):
            index = (y * width + x) * 3
            data[index] = 230
            data[index + 1] = 20
            data[index + 2] = 20

"""OpenCV 不可用时跳过形状测试，保留基础像素级测试可运行。"""
def require_opencv():
    return cv2 is not None and np is not None

"""生成红色圆形目标图像，可用背景色从右侧遮挡指定比例。"""
def draw_circle_image(width=80, height=80, color=RED, occlusion_ratio=0.0):
    image = np.full((height, width, 3), BACKGROUND, dtype=np.uint8)
    center = (width // 2, height // 2)
    radius = min(width, height) // 4
    cv2.circle(image, center, radius, color, thickness=-1)
    if occlusion_ratio > 0.0:
        occlusion_width = int((radius * 2 + 1) * occlusion_ratio)
        x_start = center[0] + radius - occlusion_width + 1
        cv2.rectangle(image, (x_start, center[1] - radius - 1),
                      (center[0] + radius + 1, center[1] + radius + 1), BACKGROUND, thickness=-1)
    return bytearray(image.tobytes())

"""生成红色正方形目标图像，用作红色立方体投影的误检对照。"""
def draw_square_image(width=80, height=80, color=RED):
    image = np.full((height, width, 3), BACKGROUND, dtype=np.uint8)
    cv2.rectangle(image, (24, 24), (56, 56), color, thickness=-1)
    return bytearray(image.tobytes())

"""生成两类红色不规则目标图像，用作非球体误检对照。"""
def draw_irregular_image(kind, width=80, height=80, color=RED):
    image = np.full((height, width, 3), BACKGROUND, dtype=np.uint8)
    if kind == 'triangle':
        points = np.array([[18, 58], [42, 18], [62, 55]], dtype=np.int32)
    elif kind == 'elongated':
        points = np.array([[14, 35], [64, 27], [68, 40], [20, 52]], dtype=np.int32)
    else:
        raise ValueError(f'Unknown irregular shape: {kind}')
    cv2.fillPoly(image, [points], color)
    return bytearray(image.tobytes())

"""验证典型红色像素会落在 HSV 红色 hue 区间内。"""
def test_rgb_to_hsv_pixel_detects_red_hue():
    h, s, v = rgb_to_hsv_pixel(230, 20, 20)
    assert h <= 10.0 or h >= 170.0
    assert s > 80.0
    assert v > 80.0

"""验证大面积红色圆形目标能输出合理 2D 包围框。"""
def test_detect_red_ball_rgb_bytes_returns_bbox():
    if not require_opencv():
        return

    width = 80
    height = 80
    data = draw_circle_image(width, height)

    detection = detect_red_ball_rgb_bytes(data, width, height, min_area_px=20)

    assert detection is not None
    assert 18 <= detection.x_min <= 22
    assert 18 <= detection.y_min <= 22
    assert 58 <= detection.x_max <= 62
    assert 58 <= detection.y_max <= 62
    assert detection.confidence >= 0.5

"""验证小红点噪声会被面积阈值过滤掉。"""
def test_detect_red_ball_rgb_bytes_ignores_small_noise():

    width = 40
    height = 30
    data = make_rgb_image(width, height)
    draw_red_square(data, width, 1, 1, 2, 2)

    detection = detect_red_ball_rgb_bytes(data, width, height, min_area_px=20)

    assert detection is None

"""验证 bgr8 输入也能检测红色圆形目标。"""
def test_detect_red_ball_bgr8_returns_bbox():

    if not require_opencv():
        return

    width = 80
    height = 80
    rgb_data = draw_circle_image(width, height)
    image = np.frombuffer(rgb_data, dtype=np.uint8).reshape((height, width, 3))
    bgr_data = bytearray(image[:, :, ::-1].tobytes())

    detection = detect_red_ball_rgb_bytes(bgr_data, width, height, encoding='bgr8', min_area_px=20)

    assert detection is not None
    assert detection.confidence >= 0.5

"""验证低亮度暗红色目标会被 HSV value 阈值过滤。"""
def test_dark_red_low_value_is_rejected_for_all_shapes():
    if not require_opencv():
        return

    width = 80
    height = 80
    samples = [
        draw_circle_image(width, height, color=DARK_RED),
        draw_square_image(width, height, color=DARK_RED),
        draw_irregular_image('triangle', width, height, color=DARK_RED),
        draw_irregular_image('elongated', width, height, color=DARK_RED),
    ]

    for data in samples:
        detection = detect_red_ball_rgb_bytes(data, width, height, min_area_px=20)
        assert detection is None

"""验证低饱和度偏灰红目标会被 HSV saturation 阈值过滤。"""
def test_low_saturation_red_is_rejected_for_all_shapes():
    if not require_opencv():
        return

    width = 80
    height = 80
    samples = [
        draw_circle_image(width, height, color=LOW_SATURATION_RED),
        draw_square_image(width, height, color=LOW_SATURATION_RED),
        draw_irregular_image('triangle', width, height, color=LOW_SATURATION_RED),
        draw_irregular_image('elongated', width, height, color=LOW_SATURATION_RED),
    ]

    for data in samples:
        detection = detect_red_ball_rgb_bytes(data, width, height, min_area_px=20)
        assert detection is None

"""验证红色方块和两类不规则物体会作为非球体误检被过滤。"""
def test_red_non_sphere_shapes_are_rejected():
    if not require_opencv():
        return

    width = 80
    height = 80
    samples = [
        draw_square_image(width, height),
        draw_irregular_image('triangle', width, height),
        draw_irregular_image('elongated', width, height),
    ]

    for data in samples:
        detection = detect_red_ball_rgb_bytes(data, width, height, min_area_px=20)
        assert detection is None

"""验证 10% 到 50% 遮挡红球可检测，70% 遮挡按保守策略拒绝。"""
def test_partially_occluded_red_balls_keep_expected_detection_boundary():
    if not require_opencv():
        return

    width = 80
    height = 80
    expected_detected = {
        0.10: True,
        0.20: True,
        0.30: True,
        0.50: True,
        0.70: False,
    }

    for occlusion_ratio, should_detect in expected_detected.items():
        data = draw_circle_image(width, height, occlusion_ratio=occlusion_ratio)
        detection = detect_red_ball_rgb_bytes(data, width, height, min_area_px=20)
        if should_detect:
            assert detection is not None, f'occlusion_ratio={occlusion_ratio}'
        else:
            assert detection is None, f'occlusion_ratio={occlusion_ratio}'


"""验证高遮挡红球不会放宽为最终检出，但会输出触发重观察的候选。"""
def test_heavily_occluded_red_ball_emits_reobservation_candidate():
    if not require_opencv():
        return

    data = draw_circle_image(80, 80, occlusion_ratio=0.70)
    detections = detect_red_balls_rgb_bytes(
        data, 80, 80, min_area_px=20, include_partial_candidates=True,
    )

    assert len(detections) == 1
    assert detections[0].is_partial is True
    assert detections[0].requires_reobservation is True
    assert detections[0].confidence < 0.5


"""验证只露出约 10% 的球面仍输出不可确认的重观察候选。"""
def test_extremely_occluded_red_ball_keeps_reobservation_candidate():
    if not require_opencv():
        return

    data = draw_circle_image(100, 100, occlusion_ratio=0.90)
    detections = detect_red_balls_rgb_bytes(
        data, 100, 100, min_area_px=20, include_partial_candidates=True,
        partial_min_circularity=0.18, partial_min_aspect_ratio=0.12,
    )

    assert len(detections) == 1
    assert detections[0].is_partial is True
    assert detections[0].requires_reobservation is True




"""验证暗红目标仅以低质量候选形式出现，不绕过严格 HSV 最终判定。"""
def test_dim_red_ball_emits_reobservation_candidate_only():
    if not require_opencv():
        return

    data = draw_circle_image(80, 80, color=DARK_RED)
    detections = detect_red_balls_rgb_bytes(
        data, 80, 80, min_area_px=20, include_partial_candidates=True,
    )

    assert len(detections) == 1
    assert detections[0].requires_reobservation is True
    assert detections[0].quality_reason == 'partial_or_low_light'

"""验证多个分离红球会输出多个候选，且旧接口仍只返回最高置信度候选。"""
def test_detect_red_balls_rgb_bytes_returns_multiple_candidates():
    if not require_opencv():
        return

    width = 140
    height = 80
    image = np.full((height, width, 3), BACKGROUND, dtype=np.uint8)
    cv2.circle(image, (38, 40), 18, RED, thickness=-1)
    cv2.circle(image, (100, 40), 18, RED, thickness=-1)
    data = bytearray(image.tobytes())

    detections = detect_red_balls_rgb_bytes(data, width, height, min_area_px=80)
    detection = detect_red_ball_rgb_bytes(data, width, height, min_area_px=80)

    assert len(detections) == 2
    assert detection is not None
    assert detections[0].confidence >= detections[1].confidence


"""验证两个轻度粘连红球会通过距离变换和 watershed 分裂为多个候选。"""
def test_touching_red_balls_can_be_split_into_multiple_candidates():
    if not require_opencv():
        return

    width = 150
    height = 100
    image = np.full((height, width, 3), BACKGROUND, dtype=np.uint8)
    cv2.circle(image, (62, 50), 28, RED, thickness=-1)
    cv2.circle(image, (88, 50), 28, RED, thickness=-1)
    data = bytearray(image.tobytes())

    detections = detect_red_balls_rgb_bytes(data, width, height, min_area_px=80)

    assert len(detections) >= 2
    centers = sorted((item.x_min + item.x_max) / 2.0 for item in detections[:2])
    assert centers[1] - centers[0] > 15.0
    assert all(item.from_merged_split for item in detections[:2])
    assert all(item.requires_reobservation for item in detections[:2])


def test_three_ball_triangle_blob_can_be_split_despite_near_square_bbox():
    if not require_opencv():
        return
    image = np.full((180, 220, 3), BACKGROUND, dtype=np.uint8)
    for center in ((80, 105), (140, 105), (110, 58)):
        cv2.circle(image, center, 36, RED, thickness=-1)

    detections = detect_red_balls_rgb_bytes(
        bytearray(image.tobytes()), 220, 180, min_area_px=80,
    )

    assert len(detections) >= 3


def test_dumbbell_lobes_are_split_candidates_not_confirmable_balls():
    if not require_opencv():
        return
    image = np.full((160, 240, 3), BACKGROUND, dtype=np.uint8)
    cv2.circle(image, (80, 80), 34, RED, thickness=-1)
    cv2.circle(image, (160, 80), 34, RED, thickness=-1)
    cv2.rectangle(image, (80, 72), (160, 88), RED, thickness=-1)

    detections = detect_red_balls_rgb_bytes(
        bytearray(image.tobytes()), 240, 160, min_area_px=80,
    )

    assert len(detections) >= 2
    assert all(item.from_merged_split for item in detections)
    assert all(item.requires_reobservation for item in detections)


"""单个大球的 Hough 内嵌小圆不能导致重复报球。"""
def test_single_large_red_ball_is_not_duplicated_by_hough_split():
    if not require_opencv():
        return

    width = 220
    height = 180
    image = np.full((height, width, 3), BACKGROUND, dtype=np.uint8)
    cv2.circle(image, (110, 90), 62, RED, thickness=-1)

    detections = detect_red_balls_rgb_bytes(bytearray(image.tobytes()), width, height, min_area_px=80)

    assert len(detections) == 1


def test_single_red_ellipse_is_not_hough_split_into_fake_balls():
    """凸椭球投影不能被 Hough 拆成多个伪球候选。"""

    image = np.zeros((220, 300, 3), dtype=np.uint8)
    cv2.ellipse(image, (150, 110), (78, 40), 0, 0, 360, (255, 0, 0), thickness=-1)

    detections = detect_red_balls_rgb_bytes(
        image.tobytes(), width=300, height=220, step=900, encoding='rgb8',
        min_area_px=80, min_confidence=0.5, split_touching=True,
    )

    assert len(detections) <= 1


def test_single_near_round_vertical_ellipse_is_not_split_into_two_balls():
    if not require_opencv():
        return
    image = np.zeros((240, 300, 3), dtype=np.uint8)
    cv2.ellipse(image, (150, 120), (60, 80), 0, 0, 360, (255, 0, 0), thickness=-1)

    detections = detect_red_balls_rgb_bytes(
        image.tobytes(), width=300, height=240,
        min_area_px=80, min_confidence=0.5, split_touching=True,
    )

    assert len(detections) <= 1


def test_convex_red_triangle_is_not_split_into_fake_balls():
    if not require_opencv():
        return
    image = np.zeros((240, 300, 3), dtype=np.uint8)
    cv2.fillPoly(
        image,
        [np.array([[150, 25], [75, 205], [225, 205]], dtype=np.int32)],
        (255, 0, 0),
    )

    detections = detect_red_balls_rgb_bytes(
        image.tobytes(), width=300, height=240,
        min_area_px=80, min_confidence=0.5, split_touching=True,
    )

    assert len(detections) <= 1
