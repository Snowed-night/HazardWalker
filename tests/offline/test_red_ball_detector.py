import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_perception'))

from hazardwalker_perception.red_ball_detector import detect_red_ball_rgb_bytes, rgb_to_hsv_pixel


def make_rgb_image(width, height, background=(30, 30, 30)):
    data = bytearray(width * height * 3)
    for i in range(0, len(data), 3):
        data[i] = background[0]
        data[i + 1] = background[1]
        data[i + 2] = background[2]
    return data


def draw_red_square(data, width, x_min, y_min, x_max, y_max):
    for y in range(y_min, y_max + 1):
        for x in range(x_min, x_max + 1):
            index = (y * width + x) * 3
            data[index] = 230
            data[index + 1] = 20
            data[index + 2] = 20


def test_rgb_to_hsv_pixel_detects_red_hue():
    h, s, v = rgb_to_hsv_pixel(230, 20, 20)
    assert h <= 10.0 or h >= 170.0
    assert s > 80.0
    assert v > 80.0


def test_detect_red_ball_rgb_bytes_returns_bbox():
    width = 40
    height = 30
    data = make_rgb_image(width, height)
    draw_red_square(data, width, 10, 8, 20, 18)

    detection = detect_red_ball_rgb_bytes(data, width, height, min_area_px=20)

    assert detection is not None
    assert detection.x_min == 10
    assert detection.y_min == 8
    assert detection.x_max == 20
    assert detection.y_max == 18
    assert detection.confidence >= 0.5


def test_detect_red_ball_rgb_bytes_ignores_small_noise():
    width = 40
    height = 30
    data = make_rgb_image(width, height)
    draw_red_square(data, width, 1, 1, 2, 2)

    detection = detect_red_ball_rgb_bytes(data, width, height, min_area_px=20)

    assert detection is None
