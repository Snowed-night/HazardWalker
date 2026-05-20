"""红色球体离线检测函数。

本文件不依赖 ROS。感知组可以直接用普通 Python 单元测试验证这些函数，
确认算法可用后，再由 `hsv_detector_node.py` 负责接入 ROS topic。
"""

from dataclasses import dataclass


@dataclass
class RedBallDetection2D:
    """图像平面上的红球候选结果。"""

    x_min: int
    y_min: int
    x_max: int
    y_max: int
    confidence: float
    red_pixel_count: int


def rgb_to_hsv_pixel(r, g, b):
    """将单个 RGB 像素转换为 OpenCV 风格 HSV。

    OpenCV 中 H 的范围通常是 [0, 180]，S/V 是 [0, 255]。
    这里手写转换是为了让离线测试不依赖 OpenCV；后续正式版本可以替换为
    cv2.cvtColor 加形态学处理。
    """

    r_f = r / 255.0
    g_f = g / 255.0
    b_f = b / 255.0
    max_c = max(r_f, g_f, b_f)
    min_c = min(r_f, g_f, b_f)
    delta = max_c - min_c

    if delta == 0.0:
        hue = 0.0
    elif max_c == r_f:
        hue = (60.0 * ((g_f - b_f) / delta) + 360.0) % 360.0
    elif max_c == g_f:
        hue = 60.0 * ((b_f - r_f) / delta + 2.0)
    else:
        hue = 60.0 * ((r_f - g_f) / delta + 4.0)

    saturation = 0.0 if max_c == 0.0 else delta / max_c
    value = max_c
    return hue / 2.0, saturation * 255.0, value * 255.0


def is_red_hsv(h, s, v, lower_h_1=0.0, upper_h_1=10.0, lower_h_2=170.0, upper_h_2=180.0,
               min_s=80.0, min_v=80.0):
    """判断 HSV 像素是否属于红色。

    红色在 HSV 色相上跨越 0 度边界，所以需要两段 hue 范围。
    """

    in_first_range = lower_h_1 <= h <= upper_h_1
    in_second_range = lower_h_2 <= h <= upper_h_2
    return (in_first_range or in_second_range) and s >= min_s and v >= min_v


def detect_red_ball_rgb_bytes(data, width, height, step=None, encoding='rgb8',
                              min_area_px=80, min_confidence=0.5):
    """在 RGB/BGR 图像字节中检测红色区域。

    Args:
        data: 图像原始 bytes/bytearray。
        width: 图像宽度。
        height: 图像高度。
        step: 每行字节数，ROS Image 中通常等于 width * 3。
        encoding: 支持 `rgb8` 或 `bgr8`。
        min_area_px: 红色像素数量低于该值时认为没有目标。
        min_confidence: 输出置信度下限。

    Returns:
        RedBallDetection2D 或 None。
    """

    normalized_encoding = encoding.lower()
    if normalized_encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'Unsupported image encoding: {encoding}')

    if step is None:
        step = width * 3

    red_pixels = []
    is_bgr = normalized_encoding == 'bgr8'
    for y in range(height):
        row = y * step
        for x in range(width):
            index = row + x * 3
            if index + 2 >= len(data):
                continue
            c0 = data[index]
            c1 = data[index + 1]
            c2 = data[index + 2]
            r, g, b = (c2, c1, c0) if is_bgr else (c0, c1, c2)
            h, s, v = rgb_to_hsv_pixel(r, g, b)
            if is_red_hsv(h, s, v):
                red_pixels.append((x, y))

    if len(red_pixels) < min_area_px:
        return None

    xs = [p[0] for p in red_pixels]
    ys = [p[1] for p in red_pixels]
    area_ratio = len(red_pixels) / float(width * height)
    confidence = min(1.0, max(float(min_confidence), area_ratio * 50.0))

    return RedBallDetection2D(
        x_min=min(xs),
        y_min=min(ys),
        x_max=max(xs),
        y_max=max(ys),
        confidence=confidence,
        red_pixel_count=len(red_pixels),
    )
