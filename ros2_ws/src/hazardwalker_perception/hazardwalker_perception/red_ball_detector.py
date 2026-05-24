"""红色球体离线检测函数。

所属组：感知组。
文件作用：
- 提供不依赖 ROS 的红球检测基础函数。
- 让感知组先用普通 Python 数据构造单测验证颜色阈值、像素筛选和 bbox 输出。

当前函数职责：
- `rgb_to_hsv_pixel`：把单个 RGB 像素转为 OpenCV 风格 HSV，供阈值判断使用。
- `is_red_hsv`：判断单个 HSV 像素是否落入红色阈值区间。
- `detect_red_ball_rgb_bytes`：扫描整张 RGB/BGR 图像字节，输出红色区域的 2D 包围框和像素计数。

后续扩展方式：
- 先把这里的像素级检测保留为纯函数，再在 `hsv_detector_node.py` 中接入 ROS Image。
- 当需要真实定位时，再新增 `localize_hazard(...)` 或同类函数，输入 `CameraInfo`、`PointCloud2`、TF 和检测框，输出三维坐标。
- 当需要更稳的检测时，可在这里增加形态学滤波、连通域筛选和圆形度评估，但保持函数接口稳定。

验证方式：
- 用 `tests/offline/test_red_ball_detector.py` 构造纯 RGB 字节图像。
- 验证红球能输出 bbox，随机小噪声不会被误检。
- 验证 BGR 输入和红色 HSV 边界值处理正确。
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

    输出用于后面的红色阈值判断，不做整图处理。
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

    这里使用两段 hue 区间，是因为红色跨越 HSV 色相边界。
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

    说明：
    - 返回结果只描述 2D 图像范围，不包含真实世界三维坐标。
    - 如果后续改成连通域或轮廓法，这个函数仍应保留同样的输入输出语义。
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
