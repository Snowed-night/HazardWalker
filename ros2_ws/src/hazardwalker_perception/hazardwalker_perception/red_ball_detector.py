"""红色球体离线检测函数。

所属组：感知组。
文件作用：
提供不依赖 ROS 的红球检测基础函数。
使用 OpenCV 完成 HSV 分割、连通域提取和圆形度筛选，优先降低红色非球体误检。
当前可输出多个红球候选；兼容旧接口时返回单个最可信候选。
每个候选包含 2D bbox、红色像素数和形状指标。
"""

from dataclasses import dataclass
import math

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - fallback is covered through environments without OpenCV.
    cv2 = None
    np = None


"""图像平面上的红球候选结果。"""
@dataclass
class RedBallDetection2D:

    x_min: int
    y_min: int
    x_max: int
    y_max: int
    confidence: float
    red_pixel_count: int
    circularity: float = 0.0
    aspect_ratio: float = 0.0
    extent: float = 0.0

"""把单个 RGB 像素转为 OpenCV 风格 HSV"""
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

"""判断单个 HSV 像素是否落入红色阈值区间"""
def is_red_hsv(h, s, v, lower_h_1=0.0, upper_h_1=10.0, lower_h_2=170.0, upper_h_2=180.0,
               min_s=80.0, min_v=80.0):
    """判断 HSV 像素是否属于红色。

    这里使用两段 hue 区间，是因为红色跨越 HSV 色相边界。
    """

    in_first_range = lower_h_1 <= h <= upper_h_1
    in_second_range = lower_h_2 <= h <= upper_h_2
    return (in_first_range or in_second_range) and s >= min_s and v >= min_v

"""检测多个红色球体候选"""
def detect_red_balls_rgb_bytes(data, width, height, step=None, encoding='rgb8',
                               min_area_px=80, min_confidence=0.5,
                               min_circularity=0.60, min_aspect_ratio=0.45,
                               min_extent=0.35, max_extent=0.92, max_detections=None):
    """
    Args:
        data: 图像原始 bytes/bytearray。
        step: 每行字节数，ROS Image 中通常等于 width * 3。
        encoding: 支持 `rgb8` 或 `bgr8`。
        min_area_px: 红色像素数量低于该值时认为没有目标。
        min_confidence: 输出置信度下限。
        min_circularity: 轮廓圆形度下限，越高越偏向完整圆形。
        min_aspect_ratio: bbox 短边/长边下限，用于过滤细长红色物体。
        min_extent: 轮廓面积 / bbox 面积下限，用于过滤过碎或过稀疏区域。
        max_extent: 轮廓面积 / bbox 面积上限，用于过滤红色方块等实心矩形。
        max_detections: 最多返回多少个候选，None 表示不限制。

    Returns:
        RedBallDetection2D 列表，按 confidence 从高到低排序。

    说明：
    返回结果只描述 2D 图像范围，不包含真实世界三维坐标。
    当前版本优先压低红色方块和不规则物体误检，因此 70% 以上遮挡目标可能被拒绝。
    """

    normalized_encoding = encoding.lower()
    if normalized_encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'Unsupported image encoding: {encoding}')

    if step is None:
        step = width * 3

    if cv2 is not None and np is not None:
        detections = _detect_red_balls_with_opencv(
            data=data,
            width=width,
            height=height,
            step=step,
            encoding=normalized_encoding,
            min_area_px=min_area_px,
            min_confidence=min_confidence,
            min_circularity=min_circularity,
            min_aspect_ratio=min_aspect_ratio,
            min_extent=min_extent,
            max_extent=max_extent,
        )
        if max_detections is not None:
            return detections[:max_detections]
        return detections

    detection = _detect_red_region_by_pixel_scan(
        data=data,
        width=width,
        height=height,
        step=step,
        encoding=normalized_encoding,
        min_area_px=min_area_px,
        min_confidence=min_confidence,
    )
    return [] if detection is None else [detection]

"""检测单个最可信红色球体候选,保留给现有 ROS 节点和旧测试使用。"""
def detect_red_ball_rgb_bytes(data, width, height, step=None, encoding='rgb8',
                              min_area_px=80, min_confidence=0.5,
                              min_circularity=0.60, min_aspect_ratio=0.45,
                              min_extent=0.35, max_extent=0.92):
    detections = detect_red_balls_rgb_bytes(
        data=data,
        width=width,
        height=height,
        step=step,
        encoding=encoding,
        min_area_px=min_area_px,
        min_confidence=min_confidence,
        min_circularity=min_circularity,
        min_aspect_ratio=min_aspect_ratio,
        min_extent=min_extent,
        max_extent=max_extent,
        max_detections=1,
    )
    return detections[0] if detections else None

"""OpenCV 不可用时的保底像素扫描，只验证颜色，不做形状筛选。"""
def _detect_red_region_by_pixel_scan(data, width, height, step, encoding, min_area_px, min_confidence):

    red_pixels = []
    is_bgr = encoding == 'bgr8'
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

"""把 ROS Image 风格字节数据转换为 OpenCV 可处理的 HxWx3 数组。"""
def _image_bytes_to_array(data, width, height, step):

    image = np.zeros((height, width, 3), dtype=np.uint8)
    for y in range(height):
        row_start = y * step
        row_end = row_start + width * 3
        row = np.frombuffer(data[row_start:row_end], dtype=np.uint8)
        if row.size == width * 3:
            image[y] = row.reshape((width, 3))
    return image

"""用 OpenCV 的 HSV mask、形态学处理和轮廓指标筛选多个红色球体。"""
def _detect_red_balls_with_opencv(data, width, height, step, encoding, min_area_px, min_confidence,
                                  min_circularity, min_aspect_ratio, min_extent, max_extent):


    image = _image_bytes_to_array(data, width, height, step)
    color_code = cv2.COLOR_BGR2HSV if encoding == 'bgr8' else cv2.COLOR_RGB2HSV
    hsv = cv2.cvtColor(image, color_code)

    lower_red_1 = np.array([0, 80, 80], dtype=np.uint8)
    upper_red_1 = np.array([10, 255, 255], dtype=np.uint8)
    lower_red_2 = np.array([170, 80, 80], dtype=np.uint8)
    upper_red_2 = np.array([180, 255, 255], dtype=np.uint8)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red_1, upper_red_1),
        cv2.inRange(hsv, lower_red_2, upper_red_2),
    )

    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections = []

    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area_px:
            continue

        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 0.0:
            continue

        x, y, box_width, box_height = cv2.boundingRect(contour)
        if box_width <= 0 or box_height <= 0:
            continue

        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        aspect_ratio = min(box_width, box_height) / float(max(box_width, box_height))
        extent = area / float(box_width * box_height)

        # 当前版本优先不误检：红色方块通常 extent 接近 1，不规则物体通常圆度或长宽比较低。
        if circularity < min_circularity:
            continue
        if aspect_ratio < min_aspect_ratio:
            continue
        if extent < min_extent or extent > max_extent:
            continue

        red_pixel_count = int(cv2.countNonZero(mask[y:y + box_height, x:x + box_width]))
        shape_score = _score_shape(circularity, aspect_ratio, extent)
        area_score = min(1.0, red_pixel_count / float(max(min_area_px, 1) * 4))
        confidence = min(1.0, max(float(min_confidence), 0.75 * shape_score + 0.25 * area_score))

        detections.append(RedBallDetection2D(
            x_min=int(x),
            y_min=int(y),
            x_max=int(x + box_width - 1),
            y_max=int(y + box_height - 1),
            confidence=confidence,
            red_pixel_count=red_pixel_count,
            circularity=circularity,
            aspect_ratio=aspect_ratio,
            extent=extent,
        ))

    detections.sort(key=lambda item: item.confidence, reverse=True)
    return detections

"""把圆度、bbox 长宽比和面积比例合成 0 到 1 形状分数,用于在多个候选中优先选择更像球体的。"""
def _score_shape(circularity, aspect_ratio, extent):

    circularity_score = max(0.0, min(1.0, circularity))
    aspect_score = max(0.0, min(1.0, aspect_ratio))
    ideal_circle_extent = math.pi / 4.0
    extent_error = abs(extent - ideal_circle_extent)
    extent_score = max(0.0, 1.0 - extent_error / ideal_circle_extent)
    return 0.45 * circularity_score + 0.30 * aspect_score + 0.25 * extent_score
