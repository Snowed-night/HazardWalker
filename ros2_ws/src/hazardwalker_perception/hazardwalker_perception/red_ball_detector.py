"""红色球体离线检测函数。

所属组：感知组。
文件作用：
提供不依赖 ROS 的红球检测基础函数。
使用 OpenCV 完成 HSV 分割、连通域提取、Hough 圆/Watershed 粘连分离和圆形度筛选，优先降低红色非球体误检。
当前可输出多个红球候选；兼容旧接口时返回单个最可信候选。
每个候选包含 2D bbox、红色像素数和形状指标。
后续可在保持 DetectionBackend 接口不变的前提下接入 YOLO、分割模型或点云辅助检测。
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
    is_partial: bool = False
    requires_reobservation: bool = False
    may_be_merged: bool = False
    from_merged_split: bool = False
    quality_reason: str = 'stable_shape'


def is_complete_candidate_for_3d_tracking(detection, image_width, image_height,
                                          edge_margin_px=2):
    """判断二维框是否完整到足以参与球心反推和三维轨迹更新。

    partial、粘连拆分和贴边框的 bbox 都不是完整球直径。它们可以保留为
    主动复查候选，但禁止套用 0.15 m 半径先验，也禁止用错误坐标污染轨迹。
    """

    if (
        bool(getattr(detection, 'requires_reobservation', False))
        or bool(getattr(detection, 'is_partial', False))
        or bool(getattr(detection, 'may_be_merged', False))
        or bool(getattr(detection, 'from_merged_split', False))
    ):
        return False
    margin = max(0, int(edge_margin_px))
    width = max(0, int(image_width))
    height = max(0, int(image_height))
    return not (
        int(detection.x_min) <= margin
        or int(detection.y_min) <= margin
        or int(detection.x_max) >= width - 1 - margin
        or int(detection.y_max) >= height - 1 - margin
    )

"""二维检测后端接口，后续 YOLO/分割模型只要实现 detect 即可接入 ROS 节点。"""
class DetectionBackend:
    name = 'base'

    def detect(self, data, width, height, step=None, encoding='rgb8', **kwargs):
        raise NotImplementedError


"""当前可展示版本使用的 HSV + OpenCV 检测后端。"""
class HsvOpenCvDetectionBackend(DetectionBackend):
    name = 'hsv_opencv'

    def detect(self, data, width, height, step=None, encoding='rgb8', **kwargs):
        return detect_red_balls_rgb_bytes(
            data=data,
            width=width,
            height=height,
            step=step,
            encoding=encoding,
            **kwargs,
        )


"""根据名称创建检测后端，暂时只内置 HSV，后续模型化方案从这里注册。"""
def create_detection_backend(name='hsv_opencv'):
    normalized = (name or 'hsv_opencv').lower()
    if normalized in ('hsv', 'hsv_opencv'):
        return HsvOpenCvDetectionBackend()
    raise ValueError(f'Unsupported detection backend: {name}')


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
                               min_extent=0.35, max_extent=0.92, max_detections=None,
                               split_touching=True, include_partial_candidates=False,
                               partial_min_area_px=20, partial_min_circularity=0.30,
                               partial_min_aspect_ratio=0.30, partial_min_value=50):
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
        split_touching: 是否尝试用距离变换和 watershed 分离粘连红球。
        include_partial_candidates: 严格球形筛选失败时，是否输出只可用于重观察的
            低质量候选；该候选不会进入三维确认。

    Returns:
        RedBallDetection2D 列表，按 confidence 从高到低排序。

    说明：
    返回结果只描述 2D 图像范围，不包含真实世界三维坐标。
    严格球形筛选仍优先压低红色方块和不规则物体误检；高遮挡目标最多输出
    `requires_reobservation` 候选，不能据此直接确认危险源。
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
            split_touching=split_touching,
            include_partial_candidates=include_partial_candidates,
            partial_min_area_px=partial_min_area_px,
            partial_min_circularity=partial_min_circularity,
            partial_min_aspect_ratio=partial_min_aspect_ratio,
            partial_min_value=partial_min_value,
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
                              min_extent=0.35, max_extent=0.92, split_touching=True):
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
        split_touching=split_touching,
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
                                  min_circularity, min_aspect_ratio, min_extent, max_extent,
                                  split_touching, include_partial_candidates,
                                  partial_min_area_px, partial_min_circularity,
                                  partial_min_aspect_ratio, partial_min_value):


    image = _image_bytes_to_array(data, width, height, step)
    color_code = cv2.COLOR_BGR2HSV if encoding == 'bgr8' else cv2.COLOR_RGB2HSV
    hsv = cv2.cvtColor(image, color_code)

    mask = _red_mask_from_hsv(hsv, min_value=80)

    contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections = []

    for contour in contours:
        split_detections = []
        if split_touching:
            split_detections = _split_touching_contour_to_detections(
                contour=contour,
                mask=mask,
                min_area_px=min_area_px,
                min_confidence=min_confidence,
                min_circularity=min_circularity,
                min_aspect_ratio=min_aspect_ratio,
                min_extent=min_extent,
                max_extent=max_extent,
            )
        if len(split_detections) >= 2:
            detections.extend(split_detections)
            continue

        contour_detections = _contour_to_detections(
            contour=contour,
            mask=mask,
            min_area_px=min_area_px,
            min_confidence=min_confidence,
            min_circularity=min_circularity,
            min_aspect_ratio=min_aspect_ratio,
            min_extent=min_extent,
            max_extent=max_extent,
        )
        if contour_detections:
            detections.extend(contour_detections)
            continue

    detections.sort(key=lambda item: item.confidence, reverse=True)
    # 不能因为同帧已经有一个完整红球，就吞掉另一处仅露出一小段的红球。
    # 真实巡检中“完整球 + 门边局部球”很常见：后者必须触发复查，绝不能因为
    # 前者存在而静默漏检。这里始终生成宽松候选，再仅剔除和已有严格框明显重叠的
    # 同一物体，确保候选不会与完整球重复计数。
    if include_partial_candidates:
        relaxed_mask = _red_mask_from_hsv(hsv, min_value=partial_min_value)
        partial_contours, _hierarchy = cv2.findContours(
            relaxed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
        )
        for contour in partial_contours:
            candidate = _contour_to_partial_candidate(
                contour=contour,
                mask=relaxed_mask,
                min_area_px=partial_min_area_px,
                min_circularity=partial_min_circularity,
                min_aspect_ratio=partial_min_aspect_ratio,
            )
            if candidate is not None and not _is_duplicate_partial_candidate(candidate, detections):
                detections.append(candidate)
        detections.sort(key=lambda item: item.confidence, reverse=True)
    return detections


def _red_mask_from_hsv(hsv, min_value):
    """生成红色掩膜；局部候选可使用较低亮度阈值但不会放宽最终确认。"""

    lower_red_1 = np.array([0, 80, int(min_value)], dtype=np.uint8)
    upper_red_1 = np.array([10, 255, 255], dtype=np.uint8)
    lower_red_2 = np.array([170, 80, int(min_value)], dtype=np.uint8)
    upper_red_2 = np.array([180, 255, 255], dtype=np.uint8)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red_1, upper_red_1),
        cv2.inRange(hsv, lower_red_2, upper_red_2),
    )
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

"""把单个轮廓转为检测结果；不符合球体形状时返回空列表。"""
def _contour_to_detections(contour, mask, min_area_px, min_confidence,
                           min_circularity, min_aspect_ratio, min_extent, max_extent):

    area = float(cv2.contourArea(contour))
    if area < min_area_px:
        return []

    perimeter = float(cv2.arcLength(contour, True))
    if perimeter <= 0.0:
        return []

    x, y, box_width, box_height = cv2.boundingRect(contour)
    if box_width <= 0 or box_height <= 0:
        return []

    circularity = 4.0 * math.pi * area / (perimeter * perimeter)
    aspect_ratio = min(box_width, box_height) / float(max(box_width, box_height))
    extent = area / float(box_width * box_height)

    # 当前版本优先不误检：红色方块通常 extent 接近 1，不规则物体通常圆度或长宽比较低。
    if circularity < min_circularity:
        return []
    if aspect_ratio < min_aspect_ratio:
        return []
    if extent < min_extent or extent > max_extent:
        return []

    red_pixel_count = int(cv2.countNonZero(mask[y:y + box_height, x:x + box_width]))
    shape_score = _score_shape(circularity, aspect_ratio, extent)
    area_score = min(1.0, red_pixel_count / float(max(min_area_px, 1) * 4))
    confidence = min(1.0, max(float(min_confidence), 0.75 * shape_score + 0.25 * area_score))

    return [RedBallDetection2D(
        x_min=int(x),
        y_min=int(y),
        x_max=int(x + box_width - 1),
        y_max=int(y + box_height - 1),
        confidence=confidence,
        red_pixel_count=red_pixel_count,
        circularity=circularity,
        aspect_ratio=aspect_ratio,
        extent=extent,
    )]


def _contour_to_partial_candidate(contour, mask, min_area_px, min_circularity,
                                  min_aspect_ratio):
    """将高遮挡或暗光的红色区域转成“待复查”候选，而不是静默漏检。"""

    area = float(cv2.contourArea(contour))
    if area < max(1, int(min_area_px)):
        return None
    perimeter = float(cv2.arcLength(contour, True))
    if perimeter <= 0.0:
        return None
    x, y, box_width, box_height = cv2.boundingRect(contour)
    if box_width <= 0 or box_height <= 0:
        return None
    circularity = 4.0 * math.pi * area / (perimeter * perimeter)
    aspect_ratio = min(box_width, box_height) / float(max(box_width, box_height))
    extent = area / float(box_width * box_height)
    # 低质量候选仍拒绝明显的长条和实心矩形，避免把红色面板大量送去复查。
    if circularity < float(min_circularity) or aspect_ratio < float(min_aspect_ratio):
        return None
    if extent < 0.20 or extent > 0.94:
        return None
    red_pixel_count = int(cv2.countNonZero(mask[y:y + box_height, x:x + box_width]))
    confidence = min(0.49, max(0.10, 0.45 * _score_shape(
        circularity, aspect_ratio, min(extent, math.pi / 4.0),
    )))
    return RedBallDetection2D(
        x_min=int(x), y_min=int(y), x_max=int(x + box_width - 1),
        y_max=int(y + box_height - 1), confidence=confidence,
        red_pixel_count=red_pixel_count, circularity=circularity,
        aspect_ratio=aspect_ratio, extent=extent, is_partial=True,
        requires_reobservation=True, quality_reason='partial_or_low_light',
    )


"""对粘连的红色连通域尝试分裂成多个红球候选。"""
def _split_touching_contour_to_detections(contour, mask, min_area_px, min_confidence,
                                          min_circularity, min_aspect_ratio, min_extent, max_extent):

    area = float(cv2.contourArea(contour))
    if area < min_area_px * 2:
        return []

    x, y, box_width, box_height = cv2.boundingRect(contour)
    if box_width <= 0 or box_height <= 0:
        return []

    # 三球三角团的外接框也可能接近正方形，不能用细长度决定是否允许分裂。
    # 这里统一依赖连接腰部的凸缺陷深度；椭球、圆锥等凸物体仍会被挡住。
    hull_indices = cv2.convexHull(contour, returnPoints=False)
    defects = cv2.convexityDefects(contour, hull_indices) if len(hull_indices) >= 3 else None
    max_defect_depth = 0.0
    if defects is not None:
        max_defect_depth = max(float(item[0][3]) / 256.0 for item in defects)
    normalized_defect_depth = max_defect_depth / float(max(1, min(box_width, box_height)))
    # 圆锥侧视、长椭球和胶囊属于凸轮廓，即使被拉长也不能拆成多球；
    # 相互粘连的球在连接腰部会形成相对于短边足够深的凹陷。
    if normalized_defect_depth < 0.02:
        return []

    roi = mask[y:y + box_height, x:x + box_width]
    if cv2.countNonZero(roi) < min_area_px * 2:
        return []

    hough_detections = _split_touching_contour_by_hough(
        roi=roi,
        offset_x=x,
        offset_y=y,
        min_area_px=min_area_px,
        min_confidence=min_confidence,
        min_circularity=min_circularity,
        min_aspect_ratio=min_aspect_ratio,
        min_extent=min_extent,
        max_extent=max_extent,
    )
    if len(hough_detections) >= 2:
        _mark_split_candidates(hough_detections)
        return hough_detections

    distance = cv2.distanceTransform(roi, cv2.DIST_L2, 5)
    max_distance = float(distance.max())
    if max_distance <= 0.0:
        return []

    sure_foreground = np.uint8(distance > max_distance * 0.45) * 255
    kernel = np.ones((3, 3), dtype=np.uint8)
    sure_foreground = cv2.morphologyEx(sure_foreground, cv2.MORPH_OPEN, kernel)
    marker_count, markers = cv2.connectedComponents(sure_foreground)
    if marker_count <= 2:
        return []

    sure_background = cv2.dilate(roi, kernel, iterations=2)
    unknown = cv2.subtract(sure_background, sure_foreground)
    markers = markers + 1
    markers[unknown == 255] = 0
    watershed_image = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
    markers = cv2.watershed(watershed_image, markers)

    split_detections = []
    for marker_id in range(2, marker_count + 1):
        segment = np.uint8(markers == marker_id) * 255
        segment = cv2.bitwise_and(segment, roi)
        segment_contours, _hierarchy = cv2.findContours(segment, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for segment_contour in segment_contours:
            shifted = segment_contour.copy()
            shifted[:, :, 0] += x
            shifted[:, :, 1] += y
            split_detections.extend(_contour_to_detections(
                contour=shifted,
                mask=mask,
                min_area_px=max(10, int(min_area_px * 0.6)),
                min_confidence=min_confidence,
                min_circularity=max(0.35, min_circularity * 0.75),
                min_aspect_ratio=max(0.35, min_aspect_ratio * 0.8),
                min_extent=min_extent,
                max_extent=max_extent,
            ))

    split_detections = _deduplicate_detections(split_detections)
    _mark_split_candidates(split_detections)
    return split_detections


def _mark_split_candidates(detections):
    """同一连通域拆出的圆必须等待独立轮廓视角，不能直接作为确认正证据。"""

    for detection in detections:
        detection.from_merged_split = True
        detection.may_be_merged = True
        detection.requires_reobservation = True
        detection.quality_reason = 'split_candidate_requires_independent_view'


"""在红色 ROI 内用 Hough 圆检测辅助分离粘连红球。"""
def _split_touching_contour_by_hough(roi, offset_x, offset_y, min_area_px, min_confidence,
                                     min_circularity, min_aspect_ratio, min_extent, max_extent):

    blurred = cv2.GaussianBlur(roi, (7, 7), 1.5)
    min_radius = max(5, int(math.sqrt(min_area_px / math.pi) * 0.7))
    max_radius = max(min_radius + 2, int(min(roi.shape[0], roi.shape[1]) * 0.55))
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=max(8, min_radius * 2),
        param1=80,
        param2=18,
        minRadius=min_radius,
        maxRadius=max_radius,
    )
    if circles is None:
        return []

    detections = []
    covered_red = np.zeros_like(roi)
    # 大小球粘连时，小球半径可能只有连通域短边的 10%--20%。凸缺陷、
    # 圆内红色填充率和总覆盖率已经共同抑制椭球/圆锥，因此不能再用 25%
    # 的全局短边门槛系统性删除小球。所有拆分结果仍只作为待复查候选。
    minimum_supported_radius = max(float(min_radius), min(roi.shape[0], roi.shape[1]) * 0.10)
    for cx, cy, radius in np.round(circles[0]).astype(int):
        if cx < 0 or cy < 0 or cx >= roi.shape[1] or cy >= roi.shape[0] or radius <= 0:
            continue
        circle_mask = np.zeros_like(roi)
        cv2.circle(circle_mask, (int(cx), int(cy)), int(radius), 255, thickness=-1)
        segment = cv2.bitwise_and(roi, circle_mask)
        red_pixel_count = int(cv2.countNonZero(segment))
        if red_pixel_count < max(10, int(min_area_px * 0.6)):
            continue
        ideal_area = math.pi * float(radius) * float(radius)
        # 椭球/胶囊常在两端产生很小的“帽状圆”，但这些圆无法覆盖主体。
        # 真正粘连球的圆半径接近连通域短边的一半，且圆内红色填充充分。
        if radius < minimum_supported_radius:
            continue
        if red_pixel_count / max(ideal_area, 1.0) < 0.90:
            continue

        x0 = int(max(0, cx - radius) + offset_x)
        y0 = int(max(0, cy - radius) + offset_y)
        x1 = int(min(roi.shape[1] - 1, cx + radius) + offset_x)
        y1 = int(min(roi.shape[0] - 1, cy + radius) + offset_y)
        box_width = x1 - x0 + 1
        box_height = y1 - y0 + 1
        if box_width <= 0 or box_height <= 0:
            continue
        circularity = 1.0
        aspect_ratio = min(box_width, box_height) / float(max(box_width, box_height))
        extent = min(max_extent, ideal_area / float(box_width * box_height))
        if aspect_ratio < max(0.35, min_aspect_ratio * 0.8):
            continue
        if extent < min_extent:
            continue

        shape_score = _score_shape(circularity, aspect_ratio, extent)
        area_score = min(1.0, red_pixel_count / float(max(min_area_px, 1) * 4))
        confidence = min(1.0, max(float(min_confidence), 0.75 * shape_score + 0.25 * area_score))
        detections.append(RedBallDetection2D(
            x_min=x0,
            y_min=y0,
            x_max=x1,
            y_max=y1,
            confidence=confidence,
            red_pixel_count=red_pixel_count,
            circularity=max(min_circularity, circularity),
            aspect_ratio=aspect_ratio,
            extent=extent,
        ))
        covered_red = cv2.bitwise_or(covered_red, segment)

    detections = _deduplicate_detections(detections)
    if len(detections) < 2:
        return detections
    coverage = cv2.countNonZero(covered_red) / float(max(cv2.countNonZero(roi), 1))
    if coverage < 0.82:
        return []
    return detections


"""去掉分裂过程产生的重复 bbox。"""
def _deduplicate_detections(detections):
    """去重 Hough/分水岭的嵌套候选，防止单个大球被重复计数。"""

    unique = []
    for detection in sorted(detections, key=lambda item: item.red_pixel_count, reverse=True):
        duplicate = False
        for existing in unique:
            # Hough 在单个较大球上偶尔会同时给出“整球圆”和“内嵌小圆”。
            # 两框 IoU 约为 0.47 时仍是同一球，原 0.6 阈值会造成重复计数；
            # 相切双球的 IoU 明显更低，仍保留给后续分离逻辑。
            if (_bbox_iou_detection(detection, existing) > 0.45
                    or _bbox_contains_detection(detection, existing, min_ratio=0.80)):
                duplicate = True
                break
        if not duplicate:
            unique.append(detection)
    return unique


def _is_duplicate_partial_candidate(candidate, detections):
    """只去掉与已有严格/分裂框代表同一红色区域的宽松候选。

    这里不能使用“画面已有严格候选”这样的全局条件，否则远处或遮挡后的另一个
    红球会被静默忽略。要求候选框实际高度重叠或近乎被包含，才认定为同一物体。
    """

    for existing in detections:
        if (_bbox_iou_detection(candidate, existing) >= 0.35
                or _bbox_contains_detection(candidate, existing, min_ratio=0.75)
                or _bbox_contains_detection(existing, candidate, min_ratio=0.75)):
            return True
    return False


def _bbox_contains_detection(first, second, min_ratio=0.80):
    """判断较小框是否大比例被另一个框包含，用于消除嵌套圆重复。"""

    x0 = max(first.x_min, second.x_min)
    y0 = max(first.y_min, second.y_min)
    x1 = min(first.x_max, second.x_max)
    y1 = min(first.y_max, second.y_max)
    if x1 < x0 or y1 < y0:
        return False
    intersection = float((x1 - x0 + 1) * (y1 - y0 + 1))
    first_area = float((first.x_max - first.x_min + 1) * (first.y_max - first.y_min + 1))
    second_area = float((second.x_max - second.x_min + 1) * (second.y_max - second.y_min + 1))
    return intersection / max(min(first_area, second_area), 1.0) >= float(min_ratio)


"""计算两个检测框的 IoU，用于分裂结果去重。"""
def _bbox_iou_detection(a, b):
    x0 = max(a.x_min, b.x_min)
    y0 = max(a.y_min, b.y_min)
    x1 = min(a.x_max, b.x_max)
    y1 = min(a.y_max, b.y_max)
    if x1 < x0 or y1 < y0:
        return 0.0
    intersection = float((x1 - x0 + 1) * (y1 - y0 + 1))
    area_a = float((a.x_max - a.x_min + 1) * (a.y_max - a.y_min + 1))
    area_b = float((b.x_max - b.x_min + 1) * (b.y_max - b.y_min + 1))
    return intersection / max(area_a + area_b - intersection, 1.0)


"""把圆度、bbox 长宽比和面积比例合成 0 到 1 形状分数,用于在多个候选中优先选择更像球体的。"""
def _score_shape(circularity, aspect_ratio, extent):

    circularity_score = max(0.0, min(1.0, circularity))
    aspect_score = max(0.0, min(1.0, aspect_ratio))
    ideal_circle_extent = math.pi / 4.0
    extent_error = abs(extent - ideal_circle_extent)
    extent_score = max(0.0, 1.0 - extent_error / ideal_circle_extent)
    return 0.45 * circularity_score + 0.30 * aspect_score + 0.25 * extent_score
