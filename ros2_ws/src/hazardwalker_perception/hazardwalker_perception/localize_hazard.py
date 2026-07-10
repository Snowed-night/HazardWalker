"""危险源三维定位纯函数。

所属组：感知组。
文件作用：
根据 2D 检测框、相机内参和深度信息估计危险源三维坐标。
当前实现边界：
只处理针孔相机反投影、bbox ROI 深度采样和通用刚体变换，不直接依赖 ROS 消息类型。
后续 ROS 节点需要把 CameraInfo、Depth Image/PointCloud2 和 TF 转成这里的纯 Python 数据结构。
验证方式：
使用 tests/offline/test_localize_hazard.py 构造相机内参、深度图和刚体变换离线测试。
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CameraIntrinsics:
    """针孔相机内参，单位使用像素。"""

    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class Point3D:
    """三维点，默认约定单位为米。"""

    x: float
    y: float
    z: float


@dataclass(frozen=True)
class RigidTransform3D:
    """从源坐标系到目标坐标系的刚体变换。"""

    translation: Point3D
    rotation: tuple


@dataclass(frozen=True)
class HazardLocalization3D:
    """单个 2D 检测框对应的三维定位结果。"""

    position: Point3D
    frame_id: str
    depth_m: float
    pixel_u: float
    pixel_v: float
    points_used: int = 1


@dataclass(frozen=True)
class DepthShapeEvidence:
    """目标 ROI 的深度曲率证据。

    球面中心比边缘更靠近相机，平面板和圆柱正面则近似等深。该证据只用于
    抑制明显平面误报；深度稀疏时返回 unknown，交给多视角复查而不是误杀目标。
    """

    status: str
    center_depth_m: float | None
    outer_depth_m: float | None
    curvature_m: float | None
    center_points: int
    outer_points: int


"""从 ROS CameraInfo 风格的 K 矩阵提取针孔相机内参。"""
def camera_intrinsics_from_k(k):
    if len(k) < 6:
        raise ValueError('CameraInfo K must contain at least 6 values.')
    return CameraIntrinsics(
        fx=float(k[0]),
        fy=float(k[4]),
        cx=float(k[2]),
        cy=float(k[5]),
    )


"""返回 bbox 中心像素坐标，bbox 可为 dict 或带 x_min 等属性的对象。"""
def bbox_center_pixel(bbox):
    x_min, y_min, x_max, y_max = _read_bbox(bbox)
    return (x_min + x_max) / 2.0, (y_min + y_max) / 2.0


"""把像素点和深度反投影到相机坐标系。"""
def pixel_to_camera_point(pixel_u, pixel_v, depth_m, intrinsics):
    depth = float(depth_m)
    if not math.isfinite(depth) or depth <= 0.0:
        raise ValueError('Depth must be a finite positive value in meters.')
    if intrinsics.fx <= 0.0 or intrinsics.fy <= 0.0:
        raise ValueError('Camera focal length must be positive.')

    x = (float(pixel_u) - intrinsics.cx) * depth / intrinsics.fx
    y = (float(pixel_v) - intrinsics.cy) * depth / intrinsics.fy
    return Point3D(x=x, y=y, z=depth)


"""对三维点应用刚体变换。"""
def transform_point(point, transform):
    rotation = transform.rotation
    if len(rotation) != 3 or any(len(row) != 3 for row in rotation):
        raise ValueError('Rotation must be a 3x3 matrix.')

    x = (
        rotation[0][0] * point.x +
        rotation[0][1] * point.y +
        rotation[0][2] * point.z +
        transform.translation.x
    )
    y = (
        rotation[1][0] * point.x +
        rotation[1][1] * point.y +
        rotation[1][2] * point.z +
        transform.translation.y
    )
    z = (
        rotation[2][0] * point.x +
        rotation[2][1] * point.y +
        rotation[2][2] * point.z +
        transform.translation.z
    )
    return Point3D(x=x, y=y, z=z)


"""构造仅绕 z 轴旋转的平面位姿变换，方便离线测试 odom/base_link 场景。"""
def make_yaw_transform(x=0.0, y=0.0, z=0.0, yaw_rad=0.0):
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return RigidTransform3D(
        translation=Point3D(float(x), float(y), float(z)),
        rotation=(
            (cos_yaw, -sin_yaw, 0.0),
            (sin_yaw, cos_yaw, 0.0),
            (0.0, 0.0, 1.0),
        ),
    )


"""从 bbox 周围的深度图 ROI 中取有效深度中位数。"""
def estimate_depth_from_bbox(depth_image, bbox, padding_px=0, max_depth_m=20.0, min_points=5):
    height = len(depth_image)
    if height <= 0:
        return None, 0
    width = len(depth_image[0])
    if width <= 0:
        return None, 0

    x_min, y_min, x_max, y_max = _read_bbox(bbox)
    pad = max(0, int(padding_px))
    x0 = max(0, int(math.floor(x_min)) - pad)
    y0 = max(0, int(math.floor(y_min)) - pad)
    x1 = min(width - 1, int(math.ceil(x_max)) + pad)
    y1 = min(height - 1, int(math.ceil(y_max)) + pad)

    values = []
    for y in range(y0, y1 + 1):
        row = depth_image[y]
        for x in range(x0, x1 + 1):
            value = float(row[x])
            if math.isfinite(value) and 0.0 < value <= max_depth_m:
                values.append(value)

    if len(values) < int(min_points):
        return None, len(values)

    values.sort()
    mid = len(values) // 2
    if len(values) % 2 == 1:
        return values[mid], len(values)
    return (values[mid - 1] + values[mid]) / 2.0, len(values)


def evaluate_sphere_depth_shape(depth_image, bbox, max_depth_m=20.0,
                                min_points_per_region=8,
                                min_curvature_m=0.008):
    """用 bbox 内椭圆的中心/外环深度差区分球面与近似平面。

    返回 ``spherical``、``flat`` 或 ``unknown``。这是保守过滤：只有中心与
    外环都有足够有效点且深度差很小时才判为 ``flat``；遮挡、过小目标和深度
    缺失均不作负判定，仍可进入主动重观察流程。
    """

    height = len(depth_image)
    if height <= 0 or len(depth_image[0]) <= 0:
        return DepthShapeEvidence('unknown', None, None, None, 0, 0)
    width = len(depth_image[0])
    x_min, y_min, x_max, y_max = _read_bbox(bbox)
    x0 = max(0, int(math.floor(x_min)))
    y0 = max(0, int(math.floor(y_min)))
    x1 = min(width - 1, int(math.ceil(x_max)))
    y1 = min(height - 1, int(math.ceil(y_max)))
    radius_x = (x1 - x0 + 1) / 2.0
    radius_y = (y1 - y0 + 1) / 2.0
    if radius_x < 2.0 or radius_y < 2.0:
        return DepthShapeEvidence('unknown', None, None, None, 0, 0)
    center_x = (x0 + x1) / 2.0
    center_y = (y0 + y1) / 2.0
    center_values = []
    outer_values = []
    for y in range(y0, y1 + 1):
        row = depth_image[y]
        for x in range(x0, x1 + 1):
            radial = math.sqrt(((x - center_x) / radius_x) ** 2 + ((y - center_y) / radius_y) ** 2)
            if radial > 0.90:
                continue
            value = float(row[x])
            if not math.isfinite(value) or value <= 0.0 or value > max_depth_m:
                continue
            if radial <= 0.35:
                center_values.append(value)
            elif 0.60 <= radial <= 0.88:
                outer_values.append(value)
    required = max(1, int(min_points_per_region))
    if len(center_values) < required or len(outer_values) < required:
        return DepthShapeEvidence(
            'unknown', _median(center_values), _median(outer_values), None,
            len(center_values), len(outer_values),
        )
    center_depth = _median(center_values)
    outer_depth = _median(outer_values)
    curvature = outer_depth - center_depth
    status = 'spherical' if curvature >= float(min_curvature_m) else 'flat'
    return DepthShapeEvidence(
        status, center_depth, outer_depth, curvature,
        len(center_values), len(outer_values),
    )


"""由完整圆形投影和已知球半径反推球心的相机 z 深度。"""
def estimate_sphere_center_depth_from_bbox(bbox, intrinsics, image_width, image_height,
                                           sphere_radius_m, min_radius_px=3.0):
    radius_m = float(sphere_radius_m)
    if radius_m <= 0.0 or image_width <= 0 or image_height <= 0:
        return None

    x_min, y_min, x_max, y_max = _read_bbox(bbox)
    # 边框截断会显著缩小表观半径，只能交给深度 ROI 的保守回退路径。
    if x_min <= 0.0 or y_min <= 0.0 or x_max >= image_width - 1 or y_max >= image_height - 1:
        return None

    radius_x = (x_max - x_min) / 2.0
    radius_y = (y_max - y_min) / 2.0
    if min(radius_x, radius_y) < float(min_radius_px):
        return None

    # 对光轴附近的球，投影半径 r=fR/sqrt(z²-R²)，反解 z。
    estimates = []
    if intrinsics.fx > 0.0:
        estimates.append(math.sqrt((intrinsics.fx * radius_m / radius_x) ** 2 + radius_m ** 2))
    if intrinsics.fy > 0.0:
        estimates.append(math.sqrt((intrinsics.fy * radius_m / radius_y) ** 2 + radius_m ** 2))
    if not estimates:
        return None
    return sum(estimates) / len(estimates)


"""使用 bbox 中心像素和深度估计危险源三维坐标。"""
def localize_bbox_with_depth(bbox, intrinsics, depth_m, camera_to_output=None, output_frame='camera_link'):
    pixel_u, pixel_v = bbox_center_pixel(bbox)
    camera_point = pixel_to_camera_point(pixel_u, pixel_v, depth_m, intrinsics)
    position = transform_point(camera_point, camera_to_output) if camera_to_output else camera_point
    return HazardLocalization3D(
        position=position,
        frame_id=output_frame,
        depth_m=float(depth_m),
        pixel_u=pixel_u,
        pixel_v=pixel_v,
        points_used=1,
    )


"""从深度图中采样 bbox ROI 深度，并输出目标坐标系下的三维定位。"""
def localize_bbox_from_depth_image(bbox, intrinsics, depth_image, camera_to_output=None,
                                   output_frame='camera_link', roi_padding_px=0,
                                   max_depth_m=20.0, min_points=5,
                                   sphere_radius_m=0.0,
                                   use_sphere_projection_geometry=True):
    depth_m, points_used = estimate_depth_from_bbox(
        depth_image=depth_image,
        bbox=bbox,
        padding_px=roi_padding_px,
        max_depth_m=max_depth_m,
        min_points=min_points,
    )
    if depth_m is None:
        return None

    # 深度相机看到的是可见球面的前沿，而比赛目标是红球球心的位置。
    # 完整球形优先利用表观投影半径反解球心；被边框截断或过小时才保守地
    # 沿光轴补回一个半径。默认关闭，避免把该先验误用于未知尺寸物体。
    radius_m = float(sphere_radius_m)
    if math.isfinite(radius_m) and radius_m > 0.0:
        sphere_depth_m = None
        if use_sphere_projection_geometry:
            sphere_depth_m = estimate_sphere_center_depth_from_bbox(
                bbox=bbox,
                intrinsics=intrinsics,
                image_width=len(depth_image[0]),
                image_height=len(depth_image),
                sphere_radius_m=radius_m,
            )
        depth_m = sphere_depth_m if sphere_depth_m is not None else depth_m + radius_m

    localization = localize_bbox_with_depth(
        bbox=bbox,
        intrinsics=intrinsics,
        depth_m=depth_m,
        camera_to_output=camera_to_output,
        output_frame=output_frame,
    )
    return HazardLocalization3D(
        position=localization.position,
        frame_id=localization.frame_id,
        depth_m=localization.depth_m,
        pixel_u=localization.pixel_u,
        pixel_v=localization.pixel_v,
        points_used=points_used,
    )


"""读取 bbox 边界，兼容 dict 和 RedBallDetection2D。"""
def _read_bbox(bbox):
    if isinstance(bbox, dict):
        return (
            float(bbox['x_min']),
            float(bbox['y_min']),
            float(bbox['x_max']),
            float(bbox['y_max']),
        )
    return (
        float(bbox.x_min),
        float(bbox.y_min),
        float(bbox.x_max),
        float(bbox.y_max),
    )


def _median(values):
    """返回数值中位数；空列表保持 None，供深度缺失分支显式处理。"""

    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0
