"""危险源三维定位离线测试。

所属组：感知组 / 测试组。
文件作用：
验证 `localize_hazard.py` 的相机反投影、深度图采样和坐标变换。
不依赖 ROS、Gazebo 或真实相机。
"""
import math
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_perception'))

from hazardwalker_perception.localize_hazard import (
    CameraIntrinsics,
    Point3D,
    RigidTransform3D,
    camera_intrinsics_from_k,
    estimate_depth_from_bbox,
    estimate_sphere_center_depth_from_bbox,
    evaluate_sphere_depth_shape,
    localize_bbox_from_depth_image,
    localize_bbox_with_depth,
    make_yaw_transform,
    optical_point_to_camera_link,
    pixel_to_camera_point,
    transform_point,
)
from hazardwalker_perception.track_hazards import distance_m


"""验证 CameraInfo K 矩阵能被转换成定位函数使用的内参。"""
def test_camera_intrinsics_from_k_reads_expected_values():
    intrinsics = camera_intrinsics_from_k([260.0, 0.0, 160.0, 0.0, 260.0, 120.0, 0.0, 0.0, 1.0])

    assert intrinsics.fx == 260.0
    assert intrinsics.fy == 260.0
    assert intrinsics.cx == 160.0
    assert intrinsics.cy == 120.0


"""验证中心像素反投影时 x/y 为 0，深度沿相机 z 轴。"""
def test_pixel_to_camera_point_projects_center_pixel():
    intrinsics = CameraIntrinsics(fx=200.0, fy=200.0, cx=100.0, cy=80.0)

    point = pixel_to_camera_point(100.0, 80.0, 3.0, intrinsics)

    assert point == Point3D(x=0.0, y=0.0, z=3.0)


"""验证偏离光心的像素会按针孔模型得到横向坐标。"""
def test_pixel_to_camera_point_projects_offset_pixel():
    intrinsics = CameraIntrinsics(fx=200.0, fy=100.0, cx=100.0, cy=50.0)

    point = pixel_to_camera_point(120.0, 40.0, 2.0, intrinsics)

    assert math.isclose(point.x, 0.2)
    assert math.isclose(point.y, -0.2)
    assert math.isclose(point.z, 2.0)


"""验证官方 Gazebo real_sense 链路会把光学 z 前向改为链路 x 前向。"""
def test_gazebo_camera_link_axis_convention_converts_optical_point():
    point = optical_point_to_camera_link(
        Point3D(x=0.2, y=-0.3, z=2.0),
        convention='gazebo_link_x_forward',
    )

    assert point == Point3D(x=2.0, y=-0.2, z=0.3)


"""验证刚体变换能把相机坐标点转换到目标坐标系。"""
def test_transform_point_applies_rotation_and_translation():
    transform = RigidTransform3D(
        translation=Point3D(1.0, 2.0, 0.5),
        rotation=((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    )

    point = transform_point(Point3D(2.0, 0.0, 3.0), transform)

    assert math.isclose(point.x, 1.0)
    assert math.isclose(point.y, 4.0)
    assert math.isclose(point.z, 3.5)


"""验证 yaw 平面变换便于表达机器人位姿。"""
def test_make_yaw_transform_rotates_planar_point():
    transform = make_yaw_transform(x=1.0, y=2.0, z=0.0, yaw_rad=math.pi / 2.0)

    point = transform_point(Point3D(1.0, 0.0, 0.0), transform)

    assert math.isclose(point.x, 1.0, abs_tol=1e-9)
    assert math.isclose(point.y, 3.0, abs_tol=1e-9)
    assert math.isclose(point.z, 0.0)


"""验证深度图 ROI 使用有效深度中位数，忽略 0、无穷和超量程值。"""
def test_estimate_depth_from_bbox_uses_valid_roi_median():
    depth_image = [
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 1.8, 2.0, float('inf')],
        [0.0, 2.2, 9.5, 0.0],
        [0.0, 0.0, 0.0, 0.0],
    ]

    depth_m, points_used = estimate_depth_from_bbox(
        depth_image=depth_image,
        bbox={'x_min': 1, 'y_min': 1, 'x_max': 2, 'y_max': 2},
        max_depth_m=5.0,
        min_points=3,
    )

    assert points_used == 3
    assert math.isclose(depth_m, 2.0)


"""验证球面中心比外环更近时会通过深度形状检查。"""
def test_depth_shape_accepts_spherical_curvature():
    depth_image = [[0.0 for _x in range(41)] for _y in range(41)]
    for y in range(5, 36):
        for x in range(5, 36):
            dx = (x - 20) / 15.5
            dy = (y - 20) / 15.5
            if dx * dx + dy * dy <= 0.90 * 0.90:
                depth_image[y][x] = 2.00 + 0.08 * (dx * dx + dy * dy)

    evidence = evaluate_sphere_depth_shape(
        depth_image, {'x_min': 5, 'y_min': 5, 'x_max': 35, 'y_max': 35},
        min_points_per_region=8, min_curvature_m=0.008,
    )

    assert evidence.status == 'spherical'
    assert evidence.curvature_m is not None and evidence.curvature_m > 0.02


"""验证红色圆柱端面等近似平面会被明确抑制，而非作为已确认红球。"""
def test_depth_shape_marks_flat_surface_as_non_spherical():
    depth_image = [[2.0 for _x in range(41)] for _y in range(41)]

    evidence = evaluate_sphere_depth_shape(
        depth_image, {'x_min': 5, 'y_min': 5, 'x_max': 35, 'y_max': 35},
        min_points_per_region=8, min_curvature_m=0.008,
    )

    assert evidence.status == 'flat'
    assert evidence.curvature_m is not None and abs(evidence.curvature_m) < 1e-9


"""验证只在水平方向弯曲的圆柱侧面不会被中心—外环差误判为球面。"""
def test_depth_shape_marks_single_axis_cylinder_surface_as_anisotropic():
    depth_image = [[0.0 for _x in range(41)] for _y in range(41)]
    for y in range(5, 36):
        for x in range(5, 36):
            dx = (x - 20) / 15.5
            dy = (y - 20) / 15.5
            if dx * dx + dy * dy <= 0.90 * 0.90:
                # 圆柱轴沿图像竖直方向：深度只随水平位置变化。
                depth_image[y][x] = 2.00 + 0.08 * dx * dx

    evidence = evaluate_sphere_depth_shape(
        depth_image, {'x_min': 5, 'y_min': 5, 'x_max': 35, 'y_max': 35},
        min_points_per_region=8, min_curvature_m=0.008,
        min_axis_points=4, min_axis_curvature_ratio=0.35,
    )

    assert evidence.status == 'anisotropic'
    assert evidence.horizontal_curvature_m is not None
    assert evidence.horizontal_curvature_m > 0.02
    assert evidence.vertical_curvature_m is not None
    assert evidence.vertical_curvature_m < 0.008
    assert evidence.curvature_isotropy_ratio is not None
    assert evidence.curvature_isotropy_ratio < 0.35


"""验证斜放圆柱也会在对角方向暴露平坦轴，不能绕过固定横纵检查。"""
def test_depth_shape_marks_rotated_cylinder_surface_as_anisotropic():
    depth_image = [[0.0 for _x in range(41)] for _y in range(41)]
    for y in range(5, 36):
        for x in range(5, 36):
            dx = (x - 20) / 15.5
            dy = (y - 20) / 15.5
            if dx * dx + dy * dy <= 0.90 * 0.90:
                perpendicular = (dx - dy) / math.sqrt(2.0)
                depth_image[y][x] = 2.00 + 0.08 * perpendicular * perpendicular

    evidence = evaluate_sphere_depth_shape(
        depth_image, {'x_min': 5, 'y_min': 5, 'x_max': 35, 'y_max': 35},
        min_points_per_region=8, min_curvature_m=0.008,
        min_axis_points=4, min_axis_curvature_ratio=0.35,
    )

    assert evidence.status == 'anisotropic'
    assert evidence.diagonal_positive_curvature_m is not None
    assert evidence.diagonal_positive_curvature_m < 0.008
    assert evidence.diagonal_negative_curvature_m is not None
    assert evidence.diagonal_negative_curvature_m > 0.02
    assert evidence.curvature_isotropy_ratio is not None
    assert evidence.curvature_isotropy_ratio < 0.35


"""验证四方向门控对 0° 到 165° 的圆柱轴方向均保持拒绝。"""
def test_depth_shape_rejects_cylinder_across_all_sampled_orientations():
    bbox = {'x_min': 5, 'y_min': 5, 'x_max': 35, 'y_max': 35}
    for angle_deg in range(0, 180, 15):
        angle = math.radians(angle_deg)
        depth_image = [[0.0 for _x in range(41)] for _y in range(41)]
        for y in range(5, 36):
            for x in range(5, 36):
                dx = (x - 20) / 15.5
                dy = (y - 20) / 15.5
                if dx * dx + dy * dy <= 0.90 * 0.90:
                    perpendicular = -math.sin(angle) * dx + math.cos(angle) * dy
                    depth_image[y][x] = 2.00 + 0.08 * perpendicular * perpendicular

        evidence = evaluate_sphere_depth_shape(
            depth_image, bbox,
            min_points_per_region=8, min_curvature_m=0.008,
            min_axis_points=4, min_axis_curvature_ratio=0.35,
        )

        assert evidence.status == 'anisotropic', (
            angle_deg, evidence.curvature_isotropy_ratio,
        )


"""验证轴向深度缺失时保持 unknown，不能用斜向外环样本伪造球面正证据。"""
def test_depth_shape_with_missing_axis_samples_remains_unknown():
    depth_image = [[0.0 for _x in range(41)] for _y in range(41)]
    for y in range(5, 36):
        for x in range(5, 36):
            dx = (x - 20) / 15.5
            dy = (y - 20) / 15.5
            radial = math.sqrt(dx * dx + dy * dy)
            if radial <= 0.35:
                depth_image[y][x] = 2.00
            elif 0.60 <= radial <= 0.88 and abs(dx) > 0.35 and abs(dy) > 0.35:
                depth_image[y][x] = 2.08

    evidence = evaluate_sphere_depth_shape(
        depth_image, {'x_min': 5, 'y_min': 5, 'x_max': 35, 'y_max': 35},
        min_points_per_region=8, min_curvature_m=0.008,
        min_axis_points=4, min_axis_curvature_ratio=0.35,
    )

    assert evidence.status == 'unknown'
    assert evidence.curvature_m is not None and evidence.curvature_m > 0.02
    assert evidence.horizontal_points < 4
    assert evidence.vertical_points < 4


"""验证 bbox + 固定深度能输出目标坐标系下三维定位。"""
def test_localize_bbox_with_depth_outputs_transformed_position():
    intrinsics = CameraIntrinsics(fx=200.0, fy=200.0, cx=100.0, cy=80.0)
    transform = make_yaw_transform(x=1.0, y=0.0, z=0.5, yaw_rad=0.0)

    result = localize_bbox_with_depth(
        bbox={'x_min': 90, 'y_min': 70, 'x_max': 110, 'y_max': 90},
        intrinsics=intrinsics,
        depth_m=4.0,
        camera_to_output=transform,
        output_frame='start',
    )

    assert result.frame_id == 'start'
    assert math.isclose(result.position.x, 1.0)
    assert math.isclose(result.position.y, 0.0)
    assert math.isclose(result.position.z, 4.5)
    assert result.points_used == 1


"""验证从深度图定位时，如果有效点太少则返回 None。"""
def test_localize_bbox_from_depth_image_rejects_sparse_depth():
    intrinsics = CameraIntrinsics(fx=200.0, fy=200.0, cx=100.0, cy=80.0)
    depth_image = [[0.0, 0.0], [0.0, 2.0]]

    result = localize_bbox_from_depth_image(
        bbox={'x_min': 0, 'y_min': 0, 'x_max': 1, 'y_max': 1},
        intrinsics=intrinsics,
        depth_image=depth_image,
        min_points=2,
    )

    assert result is None


"""验证多红球 bbox 能分别从深度图定位，并计算相对真值的三维误差。"""
def test_multiple_bboxes_localize_with_small_position_error():
    intrinsics = CameraIntrinsics(fx=200.0, fy=200.0, cx=100.0, cy=80.0)
    depth_image = [[0.0 for _x in range(200)] for _y in range(160)]
    boxes = [
        {'x_min': 88, 'y_min': 68, 'x_max': 112, 'y_max': 92},
        {'x_min': 128, 'y_min': 78, 'x_max': 152, 'y_max': 102},
    ]
    depths = [3.0, 4.0]
    expected_positions = [
        (0.0, 0.0, 3.0),
        (0.8, 0.2, 4.0),
    ]
    for bbox, depth in zip(boxes, depths):
        for y in range(int(bbox['y_min']), int(bbox['y_max']) + 1):
            for x in range(int(bbox['x_min']), int(bbox['x_max']) + 1):
                depth_image[y][x] = depth

    results = [
        localize_bbox_from_depth_image(
            bbox=bbox,
            intrinsics=intrinsics,
            depth_image=depth_image,
            min_points=10,
            output_frame='start',
        )
        for bbox in boxes
    ]
    errors = [
        distance_m(
            (result.position.x, result.position.y, result.position.z),
            expected,
        )
        for result, expected in zip(results, expected_positions)
    ]

    assert all(result is not None for result in results)
    assert max(errors) < 1e-9
