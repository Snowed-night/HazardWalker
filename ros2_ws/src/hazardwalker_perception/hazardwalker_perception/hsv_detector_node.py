"""HSV 红球检测 ROS 节点。

所属组：感知组。
文件作用：
把 `/hw/camera/image_raw` 转为危险源候选 JSON。
调用离线红球检测函数得到 2D bbox。
结合 `/hw/camera/camera_info`、`/hw/camera/depth_image` 和 TF 输出三维危险源坐标。
当前实现边界：
深度图支持 `32FC1` 米和 `16UC1` 毫米编码；点云 ROI 定位后续再接。
"""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from hazardwalker_perception.localize_hazard import (
    Point3D,
    RigidTransform3D,
    camera_intrinsics_from_k,
    evaluate_sphere_depth_shape,
    localize_bbox_from_depth_image,
)
from hazardwalker_perception.red_ball_detector import create_detection_backend
from hazardwalker_perception.track_hazards import (
    HazardObservation,
    HazardTracker,
    HazardTrackerConfig,
    track_to_hazard_dict,
)


class HsvDetectorNode(Node):
    def __init__(self):
        super().__init__('hsv_detector_node')
        self.declare_parameter('min_area_px', 200)
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('min_circularity', 0.65)
        self.declare_parameter('min_aspect_ratio', 0.45)
        self.declare_parameter('min_extent', 0.35)
        # 圆投影的 extent 理论值约 0.785；过于接近矩形的红色区域仅保留为
        # reobserve 候选，避免圆柱/方块端面在单帧成为正式正证据。
        self.declare_parameter('max_extent', 0.82)
        self.declare_parameter('max_detections', 20)
        self.declare_parameter('detector_backend', 'hsv_opencv')
        self.declare_parameter('split_touching_red_balls', True)
        self.declare_parameter('roi_padding_px', 8)
        self.declare_parameter('min_depth_points_in_roi', 5)
        self.declare_parameter('max_detection_range_m', 20.0)
        # 官方标准红球为半径 0.15 m；该先验只在严格球形候选的定位阶段使用。
        self.declare_parameter('sphere_radius_m', 0.15)
        self.declare_parameter('use_sphere_projection_geometry', True)
        # PoseInfo/相机帧来自不同桥接线程时可能只有数毫秒滞后；允许使用最新 TF
        # 能避免“未来外推”导致整帧定位被丢弃。高速运动平台可显式关闭该兜底。
        self.declare_parameter('allow_latest_tf_fallback', True)
        # 官方 SimEnv 适配层提供 world→map 静态别名，令检测结果直接满足官方提交的 world 坐标要求。
        self.declare_parameter('output_frame', 'world')
        self.declare_parameter('confirm_observation_count', 3)
        self.declare_parameter('confirm_distinct_views', 3)
        # 主动横移期间目标可能连续数秒离开视场；150 帧约等于 5 秒@30FPS，
        # 避免候选在抵达第二视角前被删除。
        self.declare_parameter('reject_after_missed_count', 300)
        self.declare_parameter('merge_distance_m', 0.5)
        self.declare_parameter('max_apparent_diameter_cv', 0.35)
        self.declare_parameter('min_multiview_aspect_ratio', 0.88)
        self.declare_parameter('max_depth_curvature_cv', 0.65)
        self.declare_parameter('min_normalized_depth_curvature', 0.10)
        self.declare_parameter('max_median_normalized_depth_curvature', 0.30)
        # 只在同一正面方向的前后移动无法排除圆柱/圆锥端面；正式确认至少需要
        # 目标相对相机的水平视线改变 25 度，促使机器人获得侧面反证。
        self.declare_parameter('min_view_bearing_span_deg', 25.0)
        self.declare_parameter('emit_partial_candidates', True)
        # 7 月 5 日真实遮挡帧中约 5% 可见球面只剩约 24--28 px 轮廓面积；
        # 该阈值只产出黄色重观察候选，严格球体确认仍使用更高的 min_area_px。
        self.declare_parameter('partial_min_area_px', 20)
        # 遮挡到 5%--15% 时球面投影会变成很窄的弓形，仍需输出“待复查”
        # 而非静默漏检。该阈值只作用于不可确认候选，最终确认仍保持严格门槛。
        self.declare_parameter('partial_min_circularity', 0.18)
        self.declare_parameter('partial_min_aspect_ratio', 0.12)
        self.declare_parameter('partial_min_value', 50)
        # 深度曲率仅否决“明确平面”的候选；缺深度/遮挡一律进入重观察而非直接漏检。
        self.declare_parameter('min_sphere_depth_curvature_m', 0.008)
        self.declare_parameter('min_sphere_depth_shape_points', 8)
        # 运动中继续发布候选供导航使用，但只有相机连续稳定若干帧后才向轨迹
        # 累积确认/反证，防止横移过渡帧让球体尺寸和曲率统计失真。
        self.declare_parameter('stable_view_min_frames', 3)
        self.declare_parameter('stable_view_max_translation_m', 0.002)
        self.declare_parameter('stable_view_max_yaw_deg', 0.3)

        self.camera_intrinsics = None
        self.latest_depth_image = None
        self.latest_depth_frame_id = ''
        self.latest_depth_stamp = None
        self._last_camera_pose_signature = None
        self._stable_view_frame_count = 0
        self._stable_view_id = ''
        # 官方 RGB-D 对接首帧常暴露 DDS、编码或 TF 时序问题；仅记录前两帧的关键阶段，
        # 既便于集成验收定位，又避免运行期逐帧刷屏。
        self._image_callback_count = 0
        self.detector_backend = create_detection_backend(str(self.get_parameter('detector_backend').value))
        self.tracker = HazardTracker(HazardTrackerConfig(
            confirm_observation_count=int(self.get_parameter('confirm_observation_count').value),
            min_distinct_views=int(self.get_parameter('confirm_distinct_views').value),
            reject_after_missed_count=int(self.get_parameter('reject_after_missed_count').value),
            merge_distance_m=float(self.get_parameter('merge_distance_m').value),
            max_apparent_diameter_cv=float(self.get_parameter('max_apparent_diameter_cv').value),
            min_multiview_aspect_ratio=float(self.get_parameter('min_multiview_aspect_ratio').value),
            max_depth_curvature_cv=float(self.get_parameter('max_depth_curvature_cv').value),
            min_normalized_depth_curvature=float(
                self.get_parameter('min_normalized_depth_curvature').value
            ),
            max_median_normalized_depth_curvature=float(
                self.get_parameter('max_median_normalized_depth_curvature').value
            ),
            min_view_bearing_span_deg=float(
                self.get_parameter('min_view_bearing_span_deg').value
            ),
        ))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 只依赖平台层输出的统一 `/hw/*` topic，不直接依赖 Gazebo/官方平台 topic。
        self.sub = self.create_subscription(Image, '/hw/camera/image_raw', self.on_image, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/hw/camera/camera_info', self.on_camera_info, 10)
        self.depth_sub = self.create_subscription(Image, '/hw/camera/depth_image', self.on_depth_image, 10)
        # 第一阶段用 String(JSON) 快速打通链路；稳定后迁移到 hazardwalker_msgs/HazardArray。
        self.pub = self.create_publisher(String, '/hw/perception/hazard_detections', 10)
        self.get_logger().info('HSV detector subscribed to camera image, camera info and depth image.')

    def on_camera_info(self, msg: CameraInfo):
        self.camera_intrinsics = camera_intrinsics_from_k(msg.k)

    def on_depth_image(self, msg: Image):
        depth_image = _depth_image_to_meters(msg)
        if depth_image is None:
            self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}', throttle_duration_sec=5.0)
            return
        self.latest_depth_image = depth_image
        self.latest_depth_frame_id = msg.header.frame_id
        self.latest_depth_stamp = msg.header.stamp

    def on_image(self, msg: Image):
        # 当前只支持最常见的 rgb8/bgr8。正式版本应通过 cv_bridge 支持更多编码。
        if msg.encoding.lower() not in ('rgb8', 'bgr8'):
            self.get_logger().warn(f'Unsupported image encoding: {msg.encoding}', throttle_duration_sec=5.0)
            return

        self._image_callback_count += 1
        if self._image_callback_count <= 2:
            self.get_logger().info('Received RGB frame %d: %dx%d %s.' % (
                self._image_callback_count, msg.width, msg.height, msg.encoding))

        detections_2d = self.detector_backend.detect(
            data=msg.data,
            width=msg.width,
            height=msg.height,
            step=msg.step,
            encoding=msg.encoding,
            min_area_px=int(self.get_parameter('min_area_px').value),
            min_confidence=float(self.get_parameter('min_confidence').value),
            min_circularity=float(self.get_parameter('min_circularity').value),
            min_aspect_ratio=float(self.get_parameter('min_aspect_ratio').value),
            min_extent=float(self.get_parameter('min_extent').value),
            max_extent=float(self.get_parameter('max_extent').value),
            max_detections=int(self.get_parameter('max_detections').value),
            split_touching=bool(self.get_parameter('split_touching_red_balls').value),
            include_partial_candidates=bool(self.get_parameter('emit_partial_candidates').value),
            partial_min_area_px=int(self.get_parameter('partial_min_area_px').value),
            partial_min_circularity=float(self.get_parameter('partial_min_circularity').value),
            partial_min_aspect_ratio=float(self.get_parameter('partial_min_aspect_ratio').value),
            partial_min_value=int(self.get_parameter('partial_min_value').value),
        )
        stamp_sec = _stamp_to_float(msg.header.stamp)
        output_frame = str(self.get_parameter('output_frame').value)
        camera_to_output = self._lookup_camera_to_output(msg.header.frame_id, output_frame, msg.header.stamp)
        if self._image_callback_count <= 2:
            self.get_logger().info('RGB frame %d processed: detections=%d tf=%s.' % (
                self._image_callback_count, len(detections_2d), bool(camera_to_output)))
        camera_stable = self._update_camera_stability(camera_to_output)
        # 一个停靠周期只允许一个视角标识；即使量化边界附近有毫米级抖动，
        # 也不能在同一次截图中凭空累计多个 distinct view。
        view_id = self._stable_view_id if camera_stable else _view_id_from_transform(camera_to_output)
        if not detections_2d:
            active_tracks = (
                self.tracker.update([], stamp_sec=stamp_sec)
                if camera_stable else self.tracker.active_tracks()
            )
            self._publish_detection_payload(
                hazards=self._tracks_to_hazards(active_tracks, output_frame),
                detections_2d=[],
                camera_stable=camera_stable,
            )
            return

        observations = []
        detections_2d_payload = []

        for index, detection_2d in enumerate(detections_2d, start=1):
            bbox = {
                'x_min': detection_2d.x_min,
                'y_min': detection_2d.y_min,
                'x_max': detection_2d.x_max,
                'y_max': detection_2d.y_max,
            }
            localization = None
            depth_shape = None
            if self.latest_depth_image is not None:
                depth_shape = evaluate_sphere_depth_shape(
                    depth_image=self.latest_depth_image,
                    bbox=bbox,
                    max_depth_m=float(self.get_parameter('max_detection_range_m').value),
                    min_points_per_region=int(self.get_parameter('min_sphere_depth_shape_points').value),
                    min_curvature_m=float(self.get_parameter('min_sphere_depth_curvature_m').value),
                )
            depth_shape_status = depth_shape.status if depth_shape else 'unknown'
            # 圆柱端面、立方体/平板等在单帧可能都有近圆形红色投影。只有深度明确
            # 显示平面或轮廓明显非圆时才抑制正证据；unknown 保留给多视角策略。
            # 非圆视角仍进入轨迹用于复查，但不能污染后续完整球视角的尺寸/圆度统计。
            confirmation_eligible = (
                not detection_2d.requires_reobservation and depth_shape_status != 'flat'
                and detection_2d.aspect_ratio >= float(
                    self.get_parameter('min_multiview_aspect_ratio').value
                )
            )
            if self.camera_intrinsics and self.latest_depth_image is not None and camera_to_output:
                localization = localize_bbox_from_depth_image(
                    bbox=bbox,
                    intrinsics=self.camera_intrinsics,
                    depth_image=self.latest_depth_image,
                    camera_to_output=camera_to_output,
                    output_frame=output_frame,
                    roi_padding_px=int(self.get_parameter('roi_padding_px').value),
                    max_depth_m=float(self.get_parameter('max_detection_range_m').value),
                    min_points=int(self.get_parameter('min_depth_points_in_roi').value),
                    sphere_radius_m=float(self.get_parameter('sphere_radius_m').value),
                    use_sphere_projection_geometry=bool(
                        self.get_parameter('use_sphere_projection_geometry').value
                    ),
                )
            apparent_diameter_m = _apparent_diameter_m(
                bbox, localization.depth_m if localization else None, self.camera_intrinsics,
            )
            view_bearing_rad = _view_bearing_from_camera(
                camera_to_output, localization.position if localization else None,
            )

            source_id = f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec}:{index}'
            detections_2d_payload.append({
                'id': index,
                'frame_id': msg.header.frame_id,
                # 量化后的相机世界位姿标签用于后续多视角确认与实验审计。
                'view_id': view_id,
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec,
                },
                'bbox': bbox,
                'confidence': detection_2d.confidence,
                'red_pixel_count': detection_2d.red_pixel_count,
                'is_partial': detection_2d.is_partial,
                'requires_reobservation': detection_2d.requires_reobservation,
                'may_be_merged': detection_2d.may_be_merged,
                'from_merged_split': detection_2d.from_merged_split,
                'quality_reason': detection_2d.quality_reason,
                'shape': {
                    'circularity': detection_2d.circularity,
                    'aspect_ratio': detection_2d.aspect_ratio,
                    'extent': detection_2d.extent,
                },
                'depth_shape': {
                    'status': depth_shape_status,
                    'center_depth_m': depth_shape.center_depth_m if depth_shape else None,
                    'outer_depth_m': depth_shape.outer_depth_m if depth_shape else None,
                    'curvature_m': depth_shape.curvature_m if depth_shape else None,
                    'center_points': depth_shape.center_points if depth_shape else 0,
                    'outer_points': depth_shape.outer_points if depth_shape else 0,
                },
                'confirmation_eligible': confirmation_eligible,
                'apparent_diameter_m': apparent_diameter_m,
                'view_bearing_deg': (
                    math.degrees(view_bearing_rad) if view_bearing_rad is not None else None
                ),
                'localization_status': (
                    'suppressed_flat_depth_shape' if depth_shape_status == 'flat'
                    else 'localized' if localization else 'unlocalized'
                ),
                'source': 'hsv_minimal',
                'detector_backend': self.detector_backend.name,
            })

            # 所有可定位候选都进入证据轨迹：严格候选累计正证据，partial/flat
            # 候选只累计反证或待复查证据，绝不能把轨迹直接推成 confirmed。
            if localization:
                observations.append(HazardObservation(
                    position=(
                        localization.position.x,
                        localization.position.y,
                        localization.position.z,
                    ),
                    confidence=detection_2d.confidence,
                    stamp_sec=stamp_sec,
                    source_id=source_id,
                    view_id=view_id,
                    confirmation_eligible=confirmation_eligible,
                    depth_shape_status=depth_shape_status,
                    apparent_diameter_m=apparent_diameter_m,
                    # 贴边框被图像裁切，长宽比不代表真实轮廓，不能拿来否决球体。
                    aspect_ratio=(
                        None if _bbox_touches_image_edge(bbox, msg.width, msg.height)
                        else detection_2d.aspect_ratio
                    ),
                    depth_curvature_m=(depth_shape.curvature_m if depth_shape else None),
                    view_bearing_rad=view_bearing_rad,
                ))

        active_tracks = (
            self.tracker.update(observations, stamp_sec=stamp_sec)
            if camera_stable else self.tracker.active_tracks()
        )
        self._publish_detection_payload(
            hazards=self._tracks_to_hazards(active_tracks, output_frame),
            detections_2d=detections_2d_payload,
            camera_stable=camera_stable,
        )

    def _tracks_to_hazards(self, tracks, output_frame):
        hazards = []
        for track in tracks:
            item = track_to_hazard_dict(track)
            item['position_frame_id'] = output_frame
            item['source'] = 'hsv_depth_tf'
            item['observation_time'] = time.time()
            hazards.append(item)
        return hazards

    def _publish_detection_payload(self, hazards, detections_2d, camera_stable=False):
        out = String()
        out.data = json.dumps({
            'hazards': hazards,
            'detections_2d': detections_2d,
            'localization_ready': self.camera_intrinsics is not None and self.latest_depth_image is not None,
            'camera_stable': bool(camera_stable),
            'stable_view_frame_count': self._stable_view_frame_count,
        }, ensure_ascii=False)
        self.pub.publish(out)
        if self._image_callback_count <= 2:
            self.get_logger().info('Published perception payload for RGB frame %d.' % self._image_callback_count)

    def _update_camera_stability(self, transform):
        """根据精确相机世界位姿判断当前帧是否属于停靠稳定视角。"""

        signature = _camera_pose_signature(transform)
        if signature is None:
            self._last_camera_pose_signature = None
            self._stable_view_frame_count = 0
            self._stable_view_id = ''
            return False
        previous = self._last_camera_pose_signature
        self._last_camera_pose_signature = signature
        if previous is None:
            self._stable_view_frame_count = 1
            self._stable_view_id = ''
        else:
            translation = math.sqrt(sum((signature[index] - previous[index]) ** 2 for index in range(3)))
            yaw_delta = abs(math.degrees(math.atan2(
                math.sin(signature[3] - previous[3]), math.cos(signature[3] - previous[3]),
            )))
            if (translation <= float(self.get_parameter('stable_view_max_translation_m').value)
                    and yaw_delta <= float(self.get_parameter('stable_view_max_yaw_deg').value)):
                self._stable_view_frame_count += 1
            else:
                self._stable_view_frame_count = 1
                self._stable_view_id = ''
        stable = self._stable_view_frame_count >= int(self.get_parameter('stable_view_min_frames').value)
        if stable and not self._stable_view_id:
            self._stable_view_id = _view_id_from_transform(transform)
        return stable

    def _lookup_camera_to_output(self, camera_frame, output_frame, stamp):
        if not camera_frame:
            return None
        if camera_frame == output_frame:
            return RigidTransform3D(
                translation=Point3D(0.0, 0.0, 0.0),
                rotation=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            )
        try:
            transform = self.tf_buffer.lookup_transform(output_frame, camera_frame, Time.from_msg(stamp))
        except TransformException as exc:
            if bool(self.get_parameter('allow_latest_tf_fallback').value):
                try:
                    latest_transform = self.tf_buffer.lookup_transform(output_frame, camera_frame, Time())
                    self.get_logger().warn(
                        f'TF at image stamp unavailable from {camera_frame} to {output_frame}; using latest TF: {exc}',
                        throttle_duration_sec=5.0,
                    )
                    return _transform_msg_to_rigid(latest_transform.transform)
                except TransformException:
                    pass
            self.get_logger().warn(
                f'TF lookup failed from {camera_frame} to {output_frame}: {exc}',
                throttle_duration_sec=5.0,
            )
            return None
        return _transform_msg_to_rigid(transform.transform)


"""把 ROS 深度图转换为米单位二维数组。"""
def _depth_image_to_meters(msg: Image):
    encoding = msg.encoding.upper()
    if encoding == '32FC1':
        dtype = np.float32
        scale = 1.0
    elif encoding == '16UC1':
        dtype = np.uint16
        scale = 0.001
    else:
        return None

    item_size = np.dtype(dtype).itemsize
    row_values = msg.step // item_size
    if row_values < msg.width:
        return None

    raw = np.frombuffer(bytes(msg.data), dtype=dtype)
    expected_values = row_values * msg.height
    if raw.size < expected_values:
        return None
    image = raw[:expected_values].reshape((msg.height, row_values))[:, :msg.width]
    return image.astype(np.float32) * scale


"""把 ROS Transform 转成定位纯函数使用的刚体变换。"""
def _transform_msg_to_rigid(transform):
    qx = float(transform.rotation.x)
    qy = float(transform.rotation.y)
    qz = float(transform.rotation.z)
    qw = float(transform.rotation.w)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    else:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    rotation = (
        (1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)),
        (2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)),
        (2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)),
    )
    return RigidTransform3D(
        translation=Point3D(
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
        ),
        rotation=rotation,
    )


"""把 ROS 时间戳转成浮点秒，供跟踪器记录观测时间。"""
def _stamp_to_float(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _apparent_diameter_m(bbox, depth_m, intrinsics):
    """由 bbox 角尺寸和深度估计目标直径，供多视角尺寸一致性门控使用。"""

    if depth_m is None or intrinsics is None or depth_m <= 0.0:
        return None
    width_px = max(1.0, float(bbox['x_max']) - float(bbox['x_min']) + 1.0)
    height_px = max(1.0, float(bbox['y_max']) - float(bbox['y_min']) + 1.0)
    focal_px = max(1.0, (float(intrinsics.fx) + float(intrinsics.fy)) / 2.0)
    return round(max(width_px, height_px) * float(depth_m) / focal_px, 4)


def _view_bearing_from_camera(camera_to_output, target_position):
    """计算目标相对相机的水平视线；用于强制产生真实侧向多视角。"""
    if camera_to_output is None or target_position is None:
        return None
    dx = float(target_position.x) - float(camera_to_output.translation.x)
    dy = float(target_position.y) - float(camera_to_output.translation.y)
    if math.hypot(dx, dy) < 1e-4:
        return None
    return math.atan2(dy, dx)


def _bbox_touches_image_edge(bbox, image_width, image_height, margin_px=2):
    return (
        bbox['x_min'] <= margin_px or bbox['y_min'] <= margin_px
        or bbox['x_max'] >= int(image_width) - 1 - margin_px
        or bbox['y_max'] >= int(image_height) - 1 - margin_px
    )


"""将相机位姿量化为多视角确认使用的稳定标签。"""
def _view_id_from_transform(transform):
    if transform is None:
        return ''
    forward_x = float(transform.rotation[0][2])
    forward_y = float(transform.rotation[1][2])
    yaw_deg = math.degrees(math.atan2(forward_y, forward_x))
    # 多视角证据只应由横向/前后基线或朝向变化产生。Gazebo 占位底盘在停稳
    # 时仍可能有轻微上下浮动；把 z 也写入标签会让同一停靠位置被误算成多个
    # 视角，从而虚高 confirmed。高度变化留给三维定位误差记录，不参与确认计数。
    return 'xy:{:.1f}:{:.1f}|yaw:{:.0f}'.format(
        round(float(transform.translation.x) / 0.4) * 0.4,
        round(float(transform.translation.y) / 0.4) * 0.4,
        round(yaw_deg / 30.0) * 30.0,
    )


def _camera_pose_signature(transform):
    if transform is None:
        return None
    forward_x = float(transform.rotation[0][2])
    forward_y = float(transform.rotation[1][2])
    return (
        float(transform.translation.x), float(transform.translation.y),
        float(transform.translation.z), math.atan2(forward_y, forward_x),
    )


"""启动 ROS 节点,用于在仿真或 fake platform 下验证图像检测链路。"""
def main():
    rclpy.init()
    node = HsvDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
