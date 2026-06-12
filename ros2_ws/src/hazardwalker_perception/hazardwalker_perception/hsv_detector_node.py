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
    localize_bbox_from_depth_image,
)
from hazardwalker_perception.red_ball_detector import detect_red_balls_rgb_bytes
from hazardwalker_perception.track_hazards import (
    HazardObservation,
    HazardTracker,
    HazardTrackerConfig,
    track_to_hazard_dict,
)


class HsvDetectorNode(Node):
    def __init__(self):
        super().__init__('hsv_detector_node')
        self.declare_parameter('min_area_px', 80)
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('min_circularity', 0.60)
        self.declare_parameter('min_aspect_ratio', 0.45)
        self.declare_parameter('min_extent', 0.35)
        self.declare_parameter('max_extent', 0.92)
        self.declare_parameter('max_detections', 20)
        self.declare_parameter('roi_padding_px', 8)
        self.declare_parameter('min_depth_points_in_roi', 5)
        self.declare_parameter('max_detection_range_m', 20.0)
        self.declare_parameter('output_frame', 'start')
        self.declare_parameter('confirm_observation_count', 3)
        self.declare_parameter('reject_after_missed_count', 10)
        self.declare_parameter('merge_distance_m', 0.5)

        self.camera_intrinsics = None
        self.latest_depth_image = None
        self.latest_depth_frame_id = ''
        self.latest_depth_stamp = None
        self.tracker = HazardTracker(HazardTrackerConfig(
            confirm_observation_count=int(self.get_parameter('confirm_observation_count').value),
            reject_after_missed_count=int(self.get_parameter('reject_after_missed_count').value),
            merge_distance_m=float(self.get_parameter('merge_distance_m').value),
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

        detections_2d = detect_red_balls_rgb_bytes(
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
        )
        stamp_sec = _stamp_to_float(msg.header.stamp)
        output_frame = str(self.get_parameter('output_frame').value)
        if not detections_2d:
            active_tracks = self.tracker.update([], stamp_sec=stamp_sec)
            self._publish_detection_payload(
                hazards=self._tracks_to_hazards(active_tracks, output_frame),
                detections_2d=[],
            )
            return

        camera_to_output = self._lookup_camera_to_output(msg.header.frame_id, output_frame, msg.header.stamp)
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
                )

            source_id = f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec}:{index}'
            detections_2d_payload.append({
                'id': index,
                'frame_id': msg.header.frame_id,
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec,
                },
                'bbox': bbox,
                'confidence': detection_2d.confidence,
                'shape': {
                    'circularity': detection_2d.circularity,
                    'aspect_ratio': detection_2d.aspect_ratio,
                    'extent': detection_2d.extent,
                },
                'localization_status': 'localized' if localization else 'unlocalized',
                'source': 'hsv_minimal',
            })

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
                ))

        active_tracks = self.tracker.update(observations, stamp_sec=stamp_sec)
        self._publish_detection_payload(
            hazards=self._tracks_to_hazards(active_tracks, output_frame),
            detections_2d=detections_2d_payload,
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

    def _publish_detection_payload(self, hazards, detections_2d):
        out = String()
        out.data = json.dumps({
            'hazards': hazards,
            'detections_2d': detections_2d,
            'localization_ready': self.camera_intrinsics is not None and self.latest_depth_image is not None,
        }, ensure_ascii=False)
        self.pub.publish(out)

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


"""启动 ROS 节点,用于在仿真或 fake platform 下验证图像检测链路。"""
def main():
    rclpy.init()
    node = HsvDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
