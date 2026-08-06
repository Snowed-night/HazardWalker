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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from hazardwalker_perception.active_view_geometry import (
    camera_pose_signature,
    quantized_camera_view_id,
)
from hazardwalker_perception.active_view_policy import (
    ActiveViewDirectionMemory,
    ActiveViewPolicyConfig,
    TransientCandidateMemory,
    annotate_detections_with_tracks,
    attach_candidate_aliases_to_hazards,
    choose_active_view_action,
    project_tracks_for_image_association,
)
from hazardwalker_perception.localize_hazard import (
    Point3D,
    RigidTransform3D,
    camera_intrinsics_from_k,
    estimate_depth_from_bbox,
    evaluate_sphere_depth_shape,
    localize_bbox_from_depth_image,
)
from hazardwalker_perception.red_ball_detector import (
    create_detection_backend,
    is_complete_candidate_for_3d_tracking,
)
from hazardwalker_perception.rgbd_pairing import DeferredRgbDepthPairer
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
        # HSV 阈值必须能由同一份受版本控制的配置文件覆盖，回放评估记录的
        # 参数哈希才与节点实际运行值一致。
        self.declare_parameter('red_hue_min_1', 0)
        self.declare_parameter('red_hue_max_1', 10)
        self.declare_parameter('red_hue_min_2', 170)
        self.declare_parameter('red_hue_max_2', 180)
        self.declare_parameter('red_min_saturation', 80)
        self.declare_parameter('red_min_value', 80)
        self.declare_parameter('roi_padding_px', 8)
        self.declare_parameter('min_depth_points_in_roi', 5)
        self.declare_parameter('max_detection_range_m', 20.0)
        # 官方标准红球为半径 0.15 m；该先验只在严格球形候选的定位阶段使用。
        self.declare_parameter('sphere_radius_m', 0.15)
        self.declare_parameter('use_sphere_projection_geometry', True)
        # RGB 与 TF 来自不同桥接线程时可能有数十毫秒到达差。允许最新 TF 兜底，
        # 但必须限制时间差，不能把任意旧位姿用于运动中的三维定位。
        self.declare_parameter('allow_latest_tf_fallback', True)
        self.declare_parameter('max_latest_tf_fallback_delta_sec', 0.10)
        # 在官方比赛模式下，绝不能把 Gazebo 的 /Odometry_gazebo 或 ground_truth TF
        # 伪装成世界定位。默认只使用由导航组合法 SLAM 发布的 map；在该 TF 未就绪前
        # 仍发布相机候选和深度距离，但不输出可提交的 world 坐标。
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('localization_provenance', 'unverified')
        self.declare_parameter('confirm_observation_count', 3)
        self.declare_parameter('confirm_distinct_views', 3)
        # 主动横移期间目标可能连续数秒离开视场；150 帧约等于 5 秒@30FPS，
        # 避免候选在抵达第二视角前被删除。
        self.declare_parameter('reject_after_missed_count', 300)
        # 正式桥接链会重复发布图像，固定漏检帧数随发布频率变化。按仿真时间
        # 保留 60 秒，覆盖一次完整侧移、回转和重新对准；纯函数仍可用按帧模式。
        # 主动复查需覆盖 A1 在低实时倍率下的一次完整侧移和回转。
        self.declare_parameter('reject_after_missed_sec', 60.0)
        self.declare_parameter('merge_distance_m', 0.5)
        self.declare_parameter('single_track_reacquire_distance_m', 1.0)
        self.declare_parameter(
            'single_track_reacquire_diameter_relative_error', 0.25,
        )
        self.declare_parameter('same_frame_duplicate_distance_m', 0.12)
        self.declare_parameter(
            'same_frame_duplicate_diameter_relative_error', 0.25,
        )
        # 官方控制链一次侧移、停稳、稳定帧和截图可超过 20 秒，局部目标连续
        # 离开视野可达约 80 秒；危险源静止，因而保留 180 秒投影以跨多个
        # 动作周期恢复同一轨迹。多目标投影近似重合时
        # 关联器会拒绝歧义，不会据此把两个红球强行合并。
        self.declare_parameter('track_projection_max_age_s', 180.0)
        self.declare_parameter('max_apparent_diameter_cv', 0.35)
        # 官方唯一目标为半径 0.15 m 的球体；用 35% 容差吸收低分辨率、深度噪声与
        # 轻度遮挡的投影误差，但拒绝尺寸明显不符的红色圆形物体。
        self.declare_parameter('expected_sphere_diameter_m', 0.30)
        self.declare_parameter('max_sphere_diameter_relative_error', 0.35)
        # 官方真实 RGB 中完整球体会因透视/低分辨率落在约 0.85；0.82 以下才视为
        # 明显拉长。深度平面和横向多视角仍是确认的强制门，不能放宽为单帧确认。
        self.declare_parameter('min_multiview_aspect_ratio', 0.82)
        # 最终确认至少要有两个不同视角的 RGB-D 球面正证据；未知深度不能把
        # 圆柱端面或圆盘升级为危险源。
        self.declare_parameter('min_spherical_views_for_confirm', 2)
        self.declare_parameter('max_depth_curvature_cv', 0.65)
        self.declare_parameter('min_normalized_depth_curvature', 0.10)
        self.declare_parameter('max_median_normalized_depth_curvature', 0.30)
        # 只在同一正面方向的前后移动无法排除圆柱/圆锥端面；正式确认至少需要
        # 目标相对相机的水平视线改变 25 度，促使机器人获得侧面反证。
        self.declare_parameter('min_view_bearing_span_deg', 25.0)
        # 当前官方 SLAM 在长距离侧移时可能产生单向漂移；用首个完整球面视角
        # 锚定静态危险源位置，后续视角只用于确认/反证，避免帧数把坐标拖走。
        self.declare_parameter(
            'track_position_fusion_mode', 'earliest_view_anchor',
        )
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
        # 球面横纵两个方向都应有凸曲率；单轴弯曲通常来自圆柱侧面或弧形板。
        self.declare_parameter('min_sphere_axis_depth_points', 4)
        self.declare_parameter('min_sphere_axis_curvature_ratio', 0.35)
        # RGB 与独立深度 WebSocket 可能跨帧到达；不同时间的深度不能用于球形判别或定位，
        # 否则会把运动中的红球错误记为平面/错误世界坐标。
        # 当前官方 ROS1↔ROS2 适配实测 RGB/深度固定相差约 50 ms，帧周期约
        # 500 ms；0.06 s 可接纳同一对帧而不会误配到相邻帧。运动时仍由相机
        # 稳定门禁止累计确认/反证，现场帧率变化后应重新实测并记录该参数。
        self.declare_parameter('max_rgb_depth_sync_delta_sec', 0.06)
        # 默认标准 ROS 光学系；官方 Gazebo ``real_sense`` 链路需设为
        # ``gazebo_link_x_forward``，由官方业务 launch 显式传入。
        self.declare_parameter('camera_axis_convention', 'optical_z_forward')
        # 运动中继续发布候选供导航使用，但只有相机连续稳定若干帧后才向轨迹
        # 累积确认/反证，防止横移过渡帧让球体尺寸和曲率统计失真。
        self.declare_parameter('stable_view_min_frames', 3)
        self.declare_parameter('stable_view_max_translation_m', 0.002)
        self.declare_parameter('stable_view_max_yaw_deg', 0.3)
        # 主动视角策略阈值全部暴露为 ROS 参数，现场调优不修改源码。
        self.declare_parameter('active_view_edge_margin_ratio', 0.05)
        self.declare_parameter('active_view_min_bbox_area_px', 900)
        self.declare_parameter('active_view_min_red_pixel_count', 300)
        self.declare_parameter('active_view_min_circularity', 0.72)
        self.declare_parameter('active_view_min_confidence', 0.70)
        self.declare_parameter('active_view_far_distance_m', 5.0)
        self.declare_parameter('active_view_dense_iou_threshold', 0.15)
        self.declare_parameter(
            'active_view_min_normalized_depth_curvature', 0.10,
        )
        self.declare_parameter(
            'active_view_max_normalized_depth_curvature', 0.30,
        )
        self.declare_parameter('candidate_memory_ttl_s', 60.0)
        self.declare_parameter('active_view_direction_memory_ttl_s', 12.0)
        self.declare_parameter('candidate_memory_min_iou', 0.05)
        self.declare_parameter(
            'candidate_memory_max_center_shift_ratio', 1.5,
        )

        self.camera_intrinsics = None
        self.latest_depth_image = None
        self.latest_depth_frame_id = ''
        self.latest_depth_stamp = None
        self._last_camera_pose_signature = None
        self._stable_view_frame_count = 0
        self._stable_view_id = ''
        self._last_depth_synchronized = False
        self._last_tf_synchronized = False
        self._last_tf_stamp_delta_sec = None
        self.rgbd_pairer = DeferredRgbDepthPairer(
            float(self.get_parameter('max_rgb_depth_sync_delta_sec').value),
        )
        # 官方 RGB-D 对接首帧常暴露 DDS、编码或 TF 时序问题；仅记录前两帧的关键阶段，
        # 既便于集成验收定位，又避免运行期逐帧刷屏。
        self._image_callback_count = 0
        self.detector_backend = create_detection_backend(str(self.get_parameter('detector_backend').value))
        self.tracker = HazardTracker(HazardTrackerConfig(
            confirm_observation_count=int(self.get_parameter('confirm_observation_count').value),
            min_distinct_views=int(self.get_parameter('confirm_distinct_views').value),
            reject_after_missed_count=int(self.get_parameter('reject_after_missed_count').value),
            reject_after_missed_sec=float(
                self.get_parameter('reject_after_missed_sec').value
            ),
            merge_distance_m=float(self.get_parameter('merge_distance_m').value),
            single_track_reacquire_distance_m=float(
                self.get_parameter(
                    'single_track_reacquire_distance_m'
                ).value
            ),
            single_track_reacquire_diameter_relative_error=float(
                self.get_parameter(
                    'single_track_reacquire_diameter_relative_error'
                ).value
            ),
            same_frame_duplicate_distance_m=float(
                self.get_parameter('same_frame_duplicate_distance_m').value
            ),
            same_frame_duplicate_diameter_relative_error=float(
                self.get_parameter(
                    'same_frame_duplicate_diameter_relative_error'
                ).value
            ),
            max_apparent_diameter_cv=float(self.get_parameter('max_apparent_diameter_cv').value),
            expected_sphere_diameter_m=float(
                self.get_parameter('expected_sphere_diameter_m').value
            ),
            max_sphere_diameter_relative_error=float(
                self.get_parameter('max_sphere_diameter_relative_error').value
            ),
            min_multiview_aspect_ratio=float(self.get_parameter('min_multiview_aspect_ratio').value),
            min_spherical_views_for_confirm=int(
                self.get_parameter('min_spherical_views_for_confirm').value
            ),
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
            position_fusion_mode=str(
                self.get_parameter('track_position_fusion_mode').value
            ),
        ))
        self.candidate_memory = TransientCandidateMemory(
            ttl_s=float(
                self.get_parameter('candidate_memory_ttl_s').value
            ),
            min_iou=float(
                self.get_parameter('candidate_memory_min_iou').value
            ),
            max_center_shift_ratio=float(
                self.get_parameter(
                    'candidate_memory_max_center_shift_ratio'
                ).value
            ),
        )
        self.active_view_direction_memory = ActiveViewDirectionMemory(
            ttl_s=float(
                self.get_parameter(
                    'active_view_direction_memory_ttl_s'
                ).value
            ),
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 只依赖平台层输出的统一 `/hw/*` topic，不直接依赖 Gazebo/官方平台 topic。
        self.sub = self.create_subscription(Image, '/hw/camera/image_raw', self.on_image, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/hw/camera/camera_info', self.on_camera_info, 10)
        self.depth_sub = self.create_subscription(Image, '/hw/camera/depth_image', self.on_depth_image, 10)
        # 第一阶段用 String(JSON) 快速打通链路；稳定后迁移到 hazardwalker_msgs/HazardArray。
        self.pub = self.create_publisher(String, '/hw/perception/hazard_detections', 10)
        # 复查建议由感知节点直接发布，不能依赖“是否启动证据记录器”；导航、
        # 控制、GUI 和 rosbag 因而消费同一份只读建议。
        self.recommendation_pub = self.create_publisher(
            String, '/hw/perception/view_recommendation', 10)
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
        # RGB 常先于同时间戳深度抵达。深度到达后立即补处理等待帧，避免始终
        # 使用上一帧 50 ms 的深度；运动错帧仍由严格时间阈值拒绝。
        for dispatch in self.rgbd_pairer.push_depth(
                _stamp_to_float(msg.header.stamp)):
            self._process_image(dispatch.payload)

    def on_image(self, msg: Image):
        # 最多延迟一帧等待对应深度。若深度已先到则立即处理；若一直缺失，
        # 下一帧 RGB 会先降级处理旧帧，因此导航候选不会被无限阻塞。
        for dispatch in self.rgbd_pairer.push_rgb(
                _stamp_to_float(msg.header.stamp), msg):
            self._process_image(dispatch.payload)

    def _process_image(self, msg: Image):
        # 当前只支持最常见的 rgb8/bgr8。正式版本应通过 cv_bridge 支持更多编码。
        if msg.encoding.lower() not in ('rgb8', 'bgr8'):
            self.get_logger().warn(f'Unsupported image encoding: {msg.encoding}', throttle_duration_sec=5.0)
            return

        processing_started = time.perf_counter()

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
            red_hue_min_1=int(self.get_parameter('red_hue_min_1').value),
            red_hue_max_1=int(self.get_parameter('red_hue_max_1').value),
            red_hue_min_2=int(self.get_parameter('red_hue_min_2').value),
            red_hue_max_2=int(self.get_parameter('red_hue_max_2').value),
            red_min_saturation=int(self.get_parameter('red_min_saturation').value),
            red_min_value=int(self.get_parameter('red_min_value').value),
        )
        stamp_sec = _stamp_to_float(msg.header.stamp)
        depth_stamp_delta_sec = _stamp_delta_sec(
            msg.header.stamp, self.latest_depth_stamp)
        depth_synchronized = (
            self.latest_depth_image is not None
            and depth_stamp_delta_sec is not None
            and depth_stamp_delta_sec <= float(
                self.get_parameter('max_rgb_depth_sync_delta_sec').value
            )
        )
        self._last_depth_synchronized = depth_synchronized
        output_frame = str(self.get_parameter('output_frame').value)
        camera_to_output = self._lookup_camera_to_output(msg.header.frame_id, output_frame, msg.header.stamp)
        if self._image_callback_count <= 2:
            self.get_logger().info('RGB frame %d processed: detections=%d tf=%s.' % (
                self._image_callback_count, len(detections_2d), bool(camera_to_output)))
        camera_stable = self._update_camera_stability(camera_to_output)
        # 一个停靠周期只允许一个视角标识；即使量化边界附近有毫米级抖动，
        # 也不能在同一次截图中凭空累计多个 distinct view。
        view_id = (
            self._stable_view_id
            if camera_stable else quantized_camera_view_id(
                camera_to_output,
                str(self.get_parameter('camera_axis_convention').value),
            )
        )
        if not detections_2d:
            if camera_stable:
                self.tracker.update([], stamp_sec=stamp_sec)
            tracks_to_publish = (
                self.tracker.published_tracks()
                if camera_stable else self.tracker.active_tracks()
            )
            self._publish_detection_payload(
                hazards=self._tracks_to_hazards(tracks_to_publish, output_frame),
                detections_2d=[],
                camera_stable=camera_stable,
                image_width=msg.width,
                image_height=msg.height,
                stamp_sec=stamp_sec,
                processing_time_ms=(
                    time.perf_counter() - processing_started
                ) * 1000.0,
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
            complete_for_3d_tracking = is_complete_candidate_for_3d_tracking(
                detection_2d, msg.width, msg.height,
            )
            # HSV 在重遮挡下偶尔仍会把带直边的红色弧段标成 stable_shape。
            # 轮廓长宽比未达到多视角球体门槛时，禁止套用完整球半径先验，
            # 也禁止把遮挡边缘的 flat/anisotropic 深度写入永久拒绝轨迹。
            shape_complete_for_3d_tracking = (
                complete_for_3d_tracking
                and detection_2d.aspect_ratio >= float(
                    self.get_parameter('min_multiview_aspect_ratio').value
                )
            )
            localization = None
            depth_shape = None
            raw_surface_depth_m = None
            if depth_synchronized:
                depth_shape = evaluate_sphere_depth_shape(
                    depth_image=self.latest_depth_image,
                    bbox=bbox,
                    max_depth_m=float(self.get_parameter('max_detection_range_m').value),
                    min_points_per_region=int(self.get_parameter('min_sphere_depth_shape_points').value),
                    min_curvature_m=float(self.get_parameter('min_sphere_depth_curvature_m').value),
                    min_axis_points=int(
                        self.get_parameter('min_sphere_axis_depth_points').value
                    ),
                    min_axis_curvature_ratio=float(
                        self.get_parameter('min_sphere_axis_curvature_ratio').value
                    ),
                )
                raw_surface_depth_m, _ = estimate_depth_from_bbox(
                    depth_image=self.latest_depth_image,
                    bbox=bbox,
                    padding_px=(
                        int(self.get_parameter('roi_padding_px').value)
                        if shape_complete_for_3d_tracking else 0
                    ),
                    max_depth_m=float(self.get_parameter('max_detection_range_m').value),
                    min_points=int(self.get_parameter('min_depth_points_in_roi').value),
                )
            depth_shape_status = depth_shape.status if depth_shape else 'unknown'
            # 圆柱端面、立方体/平板等在单帧可能都有近圆形红色投影。只有深度明确
            # 显示平面或轮廓明显非圆时才抑制正证据；unknown 保留给多视角策略。
            # 非圆视角仍进入轨迹用于复查，但不能污染后续完整球视角的尺寸/圆度统计。
            confirmation_eligible = (
                shape_complete_for_3d_tracking
                and depth_shape_status not in ('flat', 'anisotropic', 'non_spherical')
            )
            if self.camera_intrinsics and depth_synchronized and camera_to_output:
                localization = localize_bbox_from_depth_image(
                    bbox=bbox,
                    intrinsics=self.camera_intrinsics,
                    depth_image=self.latest_depth_image,
                    camera_to_output=camera_to_output,
                    output_frame=output_frame,
                    # 局部、粘连或贴边框不是完整球直径：只保留表面点供复查，
                    # 不扩大 ROI，也不允许用标准半径反推球心。
                    roi_padding_px=(
                        int(self.get_parameter('roi_padding_px').value)
                        if shape_complete_for_3d_tracking else 0
                    ),
                    max_depth_m=float(self.get_parameter('max_detection_range_m').value),
                    min_points=int(self.get_parameter('min_depth_points_in_roi').value),
                    sphere_radius_m=(
                        float(self.get_parameter('sphere_radius_m').value)
                        if shape_complete_for_3d_tracking else 0.0
                    ),
                    use_sphere_projection_geometry=(
                        shape_complete_for_3d_tracking
                        and bool(self.get_parameter('use_sphere_projection_geometry').value)
                    ),
                    camera_axis_convention=str(
                        self.get_parameter('camera_axis_convention').value
                    ),
                )
            apparent_diameter_m = (
                _apparent_diameter_m(
                    bbox, raw_surface_depth_m,
                    self.camera_intrinsics,
                )
                if shape_complete_for_3d_tracking and raw_surface_depth_m is not None
                else None
            )
            view_bearing_rad = _view_bearing_from_camera(
                camera_to_output, localization.position if localization else None,
            )

            source_id = f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec}:{index}'
            detections_2d_payload.append({
                'id': index,
                # 仅用于本回调内把二维投影关联结果传给对应三维观测，发布前删除。
                '_source_id': source_id,
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
                'requires_reobservation': (
                    detection_2d.requires_reobservation
                    or not shape_complete_for_3d_tracking
                    or not confirmation_eligible
                ),
                'may_be_merged': detection_2d.may_be_merged,
                'from_merged_split': detection_2d.from_merged_split,
                'quality_reason': detection_2d.quality_reason,
                'shape': {
                    'circularity': detection_2d.circularity,
                    'aspect_ratio': detection_2d.aspect_ratio,
                    'extent': detection_2d.extent,
                    'visible_centroid_x_ratio': (
                        detection_2d.visible_centroid_x_ratio
                    ),
                },
                'depth_shape': {
                    'status': depth_shape_status,
                    'center_depth_m': depth_shape.center_depth_m if depth_shape else None,
                    'outer_depth_m': depth_shape.outer_depth_m if depth_shape else None,
                    'curvature_m': depth_shape.curvature_m if depth_shape else None,
                    'horizontal_curvature_m': (
                        depth_shape.horizontal_curvature_m if depth_shape else None
                    ),
                    'vertical_curvature_m': (
                        depth_shape.vertical_curvature_m if depth_shape else None
                    ),
                    'diagonal_positive_curvature_m': (
                        depth_shape.diagonal_positive_curvature_m if depth_shape else None
                    ),
                    'diagonal_negative_curvature_m': (
                        depth_shape.diagonal_negative_curvature_m if depth_shape else None
                    ),
                    'curvature_isotropy_ratio': (
                        depth_shape.curvature_isotropy_ratio if depth_shape else None
                    ),
                    'center_points': depth_shape.center_points if depth_shape else 0,
                    'outer_points': depth_shape.outer_points if depth_shape else 0,
                    'horizontal_points': depth_shape.horizontal_points if depth_shape else 0,
                    'vertical_points': depth_shape.vertical_points if depth_shape else 0,
                    'diagonal_positive_points': (
                        depth_shape.diagonal_positive_points if depth_shape else 0
                    ),
                    'diagonal_negative_points': (
                        depth_shape.diagonal_negative_points if depth_shape else 0
                    ),
                },
                'depth_synchronized': depth_synchronized,
                'depth_stamp_delta_sec': depth_stamp_delta_sec,
                # 尺寸门使用深度图直接量得的可见表面深度，避免用已知球半径
                # 反推的球心深度再次验证直径，形成循环证据。
                'raw_surface_depth_m': raw_surface_depth_m,
                'tf_synchronized': self._last_tf_synchronized,
                'tf_stamp_delta_sec': self._last_tf_stamp_delta_sec,
                'confirmation_eligible': confirmation_eligible,
                'apparent_diameter_m': apparent_diameter_m,
                'view_bearing_deg': (
                    math.degrees(view_bearing_rad) if view_bearing_rad is not None else None
                ),
                'localization_status': (
                    'suppressed_non_spherical_depth_shape'
                    if depth_shape_status in ('flat', 'anisotropic', 'non_spherical')
                    else 'localized' if localization else 'unlocalized'
                ),
                'localized_position': (
                    [
                        round(localization.position.x, 4),
                        round(localization.position.y, 4),
                        round(localization.position.z, 4),
                    ]
                    if localization else None
                ),
                'source': 'hsv_minimal',
                'detector_backend': self.detector_backend.name,
            })

            # partial/粘连/贴边框的 bbox 会把完整球半径先验放大为错误世界坐标，
            # 因而只能保留二维复查语义，不能创建或更新三维轨迹。完整严格框的
            # flat/anisotropic 深度反证仍可进入轨迹，用于跨视角拒绝非球体。
            if localization and shape_complete_for_3d_tracking:
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

        detections_2d_payload = self.candidate_memory.annotate(
            detections_2d_payload, stamp_sec,
        )
        if camera_stable and observations and self.tracker.tracks:
            # 先用上一时刻轨迹投影关联当前二维框。合法 SLAM 长距离漂移会让同一
            # 静态球的世界坐标超过 merge_distance_m，但其旧轨迹投影仍可与当前
            # 图像框唯一重合；把这个非真值提示交给三维跟踪器可避免拆成重复目标。
            previous_projections = project_tracks_for_image_association(
                self.tracker.tracks,
                camera_to_output,
                self.camera_intrinsics,
                current_stamp_sec=stamp_sec,
                max_track_age_s=float(
                    self.get_parameter('track_projection_max_age_s').value
                ),
                sphere_radius_m=float(
                    self.get_parameter('sphere_radius_m').value
                ),
                max_depth_m=float(
                    self.get_parameter('max_detection_range_m').value
                ),
                camera_axis_convention=str(
                    self.get_parameter('camera_axis_convention').value
                ),
            )
            preassociated = annotate_detections_with_tracks(
                detections_2d_payload,
                self.tracker.tracks,
                merge_distance_m=float(
                    self.get_parameter('merge_distance_m').value
                ),
                projected_tracks=previous_projections,
            )
            hint_by_source_id = {
                str(item.get('_source_id')): int(
                    item.get('track_id')
                    or item.get('_candidate_track_id_hint')
                )
                for item in preassociated
                if str(item.get('_source_id', '')).strip()
                and str(
                    item.get('track_id')
                    or item.get('_candidate_track_id_hint')
                    or ''
                ).isdigit()
            }
            for observation in observations:
                observation.track_id_hint = hint_by_source_id.get(
                    str(observation.source_id),
                )
        for item in detections_2d_payload:
            item.pop('_source_id', None)

        if camera_stable:
            self.tracker.update(observations, stamp_sec=stamp_sec)
        tracks_to_publish = (
            self.tracker.published_tracks()
            if camera_stable else self.tracker.active_tracks()
        )
        projected_tracks = project_tracks_for_image_association(
            self.tracker.tracks,
            camera_to_output,
            self.camera_intrinsics,
            current_stamp_sec=stamp_sec,
            max_track_age_s=float(
                self.get_parameter('track_projection_max_age_s').value
            ),
            sphere_radius_m=float(self.get_parameter('sphere_radius_m').value),
            max_depth_m=float(self.get_parameter('max_detection_range_m').value),
            camera_axis_convention=str(
                self.get_parameter('camera_axis_convention').value
            ),
        )
        annotated_detections = annotate_detections_with_tracks(
            detections_2d_payload,
            self.tracker.tracks,
            merge_distance_m=float(self.get_parameter('merge_distance_m').value),
            projected_tracks=projected_tracks,
        )
        self.candidate_memory.remember_track_ids(annotated_detections)
        for item in annotated_detections:
            item.pop('_candidate_track_id_hint', None)
        hazards = attach_candidate_aliases_to_hazards(
            self._tracks_to_hazards(tracks_to_publish, output_frame),
            annotated_detections,
        )
        self._publish_detection_payload(
            hazards=hazards,
            detections_2d=annotated_detections,
            camera_stable=camera_stable,
            image_width=msg.width,
            image_height=msg.height,
            stamp_sec=stamp_sec,
            processing_time_ms=(
                time.perf_counter() - processing_started
            ) * 1000.0,
        )

    def _tracks_to_hazards(self, tracks, output_frame):
        hazards = []
        for track in tracks:
            item = track_to_hazard_dict(track)
            item['position_frame_id'] = output_frame
            item['localization_provenance'] = str(
                self.get_parameter('localization_provenance').value
            )
            item['source'] = 'hsv_depth_tf'
            # 最终结果层不能只相信可伪造的 evidence_status 字符串；把本轮实际
            # 确认门槛随轨迹一并发布，使决策层可复核视角数、球面深度正证据和
            # 横向视差。缺少任一字段时比赛结果必须 fail-closed。
            item['required_min_eligible_observations'] = int(
                self.get_parameter('confirm_observation_count').value
            )
            item['required_min_distinct_views'] = int(
                self.get_parameter('confirm_distinct_views').value
            )
            item['required_min_spherical_views'] = int(
                self.get_parameter('min_spherical_views_for_confirm').value
            )
            item['required_min_view_bearing_span_deg'] = float(
                self.get_parameter('min_view_bearing_span_deg').value
            )
            item['observation_time'] = time.time()
            hazards.append(item)
        return hazards

    def _active_view_policy_config(self):
        """从 ROS 参数构造主动复查配置，避免运行阈值散落在策略调用处。"""

        return ActiveViewPolicyConfig(
            edge_margin_ratio=float(
                self.get_parameter('active_view_edge_margin_ratio').value
            ),
            min_bbox_area_px=int(
                self.get_parameter('active_view_min_bbox_area_px').value
            ),
            min_red_pixel_count=int(
                self.get_parameter('active_view_min_red_pixel_count').value
            ),
            min_circularity=float(
                self.get_parameter('active_view_min_circularity').value
            ),
            min_confidence=float(
                self.get_parameter('active_view_min_confidence').value
            ),
            far_distance_m=float(
                self.get_parameter('active_view_far_distance_m').value
            ),
            dense_iou_threshold=float(
                self.get_parameter('active_view_dense_iou_threshold').value
            ),
            min_normalized_depth_curvature=float(
                self.get_parameter(
                    'active_view_min_normalized_depth_curvature'
                ).value
            ),
            max_normalized_depth_curvature=float(
                self.get_parameter(
                    'active_view_max_normalized_depth_curvature'
                ).value
            ),
        )

    def _publish_detection_payload(
            self, hazards, detections_2d, camera_stable=False,
            image_width=0, image_height=0, stamp_sec=None,
            processing_time_ms=None):
        # 已确认或已由多视角拒绝的轨迹不再请求机动。其余当前帧候选由感知侧
        # 统一计算明确动作，导航只执行契约，避免两组各自重写一套判据。
        recommendation_candidates = [
            item for item in detections_2d
            if item.get('track_status') not in (
                'confirmed', 'rejected', 'rejected_non_spherical',
            )
        ]
        recommendation = choose_active_view_action(
            recommendation_candidates,
            image_width=int(image_width),
            image_height=int(image_height),
            config=self._active_view_policy_config(),
        )
        recommendation = self.active_view_direction_memory.stabilize(
            recommendation,
            recommendation_candidates,
            image_width=int(image_width),
            stamp_sec=(
                float(stamp_sec) if stamp_sec is not None else time.time()
            ),
        ).to_dict()
        localization_provenance = str(
            self.get_parameter('localization_provenance').value).strip()
        localization_ready = (
            self.camera_intrinsics is not None
            and self._last_depth_synchronized
            and self._last_tf_synchronized
            and localization_provenance not in ('', 'unverified')
        )
        payload = {
            'hazards': hazards,
            'detections_2d': detections_2d,
            'view_recommendation': recommendation,
            'required_min_view_bearing_span_deg': float(
                self.get_parameter('min_view_bearing_span_deg').value
            ),
            'image_width': int(image_width),
            'image_height': int(image_height),
            'stamp_sec': float(stamp_sec) if stamp_sec is not None else time.time(),
            'localization_ready': localization_ready,
            'depth_synchronized': self._last_depth_synchronized,
            'tf_synchronized': self._last_tf_synchronized,
            'tf_stamp_delta_sec': self._last_tf_stamp_delta_sec,
            'localization_provenance': localization_provenance,
            'camera_stable': bool(camera_stable),
            'stable_view_frame_count': self._stable_view_frame_count,
            'processing_time_ms': (
                round(float(processing_time_ms), 3)
                if processing_time_ms is not None else None
            ),
        }
        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(out)
        recommendation_out = String()
        recommendation_out.data = json.dumps(
            recommendation, ensure_ascii=False)
        self.recommendation_pub.publish(recommendation_out)
        if self._image_callback_count <= 2:
            self.get_logger().info('Published perception payload for RGB frame %d.' % self._image_callback_count)

    def _update_camera_stability(self, transform):
        """根据精确相机世界位姿判断当前帧是否属于停靠稳定视角。"""

        axis_convention = str(
            self.get_parameter('camera_axis_convention').value
        )
        try:
            signature = camera_pose_signature(transform, axis_convention)
        except ValueError:
            signature = None
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
            self._stable_view_id = quantized_camera_view_id(
                transform, axis_convention,
            )
        return stable

    def _lookup_camera_to_output(self, camera_frame, output_frame, stamp):
        self._last_tf_synchronized = False
        self._last_tf_stamp_delta_sec = None
        if not camera_frame:
            return None
        if camera_frame == output_frame:
            self._last_tf_synchronized = True
            self._last_tf_stamp_delta_sec = 0.0
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
                    delta_sec = _stamp_delta_sec(
                        stamp, latest_transform.header.stamp)
                    maximum_delta_sec = float(
                        self.get_parameter(
                            'max_latest_tf_fallback_delta_sec').value)
                    self._last_tf_stamp_delta_sec = delta_sec
                    if delta_sec is None or delta_sec > maximum_delta_sec:
                        self.get_logger().warn(
                            f'Latest TF rejected: delta={delta_sec} sec exceeds '
                            f'{maximum_delta_sec:.3f} sec from '
                            f'{camera_frame} to {output_frame}.',
                            throttle_duration_sec=5.0,
                        )
                        return None
                    self._last_tf_synchronized = True
                    self.get_logger().warn(
                        f'TF at image stamp unavailable from {camera_frame} to '
                        f'{output_frame}; using bounded latest TF '
                        f'(delta={delta_sec:.3f}s): {exc}',
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
        self._last_tf_synchronized = True
        self._last_tf_stamp_delta_sec = 0.0
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


def _stamp_delta_sec(left, right):
    """返回两个 ROS 时间戳的绝对差；任一缺失时明确返回 None。"""

    if left is None or right is None:
        return None
    return abs(_stamp_to_float(left) - _stamp_to_float(right))


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


"""启动 ROS 节点,用于在仿真或 fake platform 下验证图像检测链路。"""
def main():
    rclpy.init()
    node = HsvDetectorNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        # 官方一键栈停止时 ROS 上下文可先于节点关闭，不把正常退出误记为感知异常。
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
