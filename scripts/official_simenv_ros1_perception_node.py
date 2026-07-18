#!/usr/bin/env python3
"""官方 SimEnv ROS1 Noetic 的 RGB-D 红球感知与安全结果导出节点。

所属组：感知定位组 / 系统集成组。
文件作用：
1. 订阅官方公开的 RealSense RGB、对齐深度图和 CameraInfo；
2. 使用 HSV/OpenCV 红球候选、深度球面证据和自建 SLAM TF 进行三维跟踪；
3. 只把三个独立稳定视角确认后的红球写入官方 results/detected_danger.json。

关键边界：
1. 不读取 danger_truth、layout_metadata、world 文件或 ground_truth 话题；
2. 红方块、圆柱、圆锥、局部可见球只能成为 reobserve/反证，不能直接提交；
3. world <- camera 的 TF 必须由团队自身 SLAM/定位系统发布，不能使用裁判真值；
4. 本文件保持 Python 3.8 语法兼容，以适配 ROS1 Noetic 默认运行时。

验证方式：
1. 离线运行 python scripts/run_offline_tests.py；
2. 官方环境 source devel/setup.bash 后运行 scripts/run_official_simenv_ros1_perception.sh；
3. 检查 /hazardwalker/perception/hazard_detections 与 results/detected_danger.json。
"""

import json
import math
import os
import sys
import time
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image, Joy
from std_msgs.msg import String


# 允许该脚本直接从官方 SimEnv 根目录调用，不要求先把 HazardWalker 打进 catkin。
REPO_ROOT = Path(__file__).resolve().parents[1]
for package_root in (
    REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception',
    REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_decision',
):
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

from hazardwalker_decision.official_simenv_contract import activation_command
from hazardwalker_decision.result_builder import build_official_detected_danger_result
from hazardwalker_perception.active_view_geometry import plan_lateral_reobservation
from hazardwalker_perception.active_view_policy import choose_active_view_action
from hazardwalker_perception.localize_hazard import (
    CameraIntrinsics,
    Point3D,
    RigidTransform3D,
    camera_intrinsics_from_k,
    evaluate_sphere_depth_shape,
    localize_bbox_from_depth_image,
)
from hazardwalker_perception.red_ball_detector import detect_red_balls_rgb_bytes
from hazardwalker_perception.track_hazards import (
    HazardObservation,
    HazardTracker,
    HazardTrackerConfig,
    track_to_hazard_dict,
)


class OfficialRgbdPerceptionNode(object):
    """把官方 RGB-D 流变为可提交的 confirmed 红球世界坐标。"""

    def __init__(self):
        rospy.init_node('hazardwalker_official_rgbd_perception', anonymous=False)
        self.bridge = CvBridge()
        self.world_frame = rospy.get_param('~world_frame', 'world')
        # 团队自己的 SLAM 应发布 localize_frame <- camera。官方不允许读 Gazebo
        # world 真值、scene_manifest 或布局文件；局部坐标到 world 的唯一外部先验
        # 是启动方显式传入的公开出生点。默认值与官方 docs/reference.md 的默认
        # ROBOT_X/Y/Z/YAW 一致；若平台改动出生点，必须在启动命令中显式记录这四项。
        self.localization_frame = rospy.get_param('~localization_frame', 'start')
        # 结果导出必须显式声明团队自建、且规则允许的定位来源。默认未验证，
        # 即使碰巧存在 TF 也只能用于候选/复查，不能写进官方结果文件。
        self.localization_provenance = str(rospy.get_param(
            '~localization_provenance', 'unverified',
        ))
        self.public_start_world_x = float(rospy.get_param('~public_start_world_x', 0.0))
        self.public_start_world_y = float(rospy.get_param('~public_start_world_y', -2.2))
        self.public_start_world_z = float(rospy.get_param('~public_start_world_z', 0.6))
        self.public_start_world_yaw = float(rospy.get_param(
            '~public_start_world_yaw', 1.5708,
        ))
        self.world_from_localization = self._build_public_start_world_transform()
        self.rgb_topic = rospy.get_param('~rgb_topic', '/real_sense/rgb/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/real_sense/depth/image_raw')
        self.rgb_info_topic = rospy.get_param(
            '~rgb_camera_info_topic', '/real_sense/rgb/camera_info',
        )
        # CameraInfo/深度反投影天然得到 ROS 光学坐标（X 右、Y 下、Z 前），
        # 但官方 Gazebo ``real_sense`` TF 使用机体链路坐标（X 前、Y 左、Z 上）。
        # 正式入口必须显式完成轴转换，否则检测框虽然正确，world 三维坐标会整体错轴。
        self.camera_axis_convention = str(rospy.get_param(
            '~camera_axis_convention', 'gazebo_link_x_forward',
        ))
        # 官方相机流的 RGB/深度时间戳并非逐帧完全相同。0.12 秒在容器繁忙时
        # 会造成同步器长期无回调；0.25 秒仍远小于机器人静止观察周期，且可由
        # 参数收紧，避免把相隔过远的帧错误配对。
        self.rgbd_sync_slop_s = float(rospy.get_param('~rgbd_sync_slop_s', 0.25))
        # 运行节点采用比离线兼容函数更严格的轮廓填充阈值；接近实心矩形的
        # 红色物体只能走 reobserve 分支，避免把其当作普通球形正证据。
        self.strict_max_extent = float(rospy.get_param('~strict_max_extent', 0.82))
        self.strict_min_area_px = int(rospy.get_param('~strict_min_area_px', 200))
        self.strict_min_circularity = float(rospy.get_param('~strict_min_circularity', 0.65))
        self.min_sphere_depth_curvature_m = float(rospy.get_param(
            '~min_sphere_depth_curvature_m', 0.008,
        ))
        self.min_sphere_depth_shape_points = int(rospy.get_param(
            '~min_sphere_depth_shape_points', 8,
        ))
        self.min_sphere_axis_depth_points = int(rospy.get_param(
            '~min_sphere_axis_depth_points', 4,
        ))
        self.min_sphere_axis_curvature_ratio = float(rospy.get_param(
            '~min_sphere_axis_curvature_ratio', 0.35,
        ))
        self.output_path = Path(rospy.get_param(
            '~output_path', 'results/detected_danger.json',
        )).expanduser()
        # 仅前后靠近目标并不会显露圆柱端面和球体的轮廓差异；该角度由真实
        # world <- camera TF 和三维候选位置计算，不能由 view_id 字符串伪造。
        self.min_view_bearing_span_deg = float(rospy.get_param(
            '~min_view_bearing_span_deg', 25.0,
        ))
        self.reobserve_max_step_distance_m = float(rospy.get_param(
            '~reobserve_max_step_distance_m', 0.45,
        ))
        self.mission_state_topic = rospy.get_param(
            '~mission_state_topic', '/hazardwalker/mission/state',
        )
        # 官方 junior_ctrl 的公开控制契约：/joy 的 button[1] 为站立，
        # button[3] 为进入 RL /cmd_vel 模式。默认关闭，避免诊断时意外接管
        # 机器人；正式自主运行由启动脚本显式打开。
        self.auto_activate_cmd_vel = bool(rospy.get_param(
            '~auto_activate_cmd_vel', False,
        ))
        self.activation_initial_delay_s = float(rospy.get_param(
            '~activation_initial_delay_s', 1.0,
        ))
        self.activation_stand_hold_s = float(rospy.get_param(
            '~activation_stand_hold_s', 0.8,
        ))
        self.activation_settle_s = float(rospy.get_param(
            '~activation_settle_s', 4.0,
        ))
        self.activation_rl_hold_s = float(rospy.get_param(
            '~activation_rl_hold_s', 0.8,
        ))
        self._activation_started_at = None
        self._activation_phase = 'disabled'
        self.camera_intrinsics = None
        self.started_monotonic = time.monotonic()
        self.finished = False
        self._last_pose = None
        self._stable_frames = 0
        self._stable_view_id = ''
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tracker = HazardTracker(HazardTrackerConfig(
            confirm_observation_count=3,
            min_distinct_views=3,
            reject_after_missed_count=300,
            merge_distance_m=0.50,
            max_apparent_diameter_cv=0.35,
            expected_sphere_diameter_m=0.30,
            max_sphere_diameter_relative_error=0.35,
            min_multiview_aspect_ratio=0.88,
            max_depth_curvature_cv=0.65,
            min_normalized_depth_curvature=0.10,
            max_median_normalized_depth_curvature=0.30,
            min_view_bearing_span_deg=self.min_view_bearing_span_deg,
            # 两个独立稳定视角必须各自有 RGB-D 球面正证据；单视角圆柱端面、
            # 圆锥端面或深度未知圆斑一律不能进入最终危险源文件。
            min_spherical_views_for_confirm=2,
        ))

        rospy.Subscriber(self.rgb_info_topic, CameraInfo, self._on_camera_info, queue_size=1)
        rospy.Subscriber(self.mission_state_topic, String, self._on_mission_state, queue_size=5)
        rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=20, slop=self.rgbd_sync_slop_s,
        )
        sync.registerCallback(self._on_rgbd)
        self.sync = sync
        self.payload_pub = rospy.Publisher(
            '/hazardwalker/perception/hazard_detections', String, queue_size=10,
        )
        # 这里只发送语义重观察请求，绝不直接占用 /cmd_vel。导航层必须结合
        # 自身避障和真实位姿反馈执行，才能把横移变成可审计的独立视角。
        self.reobserve_topic = rospy.get_param(
            '~reobserve_topic', '/hazardwalker/perception/reobservation_request',
        )
        self.reobserve_pub = rospy.Publisher(self.reobserve_topic, String, queue_size=10)
        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=4)
        if self.auto_activate_cmd_vel:
            self._activation_started_at = rospy.Time.now()
            self._activation_phase = 'waiting_for_controller'
            rospy.Timer(rospy.Duration(0.1), self._drive_controller_activation)
        rospy.on_shutdown(self._write_official_result)
        rospy.loginfo(
            'HazardWalker official RGB-D perception listens to %s + %s; '
            'uses team-owned %s <- camera TF and public start pose for world output.',
            self.rgb_topic, self.depth_topic, self.localization_frame,
        )

    def _drive_controller_activation(self, _event):
        """无需人工键盘，把官方控制器安全切换到自主 /cmd_vel 模式。"""
        if self._activation_phase in ('ready', 'disabled'):
            return
        elapsed = (rospy.Time.now() - self._activation_started_at).to_sec()
        if elapsed < self.activation_initial_delay_s:
            return
        # 先等待 junior_ctrl 订阅，避免 ROS 启动竞态导致一次性指令丢失。
        if self.joy_pub.get_num_connections() < 1:
            return
        phase, button = activation_command(
            elapsed,
            self.activation_initial_delay_s,
            self.activation_stand_hold_s,
            self.activation_settle_s,
            self.activation_rl_hold_s,
        )
        self._publish_joy_button(button)
        was_ready = self._activation_phase == 'ready'
        self._activation_phase = phase
        if phase == 'ready' and not was_ready:
            rospy.loginfo('Official junior_ctrl is now in autonomous RL /cmd_vel mode.')

    def _publish_joy_button(self, button_index):
        """按官方 keyboard_teleop 的同一 11 键协议发布单一控制指令。"""
        message = Joy()
        message.header.stamp = rospy.Time.now()
        message.axes = [0.0] * 6
        message.buttons = [0] * 11
        if button_index is not None:
            message.buttons[button_index] = 1
        self.joy_pub.publish(message)

    def _on_camera_info(self, message):
        try:
            self.camera_intrinsics = camera_intrinsics_from_k(message.K)
        except ValueError as error:
            rospy.logwarn_throttle(5.0, 'Invalid RealSense CameraInfo: %s', error)

    def _on_mission_state(self, message):
        if message.data.strip().upper() == 'FINISHED':
            self.finished = True
            self._write_official_result()

    def _on_rgbd(self, rgb_message, depth_message):
        if self.camera_intrinsics is None:
            return
        rgb = self._rgb_image(rgb_message)
        depth = self._depth_meters(depth_message)
        if rgb is None or depth is None:
            return
        transform = self._world_from_camera(rgb_message.header.frame_id, rgb_message.header.stamp)
        camera_stable = self._update_stability(transform)
        detections = detect_red_balls_rgb_bytes(
            rgb.tobytes(), rgb.shape[1], rgb.shape[0], encoding='rgb8',
            min_area_px=self.strict_min_area_px,
            min_circularity=self.strict_min_circularity,
            include_partial_candidates=True,
            partial_min_area_px=20,
            partial_min_circularity=0.18,
            partial_min_aspect_ratio=0.12,
            partial_min_value=50,
            max_extent=self.strict_max_extent,
        )
        payload_detections = []
        observations = []
        for index, detection in enumerate(detections, start=1):
            bbox = {
                'x_min': detection.x_min, 'y_min': detection.y_min,
                'x_max': detection.x_max, 'y_max': detection.y_max,
            }
            depth_shape = evaluate_sphere_depth_shape(
                depth_image=depth,
                bbox=bbox,
                max_depth_m=20.0,
                min_points_per_region=self.min_sphere_depth_shape_points,
                min_curvature_m=self.min_sphere_depth_curvature_m,
                min_axis_points=self.min_sphere_axis_depth_points,
                min_axis_curvature_ratio=self.min_sphere_axis_curvature_ratio,
            )
            confirmation_eligible = (
                camera_stable
                and not detection.requires_reobservation
                and depth_shape.status not in ('flat', 'anisotropic', 'non_spherical')
                and detection.aspect_ratio >= 0.88
            )
            # 2D 严格轮廓也可能是圆柱/圆锥端面。对外字段必须显式表达
            # “需要复查”，而不能只因 HSV 轮廓较圆就让展示端画成已完成目标。
            requires_reobservation = bool(
                detection.requires_reobservation or not confirmation_eligible
            )
            item = {
                'id': index,
                'bbox': bbox,
                'confidence': round(float(detection.confidence), 4),
                'requires_reobservation': requires_reobservation,
                'image_requires_reobservation': bool(detection.requires_reobservation),
                'from_merged_split': bool(detection.from_merged_split),
                'confirmation_eligible': bool(confirmation_eligible),
                # 记录真实稳定视角 ID，供正式证据记录器核验；运动帧为空，
                # 绝不能通过伪造字符串累计多视角。
                'view_id': self._stable_view_id if camera_stable else '',
                'red_pixel_count': int(detection.red_pixel_count),
                'shape': {
                    'circularity': round(float(detection.circularity), 4),
                    'aspect_ratio': round(float(detection.aspect_ratio), 4),
                    'extent': round(float(detection.extent), 4),
                },
                'depth_shape': {
                    'status': depth_shape.status,
                    'curvature_m': depth_shape.curvature_m,
                    'horizontal_curvature_m': depth_shape.horizontal_curvature_m,
                    'vertical_curvature_m': depth_shape.vertical_curvature_m,
                    'diagonal_positive_curvature_m': (
                        depth_shape.diagonal_positive_curvature_m
                    ),
                    'diagonal_negative_curvature_m': (
                        depth_shape.diagonal_negative_curvature_m
                    ),
                    'curvature_isotropy_ratio': depth_shape.curvature_isotropy_ratio,
                    'horizontal_points': depth_shape.horizontal_points,
                    'vertical_points': depth_shape.vertical_points,
                    'diagonal_positive_points': depth_shape.diagonal_positive_points,
                    'diagonal_negative_points': depth_shape.diagonal_negative_points,
                },
            }
            if transform is not None:
                localization = localize_bbox_from_depth_image(
                    bbox=bbox,
                    intrinsics=self.camera_intrinsics,
                    depth_image=depth,
                    camera_to_output=transform,
                    output_frame=self.world_frame,
                    sphere_radius_m=0.15,
                    use_sphere_projection_geometry=True,
                    camera_axis_convention=self.camera_axis_convention,
                )
            else:
                localization = None
            if localization is not None:
                apparent_diameter_m = _apparent_diameter_m(
                    bbox, localization.depth_m, self.camera_intrinsics,
                )
                view_bearing_rad = _view_bearing_from_camera(transform, localization.position)
                observations.append(HazardObservation(
                    position=(
                        localization.position.x, localization.position.y,
                        localization.position.z,
                    ),
                    confidence=detection.confidence,
                    stamp_sec=_stamp_to_seconds(rgb_message.header.stamp),
                    source_id='%s.%s:%d' % (
                        rgb_message.header.stamp.secs,
                        rgb_message.header.stamp.nsecs,
                        index,
                    ),
                    view_id=self._stable_view_id if camera_stable else '',
                    confirmation_eligible=confirmation_eligible,
                    depth_shape_status=depth_shape.status,
                    apparent_diameter_m=apparent_diameter_m,
                    aspect_ratio=detection.aspect_ratio,
                    depth_curvature_m=depth_shape.curvature_m,
                    view_bearing_rad=view_bearing_rad,
                ))
                item['position'] = [
                    round(localization.position.x, 4), round(localization.position.y, 4),
                    round(localization.position.z, 4),
                ]
                item['position_frame_id'] = self.world_frame
                item['depth_m'] = round(float(localization.depth_m), 4)
                if view_bearing_rad is not None:
                    item['view_bearing_rad'] = round(float(view_bearing_rad), 6)
            payload_detections.append(item)

        if camera_stable and transform is not None:
            active_tracks = self.tracker.update(
                observations, stamp_sec=_stamp_to_seconds(rgb_message.header.stamp),
            )
        else:
            # 运动帧可用于导航选视角，但绝不可累计成独立确认视角。
            active_tracks = self.tracker.active_tracks()
        hazards = []
        for track in active_tracks:
            item = track_to_hazard_dict(track)
            item['position_frame_id'] = self.world_frame
            item['localization_provenance'] = self.localization_provenance
            item['source'] = 'official_ros1_rgbd'
            hazards.append(item)
        self._publish_payload(
            hazards, payload_detections, camera_stable,
            _stamp_to_seconds(rgb_message.header.stamp),
            localization_ready=transform is not None,
        )
        self._publish_reobservation_request(
            payload_detections, rgb.shape[1], rgb.shape[0], camera_stable,
            _stamp_to_seconds(rgb_message.header.stamp), transform,
        )

    def _rgb_image(self, message):
        try:
            return self.bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
        except Exception as error:
            rospy.logwarn_throttle(5.0, 'Cannot decode RGB image: %s', error)
            return None

    def _depth_meters(self, message):
        try:
            depth = self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
        except Exception as error:
            rospy.logwarn_throttle(5.0, 'Cannot decode depth image: %s', error)
            return None
        if message.encoding.upper() == '16UC1':
            return depth.astype(np.float32) * 0.001
        if message.encoding.upper() == '32FC1':
            return depth.astype(np.float32)
        rospy.logwarn_throttle(5.0, 'Unsupported depth encoding: %s', message.encoding)
        return None

    def _world_from_camera(self, camera_frame, stamp):
        if not camera_frame:
            return None
        try:
            message = self.tf_buffer.lookup_transform(
                self.localization_frame, camera_frame, stamp, rospy.Duration(0.10),
            )
        except Exception as error:
            rospy.logwarn_throttle(
                5.0,
                'No team-owned %s <- %s TF; candidates remain unconfirmed: %s',
                self.localization_frame, camera_frame, error,
            )
            return None
        local_from_camera = _transform_message_to_rigid(message.transform)
        return _compose_transforms(self.world_from_localization, local_from_camera)

    def _build_public_start_world_transform(self):
        """只由启动方明示的公开出生点建立 world <- start 变换。

        不读取 ``scene_manifest.json``：该文件同时暴露布局元数据、危险源数量和
        裁判真值路径，即便只取其中一个字段也不应进入正式比赛算法输入链。
        """
        if self.localization_frame == self.world_frame:
            return RigidTransform3D(
                translation=Point3D(0.0, 0.0, 0.0),
                rotation=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            )
        cosine, sine = math.cos(self.public_start_world_yaw), math.sin(self.public_start_world_yaw)
        return RigidTransform3D(
            translation=Point3D(
                self.public_start_world_x,
                self.public_start_world_y,
                self.public_start_world_z,
            ),
            rotation=((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0)),
        )

    def _update_stability(self, transform):
        if transform is None:
            self._last_pose = None
            self._stable_frames = 0
            self._stable_view_id = ''
            return False
        pose = _pose_signature(transform)
        if self._last_pose is None:
            self._stable_frames = 1
            self._stable_view_id = ''
        else:
            distance = math.sqrt(sum((pose[index] - self._last_pose[index]) ** 2 for index in range(3)))
            yaw_delta = abs(math.degrees(_normalize_angle(pose[3] - self._last_pose[3])))
            if distance <= 0.002 and yaw_delta <= 0.3:
                self._stable_frames += 1
            else:
                self._stable_frames = 1
                self._stable_view_id = ''
        self._last_pose = pose
        stable = self._stable_frames >= 3
        if stable and not self._stable_view_id:
            self._stable_view_id = _stable_view_id(transform)
        return stable

    def _publish_payload(
            self, hazards, detections, camera_stable, stamp_sec, localization_ready):
        self.payload_pub.publish(String(data=json.dumps({
            'hazards': hazards,
            'detections_2d': detections,
            'camera_stable': bool(camera_stable),
            'stamp_sec': round(float(stamp_sec), 6),
            'localization_ready': bool(localization_ready),
            'localization_provenance': self.localization_provenance,
            'output_frame': self.world_frame,
        }, ensure_ascii=False)))

    def _publish_reobservation_request(
            self, detections, image_width, image_height, camera_stable, stamp_sec, transform):
        """发布候选驱动的视角建议，执行层据此移动后再回传真实 TF。"""
        recommendation = choose_active_view_action(
            detections, image_width=image_width, image_height=image_height,
        )
        # 无候选时只保留常规探索语义，避免把空帧误当作紧急控制请求。
        if recommendation.action == 'continue_exploring':
            return
        target = next(
            (item for item in detections if str(item.get('id')) == str(recommendation.target_id)),
            None,
        )
        plan = None
        # 没有团队坐标或候选三维位置时，导航只能保留语义请求，不能猜测一个
        # 会碰撞的世界坐标目标。只有横移类建议才生成绕目标的侧视弧线。
        if transform is not None and target is not None and target.get('position') is not None:
            plan = plan_lateral_reobservation(
                camera_position=(
                    transform.translation.x, transform.translation.y, transform.translation.z,
                ),
                target_position=target['position'],
                action=recommendation.action,
                min_bearing_change_deg=self.min_view_bearing_span_deg,
                max_step_distance_m=self.reobserve_max_step_distance_m,
            ).to_dict()
        self.reobserve_pub.publish(String(data=json.dumps({
            'source': 'official_ros1_rgbd',
            'stamp_sec': round(float(stamp_sec), 6),
            'camera_stable': bool(camera_stable),
            'required_min_view_bearing_span_deg': self.min_view_bearing_span_deg,
            'recommendation': recommendation.to_dict(),
            'reobservation_plan': plan,
        }, ensure_ascii=False)))

    def _write_official_result(self):
        hazards = []
        for track in self.tracker.active_tracks():
            item = track_to_hazard_dict(track)
            item['position_frame_id'] = self.world_frame
            item['localization_provenance'] = self.localization_provenance
            hazards.append(item)
        result = build_official_detected_danger_result(
            hazards, time.monotonic() - self.started_monotonic,
            expected_frame=self.world_frame, dedup_distance_m=0.30,
            require_legal_localization=True,
            require_multiview_sphere_evidence=True,
        )
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output_path.with_suffix(self.output_path.suffix + '.tmp')
        temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        temporary.replace(self.output_path)
        rospy.loginfo(
            'Official result updated with %d confirmed red balls: %s',
            len(result['detected_danger_sources']), self.output_path,
        )


def _transform_message_to_rigid(transform):
    quaternion = transform.rotation
    qx, qy, qz, qw = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    else:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    rotation = (
        (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
        (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
        (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)),
    )
    return RigidTransform3D(
        translation=Point3D(transform.translation.x, transform.translation.y, transform.translation.z),
        rotation=rotation,
    )


def _compose_transforms(first, second):
    """组合 A<-B 与 B<-C，得到 A<-C；不依赖裁判或 Gazebo 真值位姿。"""
    rotation = tuple(tuple(
        sum(first.rotation[row][index] * second.rotation[index][column] for index in range(3))
        for column in range(3)
    ) for row in range(3))
    translated = Point3D(
        first.rotation[0][0] * second.translation.x + first.rotation[0][1] * second.translation.y + first.rotation[0][2] * second.translation.z + first.translation.x,
        first.rotation[1][0] * second.translation.x + first.rotation[1][1] * second.translation.y + first.rotation[1][2] * second.translation.z + first.translation.y,
        first.rotation[2][0] * second.translation.x + first.rotation[2][1] * second.translation.y + first.rotation[2][2] * second.translation.z + first.translation.z,
    )
    return RigidTransform3D(translation=translated, rotation=rotation)


def _pose_signature(transform):
    forward_x = transform.rotation[0][2]
    forward_y = transform.rotation[1][2]
    return (
        transform.translation.x, transform.translation.y, transform.translation.z,
        math.atan2(forward_y, forward_x),
    )


def _stable_view_id(transform):
    """只使用水平基线与朝向，竖直抖动不能伪造独立观察视角。"""
    yaw = math.degrees(math.atan2(transform.rotation[1][2], transform.rotation[0][2]))
    return 'xy:{:.1f}:{:.1f}|yaw:{:.0f}'.format(
        round(transform.translation.x / 0.4) * 0.4,
        round(transform.translation.y / 0.4) * 0.4,
        round(yaw / 30.0) * 30.0,
    )


def _normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _stamp_to_seconds(stamp):
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


def _apparent_diameter_m(bbox, depth_m, intrinsics):
    width = max(1.0, float(bbox['x_max']) - float(bbox['x_min']) + 1.0)
    height = max(1.0, float(bbox['y_max']) - float(bbox['y_min']) + 1.0)
    focal = max(1.0, (float(intrinsics.fx) + float(intrinsics.fy)) / 2.0)
    return max(width, height) * float(depth_m) / focal


def _view_bearing_from_camera(world_from_camera, target_position):
    """目标相对相机的水平视线，必须变化足够大才允许最终确认。"""
    dx = float(target_position.x) - float(world_from_camera.translation.x)
    dy = float(target_position.y) - float(world_from_camera.translation.y)
    if math.hypot(dx, dy) < 1e-4:
        return None
    return math.atan2(dy, dx)


def main():
    OfficialRgbdPerceptionNode()
    rospy.spin()


if __name__ == '__main__':
    main()
