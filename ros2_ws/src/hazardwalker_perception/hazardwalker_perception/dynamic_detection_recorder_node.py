"""动态红球检测实验记录 ROS 2 节点。

所属组：感知定位组。
文件作用：
订阅统一的 ``/hw/*`` 图像和感知结果，记录连续帧候选、多帧确认状态、
可选的合法 SLAM 位姿与主动重观察建议，并在结束时输出 summary 和测试记录。
当前实现边界：
只记录和发布建议到 ``/hw/perception/view_recommendation``，不直接控制机器人。
验证方式：
先运行离线测试；官方 SimEnv 需由平台层桥接到 ``/hw/*`` 后再实际运行。
"""

import csv
import json
import signal
import shutil
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from hazardwalker_perception.active_view_policy import choose_active_view_action
from hazardwalker_perception.dynamic_detection_records import (
    build_perception_evidence_contract,
    build_dynamic_summary,
    build_dynamic_testing_record,
)


class DynamicDetectionRecorderNode(Node):
    """将动态检测过程写成可复核的逐帧记录和测试组摘要。"""

    def __init__(self):
        super().__init__('dynamic_detection_recorder_node')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('test_record_dir', '')
        self.declare_parameter('scenario_name', 'official_simenv_dynamic_detection')
        self.declare_parameter('save_images', True)
        self.declare_parameter('save_depth_evidence', True)
        self.declare_parameter('min_image_save_interval_sec', 0.5)
        self.declare_parameter('max_rgb_depth_evidence_delta_sec', 0.15)
        self.declare_parameter('trajectory_sample_interval_sec', 0.5)
        # 官方赛题禁止 /Odometry_gazebo 及其平台桥接别名参与比赛算法或正式证据。
        # 因此默认不订阅任何位姿；只有调用方显式提供合法 SLAM 的 Odometry 话题时才记录。
        self.declare_parameter('legal_pose_topic', '')
        # 默认内部回归，只有联调者显式填入固定 SEED、代码版本和合法 SLAM 来源时才可能
        # 生成“可供正式复核”的元数据；该标记不会凭空证明场景或算法合规。
        self.declare_parameter('run_mode', 'internal_regression')
        self.declare_parameter('scenario_seed', '')
        self.declare_parameter('code_version', '')
        self.declare_parameter('localization_provenance', 'unverified')
        self.declare_parameter('launch_command', '')
        # ROS2 适配层默认使用 /hw/*；直接接入 ROS1 官方节点时可显式切换到
        # /real_sense/* 与 /hazardwalker/perception/*，避免为了记录证据再造一层假桥接。
        self.declare_parameter('image_topic', '/hw/camera/image_raw')
        self.declare_parameter('depth_topic', '/hw/camera/depth_image')
        self.declare_parameter('detection_topic', '/hw/perception/hazard_detections')
        self.declare_parameter('mission_state_topic', '/hw/mission/state')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('result_json_path', 'results/detected_danger.json')
        self.declare_parameter('context_save_interval_sec', 10.0)
        self.declare_parameter('max_context_evidence_count', 80)

        output_dir = str(self.get_parameter('output_dir').value).strip()
        test_record_dir = str(self.get_parameter('test_record_dir').value).strip()
        if not output_dir or not test_record_dir:
            raise ValueError('output_dir 和 test_record_dir 必须显式指定，避免把实验结果写入未知目录。')

        self.output_dir = Path(output_dir).expanduser().resolve()
        self.test_record_dir = Path(test_record_dir).expanduser().resolve()
        self.image_dir = self.output_dir / 'selected_images'
        self.raw_image_dir = self.image_dir / 'raw'
        self.annotated_image_dir = self.image_dir / 'annotated'
        self.depth_dir = self.output_dir / 'selected_depth'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.test_record_dir.mkdir(parents=True, exist_ok=True)
        if bool(self.get_parameter('save_images').value):
            self.raw_image_dir.mkdir(parents=True, exist_ok=True)
            self.annotated_image_dir.mkdir(parents=True, exist_ok=True)
        if bool(self.get_parameter('save_depth_evidence').value):
            self.depth_dir.mkdir(parents=True, exist_ok=True)
        self.evidence_contract = build_perception_evidence_contract(
            self.get_parameter('run_mode').value,
            self.get_parameter('scenario_seed').value,
            self.get_parameter('code_version').value,
            self.get_parameter('legal_pose_topic').value,
            self.get_parameter('localization_provenance').value,
        )
        _write_json(self.output_dir / 'run_manifest.json', {
            'schema': 'hazardwalker_perception_official_evidence_v2',
            'evidence_contract': self.evidence_contract,
            'input_topics': {
                'image': str(self.get_parameter('image_topic').value),
                'depth': str(self.get_parameter('depth_topic').value),
                'detections': str(self.get_parameter('detection_topic').value),
                'legal_pose': str(self.get_parameter('legal_pose_topic').value),
                'mission_state': str(self.get_parameter('mission_state_topic').value),
                'map': str(self.get_parameter('map_topic').value),
            },
            'mission_completion_required': True,
            'launch_command': str(self.get_parameter('launch_command').value),
            'started_at_unix_sec': round(time.time(), 3),
        })

        self.records = []
        self.latest_image = None
        self.latest_image_stamp = 0.0
        self.latest_image_frame_id = ''
        self.latest_depth_image = None
        self.latest_depth_stamp = 0.0
        self.latest_depth_frame_id = ''
        self.latest_legal_pose = None
        self.latest_map = None
        self.last_image_save_sec = float('-inf')
        self.last_pose_save_sec = float('-inf')
        self.trajectory_sample_count = 0
        self.mission_completed = False
        self.last_context_save_sec = float('-inf')
        self.context_evidence_count = 0
        self.saved_confirmation_ids = set()

        self.image_sub = self.create_subscription(
            Image, str(self.get_parameter('image_topic').value), self.on_image, 10,
        )
        self.depth_sub = self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value), self.on_depth, 10,
        )
        legal_pose_topic = str(self.get_parameter('legal_pose_topic').value).strip()
        self.legal_pose_sub = None
        if 'forbidden_pose_topic' in self.evidence_contract.get('contract_violations', []):
            self.get_logger().error(
                '拒绝订阅禁用位姿话题：不得把 Gazebo 真值或 /hw/odom 写入感知正式证据。')
        elif legal_pose_topic:
            self.legal_pose_sub = self.create_subscription(
                Odometry, legal_pose_topic, self.on_legal_pose, 20,
            )
        else:
            self.get_logger().warn(
                '未配置 legal_pose_topic：记录中不会写入机器人位姿，且不能作为正式定位证据。')
        self.detection_sub = self.create_subscription(
            String, str(self.get_parameter('detection_topic').value), self.on_detections, 20,
        )
        self.mission_state_sub = self.create_subscription(
            String,
            str(self.get_parameter('mission_state_topic').value),
            self.on_mission_state,
            10,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self.on_map,
            5,
        )
        self.recommendation_pub = self.create_publisher(String, '/hw/perception/view_recommendation', 10)
        self.get_logger().info(f'动态检测记录将写入 {self.output_dir}')

    def on_image(self, msg):
        """缓存最新 RGB/BGR 帧，供对应检测结果保存可视化证据。"""

        image = _image_message_to_array(msg)
        if image is None:
            self.get_logger().warn(f'不支持保存的图像编码：{msg.encoding}', throttle_duration_sec=5.0)
            return
        self.latest_image = image
        self.latest_image_stamp = _stamp_to_sec(msg.header.stamp)
        self.latest_image_frame_id = msg.header.frame_id

    def on_depth(self, msg):
        """缓存深度米制数组；仅在与 RGB 时间接近时才作为证据帧落盘。"""

        depth = _depth_message_to_meters(msg)
        if depth is None:
            self.get_logger().warn(f'不支持保存的深度编码：{msg.encoding}', throttle_duration_sec=5.0)
            return
        self.latest_depth_image = depth
        self.latest_depth_stamp = _stamp_to_sec(msg.header.stamp)
        self.latest_depth_frame_id = msg.header.frame_id

    def on_legal_pose(self, msg):
        """保存调用方声明的合法 SLAM 位姿，供正式证据追溯。"""

        pose = msg.pose.pose
        self.latest_legal_pose = {
            'frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,
            'position': {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z},
            'orientation': {
                'x': pose.orientation.x, 'y': pose.orientation.y,
                'z': pose.orientation.z, 'w': pose.orientation.w,
            },
            'stamp_sec': _stamp_to_sec(msg.header.stamp),
        }
        pose_stamp = self.latest_legal_pose['stamp_sec']
        interval = float(self.get_parameter('trajectory_sample_interval_sec').value)
        if pose_stamp - self.last_pose_save_sec >= interval:
            with (self.output_dir / 'trajectory.jsonl').open('a', encoding='utf-8') as handle:
                handle.write(json.dumps(self.latest_legal_pose, ensure_ascii=False) + '\n')
            self.last_pose_save_sec = pose_stamp
            self.trajectory_sample_count += 1

    def on_detections(self, msg):
        """记录一条感知输出，并为当前候选生成一次主动观察建议。"""

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'忽略无法解析的感知 JSON：{exc}', throttle_duration_sec=5.0)
            return

        detections = list(payload.get('detections_2d', []))
        image_height, image_width = self._latest_image_shape()
        recommendation_dict = payload.get('view_recommendation')
        if not isinstance(recommendation_dict, dict):
            recommendation_dict = choose_active_view_action(
                detections, image_width, image_height,
            ).to_dict()
        self._publish_recommendation(recommendation_dict)

        stamp_sec = float(payload.get(
            'stamp_sec',
            _payload_stamp_to_sec(
                detections,
                fallback=self.latest_image_stamp or time.time(),
            ),
        ))
        raw_image_path, image_path, depth_path = self._save_evidence_image(
            stamp_sec, detections, list(payload.get('hazards', [])), recommendation_dict,
        )
        record = {
            'timestamp_sec': stamp_sec,
            'image_frame_id': self.latest_image_frame_id,
            'robot_pose': self.latest_legal_pose,
            'localization_provenance': str(
                payload.get('localization_provenance',
                            self.get_parameter('localization_provenance').value)
            ),
            'detections_2d': detections,
            'hazards': list(payload.get('hazards', [])),
            'localization_ready': bool(payload.get('localization_ready', False)),
            'view_recommendation': recommendation_dict,
            'evidence_raw_image': raw_image_path,
            'evidence_image': image_path,
            'evidence_depth': depth_path,
        }
        self.records.append(record)
        with (self.output_dir / 'frames.jsonl').open('a', encoding='utf-8') as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + '\n')

    def on_mission_state(self, message):
        """只有导航返航后发布 FINISHED 才允许证据被视为完整任务。"""

        if str(message.data).strip() == 'FINISHED':
            self.mission_completed = True

    def on_map(self, message):
        """缓存最新合法 SLAM 占据栅格，统一收尾时写出可视地图证据。"""

        self.latest_map = message

    def close(self):
        """在节点退出时写入汇总和测试组 CSV/JSON。"""

        summary = build_dynamic_summary(self.records)
        failure_reasons = _derive_failure_reasons(
            self.records,
            self.trajectory_sample_count,
            self.evidence_contract,
            self.mission_completed,
        )
        _write_json(self.output_dir / 'failure_reasons.json', {
            'observed_failure_reasons': failure_reasons,
            'generated_at_unix_sec': round(time.time(), 3),
            'note': '只记录本次采集可直接观察到的失败信号；不根据真值推测漏检或虚警。',
        })
        summary.update({
            'scenario': str(self.get_parameter('scenario_name').value),
            'generated_at_unix_sec': round(time.time(), 3),
            'record_file': 'frames.jsonl',
            'trajectory_file': 'trajectory.jsonl' if self.trajectory_sample_count else '',
            'trajectory_sample_count': self.trajectory_sample_count,
            'run_manifest_file': 'run_manifest.json',
            'failure_reasons_file': 'failure_reasons.json',
            'evidence_contract': self.evidence_contract,
            'mission_completed': self.mission_completed,
            'map_snapshot_file': self._save_map_snapshot(),
        })
        _write_json(self.output_dir / 'summary.json', summary)

        result_path = Path(
            str(self.get_parameter('result_json_path').value)
        ).expanduser()
        if result_path.exists():
            result_copy_path = self.output_dir / 'detected_danger.json'
            # 正式全链路通常直接把结果写入证据目录。成功收尾时若再次把文件
            # 复制到自身，shutil 会抛 SameFileError，导致后续测试组 CSV/JSON
            # 永远缺失；只有源、目标确实不同才执行复制。
            if result_path.resolve() != result_copy_path.resolve():
                shutil.copy2(result_path, result_copy_path)

        testing_record = build_dynamic_testing_record(
            summary,
            scenario=str(self.get_parameter('scenario_name').value),
        )
        _write_json(self.test_record_dir / 'testing_record_perception.json', testing_record)
        _write_single_row_csv(self.test_record_dir / 'testing_record_perception.csv', testing_record)
        self.get_logger().info(f'动态检测汇总已写入 {self.output_dir / "summary.json"}')

    def _save_map_snapshot(self):
        """用 ROS map_server 兼容格式保存最后一帧地图，不依赖 GUI 或额外服务。"""

        if self.latest_map is None:
            return ''
        info = self.latest_map.info
        width = int(info.width)
        height = int(info.height)
        values = list(self.latest_map.data)
        if width <= 0 or height <= 0 or len(values) != width * height:
            return ''

        # OccupancyGrid 原点在左下；PGM 首行在上方，因此按行反转。已知格保留
        # 0..100 概率灰度，不能把 26..64 全部涂成纯白后掩盖实时栅格问题。
        pixels = bytearray()
        for row in range(height - 1, -1, -1):
            start = row * width
            for value in values[start:start + width]:
                if value < 0:
                    pixels.append(205)
                else:
                    probability = max(0, min(100, int(value)))
                    pixels.append(int(round(254.0 - probability * 2.54)))
        pgm_path = self.output_dir / 'cartographer_map.pgm'
        pgm_path.write_bytes(
            f'P5\n{width} {height}\n255\n'.encode('ascii') + bytes(pixels)
        )
        yaml_path = self.output_dir / 'cartographer_map.yaml'
        origin = info.origin
        yaw = _quaternion_yaw(origin.orientation)
        yaml_path.write_text(
            '\n'.join([
                'image: cartographer_map.pgm',
                'mode: scale',
                f'resolution: {float(info.resolution):.8f}',
                'origin: [%.8f, %.8f, %.8f]' % (
                    float(origin.position.x),
                    float(origin.position.y),
                    yaw,
                ),
                'negate: 0',
                'occupied_thresh: 0.65',
                'free_thresh: 0.25',
                '',
            ]),
            encoding='utf-8',
        )
        _write_json(self.output_dir / 'cartographer_map_metadata.json', {
            'frame_id': str(self.latest_map.header.frame_id),
            'stamp_sec': _stamp_to_sec(self.latest_map.header.stamp),
            'width': width,
            'height': height,
            'resolution_m': float(info.resolution),
            'origin': [
                float(origin.position.x),
                float(origin.position.y),
                yaw,
            ],
            'free_cells': sum(1 for value in values if value == 0),
            'traversable_probability_cells': sum(
                1 for value in values if 0 <= value <= 49
            ),
            'uncertain_cells': sum(1 for value in values if 50 <= value < 65),
            'occupied_cells': sum(1 for value in values if value >= 65),
            'unknown_cells': sum(1 for value in values if value < 0),
        })
        return 'cartographer_map.yaml'

    def _latest_image_shape(self):
        if self.latest_image is None:
            return 0, 0
        return int(self.latest_image.shape[0]), int(self.latest_image.shape[1])

    def _save_evidence_image(self, stamp_sec, detections, hazards, recommendation):
        """保存叠加检测框和当前重观察建议的展示帧。"""

        confirmed_ids = {
            str(item.get('id')) for item in hazards
            if item.get('status') == 'confirmed'
        }
        has_new_confirmation = bool(
            confirmed_ids - self.saved_confirmation_ids)
        # confirmed 轨迹会在后续每帧持续发布；它不是每帧都重新获得的证据。
        # 只保存真正含候选的帧和每个目标首次确认帧，避免数百次重复写入
        # 1.2 MB 深度数组拖慢 SLAM。无候选画面按低频上下文策略保存。
        is_context_frame = not detections and not has_new_confirmation
        if is_context_frame:
            if self.context_evidence_count >= int(
                    self.get_parameter('max_context_evidence_count').value):
                return '', '', ''
            context_interval = float(
                self.get_parameter('context_save_interval_sec').value
            )
            if stamp_sec - self.last_context_save_sec < context_interval:
                return '', '', ''
        if not bool(self.get_parameter('save_images').value) or self.latest_image is None:
            return '', '', ''
        interval = (
            0.0 if is_context_frame
            else float(self.get_parameter('min_image_save_interval_sec').value)
        )
        if not is_context_frame and stamp_sec - self.last_image_save_sec < interval:
            return '', '', ''
        try:
            import cv2
        except ImportError:
            self.get_logger().warn('未安装 OpenCV，跳过动态截图保存。', throttle_duration_sec=10.0)
            return '', '', ''
        prefix = 'context' if is_context_frame else 'frame'
        stem = f'{prefix}_{len(self.records) + 1:06d}_{stamp_sec:.3f}'
        raw_path = self.raw_image_dir / f'{stem}_raw.png'
        path = self.annotated_image_dir / f'{stem}_annotated.png'
        annotated = self.latest_image.copy()
        for detection in detections:
            bbox = detection.get('bbox', {})
            x_min = int(bbox.get('x_min', 0))
            y_min = int(bbox.get('y_min', 0))
            x_max = int(bbox.get('x_max', x_min))
            y_max = int(bbox.get('y_max', y_min))
            # 黄色框是“仅供导航复查”的局部/粘连/非球体疑似候选，绝不能在展示图中
            # 与红色严格二维候选混为“已识别红球”。即使红框也仍需多视角确认，最终
            # 是否提交由 hazards 的 confirmed 状态决定。
            reobserve = bool(detection.get('requires_reobservation', False))
            color = (0, 165, 255) if reobserve else (0, 0, 255)
            status = 'reobserve' if reobserve else '2d_candidate'
            label = '#%s %.2f %s' % (
                detection.get('id', '?'), float(detection.get('confidence', 0.0)), status,
            )
            cv2.rectangle(annotated, (x_min, y_min), (x_max, y_max), color, 2)
            cv2.putText(annotated, label, (x_min, max(18, y_min - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        action_label = f"view: {recommendation.get('action', 'unknown')}"
        cv2.putText(annotated, action_label, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        if is_context_frame:
            cv2.putText(
                annotated,
                'official random scene context (no candidate)',
                (12, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
            )
        if not cv2.imwrite(str(raw_path), self.latest_image):
            self.get_logger().warn(f'原始 RGB 证据写入失败：{raw_path}')
            return '', '', ''
        if not cv2.imwrite(str(path), annotated):
            self.get_logger().warn(f'标注 RGB 证据写入失败：{path}')
            raw_path.unlink(missing_ok=True)
            return '', '', ''
        depth_path = (
            '' if is_context_frame
            else self._save_depth_evidence(f'{stem}.png')
        )
        if is_context_frame:
            self.last_context_save_sec = stamp_sec
            self.context_evidence_count += 1
        else:
            self.last_image_save_sec = stamp_sec
        self.saved_confirmation_ids.update(confirmed_ids)
        return (
            str(raw_path.relative_to(self.output_dir).as_posix()),
            str(path.relative_to(self.output_dir).as_posix()),
            depth_path,
        )

    def _save_depth_evidence(self, image_filename):
        """保存与 RGB 时间严格配对的米制深度，供赛后独立复核。"""

        if (not bool(self.get_parameter('save_depth_evidence').value)
                or self.latest_depth_image is None):
            return ''
        max_delta = float(self.get_parameter('max_rgb_depth_evidence_delta_sec').value)
        if abs(self.latest_depth_stamp - self.latest_image_stamp) > max_delta:
            return ''
        path = self.depth_dir / (Path(image_filename).stem + '.npy')
        np.save(str(path), self.latest_depth_image)
        return str(path.relative_to(self.output_dir).as_posix())

    def _publish_recommendation(self, recommendation):
        out = String()
        out.data = json.dumps(recommendation, ensure_ascii=False)
        self.recommendation_pub.publish(out)


def _image_message_to_array(msg):
    if msg.encoding.lower() not in ('rgb8', 'bgr8') or msg.step < msg.width * 3:
        return None
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    expected = msg.step * msg.height
    if raw.size < expected:
        return None
    image = raw[:expected].reshape((msg.height, msg.step))[:, :msg.width * 3]
    image = image.reshape((msg.height, msg.width, 3)).copy()
    if msg.encoding.lower() == 'rgb8':
        image = image[:, :, ::-1]
    return image


def _depth_message_to_meters(msg):
    """解码官方常用 16UC1/32FC1 深度图，输出米制 float32 数组。"""

    encoding = msg.encoding.upper()
    if encoding == '16UC1':
        dtype = np.dtype('<u2')
        scale = 0.001
    elif encoding == '32FC1':
        dtype = np.dtype('<f4')
        scale = 1.0
    else:
        return None
    item_size = dtype.itemsize
    if msg.step < msg.width * item_size:
        return None
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    expected = msg.step * msg.height
    if raw.size < expected:
        return None
    rows = raw[:expected].reshape((msg.height, msg.step))[:, :msg.width * item_size].copy()
    depth = rows.reshape((msg.height, msg.width * item_size)).view(dtype).reshape(
        (msg.height, msg.width),
    )
    return depth.astype(np.float32, copy=True) * scale


def _stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _quaternion_yaw(orientation):
    """将地图原点四元数转换为二维 yaw。"""

    siny_cosp = 2.0 * (
        float(orientation.w) * float(orientation.z)
        + float(orientation.x) * float(orientation.y)
    )
    cosy_cosp = 1.0 - 2.0 * (
        float(orientation.y) ** 2 + float(orientation.z) ** 2
    )
    return float(np.arctan2(siny_cosp, cosy_cosp))


def _payload_stamp_to_sec(detections, fallback):
    if not detections:
        return float(fallback)
    stamp = detections[0].get('stamp', {})
    if 'sec' in stamp:
        return float(stamp.get('sec', 0.0)) + float(stamp.get('nanosec', 0.0)) * 1e-9
    return float(fallback)


def _write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def _write_single_row_csv(path, value):
    row = dict(value)
    row['view_action_counts'] = json.dumps(row.get('view_action_counts', {}), ensure_ascii=False)
    with path.open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
        writer.writeheader()
        writer.writerow(row)


def _derive_failure_reasons(
        records, trajectory_sample_count, evidence_contract, mission_completed):
    """仅从实际采集状态归纳失败原因，禁止使用场景真值猜测指标。"""

    reasons = []
    if not records:
        reasons.append('no_detection_messages_recorded')
    if not trajectory_sample_count:
        reasons.append('no_legal_slam_pose_samples_recorded')
    if not evidence_contract.get('formal_evidence_eligible', False):
        reasons.extend(evidence_contract.get('contract_violations', []))
    if not mission_completed:
        reasons.append('mission_not_completed')
    if records and not any(
            hazard.get('status') == 'confirmed'
            for record in records for hazard in record.get('hazards', [])):
        reasons.append('no_confirmed_red_ball_recorded')
    return sorted(set(reasons))


def main():
    rclpy.init()
    node = DynamicDetectionRecorderNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        # 仍需在外部关闭后落盘 frames/summary，随后跳过已失效上下文的 rclpy.shutdown。
        pass
    finally:
        # ros2 launch 会在进程组 SIGINT 后再次向子进程转发信号。封存阶段忽略
        # 重复终止信号，避免 summary/result 在写到一半时再次 KeyboardInterrupt。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.close()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
