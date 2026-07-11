"""动态红球检测实验记录 ROS 2 节点。

所属组：感知定位组。
文件作用：
订阅统一的 ``/hw/*`` 图像、里程计和感知结果，记录连续帧候选、多帧确认状态、
机器人位姿与主动重观察建议，并在结束时输出 summary 和测试记录。
当前实现边界：
只记录和发布建议到 ``/hw/perception/view_recommendation``，不直接控制机器人。
验证方式：
先运行离线测试；官方 SimEnv 需由平台层桥接到 ``/hw/*`` 后再实际运行。
"""

import csv
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from hazardwalker_perception.active_view_policy import choose_active_view_action
from hazardwalker_perception.dynamic_detection_records import (
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
        self.declare_parameter('min_image_save_interval_sec', 0.5)

        output_dir = str(self.get_parameter('output_dir').value).strip()
        test_record_dir = str(self.get_parameter('test_record_dir').value).strip()
        if not output_dir or not test_record_dir:
            raise ValueError('output_dir 和 test_record_dir 必须显式指定，避免把实验结果写入未知目录。')

        self.output_dir = Path(output_dir).expanduser().resolve()
        self.test_record_dir = Path(test_record_dir).expanduser().resolve()
        self.image_dir = self.output_dir / 'selected_images'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.test_record_dir.mkdir(parents=True, exist_ok=True)
        if bool(self.get_parameter('save_images').value):
            self.image_dir.mkdir(parents=True, exist_ok=True)

        self.records = []
        self.latest_image = None
        self.latest_image_stamp = 0.0
        self.latest_image_frame_id = ''
        self.latest_odom = None
        self.last_image_save_sec = float('-inf')

        self.image_sub = self.create_subscription(Image, '/hw/camera/image_raw', self.on_image, 10)
        self.odom_sub = self.create_subscription(Odometry, '/hw/odom', self.on_odom, 20)
        self.detection_sub = self.create_subscription(
            String, '/hw/perception/hazard_detections', self.on_detections, 20,
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

    def on_odom(self, msg):
        """保存最近机器人位姿；记录时同时保留坐标系和姿态四元数。"""

        pose = msg.pose.pose
        self.latest_odom = {
            'frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,
            'position': {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z},
            'orientation': {
                'x': pose.orientation.x, 'y': pose.orientation.y,
                'z': pose.orientation.z, 'w': pose.orientation.w,
            },
            'stamp_sec': _stamp_to_sec(msg.header.stamp),
        }

    def on_detections(self, msg):
        """记录一条感知输出，并为当前候选生成一次主动观察建议。"""

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'忽略无法解析的感知 JSON：{exc}', throttle_duration_sec=5.0)
            return

        detections = list(payload.get('detections_2d', []))
        image_height, image_width = self._latest_image_shape()
        recommendation = choose_active_view_action(detections, image_width, image_height)
        recommendation_dict = recommendation.to_dict()
        self._publish_recommendation(recommendation_dict)

        stamp_sec = _payload_stamp_to_sec(detections, fallback=time.time())
        image_path = self._save_evidence_image(stamp_sec, detections, recommendation_dict)
        record = {
            'timestamp_sec': stamp_sec,
            'image_frame_id': self.latest_image_frame_id,
            'robot_pose': self.latest_odom,
            'detections_2d': detections,
            'hazards': list(payload.get('hazards', [])),
            'localization_ready': bool(payload.get('localization_ready', False)),
            'view_recommendation': recommendation_dict,
            'evidence_image': image_path,
        }
        self.records.append(record)
        with (self.output_dir / 'frames.jsonl').open('a', encoding='utf-8') as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + '\n')

    def close(self):
        """在节点退出时写入汇总和测试组 CSV/JSON。"""

        summary = build_dynamic_summary(self.records)
        summary.update({
            'scenario': str(self.get_parameter('scenario_name').value),
            'generated_at_unix_sec': round(time.time(), 3),
            'record_file': 'frames.jsonl',
        })
        _write_json(self.output_dir / 'summary.json', summary)

        testing_record = build_dynamic_testing_record(
            summary,
            scenario=str(self.get_parameter('scenario_name').value),
        )
        _write_json(self.test_record_dir / 'testing_record_perception.json', testing_record)
        _write_single_row_csv(self.test_record_dir / 'testing_record_perception.csv', testing_record)
        self.get_logger().info(f'动态检测汇总已写入 {self.output_dir / "summary.json"}')

    def _latest_image_shape(self):
        if self.latest_image is None:
            return 0, 0
        return int(self.latest_image.shape[0]), int(self.latest_image.shape[1])

    def _save_evidence_image(self, stamp_sec, detections, recommendation):
        """保存叠加检测框和当前重观察建议的展示帧。"""

        if not bool(self.get_parameter('save_images').value) or self.latest_image is None:
            return ''
        interval = float(self.get_parameter('min_image_save_interval_sec').value)
        if stamp_sec - self.last_image_save_sec < interval:
            return ''
        try:
            import cv2
        except ImportError:
            self.get_logger().warn('未安装 OpenCV，跳过动态截图保存。', throttle_duration_sec=10.0)
            return ''
        filename = f'frame_{len(self.records) + 1:06d}_{stamp_sec:.3f}.png'
        path = self.image_dir / filename
        annotated = self.latest_image.copy()
        for detection in detections:
            bbox = detection.get('bbox', {})
            x_min = int(bbox.get('x_min', 0))
            y_min = int(bbox.get('y_min', 0))
            x_max = int(bbox.get('x_max', x_min))
            y_max = int(bbox.get('y_max', y_min))
            label = f"id={detection.get('id', '?')} conf={float(detection.get('confidence', 0.0)):.2f}"
            cv2.rectangle(annotated, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(annotated, label, (x_min, max(18, y_min - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        action_label = f"view: {recommendation.get('action', 'unknown')}"
        cv2.putText(annotated, action_label, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.imwrite(str(path), annotated)
        self.last_image_save_sec = stamp_sec
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


def _stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


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


def main():
    rclpy.init()
    node = DynamicDetectionRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
