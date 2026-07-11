#!/usr/bin/env python3
"""采集官方 SimEnv ROS2 RGB-D 感知测试的单个原生案例。

该工具订阅正在运行的官方平台相机与指定感知输出话题，保存原始 RGB、
带实际检测框的标注图和精简 JSON 快照。它不读取场景真值；预期标签应由
调用方依据临时受控模型配置写入测试记录。
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CaseCapture(Node):
    """缓存同一测试窗口内最新 RGB 帧及感知 JSON，避免命令行 echo 截断。"""

    def __init__(self, image_topic: str, detection_topic: str):
        super().__init__('official_simenv_rgbd_case_capture')
        self.image = None
        self.image_stamp = None
        self.payload = None
        self.payload_received_at = 0.0
        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(String, detection_topic, self._on_detection, 10)

    def _on_image(self, message: Image):
        """将 ROS RGB/BGR 图像转换为 OpenCV BGR，便于原样归档与标注。"""
        if message.encoding.lower() not in ('rgb8', 'bgr8'):
            return
        raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
        packed = raw.reshape((message.height, message.step))[:, :message.width * 3]
        image = packed.reshape((message.height, message.width, 3))
        self.image = image[:, :, ::-1].copy() if message.encoding.lower() == 'rgb8' else image.copy()
        self.image_stamp = (message.header.stamp.sec, message.header.stamp.nanosec)

    def _on_detection(self, message: String):
        """仅接收格式正确的实际节点输出，不对检测结果作人工补写。"""
        try:
            self.payload = json.loads(message.data)
            self.payload_received_at = time.monotonic()
        except json.JSONDecodeError:
            self.get_logger().warning('忽略无法解析的感知输出 JSON。')


def _compact_detection(item: dict) -> dict:
    """移除长轨迹 source_ids，仅保留复现实验判读所需字段。"""
    return {
        key: item[key]
        for key in (
            'id', 'frame_id', 'view_id', 'stamp', 'bbox', 'confidence', 'red_pixel_count',
            'is_partial', 'requires_reobservation', 'may_be_merged',
            'quality_reason', 'shape', 'depth_shape', 'confirmation_eligible',
            'localization_status', 'source',
            'detector_backend',
        )
        if key in item
    }


def _compact_hazard(item: dict) -> dict:
    """保存轨迹状态和位置，避免将运行期内部字段写入报告。"""
    return {
        key: item[key]
        for key in (
            'track_id', 'status', 'position', 'position_frame_id', 'confidence',
            'observation_count', 'distinct_view_count', 'source',
        )
        if key in item
    }


def _draw_annotations(image: np.ndarray, detections: list[dict]) -> np.ndarray:
    """在原始图上绘制真实节点返回的 2D 框和关键风险标签。"""
    annotated = image.copy()
    for index, item in enumerate(detections, start=1):
        bbox = item.get('bbox') or {}
        try:
            x0, y0 = int(bbox['x_min']), int(bbox['y_min'])
            x1, y1 = int(bbox['x_max']), int(bbox['y_max'])
        except (KeyError, TypeError, ValueError):
            continue
        color = (0, 165, 255) if item.get('requires_reobservation') else (0, 0, 255)
        cv2.rectangle(annotated, (x0, y0), (x1, y1), color, 2)
        status = 'reobserve' if item.get('requires_reobservation') else 'stable'
        label = f'#{index} {item.get("confidence", 0.0):.2f} {status}'
        cv2.putText(annotated, label, (x0, max(18, y0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    if not detections:
        cv2.putText(annotated, 'no 2D candidate', (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, (0, 0, 180), 2, cv2.LINE_AA)
    return annotated


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case-id', required=True)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--image-topic', default='/hw/camera/image_raw')
    parser.add_argument('--detection-topic', required=True)
    parser.add_argument('--timeout-sec', type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = CaseCapture(args.image_topic, args.detection_topic)
    deadline = time.monotonic() + args.timeout_sec
    # 等待图像与感知输出都到达。节点输出每帧更新，保证快照来自当前场景。
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
        if node.image is not None and node.payload is not None:
            break

    if node.image is None or node.payload is None:
        node.destroy_node()
        rclpy.shutdown()
        missing = 'image' if node.image is None else 'detection payload'
        raise RuntimeError(f'Timed out waiting for {missing}.')

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / f'{args.case_id}_raw.png'
    annotated_path = output_dir / f'{args.case_id}_annotated.png'
    snapshot_path = output_dir / f'{args.case_id}_snapshot.json'
    detections = node.payload.get('detections_2d', [])
    cv2.imwrite(str(raw_path), node.image)
    cv2.imwrite(str(annotated_path), _draw_annotations(node.image, detections))
    snapshot = {
        'case_id': args.case_id,
        'image_stamp': {'sec': node.image_stamp[0], 'nanosec': node.image_stamp[1]},
        'localization_ready': bool(node.payload.get('localization_ready')),
        'detections_2d': [_compact_detection(item) for item in detections],
        'hazards': [_compact_hazard(item) for item in node.payload.get('hazards', [])],
        'raw_image': raw_path.name,
        'annotated_image': annotated_path.name,
    }
    snapshot_path.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({
        'case_id': args.case_id,
        'detection_count': len(detections),
        'hazard_count': len(snapshot['hazards']),
        'raw_image': str(raw_path),
        'annotated_image': str(annotated_path),
        'snapshot': str(snapshot_path),
    }, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
