#!/usr/bin/env python3
"""采集官方 SimEnv ROS2 RGB-D 感知测试的单个原生案例。

该工具订阅正在运行的官方平台 RGB、深度与指定感知输出话题，保存原始 RGB、
可恢复米制深度、深度可视化、实际检测框标注图和精简 JSON 快照。它不读取场景真值；预期标签应由
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

    def __init__(self, image_topic: str, depth_topic: str, detection_topic: str):
        super().__init__('official_simenv_rgbd_case_capture')
        self.image = None
        self.image_stamp = None
        self.depth_m = None
        self.depth_stamp = None
        self.payload = None
        self.payload_received_at = 0.0
        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
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

    def _on_depth(self, message: Image):
        """把 16UC1 毫米或 32FC1 米深度统一为 float32 米。"""

        encoding = message.encoding.upper()
        dtype = np.uint16 if encoding == '16UC1' else np.float32 if encoding == '32FC1' else None
        if dtype is None:
            return
        item_size = np.dtype(dtype).itemsize
        raw = np.frombuffer(bytes(message.data), dtype=dtype)
        row_items = message.step // item_size
        packed = raw.reshape((message.height, row_items))[:, :message.width]
        depth = packed.astype(np.float32)
        if encoding == '16UC1':
            depth *= 0.001
        self.depth_m = depth
        self.depth_stamp = (message.header.stamp.sec, message.header.stamp.nanosec)

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
            'depth_synchronized', 'depth_stamp_delta_sec', 'localization_status',
            'localized_position', 'raw_surface_depth_m', 'candidate_id', 'track_id',
            'track_status', 'track_association', 'source', 'detector_backend',
        )
        if key in item
    }


def _compact_hazard(item: dict) -> dict:
    """保存可复核确认门槛的轨迹摘要，排除冗长逐帧 source_ids。"""
    return {
        key: item[key]
        for key in (
            'id', 'status', 'position', 'position_frame_id', 'confidence',
            'observation_count', 'missed_count', 'distinct_view_count',
            'eligible_observation_count', 'eligible_view_ids', 'flat_view_ids',
            'spherical_view_ids', 'apparent_diameter_cv',
            'median_apparent_diameter_m', 'min_multiview_aspect_ratio',
            'depth_curvature_cv', 'median_normalized_depth_curvature',
            'view_bearing_span_deg', 'evidence_status',
            'required_min_eligible_observations', 'required_min_distinct_views',
            'required_min_spherical_views', 'required_min_view_bearing_span_deg',
            'localization_provenance', 'source',
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
        track_status = str(item.get('track_status') or '').strip()
        status = (
            'confirmed'
            if track_status == 'confirmed'
            else 'reobserve'
            if item.get('requires_reobservation')
            else track_status or 'stable'
        )
        label = f'#{index} {item.get("confidence", 0.0):.2f} {status}'
        cv2.putText(annotated, label, (x0, max(18, y0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    if not detections:
        cv2.putText(annotated, 'no 2D candidate', (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, (0, 0, 180), 2, cv2.LINE_AA)
    return annotated


def _depth_archives(depth_m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """生成可恢复米制值的 uint16 毫米图和便于人工检查的伪彩色图。"""

    valid = np.isfinite(depth_m) & (depth_m > 0.0) & (depth_m < 65.535)
    depth_mm = np.zeros(depth_m.shape, dtype=np.uint16)
    depth_mm[valid] = np.clip(
        np.rint(depth_m[valid] * 1000.0), 1, 65535,
    ).astype(np.uint16)
    visual = np.zeros(depth_m.shape, dtype=np.uint8)
    if np.any(valid):
        low, high = np.percentile(depth_m[valid], (2.0, 98.0))
        if high <= low:
            high = low + 0.001
        normalized = np.clip((depth_m - low) / (high - low), 0.0, 1.0)
        visual[valid] = np.rint((1.0 - normalized[valid]) * 255.0).astype(np.uint8)
    colored = cv2.applyColorMap(visual, cv2.COLORMAP_TURBO)
    colored[~valid] = 0
    return depth_mm, colored


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case-id', required=True)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--image-topic', default='/hw/camera/image_raw')
    parser.add_argument('--depth-topic', default='/hw/camera/depth_image')
    parser.add_argument('--detection-topic', required=True)
    parser.add_argument('--timeout-sec', type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = CaseCapture(args.image_topic, args.depth_topic, args.detection_topic)
    deadline = time.monotonic() + args.timeout_sec
    # 等待图像与感知输出都到达。节点输出每帧更新，保证快照来自当前场景。
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
        if node.image is not None and node.depth_m is not None and node.payload is not None:
            break

    if node.image is None or node.depth_m is None or node.payload is None:
        node.destroy_node()
        rclpy.shutdown()
        missing = (
            'RGB image' if node.image is None
            else 'depth image' if node.depth_m is None
            else 'detection payload'
        )
        raise RuntimeError(f'Timed out waiting for {missing}.')

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / f'{args.case_id}_raw.png'
    annotated_path = output_dir / f'{args.case_id}_annotated.png'
    depth_metric_path = output_dir / f'{args.case_id}_depth_mm.png'
    depth_visual_path = output_dir / f'{args.case_id}_depth_visual.png'
    snapshot_path = output_dir / f'{args.case_id}_snapshot.json'
    detections = node.payload.get('detections_2d', [])
    cv2.imwrite(str(raw_path), node.image)
    cv2.imwrite(str(annotated_path), _draw_annotations(node.image, detections))
    depth_mm, depth_visual = _depth_archives(node.depth_m)
    cv2.imwrite(str(depth_metric_path), depth_mm)
    cv2.imwrite(str(depth_visual_path), depth_visual)
    snapshot = {
        'case_id': args.case_id,
        'image_stamp': {'sec': node.image_stamp[0], 'nanosec': node.image_stamp[1]},
        'depth_stamp': {'sec': node.depth_stamp[0], 'nanosec': node.depth_stamp[1]},
        'localization_ready': bool(node.payload.get('localization_ready')),
        'detections_2d': [_compact_detection(item) for item in detections],
        'hazards': [_compact_hazard(item) for item in node.payload.get('hazards', [])],
        'view_recommendation': (
            node.payload.get('view_recommendation')
            if isinstance(node.payload.get('view_recommendation'), dict)
            else {}
        ),
        'raw_image': raw_path.name,
        'annotated_image': annotated_path.name,
        'depth_metric_image': depth_metric_path.name,
        'depth_visual_image': depth_visual_path.name,
    }
    snapshot_path.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({
        'case_id': args.case_id,
        'detection_count': len(detections),
        'hazard_count': len(snapshot['hazards']),
        'raw_image': str(raw_path),
        'annotated_image': str(annotated_path),
        'depth_metric_image': str(depth_metric_path),
        'depth_visual_image': str(depth_visual_path),
        'snapshot': str(snapshot_path),
    }, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
