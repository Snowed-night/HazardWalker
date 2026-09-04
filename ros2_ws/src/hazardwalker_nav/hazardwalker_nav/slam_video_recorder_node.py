#!/usr/bin/env python3
"""三层SLAM录像节点：同步展示分层占用地图、轨迹和增量三维点云。

所属组：导航探索组。负责人：姜晨。
文件作用：仅订阅公开SLAM输出并生成低开销MP4，不参与定位、规划或控制。
视频每收到一帧有界三维地图写入一帧，适合把长时低实时率仿真压缩成演示视频。
"""

from pathlib import Path
import threading

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformException, TransformListener


FRAME_WIDTH = 1280
FRAME_HEIGHT = 720


class SlamVideoRecorder(Node):
    """以点云发布节拍写视频，避免额外高频定时器抢占SLAM。"""

    def __init__(self):
        super().__init__('hazardwalker_slam_video_recorder')
        self.declare_parameter('output_path', '')
        self.declare_parameter('video_fps', 5.0)
        self.declare_parameter('max_render_points', 80000)
        self.output_path = Path(str(
            self.get_parameter('output_path').value)).expanduser()
        if not str(self.output_path):
            raise ValueError('slam video output_path不得为空')
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        fps = max(1.0, float(self.get_parameter('video_fps').value))
        self.writer = cv2.VideoWriter(
            str(self.output_path), cv2.VideoWriter_fourcc(*'mp4v'),
            fps, (FRAME_WIDTH, FRAME_HEIGHT))
        if not self.writer.isOpened():
            raise RuntimeError(f'无法创建SLAM视频：{self.output_path}')
        self.max_render_points = max(
            1000, int(self.get_parameter('max_render_points').value))
        self.latest_map = None
        self.floor_index = 0
        self.nav_state = 'INIT'
        self.paths = {}
        self.frame_count = 0
        self._lock = threading.Lock()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map, 10)
        self.create_subscription(
            Int32, '/hazardwalker/navigation/floor_index',
            self._on_floor, 10)
        self.create_subscription(
            String, '/hw/nav/state', self._on_state, 10)
        self.create_subscription(
            PointCloud2, '/hazardwalker/slam/cloud_map',
            self._on_cloud, qos_profile_sensor_data)
        self.get_logger().info(f'SLAM视频录制：{self.output_path}')

    def _on_map(self, message):
        self.latest_map = message

    def _on_floor(self, message):
        self.floor_index = int(message.data)

    def _on_state(self, message):
        self.nav_state = str(message.data)

    def _current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base', rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except TransformException:
            return None
        translation = transform.transform.translation
        return float(translation.x), float(translation.y), float(translation.z)

    def _on_cloud(self, message):
        with self._lock:
            points = point_cloud2.read_points_numpy(
                message, field_names=('x', 'y', 'z'), skip_nans=True)
            points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
            if len(points) > self.max_render_points:
                stride = max(1, len(points) // self.max_render_points)
                points = points[::stride][:self.max_render_points]
            pose = self._current_pose()
            if pose is not None:
                self.paths.setdefault(self.floor_index, []).append(pose)
            frame = self._render_frame(points, pose)
            self.writer.write(frame)
            self.frame_count += 1
            if self.frame_count % 10 == 0:
                cv2.imwrite(
                    str(self.output_path.with_name(
                        self.output_path.stem + '_latest.png')),
                    frame)

    def _render_frame(self, points, pose):
        frame = np.full((FRAME_HEIGHT, FRAME_WIDTH, 3), 18, dtype=np.uint8)
        cv2.putText(
            frame, 'HazardWalker 3D SLAM + Multi-floor Exploration',
            (24, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (245, 245, 245), 2,
            cv2.LINE_AA)
        cv2.putText(
            frame,
            f'floor={self.floor_index}  state={self.nav_state}  '
            f'frame={self.frame_count}  voxels~{len(points)}',
            (24, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (120, 210, 255), 1,
            cv2.LINE_AA)
        frame[88:688, 18:618] = self._render_occupancy(pose)
        frame[78:698, 638:1262] = self._render_cloud(points)
        cv2.rectangle(frame, (18, 88), (617, 687), (100, 100, 100), 1)
        cv2.rectangle(frame, (638, 78), (1261, 697), (100, 100, 100), 1)
        cv2.putText(frame, 'Layered occupancy + trajectory', (32, 112),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 220), 2)
        cv2.putText(frame, '3D voxel map (height color)', (652, 102),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1)
        return frame

    def _render_occupancy(self, pose):
        panel = np.full((600, 600, 3), 55, dtype=np.uint8)
        message = self.latest_map
        if message is None or not message.data:
            cv2.putText(panel, 'waiting for /map', (170, 300),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2)
            return panel
        height = int(message.info.height)
        width = int(message.info.width)
        grid = np.asarray(message.data, dtype=np.int16).reshape(height, width)
        image = np.full((height, width, 3), 90, dtype=np.uint8)
        image[grid == 0] = (235, 235, 235)
        image[grid >= 50] = (20, 20, 20)
        image = np.flipud(image)
        image = cv2.resize(image, (600, 600), interpolation=cv2.INTER_NEAREST)
        resolution = float(message.info.resolution)
        origin_x = float(message.info.origin.position.x)
        origin_y = float(message.info.origin.position.y)

        def pixel(x_value, y_value):
            gx = (x_value - origin_x) / max(resolution, 1e-6)
            gy = (y_value - origin_y) / max(resolution, 1e-6)
            return (
                int(np.clip(gx / max(1, width - 1) * 599, 0, 599)),
                int(np.clip((1.0 - gy / max(1, height - 1)) * 599, 0, 599)),
            )

        for floor, path in sorted(self.paths.items()):
            if len(path) < 2:
                continue
            color = ((40, 40, 230), (40, 180, 40), (230, 120, 30))[floor % 3]
            polyline = np.asarray([pixel(x, y) for x, y, _ in path], np.int32)
            cv2.polylines(image, [polyline], False, color, 2, cv2.LINE_AA)
        if pose is not None:
            cv2.circle(image, pixel(pose[0], pose[1]), 6, (0, 0, 255), -1)
        return image

    def _render_cloud(self, points):
        panel = np.full((620, 624, 3), 12, dtype=np.uint8)
        if points.size == 0:
            return panel
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        # 固定等轴投影，三层高度不会重叠；边界外点只是不显示，不改地图。
        u = np.rint((x - y) * 17.0 + 312.0).astype(np.int32)
        v = np.rint(550.0 - (x + y) * 5.0 - z * 48.0).astype(np.int32)
        valid = (
            (u >= 2) & (u < 622) & (v >= 2) & (v < 618)
            & np.isfinite(x) & np.isfinite(y) & np.isfinite(z))
        if not np.any(valid):
            return panel
        u = u[valid]
        v = v[valid]
        z_value = np.clip((z[valid] + 0.5) / 8.0 * 255.0, 0, 255).astype(np.uint8)
        colors = cv2.applyColorMap(z_value.reshape(-1, 1), cv2.COLORMAP_TURBO)
        colors = colors.reshape(-1, 3)
        order = np.argsort(v)
        panel[v[order], u[order]] = colors[order]
        for floor, path in sorted(self.paths.items()):
            if len(path) < 2:
                continue
            path_array = np.asarray(path, dtype=np.float32)
            pu = np.rint(
                (path_array[:, 0] - path_array[:, 1]) * 17.0 + 312.0
            ).astype(np.int32)
            pv = np.rint(
                550.0 - (path_array[:, 0] + path_array[:, 1]) * 5.0
                - path_array[:, 2] * 48.0
            ).astype(np.int32)
            mask = (pu >= 0) & (pu < 624) & (pv >= 0) & (pv < 620)
            polyline = np.column_stack((pu[mask], pv[mask])).astype(np.int32)
            if len(polyline) >= 2:
                cv2.polylines(panel, [polyline], False, (255, 255, 255), 2,
                              cv2.LINE_AA)
        return panel

    def close(self):
        with self._lock:
            if self.writer is not None:
                self.writer.release()
                self.writer = None
                self.get_logger().info(
                    f'SLAM视频已保存：{self.output_path}，帧数={self.frame_count}')

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = SlamVideoRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
