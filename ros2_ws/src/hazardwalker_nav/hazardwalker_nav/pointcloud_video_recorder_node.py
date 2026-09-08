#!/usr/bin/env python3
"""独立生成 1920×1080 三维点云建图视频。

所属组：导航探索组。负责人：姜晨。
节点只订阅已经体素化的公开三维地图、导航状态和合法 TF，不参与二维 SLAM、
Frontier 或控制。采用高度着色、缓慢环绕相机、轨迹和状态叠加生成展示视频；
与二维地图录像完全分离，关闭本节点不会改变导航结果。
"""

import math
from pathlib import Path
import threading

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformException, TransformListener


FRAME_WIDTH = 1920
FRAME_HEIGHT = 1080


class PointcloudVideoRecorder(Node):
    """按低频体素地图更新节拍生成独立三维视频。"""

    def __init__(self):
        super().__init__('hazardwalker_pointcloud_video_recorder')
        self.declare_parameter('input_topic', '/hazardwalker/slam/cloud_map')
        self.declare_parameter('output_path', '')
        self.declare_parameter('video_fps', 5.0)
        self.declare_parameter('max_render_points', 120000)
        self.declare_parameter('orbit_degrees_per_frame', 0.35)
        self.output_path = Path(str(
            self.get_parameter('output_path').value)).expanduser()
        if not str(self.output_path):
            raise ValueError('3D video output_path不得为空')
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.writer = cv2.VideoWriter(
            str(self.output_path), cv2.VideoWriter_fourcc(*'mp4v'),
            max(1.0, float(self.get_parameter('video_fps').value)),
            (FRAME_WIDTH, FRAME_HEIGHT))
        if not self.writer.isOpened():
            raise RuntimeError(f'无法创建三维点云视频：{self.output_path}')
        self.max_render_points = max(
            1000, int(self.get_parameter('max_render_points').value))
        self.orbit_radians_per_frame = math.radians(float(
            self.get_parameter('orbit_degrees_per_frame').value))
        self.floor_index = 0
        self.nav_state = 'INIT'
        self.frame_count = 0
        self.path = []
        self._scene_center = None
        self._lock = threading.Lock()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('input_topic').value),
            self.on_cloud,
            qos_profile_sensor_data)
        self.create_subscription(
            Int32, '/hazardwalker/navigation/floor_index',
            self.on_floor, 10)
        self.create_subscription(
            String, '/hw/nav/state', self.on_state, 10)
        self.get_logger().info(
            f'独立三维点云录像已启动：{self.output_path}')

    def on_floor(self, message):
        self.floor_index = int(message.data)

    def on_state(self, message):
        self.nav_state = str(message.data)

    def _current_pose(self, target_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                str(target_frame), 'base', rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except TransformException:
            return None
        translation = transform.transform.translation
        return float(translation.x), float(translation.y), float(translation.z)

    def on_cloud(self, message):
        with self._lock:
            raw = point_cloud2.read_points_numpy(
                message, field_names=('x', 'y', 'z'), skip_nans=True)
            points = np.asarray(raw, dtype=np.float32).reshape(-1, 3)
            if len(points) > self.max_render_points:
                stride = max(1, len(points) // self.max_render_points)
                points = points[::stride][:self.max_render_points]
            if points.size == 0:
                return
            pose = self._current_pose(message.header.frame_id)
            if pose is not None:
                self.path.append(pose)
            frame = self._render(points)
            self.writer.write(frame)
            self.frame_count += 1
            if self.frame_count % 5 == 0:
                cv2.imwrite(
                    str(self.output_path.with_name(
                        self.output_path.stem + '_latest.png')),
                    frame)

    def _render(self, points):
        frame = np.full((FRAME_HEIGHT, FRAME_WIDTH, 3), 10, dtype=np.uint8)
        xy_min = np.percentile(points[:, :2], 1.0, axis=0)
        xy_max = np.percentile(points[:, :2], 99.0, axis=0)
        observed_center = 0.5 * (xy_min + xy_max)
        if self._scene_center is None:
            self._scene_center = observed_center
        else:
            self._scene_center = (
                0.96 * self._scene_center + 0.04 * observed_center)

        centered = points.copy()
        centered[:, 0] -= self._scene_center[0]
        centered[:, 1] -= self._scene_center[1]
        yaw = self.frame_count * self.orbit_radians_per_frame
        cosine, sine = math.cos(yaw), math.sin(yaw)
        rotated_x = cosine * centered[:, 0] - sine * centered[:, 1]
        rotated_y = sine * centered[:, 0] + cosine * centered[:, 1]
        pitch = math.radians(32.0)
        screen_y_axis = (
            math.sin(pitch) * rotated_y
            - math.cos(pitch) * centered[:, 2])
        depth_axis = (
            math.cos(pitch) * rotated_y
            + math.sin(pitch) * centered[:, 2])

        horizontal_extent = max(
            1.0, float(np.percentile(np.abs(rotated_x), 99.0)))
        vertical_extent = max(
            1.0, float(np.percentile(np.abs(screen_y_axis), 99.0)))
        scale = min(820.0 / horizontal_extent, 410.0 / vertical_extent)
        u = np.rint(960.0 + rotated_x * scale).astype(np.int32)
        v = np.rint(590.0 + screen_y_axis * scale).astype(np.int32)
        valid = (u >= 8) & (u < FRAME_WIDTH - 8) & (v >= 92) & (v < 1028)
        if np.any(valid):
            u = u[valid]
            v = v[valid]
            z = points[valid, 2]
            depth = depth_axis[valid]
            z_low = float(np.percentile(z, 1.0))
            z_high = max(z_low + 0.1, float(np.percentile(z, 99.0)))
            z_color = np.clip(
                (z - z_low) / (z_high - z_low) * 255.0,
                0, 255).astype(np.uint8)
            colors = cv2.applyColorMap(
                z_color.reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)
            # 先画远点再画近点，使墙面和家具轮廓更清楚。
            order = np.argsort(depth)[::-1]
            frame[v[order], u[order]] = colors[order]
            frame[
                np.clip(v[order] + 1, 0, FRAME_HEIGHT - 1),
                u[order],
            ] = colors[order]

        self._draw_path(frame, yaw, pitch, scale)
        cv2.rectangle(frame, (0, 0), (FRAME_WIDTH - 1, 76), (18, 18, 18), -1)
        cv2.putText(
            frame, 'HazardWalker 3D Voxel Map', (30, 42),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (245, 245, 245), 2,
            cv2.LINE_AA)
        cv2.putText(
            frame,
            f'floor={self.floor_index}  state={self.nav_state}  '
            f'voxels={len(points):,}  orbit={math.degrees(yaw) % 360.0:.1f} deg',
            (1030, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
            (120, 210, 255), 1, cv2.LINE_AA)
        self._draw_height_legend(frame)
        return frame

    def _draw_path(self, frame, yaw, pitch, scale):
        if len(self.path) < 2 or self._scene_center is None:
            return
        path = np.asarray(self.path, dtype=np.float32)
        x = path[:, 0] - self._scene_center[0]
        y = path[:, 1] - self._scene_center[1]
        cosine, sine = math.cos(yaw), math.sin(yaw)
        rotated_x = cosine * x - sine * y
        rotated_y = sine * x + cosine * y
        screen_y_axis = math.sin(pitch) * rotated_y - math.cos(pitch) * path[:, 2]
        u = np.rint(960.0 + rotated_x * scale).astype(np.int32)
        v = np.rint(590.0 + screen_y_axis * scale).astype(np.int32)
        mask = (u >= 0) & (u < FRAME_WIDTH) & (v >= 78) & (v < FRAME_HEIGHT)
        polyline = np.column_stack((u[mask], v[mask])).astype(np.int32)
        if len(polyline) >= 2:
            cv2.polylines(
                frame, [polyline], False, (245, 245, 245), 3,
                cv2.LINE_AA)
            cv2.circle(frame, tuple(polyline[-1]), 7, (0, 80, 255), -1)

    @staticmethod
    def _draw_height_legend(frame):
        gradient = np.arange(255, -1, -1, dtype=np.uint8).reshape(256, 1)
        colors = cv2.applyColorMap(gradient, cv2.COLORMAP_TURBO)
        colors = cv2.resize(colors, (24, 256), interpolation=cv2.INTER_NEAREST)
        frame[790:1046, 1850:1874] = colors
        cv2.putText(frame, 'high', (1785, 806), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (230, 230, 230), 1, cv2.LINE_AA)
        cv2.putText(frame, 'low', (1792, 1042), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (230, 230, 230), 1, cv2.LINE_AA)

    def close(self):
        with self._lock:
            if self.writer is not None:
                self.writer.release()
                self.writer = None
                self.get_logger().info(
                    f'三维点云视频已保存：{self.output_path}，'
                    f'帧数={self.frame_count}')

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = PointcloudVideoRecorder()
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
