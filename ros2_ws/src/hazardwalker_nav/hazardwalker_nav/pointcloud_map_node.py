#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""将官方 Mid-360 点云按合法 SLAM 位姿累积成有界三维体素地图。

所属组：SLAM与导航组。负责人：姜晨。
本节点只消费公开点云与 TF，不读取 Gazebo 真值、场景文件或目标位置。水平
位姿由 Cartographer 位姿图提供，点云在 map 坐标系内体素化，既避免重复点
无限增长，也可在任务结束时保存 PLY/PCD 供整栋楼地图展示和视频制作。
"""

from __future__ import annotations

import math
import os
import threading
from typing import Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMessage
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .pointcloud_map import (
    quaternion_transform_matrix,
    transform_points,
    voxel_indices,
)


class PointcloudMapNode(Node):
    """在线累积、发布并保存多层三维体素地图。"""

    def __init__(self) -> None:
        super().__init__('hazardwalker_pointcloud_map')
        self.declare_parameter('input_topic', '/hw/lidar/points')
        self.declare_parameter('output_topic', '/hazardwalker/slam/cloud_map')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('voxel_size_m', 0.08)
        self.declare_parameter('max_range_m', 30.0)
        self.declare_parameter('max_voxels', 1_000_000)
        self.declare_parameter('publish_period_s', 1.0)
        self.declare_parameter('tf_timeout_s', 0.20)
        self.declare_parameter('output_dir', '')

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.voxel_size_m = float(self.get_parameter('voxel_size_m').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        self.max_voxels = int(self.get_parameter('max_voxels').value)
        self.tf_timeout_s = float(self.get_parameter('tf_timeout_s').value)
        publish_period_s = float(
            self.get_parameter('publish_period_s').value)
        numeric = (
            self.voxel_size_m, self.max_range_m, self.tf_timeout_s,
            publish_period_s,
        )
        if (not all(math.isfinite(value) and value > 0.0 for value in numeric)
                or self.max_voxels <= 0 or not self.target_frame):
            raise ValueError('三维地图参数无效')

        self._voxels: set[tuple[int, int, int]] = set()
        self._lock = threading.Lock()
        self._dropped_tf_frames = 0
        self._overflowed = False
        self._last_stamp = TimeMessage()
        self._closed = False
        self._saved_voxel_count = -1
        self._mission_finished = False

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter('output_topic').value), 1)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('input_topic').value),
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.create_service(
            Trigger, '/hazardwalker/slam/save_cloud_map', self.on_save)
        self.create_subscription(
            String, '/hw/nav/state', self.on_nav_state, 10)
        self.create_timer(publish_period_s, self.publish_map)
        self.get_logger().info(
            f'三维体素地图已启动：voxel={self.voxel_size_m:.3f} m，'
            f'max_voxels={self.max_voxels}，frame={self.target_frame}'
        )

    def on_cloud(self, message: PointCloud2) -> None:
        """将一帧点云变换至 map，并以体素集合实现有界去重。"""

        if (self._overflowed or self._mission_finished
                or not message.header.frame_id):
            return
        try:
            raw = point_cloud2.read_points_numpy(
                message, field_names=('x', 'y', 'z'), skip_nans=True)
            points = np.asarray(raw, dtype=np.float64).reshape(-1, 3)
        except (AssertionError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f'拒绝无效三维点云：{exc}', throttle_duration_sec=2.0)
            return
        if points.size == 0:
            return
        distances = np.linalg.norm(points, axis=1)
        points = points[
            np.isfinite(points).all(axis=1)
            & (distances >= 0.40)
            & (distances <= self.max_range_m)
        ]
        if points.size == 0:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self.tf_timeout_s),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            matrix = quaternion_transform_matrix(
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
            )
            world_points = transform_points(points, matrix)
        except TransformException as exact_error:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame, message.header.frame_id, Time(),
                    timeout=Duration(seconds=self.tf_timeout_s),
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                matrix = quaternion_transform_matrix(
                    (translation.x, translation.y, translation.z),
                    (rotation.x, rotation.y, rotation.z, rotation.w),
                )
                world_points = transform_points(points, matrix)
            except (TransformException, ValueError):
                self._dropped_tf_frames += 1
                self.get_logger().warning(
                    f'三维点云等待合法 TF：{exact_error}',
                    throttle_duration_sec=3.0)
                return
        except ValueError as exc:
            self._dropped_tf_frames += 1
            self.get_logger().warning(
                f'拒绝无效三维 TF：{exc}', throttle_duration_sec=3.0)
            return

        unique = voxel_indices(world_points, self.voxel_size_m)
        incoming = set(map(tuple, unique.tolist()))
        with self._lock:
            if len(self._voxels) + len(incoming - self._voxels) > self.max_voxels:
                self._overflowed = True
                self.get_logger().error(
                    '三维地图达到体素上限，已停止累积；请增大体素或上限')
                return
            self._voxels.update(incoming)
            self._last_stamp = message.header.stamp

    def _points_snapshot(self) -> np.ndarray:
        with self._lock:
            if not self._voxels:
                return np.empty((0, 3), dtype=np.float32)
            # 排序百万级 Python tuple 会长时间持有 GIL，并可能被 launch 的
            # SIGINT 打断。体素地图无顺序语义，直接复制集合即可快速封存。
            indices = np.asarray(list(self._voxels), dtype=np.float32)
        return (indices + 0.5) * self.voxel_size_m

    def publish_map(self) -> None:
        points = self._points_snapshot()
        if points.size == 0 or self.publisher.get_subscription_count() == 0:
            return
        header = Header()
        header.frame_id = self.target_frame
        header.stamp = self._last_stamp
        self.publisher.publish(point_cloud2.create_cloud_xyz32(header, points))

    def _save(self) -> tuple[bool, str]:
        output_dir = str(self.get_parameter('output_dir').value).strip()
        if not output_dir:
            return False, '未配置 output_dir'
        points = self._points_snapshot().astype('<f4', copy=False)
        if points.size == 0:
            return False, '三维地图为空'
        os.makedirs(output_dir, exist_ok=True)
        pcd_path = os.path.join(output_dir, 'cloud_map.pcd')
        ply_path = os.path.join(output_dir, 'cloud_map.ply')
        with open(pcd_path, 'wb') as handle:
            handle.write((
                '# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\n'
                'SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                f'POINTS {len(points)}\nDATA binary\n'
            ).encode('ascii'))
            handle.write(points.tobytes(order='C'))
        with open(ply_path, 'wb') as handle:
            handle.write((
                'ply\nformat binary_little_endian 1.0\n'
                f'element vertex {len(points)}\n'
                'property float x\nproperty float y\nproperty float z\n'
                'end_header\n'
            ).encode('ascii'))
            handle.write(points.tobytes(order='C'))
        self._saved_voxel_count = len(points)
        return True, f'已保存 {len(points)} 个体素：{output_dir}'

    def on_save(self, request, response):
        del request
        with self._lock:
            voxel_count = len(self._voxels)
        if voxel_count > 0 and voxel_count == self._saved_voxel_count:
            response.success = True
            response.message = '三维地图已封存，无新增体素'
            return response
        response.success, response.message = self._save()
        return response

    def on_nav_state(self, message: String) -> None:
        """导航完成后、launch 发出 SIGINT 前主动封存三维地图。"""

        if (message.data.strip().upper() != 'FINISHED'
                or self._mission_finished):
            return
        self._mission_finished = True
        success, detail = self._save()
        if success:
            self.get_logger().info(detail)
        else:
            self.get_logger().error(f'导航完成但三维地图封存失败：{detail}')

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        with self._lock:
            voxel_count = len(self._voxels)
        if voxel_count == self._saved_voxel_count:
            return
        success, message = self._save()
        if success:
            self.get_logger().info(message)
        elif self._voxels:
            self.get_logger().warning(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[PointcloudMapNode] = None
    try:
        node = PointcloudMapNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
