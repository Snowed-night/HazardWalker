#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""依据三维 SLAM 位姿，把水平激光维护成互不覆盖的逐楼层占用地图。

所属组：SLAM与导航组。负责人：姜晨。
节点只消费公开 LaserScan、楼层动作和 Cartographer TF。每层独立维护 log-odds
栅格，当前层统一发布 `/map`，避免三层墙体压到同一二维平面阻塞 Frontier。
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformException, TransformListener

from .multifloor_occupancy import (
    grid_to_occupancy,
    update_log_odds_ray,
    world_to_cell,
)


class MultifloorOccupancyMapperNode(Node):
    """按楼层分离的在线二维激光建图器。"""

    def __init__(self) -> None:
        super().__init__('hazardwalker_multifloor_occupancy_mapper')
        self.declare_parameter('scan_topic', '/hw/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter(
            'floor_index_topic', '/hazardwalker/navigation/floor_index')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('initial_floor_index', 0)
        self.declare_parameter('resolution_m', 0.10)
        self.declare_parameter('map_size_m', 60.0)
        self.declare_parameter('max_ray_length_m', 12.0)
        self.declare_parameter('occupied_score', 3)
        self.declare_parameter('publish_period_s', 1.0)
        self.declare_parameter('tf_timeout_s', 0.20)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.resolution_m = float(self.get_parameter('resolution_m').value)
        map_size_m = float(self.get_parameter('map_size_m').value)
        self.max_ray_length_m = float(
            self.get_parameter('max_ray_length_m').value)
        self.occupied_score = int(self.get_parameter('occupied_score').value)
        self.tf_timeout_s = float(self.get_parameter('tf_timeout_s').value)
        publish_period_s = float(
            self.get_parameter('publish_period_s').value)
        numeric = (
            self.resolution_m, map_size_m, self.max_ray_length_m,
            self.tf_timeout_s, publish_period_s,
        )
        if (not all(math.isfinite(value) and value > 0.0 for value in numeric)
                or not self.map_frame):
            raise ValueError('分层占用地图参数无效')
        self.side_cells = int(math.ceil(map_size_m / self.resolution_m))
        if self.side_cells < 100:
            raise ValueError('分层地图边长过小')

        self.current_floor = int(
            self.get_parameter('initial_floor_index').value)
        self.layers: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        self.origin_x = None
        self.origin_y = None
        self._last_stamp = None
        self._last_publish_monotonic = 0.0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('map_topic').value), map_qos)
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            self.on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Int32, str(self.get_parameter('floor_index_topic').value),
            self.on_floor_index, 10)
        self.get_logger().info(
            f'分层占用地图已启动：{self.side_cells}x{self.side_cells}，'
            f'{self.resolution_m:.2f} m/cell，floor={self.current_floor}'
        )

    def _layer(self, floor_index: int):
        if floor_index not in self.layers:
            shape = (self.side_cells, self.side_cells)
            self.layers[floor_index] = (
                np.zeros(shape, dtype=np.int16),
                np.zeros(shape, dtype=np.bool_),
            )
        return self.layers[floor_index]

    def on_floor_index(self, message: Int32) -> None:
        new_floor = int(message.data)
        if new_floor == self.current_floor:
            return
        self.current_floor = new_floor
        self._layer(new_floor)
        self.get_logger().info(f'切换导航占用层：floor={new_floor}')
        self.publish_map(force=True)

    @staticmethod
    def _yaw(quaternion: Quaternion) -> float:
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z
                   + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y
                         + quaternion.z * quaternion.z),
        )

    def on_scan(self, message: LaserScan) -> None:
        if not message.header.frame_id or message.angle_increment <= 0.0:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self.tf_timeout_s),
            )
        except TransformException as exc:
            # rosbridge 到达顺序可能让 scan 比同时间 TF 早 2~5 ms。只在精确
            # 时刻不可用时回退最新合法 TF；10 Hz 扫描下最大位移误差受限于
            # 一帧，避免因为微小未来外推差把所有扫描静默丢弃。
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, message.header.frame_id, Time(),
                    timeout=Duration(seconds=self.tf_timeout_s),
                )
            except TransformException:
                self.get_logger().warning(
                    f'分层地图等待 SLAM TF：{exc}',
                    throttle_duration_sec=3.0)
                return
        tx = float(transform.transform.translation.x)
        ty = float(transform.transform.translation.y)
        yaw = self._yaw(transform.transform.rotation)
        if self.origin_x is None:
            half = self.side_cells * self.resolution_m * 0.5
            self.origin_x = math.floor(
                (tx - half) / self.resolution_m) * self.resolution_m
            self.origin_y = math.floor(
                (ty - half) / self.resolution_m) * self.resolution_m
        start = world_to_cell(
            tx, ty, self.origin_x, self.origin_y, self.resolution_m)
        scores, seen = self._layer(self.current_floor)
        for index, raw_range in enumerate(message.ranges):
            angle = yaw + message.angle_min + index * message.angle_increment
            finite_hit = (
                math.isfinite(raw_range)
                and message.range_min <= raw_range <= message.range_max
                and raw_range <= self.max_ray_length_m
            )
            ray_range = (
                float(raw_range) if finite_hit else self.max_ray_length_m)
            end_x = tx + ray_range * math.cos(angle)
            end_y = ty + ray_range * math.sin(angle)
            end = world_to_cell(
                end_x, end_y, self.origin_x, self.origin_y,
                self.resolution_m)
            update_log_odds_ray(
                scores, seen, start, end, endpoint_is_hit=finite_hit)
        self._last_stamp = message.header.stamp
        self.publish_map()

    def publish_map(self, force: bool = False) -> None:
        if self.origin_x is None or self._last_stamp is None:
            return
        now = time.monotonic()
        period = float(self.get_parameter('publish_period_s').value)
        if not force and now - self._last_publish_monotonic < period:
            return
        self._last_publish_monotonic = now
        scores, seen = self._layer(self.current_floor)
        values = grid_to_occupancy(
            scores, seen, occupied_score=self.occupied_score)
        message = OccupancyGrid()
        message.header.stamp = self._last_stamp
        message.header.frame_id = self.map_frame
        message.info.map_load_time = self._last_stamp
        message.info.resolution = self.resolution_m
        message.info.width = self.side_cells
        message.info.height = self.side_cells
        message.info.origin.position.x = self.origin_x
        message.info.origin.position.y = self.origin_y
        message.info.origin.orientation.w = 1.0
        message.data = values.reshape(-1).tolist()
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultifloorOccupancyMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
