#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mid-360仿真原始PointCloud预处理：三维点云、FAST-LIO消息和二维安全扫描。

所属组：SLAM与导航组。负责人：姜晨。
文件作用：只消费公开 `/livox/lidar_raw`，不订阅任何Gazebo里程计或真值；
保持原始传感器坐标发布PointCloud2/CustomMsg，同时按公开安装俯仰生成水平
LaserScan，供二维占用图和局部避障复用。
"""

import math

import numpy as np
import rospy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import LaserScan, PointCloud, PointCloud2
from std_msgs.msg import Header
from unitree_guide.msg import CustomMsg, CustomPoint


class Livox3dPreprocessor:
    """将一个三维扫描拆成LIO输入、可视化点云和水平二维扫描。"""

    def __init__(self):
        self.input_topic = rospy.get_param(
            '~input_topic', '/livox/lidar_raw')
        self.sensor_frame = rospy.get_param('~sensor_frame', 'laser_livox')
        self.scan_frame = rospy.get_param('~scan_frame', 'laser_scan')
        self.blind_m = float(rospy.get_param('~blind_m', 0.40))
        # 官方楼宇室内结构在 20 m 内已可充分覆盖；远端量程边缘主要是
        # Gazebo multiray 无回波噪声，截断后可显著降低三维伪球壳。
        self.max_range_m = float(rospy.get_param('~max_range_m', 20.0))
        self.mount_pitch_rad = float(rospy.get_param(
            '~mount_pitch_rad', math.radians(45.0)))
        self.scan_height_min_m = float(rospy.get_param(
            '~scan_height_min_m', -0.25))
        self.scan_height_max_m = float(rospy.get_param(
            '~scan_height_max_m', 0.50))
        self.scan_bins = int(rospy.get_param('~scan_bins', 360))
        self.scan_period_sec = float(rospy.get_param(
            '~scan_period_sec', 0.10))
        if (self.scan_bins < 16 or self.scan_period_sec <= 0.0
                or self.blind_m < 0.0
                or self.max_range_m <= self.blind_m):
            raise ValueError('Mid-360预处理参数无效')

        self.cloud_pub = rospy.Publisher(
            '/livox/Pointcloud2', PointCloud2, queue_size=1)
        self.custom_pub = rospy.Publisher(
            '/livox/lidar2', CustomMsg, queue_size=1)
        self.scan_pub = rospy.Publisher(
            '/livox/scan_projection', LaserScan, queue_size=1)
        self.sub = rospy.Subscriber(
            self.input_topic, PointCloud, self.on_cloud,
            queue_size=1, buff_size=32 * 1024 * 1024)
        rospy.loginfo(
            'Mid-360 3D preprocessor ready: %s -> '
            '/livox/Pointcloud2 + /livox/lidar2 + /livox/scan_projection',
            self.input_topic)

    def on_cloud(self, message):
        if not message.points:
            return
        points = np.asarray(
            [(point.x, point.y, point.z) for point in message.points],
            dtype=np.float32)
        distance = np.linalg.norm(points, axis=1)
        mask = (
            np.isfinite(points).all(axis=1)
            & (distance >= self.blind_m)
            & (distance <= self.max_range_m)
        )
        points = points[mask]
        if points.size == 0:
            return
        stamp = message.header.stamp
        if stamp == rospy.Time():
            stamp = rospy.Time.now()

        if self.cloud_pub.get_num_connections() > 0:
            header = Header(stamp=stamp, frame_id=self.sensor_frame)
            self.cloud_pub.publish(pc2.create_cloud_xyz32(header, points))
        if self.custom_pub.get_num_connections() > 0:
            self.custom_pub.publish(self._custom_message(points, stamp))
        self.scan_pub.publish(self._horizontal_scan(points, stamp))

    def _custom_message(self, points, stamp):
        message = CustomMsg()
        message.header.stamp = stamp
        message.header.frame_id = self.sensor_frame
        message.timebase = stamp.to_nsec()
        message.point_num = int(points.shape[0])
        message.lidar_id = 1
        message.rsvd = [0, 0, 0]
        output = []
        for x, y, z in points:
            point = CustomPoint()
            # Gazebo 插件在一次 update 内同时完成所有 ray 查询，并没有
            # Mid-360 真机逐点曝光时刻。不得虚构 0~100 ms 扫描时间，否则
            # FAST-LIO 会用 IMU 对一帧静态点云做错误去畸变并产生明显漂移。
            point.offset_time = 0
            point.x = float(x)
            point.y = float(y)
            point.z = float(z)
            point.reflectivity = 0
            point.tag = 0x10
            point.line = 0
            output.append(point)
        message.points = output
        return message

    def _horizontal_scan(self, points, stamp):
        cosine = math.cos(self.mount_pitch_rad)
        sine = math.sin(self.mount_pitch_rad)
        base_x = cosine * points[:, 0] + sine * points[:, 2]
        base_y = points[:, 1]
        base_z = -sine * points[:, 0] + cosine * points[:, 2]
        planar_range = np.hypot(base_x, base_y)
        valid = (
            (base_z >= self.scan_height_min_m)
            & (base_z <= self.scan_height_max_m)
            & (planar_range >= self.blind_m)
            & (planar_range <= self.max_range_m)
        )
        angles = np.arctan2(base_y[valid], base_x[valid])
        ranges = planar_range[valid]
        bins = np.floor(
            (angles + math.pi) * self.scan_bins / (2.0 * math.pi)
        ).astype(np.int32)
        bins = np.clip(bins, 0, self.scan_bins - 1)
        scan_ranges = np.full(self.scan_bins, np.inf, dtype=np.float32)
        np.minimum.at(scan_ranges, bins, ranges)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.scan_frame
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / self.scan_bins
        scan.angle_max = math.pi - scan.angle_increment
        scan.time_increment = self.scan_period_sec / self.scan_bins
        scan.scan_time = self.scan_period_sec
        scan.range_min = self.blind_m
        scan.range_max = self.max_range_m
        scan.ranges = scan_ranges.tolist()
        return scan


def main():
    rospy.init_node('hazardwalker_livox_3d_preprocessor')
    Livox3dPreprocessor()
    rospy.spin()


if __name__ == '__main__':
    main()
