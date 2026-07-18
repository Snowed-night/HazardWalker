"""官方 SimEnv RGB-D 前向深度转 LaserScan 节点。

所属组：感知定位组。只订阅公开 `/hw/camera/depth_*`，发布
`/hw/depth_scan`，用于补充 360° 稀疏雷达的前向墙体/家具轮廓。
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from hazardwalker_perception.depth_scan import depth_image_to_planar_scan


class DepthToScanNode(Node):
    """将 32FC1/16UC1 深度图转换为机体系前向水平扫描。"""

    def __init__(self):
        super().__init__('hazardwalker_depth_to_scan')
        self.declare_parameter('depth_topic', '/hw/camera/depth_image')
        self.declare_parameter(
            'camera_info_topic', '/hw/camera/depth_camera_info',
        )
        self.declare_parameter('output_topic', '/hw/depth_scan')
        self.declare_parameter('output_frame', 'real_sense')
        self.declare_parameter('row_min_fraction', 0.35)
        self.declare_parameter('row_max_fraction', 0.65)
        self.declare_parameter('column_stride', 2)
        self.declare_parameter('depth_percentile', 10.0)
        self.declare_parameter('min_range_m', 0.40)
        self.declare_parameter('max_range_m', 8.0)
        self._camera_info = None
        self._publisher = self.create_publisher(
            LaserScan, str(self.get_parameter('output_topic').value), 10,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_topic').value),
            self._on_depth,
            qos_profile_sensor_data,
        )

    def _on_camera_info(self, message):
        self._camera_info = message

    @staticmethod
    def _decode_depth(message):
        encoding = str(message.encoding).upper()
        if encoding == '32FC1':
            dtype = np.dtype('>f4' if message.is_bigendian else '<f4')
            scale = 1.0
        elif encoding in ('16UC1', 'MONO16'):
            dtype = np.dtype('>u2' if message.is_bigendian else '<u2')
            scale = 0.001
        else:
            raise ValueError('unsupported depth encoding: %s' % encoding)
        row_elements = int(message.step) // dtype.itemsize
        array = np.frombuffer(bytes(message.data), dtype=dtype).reshape(
            int(message.height), row_elements,
        )
        return array[:, :int(message.width)].astype(np.float32) * scale

    def _on_depth(self, message):
        if self._camera_info is None or len(self._camera_info.k) < 3:
            self.get_logger().warn(
                '等待深度 CameraInfo 后再发布 /hw/depth_scan。',
                throttle_duration_sec=5.0,
            )
            return
        try:
            depth = self._decode_depth(message)
            angle_min, angle_max, angle_increment, ranges = (
                depth_image_to_planar_scan(
                    depth,
                    fx=float(self._camera_info.k[0]),
                    cx=float(self._camera_info.k[2]),
                    row_min_fraction=float(
                        self.get_parameter('row_min_fraction').value),
                    row_max_fraction=float(
                        self.get_parameter('row_max_fraction').value),
                    column_stride=int(
                        self.get_parameter('column_stride').value),
                    depth_percentile=float(
                        self.get_parameter('depth_percentile').value),
                    min_range_m=float(
                        self.get_parameter('min_range_m').value),
                    max_range_m=float(
                        self.get_parameter('max_range_m').value),
                )
            )
        except (TypeError, ValueError) as error:
            self.get_logger().error(
                '深度转扫描失败：%s' % error,
                throttle_duration_sec=5.0,
            )
            return

        scan = LaserScan()
        scan.header.stamp = message.header.stamp
        scan.header.frame_id = str(
            self.get_parameter('output_frame').value,
        )
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.range_min = float(self.get_parameter('min_range_m').value)
        scan.range_max = float(self.get_parameter('max_range_m').value)
        scan.ranges = ranges
        self._publisher.publish(scan)


def main():
    rclpy.init()
    node = DepthToScanNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
