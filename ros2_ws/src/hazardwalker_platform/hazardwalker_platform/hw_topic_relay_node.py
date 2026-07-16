"""官方平台 ROS1 Docker → HazardWalker /hw/* 话题中继节点。

数据流:
  Docker ROS1 → ros1_bridge → ROS2 /Odometry_gazebo → 本节点 → /hw/Odometry_gazebo

命名规则: /hw/<原始ROS1话题名>, 保持透明可追溯。
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage


class HwTopicRelayNode(Node):
    """把 ros1_bridge 出来的 ROS2 话题改名发布到 /hw/*。"""

    def __init__(self) -> None:
        super().__init__("hw_topic_relay")

        # ---- 里程计: /Odometry_gazebo → /hw/Odometry_gazebo ----
        self._pub_odom = self.create_publisher(Odometry, "/hw/Odometry_gazebo", 10)
        self.create_subscription(Odometry, "/Odometry_gazebo", self._forward_odom, 10)

        # ---- Livox 激光雷达 ----

        self._pub_livox_cloud = self.create_publisher(PointCloud2, "/hw/livox/Pointcloud2", 10)
        self.create_subscription(PointCloud2, "/livox/Pointcloud2", self._forward_livox_cloud, 10)


        self._pub_livox_imu = self.create_publisher(Imu, "/hw/livox/imu", 10)
        self.create_subscription(Imu, "/livox/imu", self._forward_livox_imu, 10)

        # ---- 躯干 IMU: /trunk_imu → /hw/trunk_imu ----
        self._pub_imu = self.create_publisher(Imu, "/hw/trunk_imu", 10)
        self.create_subscription(Imu, "/trunk_imu", self._forward_imu, 10)

        # ---- RealSense D415 深度相机 (主相机) ----
        self._pub_rs_rgb = self.create_publisher(Image, "/hw/real_sense/rgb/image_raw", 10)
        self.create_subscription(Image, "/real_sense/rgb/image_raw", self._forward_rs_rgb, 10)

        self._pub_rs_depth_img = self.create_publisher(Image, "/hw/real_sense/depth/image_raw", 10)
        self.create_subscription(Image, "/real_sense/depth/image_raw", self._forward_rs_depth_img, 10)

        self._pub_rs_depth_pts = self.create_publisher(
            PointCloud2, "/hw/real_sense/depth/points", 10
        )
        self.create_subscription(
            PointCloud2, "/real_sense/depth/points", self._forward_rs_depth_pts, 10
        )

        # ---- LaserScan: /scan → /hw/scan ----
        self._pub_scan = self.create_publisher(LaserScan, "/hw/scan", 10)
        self.create_subscription(LaserScan, "/scan", self._forward_scan, 10)

        # ---- TF 坐标变换: /tf → /hw/tf ----
        self._pub_tf = self.create_publisher(TFMessage, "/hw/tf", 10)
        self.create_subscription(TFMessage, "/tf", self._forward_tf, 10)

        # ---- 控制 (HazardWalker → Docker): /hw/cmd_vel → /cmd_vel ----
        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Twist, "/hw/cmd_vel", self._forward_cmd, 10)

        self.get_logger().info("ROS1 Docker → /hw/* topic relay ready")

    # ---- 回调: 直接转发, 不改消息内容 ----

    def _forward_odom(self, msg: Odometry) -> None:
        self._pub_odom.publish(msg)


    def _forward_livox_cloud(self, msg: PointCloud2) -> None:
        self._pub_livox_cloud.publish(msg)


    def _forward_livox_imu(self, msg: Imu) -> None:
        self._pub_livox_imu.publish(msg)

    def _forward_scan(self, msg: LaserScan) -> None:
        self._pub_scan.publish(msg)

    def _forward_imu(self, msg: Imu) -> None:
        self._pub_imu.publish(msg)

    def _forward_rs_rgb(self, msg: Image) -> None:
        self._pub_rs_rgb.publish(msg)

    def _forward_rs_depth_img(self, msg: Image) -> None:
        self._pub_rs_depth_img.publish(msg)

    def _forward_rs_depth_pts(self, msg: PointCloud2) -> None:
        self._pub_rs_depth_pts.publish(msg)

    def _forward_tf(self, msg: TFMessage) -> None:
        self._pub_tf.publish(msg)

    def _forward_cmd(self, msg: Twist) -> None:
        self._pub_cmd.publish(msg)


def main() -> None:
    rclpy.init()
    node = HwTopicRelayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
