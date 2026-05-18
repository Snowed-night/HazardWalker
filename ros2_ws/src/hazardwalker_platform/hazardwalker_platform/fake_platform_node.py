import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import TransformBroadcaster


class FakePlatformNode(Node):
    """Minimal in-process platform adapter for early integration tests.

    This node is not a simulator replacement. It only publishes normalized
    HazardWalker topics so nav/perception/decision nodes can be wired together
    before Gazebo or the official platform is available.
    """

    def __init__(self):
        super().__init__('fake_platform_node')
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('red_ball_visible', True)

        self.width = int(self.get_parameter('image_width').value)
        self.height = int(self.get_parameter('image_height').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self.image_pub = self.create_publisher(Image, '/hw/camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/hw/camera/camera_info', 10)
        self.odom_pub = self.create_publisher(Odometry, '/hw/odom', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/hw/lidar/points', 10)
        self.cmd_sub = self.create_subscription(Twist, '/hw/cmd_vel', self.on_cmd_vel, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.cmd = Twist()
        self.last_time = time.monotonic()
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info('Fake platform publishing /hw sensor and odom topics.')

    def on_cmd_vel(self, msg: Twist):
        self.cmd = msg

    def on_timer(self):
        now_monotonic = time.monotonic()
        dt = max(0.0, now_monotonic - self.last_time)
        self.last_time = now_monotonic

        vx = float(self.cmd.linear.x)
        wz = float(self.cmd.angular.z)
        self.x += math.cos(self.yaw) * vx * dt
        self.y += math.sin(self.yaw) * vx * dt
        self.yaw += wz * dt

        stamp = self.get_clock().now().to_msg()
        self.publish_tf(stamp)
        self.publish_odom(stamp, vx, wz)
        self.publish_camera(stamp)
        self.publish_empty_cloud(stamp)

    def publish_tf(self, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(self.yaw / 2.0)
        transform.transform.rotation.w = math.cos(self.yaw / 2.0)
        camera_tf = TransformStamped()
        camera_tf.header.stamp = stamp
        camera_tf.header.frame_id = 'base_link'
        camera_tf.child_frame_id = 'camera_link'
        camera_tf.transform.translation.x = 0.25
        camera_tf.transform.translation.y = 0.0
        camera_tf.transform.translation.z = 0.35
        camera_tf.transform.rotation.w = 1.0

        lidar_tf = TransformStamped()
        lidar_tf.header.stamp = stamp
        lidar_tf.header.frame_id = 'base_link'
        lidar_tf.child_frame_id = 'lidar_link'
        lidar_tf.transform.translation.x = 0.15
        lidar_tf.transform.translation.y = 0.0
        lidar_tf.transform.translation.z = 0.45
        lidar_tf.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([transform, camera_tf, lidar_tf])

    def publish_odom(self, stamp, vx, wz):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

    def publish_camera(self, stamp):
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = 'camera_link'
        image.height = self.height
        image.width = self.width
        image.encoding = 'rgb8'
        image.is_bigendian = 0
        image.step = self.width * 3

        data = bytearray(self.width * self.height * 3)
        for i in range(0, len(data), 3):
            data[i] = 30
            data[i + 1] = 30
            data[i + 2] = 30

        if bool(self.get_parameter('red_ball_visible').value):
            cx = int(self.width * 0.55)
            cy = int(self.height * 0.48)
            radius = int(min(self.width, self.height) * 0.08)
            radius_sq = radius * radius
            for py in range(max(0, cy - radius), min(self.height, cy + radius + 1)):
                for px in range(max(0, cx - radius), min(self.width, cx + radius + 1)):
                    if (px - cx) * (px - cx) + (py - cy) * (py - cy) <= radius_sq:
                        index = (py * self.width + px) * 3
                        data[index] = 230
                        data[index + 1] = 20
                        data[index + 2] = 20

        image.data = bytes(data)
        self.image_pub.publish(image)

        info = CameraInfo()
        info.header = image.header
        info.height = self.height
        info.width = self.width
        fx = fy = 260.0
        cx = self.width / 2.0
        cy = self.height / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(info)

    def publish_empty_cloud(self, stamp):
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = 'lidar_link'
        cloud.height = 1
        cloud.width = 0
        cloud.is_dense = False
        self.cloud_pub.publish(cloud)


def main():
    rclpy.init()
    node = FakePlatformNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
