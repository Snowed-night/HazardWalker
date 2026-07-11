"""HazardWalker 平台适配节点（官方平台规格版）。

所属组：平台组。
文件作用：
- 在没有真实 Gazebo 或官方平台时，模拟 /hw/* 话题输出和简单控制响应。
- 传感器规格对齐官方比赛平台（SimEnv）：800x800 前视相机、IMU、深度点云、Livox 雷达。

当前职责：
- 发布相机图像、CameraInfo、里程计、IMU、深度点云和空雷达点云。
- 接收 /hw/cmd_vel 并用简化运动学更新位置。
- 为感知组提供可控红球图像，为导航组提供可控里程计输入。

官方平台参考：
- ROS 1 Noetic + Gazebo Classic + Unitree A1
- 前视相机: /camera/image_raw, 800x800, 30Hz
- Livox Mid-360: /scan, 360°x57°, 10Hz
- RealSense D415: /real_sense/depth/points, 640x480, 10Hz
- IMU: /trunk_imu, 1000Hz
- 里程计: /Odometry_gazebo, 100Hz
- 控制: /cmd_vel (RL 模式下生效)

后续扩展方式：
- 有真实仿真或官方平台后，替换成 gazebo_adapter_node.py 或 official_adapter_node.py。
- 替换时保持 /hw/* 话题名不变，只改数据来源。
"""
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, PointCloud2
from tf2_ros import TransformBroadcaster


class FakePlatformNode(Node):
    def __init__(self):
        super().__init__('fake_platform_node')

        # ---- 相机参数（对齐官方前视相机: 800x800） ----
        self.declare_parameter('image_width', 800)
        self.declare_parameter('image_height', 800)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('red_ball_visible', True)

        self.width = int(self.get_parameter('image_width').value)
        self.height = int(self.get_parameter('image_height').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        # ---- 话题发布（全部使用 /hw/* 内部接口） ----
        # 传感器输出
        self.image_pub = self.create_publisher(Image, '/hw/real_sense/rgb/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, '/hw/trunk_imu', 10)
        self.scan_pub = self.create_publisher(PointCloud2, '/hw/scan', 10)
        self.livox_pub = self.create_publisher(PointCloud2, '/hw/livox/Pointcloud2', 10)
        self.depth_cloud_pub = self.create_publisher(PointCloud2, '/hw/real_sense/depth/points', 10)
        self.odom_pub = self.create_publisher(Odometry, '/hw/Odometry_gazebo', 10)
        # 控制输入
        self.cmd_sub = self.create_subscription(Twist, '/hw/cmd_vel', self.on_cmd_vel, 10)
        # TF 广播
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---- 机器人位姿（简化2D模型） ----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.cmd = Twist()
        self.last_time = time.monotonic()
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'Fake platform (official spec): camera={self.width}x{self.height}, '
            f'publishing /hw sensor and odom topics.'
        )

    # =====================================================================
    # 回调
    # =====================================================================

    def on_cmd_vel(self, msg: Twist):
        self.cmd = msg

    def on_timer(self):
        now_monotonic = time.monotonic()
        dt = max(0.0, now_monotonic - self.last_time)
        self.last_time = now_monotonic

        # 简化运动学：linear.x 前进，angular.z 转向
        vx = float(self.cmd.linear.x)
        wz = float(self.cmd.angular.z)
        self.x += math.cos(self.yaw) * vx * dt
        self.y += math.sin(self.yaw) * vx * dt
        self.yaw += wz * dt

        stamp = self.get_clock().now().to_msg()
        self.publish_tf(stamp)
        self.publish_odom(stamp, vx, wz)
        self.publish_imu(stamp)
        self.publish_camera(stamp)
        self.publish_depth_cloud(stamp)
        self.publish_lidar_cloud(stamp)

    # =====================================================================
    # TF 链（对齐官方 A1 坐标系）
    #
    # odom ──→ base_link ──→ trunk ──→ imu_link
    #                    ├──→ front_camera    (0.25, 0, 0.35)  前视RGB
    #                    ├──→ laser_livox     (0.20, 0, 0.08)  Livox雷达
    #                    └──→ real_sense      (0.28, 0, 0.043) 深度相机
    # =====================================================================

    def publish_tf(self, stamp):
        # odom → base_link
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(self.yaw / 2.0)
        transform.transform.rotation.w = math.cos(self.yaw / 2.0)

        # base_link → trunk（躯干/质心，略高于底盘中心）
        trunk_tf = TransformStamped()
        trunk_tf.header.stamp = stamp
        trunk_tf.header.frame_id = 'base_link'
        trunk_tf.child_frame_id = 'trunk'
        trunk_tf.transform.translation.x = 0.0
        trunk_tf.transform.translation.y = 0.0
        trunk_tf.transform.translation.z = 0.05
        trunk_tf.transform.rotation.w = 1.0

        # trunk → imu_link（IMU在躯干中心）
        imu_tf = TransformStamped()
        imu_tf.header.stamp = stamp
        imu_tf.header.frame_id = 'trunk'
        imu_tf.child_frame_id = 'imu_link'
        imu_tf.transform.translation.x = 0.0
        imu_tf.transform.translation.y = 0.0
        imu_tf.transform.translation.z = 0.0
        imu_tf.transform.rotation.w = 1.0

        # base_link → front_camera（前视RGB相机）
        camera_tf = TransformStamped()
        camera_tf.header.stamp = stamp
        camera_tf.header.frame_id = 'base_link'
        camera_tf.child_frame_id = 'front_camera'
        camera_tf.transform.translation.x = 0.25
        camera_tf.transform.translation.y = 0.0
        camera_tf.transform.translation.z = 0.35
        camera_tf.transform.rotation.w = 1.0

        # base_link → laser_livox（Livox雷达，前上方）
        lidar_tf = TransformStamped()
        lidar_tf.header.stamp = stamp
        lidar_tf.header.frame_id = 'base_link'
        lidar_tf.child_frame_id = 'laser_livox'
        lidar_tf.transform.translation.x = 0.20
        lidar_tf.transform.translation.y = 0.0
        lidar_tf.transform.translation.z = 0.08
        # 官方: 绕 Y 轴倾斜 45°(0.785 rad)，优化前方及地面覆盖
        half_45 = math.sin(0.785 / 2.0)
        lidar_tf.transform.rotation.y = half_45
        lidar_tf.transform.rotation.w = math.cos(0.785 / 2.0)

        # base_link → real_sense（深度相机，最前端）
        depth_tf = TransformStamped()
        depth_tf.header.stamp = stamp
        depth_tf.header.frame_id = 'base_link'
        depth_tf.child_frame_id = 'real_sense'
        depth_tf.transform.translation.x = 0.28
        depth_tf.transform.translation.y = 0.0
        depth_tf.transform.translation.z = 0.043
        depth_tf.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(
            [transform, trunk_tf, imu_tf, camera_tf, lidar_tf, depth_tf]
        )

    # =====================================================================
    # 传感器数据发布
    # =====================================================================

    def publish_odom(self, stamp, vx, wz):
        """里程计（官方: /Odometry_gazebo, 100Hz）。"""
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

    def publish_imu(self, stamp):
        """躯干 IMU（官方: /trunk_imu, 1000Hz）。

        当前简化版只输出重力方向加速度，角速度跟随转向指令变化。
        真实数据应由 Gazebo IMU 传感器提供。
        """
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'
        # 重力加速度（静止时只有 Z 方向 -9.81）
        imu.linear_acceleration.z = 9.81
        # 角速度（简化：只有绕 Z 轴的旋转）
        imu.angular_velocity.z = float(self.cmd.angular.z)
        # 协方差暂不填充（真实传感器会提供）
        self.imu_pub.publish(imu)

    def publish_camera(self, stamp):
        """RGB 相机（RealSense RGB, 640×480, 10Hz）。

        生成灰色背景 + 可选红色圆形区域，模拟相机看到红色球体危险源。
        """
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = 'real_sense'
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

    def publish_depth_cloud(self, stamp):
        """深度相机点云（官方: /real_sense/depth/points, 640x480, 10Hz）。

        当前发布空点云占位，后续由 Gazebo RealSense 插件填充真实数据。
        """
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = 'real_sense'
        cloud.height = 1
        cloud.width = 0
        cloud.is_dense = False
        self.depth_cloud_pub.publish(cloud)

    def publish_lidar_cloud(self, stamp):
        """LiDAR 点云（官方: /scan, Livox Mid-360, 360°x57°, 10Hz）。

        当前发布空点云占位，后续由 Gazebo LiDAR 插件填充真实数据。
        """
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = 'laser_livox'
        cloud.height = 1
        cloud.width = 0
        cloud.is_dense = False
        self.scan_pub.publish(cloud)
        self.livox_pub.publish(cloud)


def main():
    rclpy.init()
    node = FakePlatformNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
