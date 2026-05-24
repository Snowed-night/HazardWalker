"""HazardWalker 最小平台适配节点。

所属组：平台组。
文件作用：
- 在没有 Gazebo 或官方平台时，模拟 `/hw/*` 话题输出和简单控制响应。
- 让感知、导航、决策、结果写入可以先串成一条完整链路。

当前职责：
- 发布相机图像、CameraInfo、里程计、TF 和空点云。
- 接收 `/hw/cmd_vel` 并用简化运动学更新位置。
- 为感知组提供可控红球图像，为导航组提供可控里程计输入。

后续扩展方式：
- 有真实仿真或官方平台后，这个文件可以替换成 `gazebo_adapter_node.py` 或 `official_adapter_node.py`。
- 替换时保持 `/hw/camera/image_raw`、`/hw/odom`、`/hw/cmd_vel` 等接口不变，只改数据来源。
- 如果需要接入真实传感器，可在这里补真实点云、真实外参和更准确的 TF。

验证方式：
- 启动后确认 `odom -> base_link`、`base_link -> camera_link`、`base_link -> lidar_link` 存在。
- 确认图像里能切出红球，`/hw/odom` 会随 `/hw/cmd_vel` 改变。
"""
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import TransformBroadcaster


class FakePlatformNode(Node):
    def __init__(self):
        super().__init__('fake_platform_node')
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('red_ball_visible', True)

        # 图像尺寸和发布频率都做成 ROS 参数，后续可以在 launch/yaml 里覆盖。
        self.width = int(self.get_parameter('image_width').value)
        self.height = int(self.get_parameter('image_height').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        # 这些 topic 是项目内部接口。算法模块只依赖这些 `/hw/*` topic，
        # 不直接依赖 Gazebo、Isaac 或官方平台原始 topic 名。
        self.image_pub = self.create_publisher(Image, '/hw/camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/hw/camera/camera_info', 10)
        self.odom_pub = self.create_publisher(Odometry, '/hw/odom', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/hw/lidar/points', 10)
        self.cmd_sub = self.create_subscription(Twist, '/hw/cmd_vel', self.on_cmd_vel, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 用一个极简的 2D 位姿表示机器人状态：x/y 是平面位置，yaw 是朝向。
        # 当前模型只根据 /hw/cmd_vel 做积分，不考虑碰撞、打滑、动力学约束。
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.cmd = Twist()
        self.last_time = time.monotonic()
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info('Fake platform publishing /hw sensor and odom topics.')

    def on_cmd_vel(self, msg: Twist):
        # 导航节点发布的速度命令会被保存下来，在定时器中用于更新简化位姿。
        self.cmd = msg

    def on_timer(self):
        # 所有传感器和里程计都在同一个 timer 中发布，方便保证时间戳一致。
        now_monotonic = time.monotonic()
        dt = max(0.0, now_monotonic - self.last_time)
        self.last_time = now_monotonic

        # 极简差速/全向抽象：linear.x 表示前进速度，angular.z 表示角速度。
        # 这只用于测试接口，不代表真实四足机器人的运动控制。
        vx = float(self.cmd.linear.x)
        wz = float(self.cmd.angular.z)
        self.x += math.cos(self.yaw) * vx * dt
        self.y += math.sin(self.yaw) * vx * dt
        self.yaw += wz * dt

        stamp = self.get_clock().now().to_msg()
        # 按固定顺序发布 TF、里程计、相机和空点云。后续真实平台接入时，
        # 这些函数应被官方/仿真数据源替换，但输出 topic 和 frame 尽量不变。
        self.publish_tf(stamp)
        self.publish_odom(stamp, vx, wz)
        self.publish_camera(stamp)
        self.publish_empty_cloud(stamp)

    def publish_tf(self, stamp):
        # odom -> base_link：描述机器人底盘在里程计坐标系下的位置。
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(self.yaw / 2.0)
        transform.transform.rotation.w = math.cos(self.yaw / 2.0)

        # base_link -> camera_link：相机相对机器人底盘的安装位置。
        # 这里的数值是占位外参，后续 Gazebo/官方平台必须替换为真实外参。
        camera_tf = TransformStamped()
        camera_tf.header.stamp = stamp
        camera_tf.header.frame_id = 'base_link'
        camera_tf.child_frame_id = 'camera_link'
        camera_tf.transform.translation.x = 0.25
        camera_tf.transform.translation.y = 0.0
        camera_tf.transform.translation.z = 0.35
        camera_tf.transform.rotation.w = 1.0

        # base_link -> lidar_link：雷达相对机器人底盘的安装位置。
        # 当前 fake 节点发布空点云，因此只用于保证 TF 链完整。
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
        # /hw/odom 给导航和决策模块使用。真实系统中这个数据可能来自官方平台、
        # SLAM、轮速计/IMU 融合或 robot_localization。
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
        # 生成一张 RGB 图像：灰色背景 + 可选红色圆形区域。
        # 这样感知组即使没有相机和仿真器，也可以测试 HSV 检测链路。
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = 'camera_link'
        image.height = self.height
        image.width = self.width
        image.encoding = 'rgb8'
        image.is_bigendian = 0
        image.step = self.width * 3

        data = bytearray(self.width * self.height * 3)
        # 背景填成暗灰色，避免整个图像都是黑色导致调试不直观。
        for i in range(0, len(data), 3):
            data[i] = 30
            data[i + 1] = 30
            data[i + 2] = 30

        if bool(self.get_parameter('red_ball_visible').value):
            # 在图像中偏右位置画一个红色圆，模拟相机看到红色球体危险源。
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

        # CameraInfo 提供相机内参。当前数值是占位值，点云投影/三维定位前
        # 必须由平台组替换为仿真相机或官方相机的真实参数。
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
        # 当前只发布空 PointCloud2，用来提前固定 `/hw/lidar/points` 接口。
        # 后续 Gazebo/官方平台接入后，应发布真实点云。
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
