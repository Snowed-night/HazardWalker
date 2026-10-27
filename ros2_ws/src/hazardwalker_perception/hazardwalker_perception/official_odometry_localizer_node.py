"""把赛事公开 SimEnv 里程计归一化为统一 SLAM/感知坐标链。

所属组：感知定位组。负责人：姜晨。
节点只订阅平台转发的公开 ``/hw/odom`` 和公开电梯动作确认的楼层编号，发布
``odom -> base`` 与 ``/hazardwalker/slam/odometry``。起点首帧只用于消除官方
世界坐标平移和初始朝向；危险源真值、场景布局和裁判文件从不进入本节点。
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int32, String
from tf2_ros import TransformBroadcaster

from hazardwalker_perception.official_odometry import (
    PlanarOdomPose,
    relative_planar_pose,
)
from hazardwalker_perception.scan_imu_localization import (
    floor_index_to_elevation,
    quaternion_to_yaw,
)


class OfficialOdometryLocalizerNode(Node):
    """发布以机器人启动位姿为原点的官方公开里程计。"""

    def __init__(self):
        super().__init__('hazardwalker_official_odometry_localizer')
        self.declare_parameter('input_topic', '/hw/odom')
        self.declare_parameter('output_topic', '/hazardwalker/slam/odometry')
        self.declare_parameter(
            'floor_index_topic', '/hazardwalker/navigation/floor_index')
        self.declare_parameter(
            'localization_provenance', 'official_simenv_odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('floor_height_m', 2.6)
        self.declare_parameter('initial_floor_index', 0)
        self.declare_parameter('min_floor_index', 0)
        self.declare_parameter('max_floor_index', 31)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.floor_height_m = float(self.get_parameter('floor_height_m').value)
        self.min_floor_index = int(self.get_parameter('min_floor_index').value)
        self.max_floor_index = int(self.get_parameter('max_floor_index').value)
        self.floor_index = int(self.get_parameter('initial_floor_index').value)
        self.floor_elevation_m = floor_index_to_elevation(
            self.floor_index,
            floor_height_m=self.floor_height_m,
            min_floor_index=self.min_floor_index,
            max_floor_index=self.max_floor_index,
        )
        self.anchor = None
        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter('output_topic').value), 20)
        self.tf_broadcaster = (
            TransformBroadcaster(self)
            if bool(self.get_parameter('publish_tf').value)
            else None
        )
        self.localization_provenance = str(
            self.get_parameter('localization_provenance').value).strip()
        allowed = {
            'official_simenv_odometry',
            'official_simenv_odometry+public_floor_action',
        }
        if self.localization_provenance not in allowed:
            raise ValueError(
                '官方里程计定位来源声明不合法：'
                f'{self.localization_provenance!r}')
        provenance_qos = QoSProfile(depth=1)
        provenance_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.provenance_publisher = self.create_publisher(
            String, '/hazardwalker/slam/localization_provenance',
            provenance_qos)
        self.provenance_publisher.publish(
            String(data=self.localization_provenance))
        self.create_subscription(
            Odometry,
            str(self.get_parameter('input_topic').value),
            self.on_odometry,
            20,
        )
        self.create_subscription(
            Int32,
            str(self.get_parameter('floor_index_topic').value),
            self.on_floor_index,
            10,
        )
        self.get_logger().info(
            'Official competition odometry ready: %s -> %s -> %s'
            % (
                self.get_parameter('input_topic').value,
                self.odom_frame,
                self.base_frame,
            ))

    def on_floor_index(self, message):
        try:
            elevation = floor_index_to_elevation(
                message.data,
                floor_height_m=self.floor_height_m,
                min_floor_index=self.min_floor_index,
                max_floor_index=self.max_floor_index,
            )
        except ValueError as error:
            self.get_logger().error('拒绝非法楼层编号：%s' % error)
            return
        self.floor_index = int(message.data)
        self.floor_elevation_m = elevation

    def on_odometry(self, source):
        if source.header.frame_id not in ('', 'odom'):
            self.get_logger().error(
                '拒绝非 odom 官方里程计帧：%s' % source.header.frame_id,
                throttle_duration_sec=5.0)
            return
        orientation = source.pose.pose.orientation
        current = PlanarOdomPose(
            float(source.pose.pose.position.x),
            float(source.pose.pose.position.y),
            quaternion_to_yaw(
                orientation.x, orientation.y,
                orientation.z, orientation.w),
        )
        if self.anchor is None:
            self.anchor = current
        relative = relative_planar_pose(self.anchor, current)
        half_yaw = 0.5 * relative.yaw

        message = Odometry()
        message.header.stamp = source.header.stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = relative.x
        message.pose.pose.position.y = relative.y
        message.pose.pose.position.z = self.floor_elevation_m
        message.pose.pose.orientation.z = math.sin(half_yaw)
        message.pose.pose.orientation.w = math.cos(half_yaw)
        message.pose.covariance = source.pose.covariance
        message.twist = source.twist
        self.publisher.publish(message)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = relative.x
            transform.transform.translation.y = relative.y
            transform.transform.translation.z = self.floor_elevation_m
            transform.transform.rotation = message.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = OfficialOdometryLocalizerNode()
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
