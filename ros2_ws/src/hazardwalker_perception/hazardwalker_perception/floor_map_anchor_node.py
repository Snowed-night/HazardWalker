"""为多层任务发布每层独立的合法 SLAM map→world 锚点。

节点只读取 Cartographer TF、trunk IMU、公开楼层动作以及公开起点/电梯落点。
它不订阅 `/hw/odom`、`/Odometry_gazebo`、场景清单或危险源真值。
"""

import json
import math

import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, String

from hazardwalker_perception.floor_frame_alignment import (
    world_from_map_at_robot_anchor,
)
from hazardwalker_perception.scan_imu_localization import quaternion_to_yaw


class FloorMapAnchorNode(Node):
    """在每层首次到达的已知公共锚点冻结一份 world←map 变换。"""

    def __init__(self):
        super().__init__('hazardwalker_floor_map_anchor')
        self.declare_parameter(
            'floor_index_topic', '/hazardwalker/navigation/floor_index')
        self.declare_parameter(
            'anchor_topic', '/hazardwalker/slam/floor_anchors')
        self.declare_parameter('imu_topic', '/hw/trunk_imu')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('initial_floor_index', 0)
        self.declare_parameter('official_elevator_cabin_x_m', 2.7)
        self.declare_parameter('official_elevator_y_m', 2.6)

        self.latest_world_yaw = None
        self.pending_floor = None
        self.anchors = {}
        self.seen_floors = set()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=8)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.anchor_pub = self.create_publisher(
            String, str(self.get_parameter('anchor_topic').value), qos)
        self.create_subscription(
            Imu,
            str(self.get_parameter('imu_topic').value),
            self.on_imu,
            10,
        )
        self.create_subscription(
            Int32,
            str(self.get_parameter('floor_index_topic').value),
            self.on_floor_index,
            qos,
        )
        self.create_timer(0.1, self.try_publish_anchor)

    def on_imu(self, message):
        q = message.orientation
        self.latest_world_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def on_floor_index(self, message):
        floor = int(message.data)
        if floor in self.seen_floors:
            return
        self.seen_floors.add(floor)
        initial_floor = int(self.get_parameter('initial_floor_index').value)
        if floor == initial_floor:
            # 首次 floor_index 在运行器完成入门、释放导航时发布；此时机器人
            # 已不在出生点。首层必须使用运行器在运动前锁定的全局变换。
            self.get_logger().info(
                'Initial floor keeps the pre-motion runner map anchor.')
            return
        self.pending_floor = floor

    def try_publish_anchor(self):
        if self.pending_floor is None or self.latest_world_yaw is None:
            return
        floor = self.pending_floor
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter('map_frame').value),
                str(self.get_parameter('base_frame').value),
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        q = transform.transform.rotation
        map_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        world_x = float(self.get_parameter(
            'official_elevator_cabin_x_m').value)
        world_y = float(self.get_parameter(
            'official_elevator_y_m').value)
        anchor_kind = 'public_elevator_arrival'
        world_from_map = world_from_map_at_robot_anchor(
            transform.transform.translation.x,
            transform.transform.translation.y,
            map_yaw,
            world_x,
            world_y,
            self.latest_world_yaw,
        )
        payload = {
            'schema': 'hazardwalker_floor_map_anchor_v1',
            'floor': floor,
            'world_from_map': [round(value, 9) for value in world_from_map],
            'source': f'lidar_imu_slam+{anchor_kind}',
        }
        self.anchors[floor] = payload
        self.pending_floor = None
        self.anchor_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f'Floor {floor} map anchored from {anchor_kind}: '
            f'{payload["world_from_map"]}')


def main():
    rclpy.init()
    node = FloorMapAnchorNode()
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
