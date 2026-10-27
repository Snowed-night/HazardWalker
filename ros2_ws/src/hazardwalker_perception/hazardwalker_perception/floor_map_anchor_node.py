"""为多层任务发布每层独立的合法定位源→world 锚点。

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
        self.declare_parameter(
            'final_anchor_request_topic',
            '/hazardwalker/navigation/final_floor_anchor')
        self.declare_parameter(
            'floor_session_topic', '/hazardwalker/slam/floor_session')
        self.declare_parameter('imu_topic', '/hw/trunk_imu')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('initial_floor_index', 0)
        self.declare_parameter('official_home_x_m', 0.0)
        self.declare_parameter('official_home_y_m', -2.2)
        self.declare_parameter('official_elevator_cabin_x_m', 2.7)
        self.declare_parameter('official_elevator_y_m', 2.6)

        self.latest_world_yaw = None
        self.pending_anchors = []
        self.anchors = {}
        self.last_floor = None
        self.awaiting_arrival_floor = None
        self.floor_session_generation = {}
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
        self.create_subscription(
            Int32,
            str(self.get_parameter('final_anchor_request_topic').value),
            self.on_final_anchor_request,
            qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('floor_session_topic').value),
            self.on_floor_session,
            qos,
        )
        self.create_timer(0.1, self.try_publish_anchor)

    def _enqueue_anchor(self, floor, anchor_kind, generation=None):
        request = {
            'floor': int(floor),
            'anchor_kind': str(anchor_kind),
            'generation': generation,
        }
        key = (request['floor'], request['anchor_kind'], generation)
        if any(
                (item['floor'], item['anchor_kind'], item['generation']) == key
                for item in self.pending_anchors):
            return
        self.pending_anchors.append(request)

    def on_imu(self, message):
        q = message.orientation
        self.latest_world_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def on_floor_index(self, message):
        floor = int(message.data)
        if self.last_floor is None:
            self.last_floor = floor
            # 首次 floor_index 在运行器完成入门、释放导航时发布；此时机器人
            # 已不在出生点。首层暂用运行器在运动前锁定的全局变换，待离开
            # 本层、进入电梯轿厢时再用闭环后的 map 位姿结算。
            self.get_logger().info(
                'Initial floor keeps the pre-motion runner map anchor '
                'until the first elevator transition.')
            return
        if floor == self.last_floor:
            return
        previous_floor = self.last_floor
        self.last_floor = floor
        # 会话管理器会在短暂宽限后结束旧 Cartographer。先用电梯轿厢这个
        # 公开公共点冻结刚完成楼层；新楼层必须等全新会话 ready 后另行锚定。
        self._enqueue_anchor(
            previous_floor,
            'public_elevator_departure',
            self.floor_session_generation.get(previous_floor),
        )
        self.awaiting_arrival_floor = floor

    def on_floor_session(self, message):
        """新 Cartographer 会话产出首张地图后锚定当前到达楼层。"""

        try:
            payload = json.loads(message.data)
            floor = int(payload['floor_index'])
            generation = int(payload['generation'])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if (payload.get('schema') != 'hazardwalker_floor_slam_session_v1'
                or payload.get('state') != 'ready'):
            return
        self.floor_session_generation[floor] = generation
        if floor != self.awaiting_arrival_floor:
            return
        # 最终返航会为 0 层启动一个只供回程控制的新会话；早期红球属于最初
        # 的 0 层会话，已在首次离层时冻结，绝不能被返航会话覆盖。
        if floor in self.anchors:
            self.awaiting_arrival_floor = None
            return
        self._enqueue_anchor(
            floor, 'public_elevator_arrival', generation)
        self.awaiting_arrival_floor = None

    def on_final_anchor_request(self, message):
        """单层任务回到公开 home 后，补齐尚未由电梯闭环结算的楼层。"""

        floor = int(message.data)
        if floor in self.anchors:
            return
        self._enqueue_anchor(
            floor,
            'public_home',
            self.floor_session_generation.get(floor),
        )

    def try_publish_anchor(self):
        if not self.pending_anchors or self.latest_world_yaw is None:
            return
        request = self.pending_anchors[0]
        floor = int(request['floor'])
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
        anchor_kind = str(request['anchor_kind'])
        if anchor_kind == 'public_home':
            world_x = float(self.get_parameter('official_home_x_m').value)
            world_y = float(self.get_parameter('official_home_y_m').value)
        else:
            world_x = float(self.get_parameter(
                'official_elevator_cabin_x_m').value)
            world_y = float(self.get_parameter(
                'official_elevator_y_m').value)
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
            'applies_to_floors': [floor],
            'source_frame': str(self.get_parameter('map_frame').value),
            'world_from_map': [round(value, 9) for value in world_from_map],
            'source': f'lidar_imu_slam+{anchor_kind}',
            'session_generation': request['generation'],
        }
        self.anchors[floor] = payload
        self.pending_anchors.pop(0)
        self.anchor_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f'Floors {payload["applies_to_floors"]} map anchored '
            f'from {anchor_kind}: '
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
