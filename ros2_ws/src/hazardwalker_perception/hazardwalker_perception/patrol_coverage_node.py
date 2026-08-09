"""发布官方人工巡检运动覆盖状态的 ROS2 节点。

节点只订阅合法 SLAM 里程计，周期发布累计路径与跨度 JSON，供实时预检和
rosbag 完成门禁使用；它不参与定位、不发布速度，也不读取仿真真值。
"""

import json

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .patrol_coverage import PatrolCoverageTracker


class PatrolCoverageNode(Node):
    """把合法里程计流压缩为低带宽覆盖心跳。"""

    def __init__(self):
        super().__init__('hazardwalker_patrol_coverage')
        self.declare_parameter(
            'odometry_topic', '/hazardwalker/slam/odometry')
        self.declare_parameter(
            'coverage_topic', '/hw/perception/patrol_coverage')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('max_planar_step_m', 1.5)
        self.declare_parameter('max_vertical_step_m', 3.0)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('publish_rate_hz 必须为正数')
        self.tracker = self._new_tracker()
        self.publisher = self.create_publisher(
            String, str(self.get_parameter('coverage_topic').value), 10)
        self.subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter('odometry_topic').value),
            self.on_odometry,
            20,
        )
        self.reset_service = self.create_service(
            Trigger, '/hw/perception/patrol_coverage/reset', self.on_reset)
        self.timer = self.create_timer(1.0 / rate, self.publish_status)

    def _new_tracker(self):
        return PatrolCoverageTracker(
            max_planar_step_m=float(
                self.get_parameter('max_planar_step_m').value),
            max_vertical_step_m=float(
                self.get_parameter('max_vertical_step_m').value),
        )

    def on_reset(self, _request, response):
        """正式录包开始前清零，避免把预检阶段运动计入本轮覆盖。"""

        self.tracker = self._new_tracker()
        self.publish_status()
        response.success = True
        response.message = '巡检覆盖计数已清零'
        return response

    def on_odometry(self, message):
        pose = message.pose.pose.position
        stamp = message.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        self.tracker.update(pose.x, pose.y, pose.z, stamp_sec)

    def publish_status(self):
        message = String()
        message.data = json.dumps(
            self.tracker.snapshot(), ensure_ascii=False, sort_keys=True)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolCoverageNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
