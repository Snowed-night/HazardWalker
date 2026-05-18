import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class WaypointPatrolNode(Node):
    """Minimal waypoint follower for the first integration loop."""

    def __init__(self):
        super().__init__('waypoint_patrol_node')
        self.declare_parameter('goal_tolerance_m', 0.5)
        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('heading_tolerance_rad', 0.25)
        self.declare_parameter('waypoints', [1.0, 0.0, 2.0, 0.0, 2.0, 1.0, 0.0, 0.0])

        raw_waypoints = list(self.get_parameter('waypoints').value)
        self.waypoints = []
        for i in range(0, len(raw_waypoints), 2):
            if i + 1 < len(raw_waypoints):
                self.waypoints.append((float(raw_waypoints[i]), float(raw_waypoints[i + 1])))

        self.current_pose = None
        self.goal_index = 0
        self.completed = False
        self.cmd_pub = self.create_publisher(Twist, '/hw/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/hw/nav/state', 10)
        self.odom_sub = self.create_subscription(Odometry, '/hw/odom', self.on_odom, 10)
        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info(f'Waypoint patrol loaded {len(self.waypoints)} goals.')

    def on_odom(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def on_timer(self):
        state = String()
        cmd = Twist()

        if self.current_pose is None:
            state.data = 'IDLE'
            self.state_pub.publish(state)
            return

        if self.completed or not self.waypoints:
            state.data = 'FINISHED'
            self.state_pub.publish(state)
            self.cmd_pub.publish(cmd)
            return

        goal_x, goal_y = self.waypoints[self.goal_index]
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        dx = goal_x - x
        dy = goal_y - y
        distance = math.hypot(dx, dy)
        tolerance = float(self.get_parameter('goal_tolerance_m').value)

        if distance <= tolerance:
            self.goal_index += 1
            if self.goal_index >= len(self.waypoints):
                self.completed = True
                state.data = 'FINISHED'
                self.state_pub.publish(state)
                self.cmd_pub.publish(cmd)
                self.get_logger().info('Waypoint patrol finished.')
                return
            self.get_logger().info(f'Moving to waypoint {self.goal_index + 1}/{len(self.waypoints)}.')

        state.data = 'NAVIGATING' if self.goal_index < len(self.waypoints) - 1 else 'RETURNING'
        yaw = self.get_yaw()
        target_yaw = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_yaw - yaw)
        heading_tolerance = float(self.get_parameter('heading_tolerance_rad').value)
        angular_speed = float(self.get_parameter('angular_speed').value)

        if abs(heading_error) > heading_tolerance:
            cmd.angular.z = max(-angular_speed, min(angular_speed, heading_error))
        else:
            cmd.linear.x = min(float(self.get_parameter('linear_speed').value), distance)
            cmd.angular.z = max(-angular_speed, min(angular_speed, heading_error))
        self.state_pub.publish(state)
        self.cmd_pub.publish(cmd)

    def get_yaw(self):
        q = self.current_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main():
    rclpy.init()
    node = WaypointPatrolNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
