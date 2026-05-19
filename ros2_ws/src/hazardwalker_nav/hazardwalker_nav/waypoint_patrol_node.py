import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class WaypointPatrolNode(Node):
    """第一阶段固定航点巡检节点。

    这个节点不是最终导航方案，也不是 Nav2 的替代品。它的作用是：
    1. 在没有 SLAM / Nav2 / Gazebo 的早期阶段，先验证 `/hw/odom` -> `/hw/cmd_vel` 链路；
    2. 模拟“巡检若干航点后返航”的任务流程；
    3. 给决策节点发布 `/hw/nav/state`，让最小闭环能产生结果文件。
    """

    def __init__(self):
        super().__init__('waypoint_patrol_node')
        self.declare_parameter('goal_tolerance_m', 0.5)
        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('heading_tolerance_rad', 0.25)
        self.declare_parameter('waypoints', [1.0, 0.0, 2.0, 0.0, 2.0, 1.0, 0.0, 0.0])

        # waypoints 参数用一维数组表达，格式为 [x1, y1, x2, y2, ...]。
        # 最后一个 [0.0, 0.0] 代表回到起点，用来模拟 RETURNING。
        raw_waypoints = list(self.get_parameter('waypoints').value)
        self.waypoints = []
        for i in range(0, len(raw_waypoints), 2):
            if i + 1 < len(raw_waypoints):
                self.waypoints.append((float(raw_waypoints[i]), float(raw_waypoints[i + 1])))

        # current_pose 从 /hw/odom 更新；goal_index 表示当前正在追踪第几个航点。
        self.current_pose = None
        self.goal_index = 0
        self.completed = False

        # 输出速度命令给平台层。fake_platform_node 或后续 Gazebo/官方 adapter
        # 都应该接收这个统一的 /hw/cmd_vel。
        self.cmd_pub = self.create_publisher(Twist, '/hw/cmd_vel', 10)
        # 输出导航状态给决策层。当前只发布 IDLE/NAVIGATING/RETURNING/FINISHED。
        self.state_pub = self.create_publisher(String, '/hw/nav/state', 10)
        self.odom_sub = self.create_subscription(Odometry, '/hw/odom', self.on_odom, 10)
        # 10Hz 控制循环。真实 Nav2 接入后，这个节点会被 Nav2 goal client 替换。
        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info(f'Waypoint patrol loaded {len(self.waypoints)} goals.')

    def on_odom(self, msg: Odometry):
        # 保存最新机器人位姿。这里只使用 position 和 orientation，不处理协方差。
        self.current_pose = msg.pose.pose

    def on_timer(self):
        # 每个控制周期都重新计算当前目标、状态和速度命令。
        state = String()
        cmd = Twist()

        if self.current_pose is None:
            # 还没有收到里程计时不能控制机器人，只发布 IDLE。
            state.data = 'IDLE'
            self.state_pub.publish(state)
            return

        if self.completed or not self.waypoints:
            # 任务完成后持续发布 FINISHED，同时发布零速度，防止机器人继续运动。
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
            # 到达当前航点后切换到下一个航点。如果所有航点都完成，任务结束。
            self.goal_index += 1
            if self.goal_index >= len(self.waypoints):
                self.completed = True
                state.data = 'FINISHED'
                self.state_pub.publish(state)
                self.cmd_pub.publish(cmd)
                self.get_logger().info('Waypoint patrol finished.')
                return
            self.get_logger().info(f'Moving to waypoint {self.goal_index + 1}/{len(self.waypoints)}.')

        # 最后一个航点默认是起点，因此接近最后一个航点时发布 RETURNING。
        state.data = 'NAVIGATING' if self.goal_index < len(self.waypoints) - 1 else 'RETURNING'

        # 简单控制律：
        # 1. 先计算机器人当前朝向 yaw 和目标方向 target_yaw；
        # 2. 如果角度误差较大，原地转向；
        # 3. 如果朝向基本正确，再向前走。
        # 这只是为了 fake 平台测试，真实项目应由 Nav2 局部规划器接管。
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
        # Odometry 中姿态是四元数。这里把四元数转换为平面 yaw 角。
        # 当前 fake 平台只绕 z 轴旋转，因此这个公式足够使用。
        q = self.current_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        # 把角度规范到 [-pi, pi]，避免 179 度和 -179 度之间出现错误的大角度差。
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
