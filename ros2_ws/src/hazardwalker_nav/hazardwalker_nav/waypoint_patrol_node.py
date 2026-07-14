"""固定航点巡检 ROS 节点。

所属组：导航组。
文件作用：
- 把 `/hw/odom` 转成 `/hw/cmd_vel` 和 `/hw/nav/state`。
- 作为离线 `waypoint_controller.py` 到 ROS 话题的桥接层。

当前职责：
- 读取航点参数并维护当前目标下标。
- 根据当前里程计调用控制函数，输出速度和状态。
- 在最小 demo 里模拟“巡检 -> 返航 -> 完成”的任务节奏。

后续扩展方式：
- 如果接 Nav2，可在这个节点里增加 action client，把这里的 `WaypointCommand` 映射成 Nav2 goal 发送逻辑。
- 如果要做 Frontier，可新增一个目标生成模块，仍复用当前控制输出和状态发布。
- 如果要支持重规划或避障，可在这里增加失败计数、目标重试和状态切换。

验证方式：
- 用 fake platform 的 `odom` 验证能推动机器人走航点。
- 用离线测试先验证控制律，再检查 ROS 话题输出。
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from hazardwalker_nav.waypoint_controller import compute_waypoint_command


class WaypointPatrolNode(Node):
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
        # 官方 ROS1 适配层将 /Odometry_gazebo 统一输出为 /hw/odom；不能泄漏官方原话题名，
        # 否则官方 profile 启动导航后永远收不到位姿，表面上却没有节点异常。
        self.odom_sub = self.create_subscription(Odometry, '/hw/odom', self.on_odom, 10)
        # 10Hz 控制循环。真实 Nav2 接入后，这里可以改成 action/result 驱动，而不是定时轮询。
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

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = self.get_yaw()
        result = compute_waypoint_command(
            x=x,
            y=y,
            yaw=yaw,
            waypoints=self.waypoints,
            goal_index=self.goal_index,
            completed=self.completed,
            goal_tolerance_m=float(self.get_parameter('goal_tolerance_m').value),
            linear_speed=float(self.get_parameter('linear_speed').value),
            angular_speed=float(self.get_parameter('angular_speed').value),
            heading_tolerance_rad=float(self.get_parameter('heading_tolerance_rad').value),
        )
        if result.goal_index != self.goal_index and not result.completed:
            self.get_logger().info(f'Moving to waypoint {result.goal_index + 1}/{len(self.waypoints)}.')
        if result.completed and not self.completed:
            self.get_logger().info('Waypoint patrol finished.')

        self.goal_index = result.goal_index
        self.completed = result.completed
        state.data = result.state
        cmd.linear.x = result.linear_x
        cmd.angular.z = result.angular_z
        self.state_pub.publish(state)
        self.cmd_pub.publish(cmd)

    def get_yaw(self):
        # Odometry 中姿态是四元数。这里把四元数转换为平面 yaw 角。
        # 当前 fake 平台只绕 z 轴旋转，因此这个公式足够使用。
        q = self.current_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main():
    rclpy.init()
    node = WaypointPatrolNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
