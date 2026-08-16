"""固定航点巡检 ROS 节点。

所属组：导航组。
文件作用：
- 把 `/hw/odom` 转成 `/hw/cmd_vel` 和 `/hw/nav/state`。
- 作为离线 `waypoint_controller.py` 到 ROS 话题的桥接层。

当前职责：
- 读取航点参数并维护当前目标下标。
- 根据当前里程计调用控制函数，输出速度和状态。
- 在最小 demo 里模拟「巡检 -> 返航 -> 完成」的任务节奏。

后续扩展方式：
- 如果接 Nav2，可在这个节点里增加 action client，把这里的 `WaypointCommand` 映射成 Nav2 goal 发送逻辑。
- 如果要做 Frontier，可新增一个目标生成模块，仍复用当前控制输出和状态发布。
- 如果要支持重规划或避障，可在这里增加失败计数、目标重试和状态切换。

验证方式：
- 用 fake platform 的 `odom` 验证能推动机器人走航点。
- 用离线测试先验证控制律，再检查 ROS 话题输出。
"""
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
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
        # 控制输出话题。默认 /hw/cmd_vel 保持 fake 平台（minimal_demo /
        # gazebo_minimal）诊断兼容；官方 SimEnv 有 hazardwalker_command_mux
        # 仲裁器，launch 会显式覆盖为 /hw/control/navigation_cmd_vel。
        self.declare_parameter('cmd_vel_topic', '/hw/cmd_vel')
        # command_mux 默认 default_mode=keyboard，需显式请求 navigation 才会
        # 转发导航命令。启动期按 1 Hz 重试 3 次覆盖 DDS 匹配窗口，之后依赖
        # 模式一次锁存不再发；fake 平台无此节点，发布 mode_request 无害。
        self.declare_parameter(
            'control_mode_request_topic', '/hw/control/mode_request')
        self.declare_parameter('control_mode_value', 'navigation')

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
        self._mode_request_sent_count = 0
        self._last_mode_request_time = 0.0

        # 输出速度命令给平台层。话题由 cmd_vel_topic 参数决定：fake 平台用
        # /hw/cmd_vel，官方 SimEnv 经 launch 覆盖为仲裁器话题。
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        # 固定航点只用于诊断，状态必须与正式 Frontier 任务隔离。若复用
        # /hw/nav/state，其 FINISHED 会让决策层误写 detected_danger.json，
        # 把四个固定航点冒充成未知楼宇自主探索完成。
        self.state_pub = self.create_publisher(
            String, '/hw/nav/diagnostic_state', 10,
        )
        self.mode_request_pub = self.create_publisher(
            String,
            str(self.get_parameter('control_mode_request_topic').value),
            10,
        )
        # 官方 ROS1 适配层将 /Odometry_gazebo 统一输出为 /hw/odom；不能泄漏官方原话题名，
        # 否则官方 profile 启动导航后永远收不到位姿，表面上却没有节点异常。
        self.odom_sub = self.create_subscription(Odometry, '/hw/odom', self.on_odom, 10)
        # 10Hz 控制循环。真实 Nav2 接入后，这里可以改成 action/result 驱动，而不是定时轮询。
        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info(f'Waypoint patrol loaded {len(self.waypoints)} goals.')

    def on_odom(self, msg: Odometry):
        # 保存最新机器人位姿。这里只使用 position 和 orientation，不处理协方差。
        self.current_pose = msg.pose.pose

    def _ensure_control_mode(self):
        """请求 command_mux 切到导航模式（启动期重试 3 次后停，依赖一次锁存）。"""
        if self._mode_request_sent_count >= 3:
            return
        now = time.monotonic()
        if now - self._last_mode_request_time < 1.0:
            return
        self._last_mode_request_time = now
        self._mode_request_sent_count += 1
        msg = String()
        msg.data = str(self.get_parameter('control_mode_value').value)
        self.mode_request_pub.publish(msg)

    def on_timer(self):
        # 每个控制周期都重新计算当前目标、状态和速度命令。
        state = String()
        cmd = Twist()
        self._ensure_control_mode()

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
    except ExternalShutdownException:
        # launch/stack 收到 SIGTERM 时上下文可能已被外部关闭；这属于正常收尾，不能报成导航崩溃。
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
