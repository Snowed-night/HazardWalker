"""自主探索 ROS 节点：Frontier 驱动的楼层覆盖、避障、重观察与返航。

所属组：导航组。
- 订阅 SLAM 地图和感知检测结果。
- 使用 tf2 获取机器人位姿（map 帧），不依赖 /hw/Odometry_gazebo。
- 前沿检测 → A* 路径规划 → cmd_vel 控制。
- 接收感知重观察请求，执行靠近、横移、侧视复查。
- 探索完成后返航到起点，发布 FINISHED。

状态机: INIT → EXPLORING → REOBSERVING → RETURNING → FINISHED
"""
from __future__ import annotations

import json
import math
import time
from collections import deque
from typing import List, Optional, Tuple

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String

from hazardwalker_nav.frontier_detector import (
    a_star_path,
    cluster_frontiers,
    find_frontiers,
    occupancy_grid_to_array,
    select_best_frontier,
)
from hazardwalker_nav.waypoint_controller import normalize_angle


class FrontierExplorerNode(Node):
    """Frontier 探索节点——自主覆盖楼层、复查候选、返航。"""

    def __init__(self):
        super().__init__('frontier_explorer_node')

        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('min_frontier_size', 10)
        self.declare_parameter('goal_tolerance_m', 0.8)
        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('heading_tolerance_rad', 0.25)
        self.declare_parameter('reobserve_duration_s', 4.0)
        self.declare_parameter('stuck_timeout_s', 15.0)
        self.declare_parameter('exploration_timeout_s', 540.0)
        self.declare_parameter('replan_interval_s', 3.0)

        self.state = 'INIT'
        self.start_time = self.get_clock().now()
        self._state_entry_time = time.monotonic()

        self.latest_map: Optional[OccupancyGrid] = None
        self.grid = None

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_target = None
        self.last_target_world: Optional[Tuple[float, float]] = None
        self.current_path: List[Tuple[float, float]] = []
        self.path_index: int = 0
        self._last_replan_time = 0.0
        self._visited_frontiers: set = set()

        self.reobserve_action: Optional[str] = None
        self.reobserve_end_time: float = 0.0

        self._pose_history: deque = deque(maxlen=30)
        self._last_cmd_time: float = 0.0

        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.on_map, 10)
        self.hazard_sub = self.create_subscription(
            String, '/hw/perception/hazard_detections', self.on_hazard, 10)
        self.cmd_pub = self.create_publisher(Twist, '/hw/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/hw/nav/state', 10)
        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info(
            f'Frontier explorer ready. Home=({self.start_x:.1f}, {self.start_y:.1f})')

    def on_map(self, msg: OccupancyGrid):
        self.latest_map = msg
        try:
            self.grid = occupancy_grid_to_array(msg)
        except Exception:
            self.grid = None

    def on_hazard(self, msg: String):
        if self.state in ('RETURNING', 'FINISHED'):
            return
        try:
            payload = json.loads(msg.data)
            detections = payload.get('hazards', [])
        except json.JSONDecodeError:
            return
        for d in detections:
            if (d.get('requires_reobservation', False) or
                    d.get('is_partial', False) or
                    d.get('confidence', 1.0) < 0.6):
                self._trigger_reobservation(d)
                return

    def _trigger_reobservation(self, detection: dict):
        bbox = detection.get('bbox', {})
        confidence = detection.get('confidence', 0.5)
        try:
            from hazardwalker_perception.active_view_policy import (
                choose_active_view_action)
            det_dict = {
                'id': detection.get('id', '0'), 'bbox': bbox,
                'confidence': confidence,
                'requires_reobservation': detection.get('requires_reobservation', False),
                'red_pixel_count': detection.get('red_pixel_count', 100),
                'circularity': detection.get('circularity', 0.8),
                'depth_m': detection.get('depth_m'),
                'depth_shape': detection.get('depth_shape', {}),
            }
            rec = choose_active_view_action([det_dict], 640, 480)
            action = rec.action
            self.get_logger().info(f'Reobservation: {action} ({rec.reason})')
        except ImportError:
            action = 'move_forward'
            self.get_logger().info('Reobservation: move_forward (fallback)')
        if action in ('turn_left', 'turn_right', 'move_laterally',
                      'move_forward', 'hold_observation'):
            self.reobserve_action = action
            self.state = 'REOBSERVING'
            self.reobserve_end_time = time.monotonic() + float(
                self.get_parameter('reobserve_duration_s').value)
            self._state_entry_time = time.monotonic()
            self.get_logger().info(f'Entering REOBSERVING: {action}')

    def on_timer(self):
        self._update_pose()
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        cmd = Twist()
        if self.state == 'INIT':
            cmd = self._handle_init()
        elif self.state == 'EXPLORING':
            cmd = self._handle_exploring()
        elif self.state == 'REOBSERVING':
            cmd = self._handle_reobserving()
        elif self.state == 'RETURNING':
            cmd = self._handle_returning()
        self._update_stuck_detection(cmd)
        self.cmd_pub.publish(cmd)

    def _handle_init(self) -> Twist:
        cmd = Twist()
        elapsed = time.monotonic() - self._state_entry_time
        if self.latest_map is not None and self.grid is not None:
            free_cells = (self.grid == 0).sum()
            if free_cells > 100 or elapsed > 10.0:
                self.start_x = self.robot_x
                self.start_y = self.robot_y
                self.get_logger().info(
                    f'Map ready ({free_cells} free cells). '
                    f'Start=({self.start_x:.2f}, {self.start_y:.2f})')
                self._transition('EXPLORING')
                return cmd
        cmd.angular.z = 0.5
        return cmd

    def _handle_exploring(self) -> Twist:
        cmd = Twist()
        mission_elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        timeout = float(self.get_parameter('exploration_timeout_s').value)
        if mission_elapsed > timeout:
            self.get_logger().warn('Exploration timeout, returning home.')
            self._transition('RETURNING')
            return cmd
        if self.grid is None or self.latest_map is None:
            return cmd
        now = time.monotonic()
        replan_interval = float(self.get_parameter('replan_interval_s').value)
        if (self.current_target is None or len(self.current_path) == 0 or
                now - self._last_replan_time > replan_interval):
            self._replan()
            self._last_replan_time = now
        if self.current_target is None and len(self.current_path) == 0:
            self.get_logger().info('No frontiers remaining. Returning home.')
            self._transition('RETURNING')
            return cmd
        return self._follow_path()

    def _handle_reobserving(self) -> Twist:
        cmd = Twist()
        now = time.monotonic()
        if now >= self.reobserve_end_time:
            self.get_logger().info('Reobservation complete, resuming exploration.')
            self.reobserve_action = None
            self.current_target = None
            self.current_path = []
            self._transition('EXPLORING')
            return cmd
        action = self.reobserve_action or 'hold_observation'
        dur = max(self.reobserve_end_time - now, 0.1)
        half = float(self.get_parameter('reobserve_duration_s').value) / 2.0
        if action == 'move_forward':
            cmd.linear.x = 0.2
        elif action == 'turn_left':
            cmd.angular.z = 0.6
        elif action == 'turn_right':
            cmd.angular.z = -0.6
        elif action == 'move_laterally':
            phase = self.reobserve_end_time - now
            cmd.linear.y = 0.15 if phase > half else -0.15
        return cmd

    def _handle_returning(self) -> Twist:
        cmd = Twist()
        goal_tol = float(self.get_parameter('goal_tolerance_m').value)
        dist_home = math.hypot(self.robot_x - self.start_x,
                               self.robot_y - self.start_y)
        if dist_home <= goal_tol:
            self.get_logger().info(f'Arrived home. Distance={dist_home:.2f}m')
            self._transition('FINISHED')
            return cmd
        if self.grid is None:
            target_yaw = math.atan2(self.start_y - self.robot_y,
                                    self.start_x - self.robot_x)
            heading_error = normalize_angle(target_yaw - self.robot_yaw)
            angular_speed = float(self.get_parameter('angular_speed').value)
            heading_tol = float(self.get_parameter('heading_tolerance_rad').value)
            cmd.angular.z = max(-angular_speed, min(angular_speed, heading_error))
            if abs(heading_error) <= heading_tol:
                cmd.linear.x = min(float(self.get_parameter('linear_speed').value), dist_home)
            return cmd
        if len(self.current_path) == 0:
            self.current_path = a_star_path(
                self.grid, self.latest_map,
                self.robot_x, self.robot_y, self.start_x, self.start_y)
            self.path_index = 0
            if len(self.current_path) == 0:
                self.get_logger().warn('No path home found, blind return.')
                target_yaw = math.atan2(self.start_y - self.robot_y,
                                        self.start_x - self.robot_x)
                cmd.angular.z = max(-0.8, min(0.8, normalize_angle(target_yaw - self.robot_yaw)))
                return cmd
        return self._follow_path()

    def _update_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5))
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            q = transform.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

    def _replan(self):
        if self.grid is None:
            return
        frontier_mask = find_frontiers(self.grid)
        if frontier_mask.sum() == 0:
            self.current_target = None
            self.current_path = []
            return
        frontiers = cluster_frontiers(frontier_mask, self.grid, self.latest_map)
        min_size = int(self.get_parameter('min_frontier_size').value)
        new_frontiers = []
        for f in frontiers:
            key = (round(f.centroid[0], 1), round(f.centroid[1], 1))
            if key not in self._visited_frontiers and f.size >= min_size:
                new_frontiers.append(f)
        if not new_frontiers:
            self.current_target = None
            self.current_path = []
            self.get_logger().info('All frontiers visited.')
            return
        best = select_best_frontier(
            new_frontiers, self.robot_x, self.robot_y,
            last_target=self.last_target_world, min_frontier_size=min_size)
        if best is None:
            self.current_target = None
            self.current_path = []
            return
        self.current_target = best
        self.last_target_world = best.centroid
        key = (round(best.centroid[0], 1), round(best.centroid[1], 1))
        self._visited_frontiers.add(key)
        self.current_path = a_star_path(
            self.grid, self.latest_map,
            self.robot_x, self.robot_y, best.centroid[0], best.centroid[1])
        self.path_index = 0
        self.get_logger().info(
            f'New frontier: ({best.centroid[0]:.2f}, {best.centroid[1]:.2f}), '
            f'size={best.size}, path={len(self.current_path)} steps')

    def _follow_path(self) -> Twist:
        cmd = Twist()
        goal_tol = float(self.get_parameter('goal_tolerance_m').value)
        linear_speed = float(self.get_parameter('linear_speed').value)
        angular_speed = float(self.get_parameter('angular_speed').value)
        heading_tol = float(self.get_parameter('heading_tolerance_rad').value)
        if len(self.current_path) == 0:
            return cmd
        while self.path_index < len(self.current_path):
            wx, wy = self.current_path[self.path_index]
            if math.hypot(wx - self.robot_x, wy - self.robot_y) <= goal_tol:
                self.path_index += 1
            else:
                break
        if self.path_index >= len(self.current_path):
            self.current_path = []
            self.current_target = None
            return cmd
        goal_x, goal_y = self.current_path[self.path_index]
        target_yaw = math.atan2(goal_y - self.robot_y, goal_x - self.robot_x)
        heading_error = normalize_angle(target_yaw - self.robot_yaw)
        cmd.angular.z = max(-angular_speed, min(angular_speed, heading_error))
        if abs(heading_error) <= heading_tol:
            dist = math.hypot(goal_x - self.robot_x, goal_y - self.robot_y)
            cmd.linear.x = min(linear_speed, dist)
        return cmd

    def _update_stuck_detection(self, cmd: Twist):
        if self.state in ('INIT', 'FINISHED', 'REOBSERVING'):
            return
        self._pose_history.append((self.robot_x, self.robot_y))
        if cmd.linear.x != 0.0:
            self._last_cmd_time = time.monotonic()
        if len(self._pose_history) < self._pose_history.maxlen:
            return
        first_x, first_y = self._pose_history[0]
        last_x, last_y = self._pose_history[-1]
        moved = math.hypot(last_x - first_x, last_y - first_y)
        stuck_timeout = float(self.get_parameter('stuck_timeout_s').value)
        if (moved < 0.1 and
                time.monotonic() - self._last_cmd_time > stuck_timeout and
                self.state == 'EXPLORING'):
            self.get_logger().warn(f'Stuck detected (moved {moved:.3f}m). Recovery.')
            self.current_path = []
            self.current_target = None
            self._pose_history.clear()

    def _transition(self, new_state: str):
        self.get_logger().info(f'State: {self.state} → {new_state}')
        self.state = new_state
        self._state_entry_time = time.monotonic()


def main():
    rclpy.init()
    node = FrontierExplorerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
