"""自主探索 ROS 节点：Frontier 驱动的楼层覆盖、避障、重观察与返航。

所属组：导航组。
功能：
- 订阅 SLAM 地图 (OccupancyGrid) 和感知检测结果。
- 使用 tf2 获取机器人位姿（map 帧），不依赖 /hw/Odometry_gazebo。
- 前沿检测 → A* 路径规划 → cmd_vel 控制。
- 接收感知重观察请求，执行靠近、横移、侧视复查。
- 探索完成后返航，到达起点时发布 FINISHED。

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
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from hazardwalker_nav.frontier_detector import (
    Frontier,
    a_star_path,
    cluster_frontiers,
    compute_frontier_backoff_ttl_s,
    compute_exploration_time_limit_s,
    find_frontiers,
    grid_to_world,
    nearest_frontier_basin_key,
    occupancy_grid_to_array,
    return_pose_has_progress,
    select_best_frontier,
    should_switch_frontier,
    world_to_grid,
    OCCUPIED,
    FREE_MAX,
)
from hazardwalker_nav.reobservation_contract import (
    action_has_scan_clearance,
    bearing_change_deg,
    find_target_detection,
    find_target_status,
    parse_reobservation_request,
    reobservation_request_is_eligible,
)
from hazardwalker_nav.waypoint_controller import normalize_angle


class FrontierExplorerNode(Node):
    """Frontier 探索节点——自主覆盖楼层、复查候选、返航。"""

    def __init__(self):
        super().__init__('frontier_explorer_node')

        # ---- 参数 ----
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('min_frontier_size', 10)
        # 优先覆盖最近前沿附近的房间入口，避免远端长走廊/自由射线大簇凭
        # 线性信息增益耗尽任务预算；0 表示只比较等距最近候选。
        self.declare_parameter('frontier_locality_slack_m', 3.0)
        self.declare_parameter('frontier_switch_margin_m', 1.0)
        self.declare_parameter('frontier_minimum_hold_s', 8.0)
        self.declare_parameter('frontier_net_progress_timeout_s', 30.0)
        self.declare_parameter('frontier_net_progress_distance_m', 0.25)
        # 非官方环境默认从第一条合法 TF 推断。官方 profile 会显式传入公开
        # 起点在 map 帧中的朝向，避免 INIT 建图旋转污染入楼方向。
        self.declare_parameter('entry_heading_yaw', float('nan'))
        self.declare_parameter('entry_forward_half_angle_deg', 35.0)
        # 0 表示通用环境不限制；官方 profile 按公开 20 m 楼宽上限加安全裕量。
        self.declare_parameter('entry_lateral_limit_m', 0.0)
        # 官方场景前沿通常距离较近；过大的容差会把首个目标直接误判为“已到达”。
        self.declare_parameter('goal_tolerance_m', 0.25)
        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('minimum_linear_speed', 0.30)
        # 官方 A1 RL 控制器对小角速度响应明显偏弱；1.5 rad/s 指令在固定
        # SEED 实测能产生可控转向，控制器仍会在底层限幅。
        self.declare_parameter('angular_speed', 1.5)
        self.declare_parameter('minimum_turn_speed', 0.45)
        self.declare_parameter('heading_tolerance_rad', 0.25)
        self.declare_parameter('reobserve_motion_duration_s', 2.0)
        # 横移必须真正形成至少约 25° 视线变化；固定 SEED 中目标约 3 m 远，
        # 2 秒动作不足以产生有效基线。达到方位目标后会由感知反馈提前停车。
        self.declare_parameter('reobserve_lateral_motion_duration_s', 10.0)
        self.declare_parameter('reobserve_settle_duration_s', 1.0)
        self.declare_parameter('reobserve_observe_duration_s', 1.5)
        self.declare_parameter('reobserve_lateral_speed', 0.15)
        self.declare_parameter('reobserve_forward_speed', 0.18)
        self.declare_parameter('reobserve_turn_speed', 0.60)
        self.declare_parameter('reobserve_max_attempts_per_target', 4)
        self.declare_parameter('stuck_timeout_s', 15.0)
        # 正式任务总预算 600 秒，默认最多探索 480 秒，至少留 120 秒返航。
        # 实际返航预留还会根据距家距离和保守速度动态增加。
        self.declare_parameter('exploration_timeout_s', 480.0)
        self.declare_parameter('mission_time_budget_s', 600.0)
        self.declare_parameter('minimum_return_reserve_s', 120.0)
        self.declare_parameter('return_time_safety_factor', 2.0)
        self.declare_parameter('return_fixed_overhead_s', 30.0)
        self.declare_parameter('replan_interval_s', 3.0)
        self.declare_parameter('return_progress_timeout_s', 12.0)
        self.declare_parameter('return_progress_distance_m', 0.10)
        self.declare_parameter('return_net_progress_timeout_s', 45.0)
        self.declare_parameter('return_net_progress_distance_m', 0.25)
        self.declare_parameter('pose_fresh_timeout_s', 1.0)
        self.declare_parameter('scan_fresh_timeout_s', 1.0)
        self.declare_parameter('navigation_min_clearance_m', 0.45)
        # 安全门禁把期望运动归零时，普通卡死检测看不到“已请求但被拦截”的
        # 动作。超过该仿真时间后退避当前前沿，避免在门口永久静止。
        self.declare_parameter('safety_blocked_timeout_s', 8.0)
        self.declare_parameter('reobserve_min_clearance_m', 0.60)
        # 官方 A1 激光存在约 0.34 m 的固定近场机身回波；略低于该值，既保留
        # 原地转向能力，又不放宽前进/横移净空。
        self.declare_parameter('rotation_min_clearance_m', 0.30)
        self.declare_parameter('frontier_recovery_turn_speed', 0.60)
        self.declare_parameter('unreachable_frontier_ttl_s', 45.0)
        self.declare_parameter('unreachable_frontier_max_ttl_s', 180.0)
        self.declare_parameter('unreachable_frontier_radius_m', 0.45)
        # 必须长于基础 unreachable TTL + 一次重规划周期，否则目标刚过期前
        # 就会误判完成，永远没有机会用扩展后的地图重试。
        self.declare_parameter('frontier_completion_grace_s', 60.0)

        # ---- 状态机 ----
        self.state = 'INIT'
        self.prev_state = ''
        self.start_time = self.get_clock().now()
        self._mission_start_ros_sec: Optional[float] = None
        self._state_entry_time = time.monotonic()

        # ---- 地图 ----
        self.latest_map: Optional[OccupancyGrid] = None
        self.grid: Optional['np.ndarray'] = None

        # ---- 位姿 (通过 tf2 获取) ----
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        # 第一条合法 TF 代表官方公开起点朝向。INIT 原地旋转只用于积累扫描，
        # 不能把旋转后的随机朝向误当作“进入建筑”的方向。
        self._initial_heading_yaw: Optional[float] = None
        self._last_pose_stamp: Optional[Tuple[int, int]] = None
        self._last_pose_monotonic: Optional[float] = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- 局部安全扫描 ----
        self.latest_scan: Optional[LaserScan] = None
        self._last_scan_stamp: Optional[Tuple[int, int]] = None
        self._last_scan_monotonic: Optional[float] = None

        # ---- 探索 ----
        self.frontiers: list = []
        self.current_target: Optional[Frontier] = None
        self.last_target_world: Optional[Tuple[float, float]] = None
        self.current_path: List[Tuple[float, float]] = []
        self.path_index: int = 0
        self._current_target_selected_ros_sec: Optional[float] = None
        self._frontier_last_net_progress_ros: Optional[float] = None
        self._frontier_progress_reference_distance: Optional[float] = None
        self._last_replan_time = 0.0
        self._last_return_plan_time: Optional[float] = None
        self._visited_frontiers: set = set()  # 真正到达的前沿质心
        self._entry_origin: Optional[Tuple[float, float]] = None
        self._entry_axis: Optional[Tuple[float, float]] = None
        # 暂时不可达不能永久拉黑：按空间盆地合并相邻质心，使用仿真时间
        # 指数退避；value=(expiry_ros_sec, failure_count)。
        self._unreachable_frontiers: dict = {}
        self._no_reachable_frontier_since: Optional[float] = None

        # ---- 重观察 ----
        self.reobserve_action: Optional[str] = None
        self.reobserve_target_id: str = ''
        self.reobserve_motion_end_time: float = 0.0
        self.reobserve_end_time: float = 0.0
        self.reobserve_settle_duration_s: float = 0.0
        self.reobserve_observe_duration_s: float = 0.0
        self.reobserve_baseline_bearing_deg: Optional[float] = None
        self.reobserve_required_bearing_change_deg: float = 25.0
        self._reobserve_bearing_goal_met = False
        self._reobserve_attempts: dict = {}

        # ---- 卡死检测 ----
        self._pose_history: deque = deque(maxlen=30)  # 3秒位置与朝向历史 (10Hz)
        self._stuck_since: Optional[float] = None
        self._safety_blocked_since_ros: Optional[float] = None

        # ---- 返航 ----
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self._return_best_distance_home: Optional[float] = None
        self._return_last_progress_time: Optional[float] = None
        self._return_last_progress_pose: Optional[Tuple[float, float]] = None
        self._return_last_net_progress_time: Optional[float] = None
        self._return_net_progress_reference_distance: Optional[float] = None

        # ---- ROS 接口 ----
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.on_map, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/hw/scan', self.on_scan, 10)
        self.hazard_sub = self.create_subscription(
            String, '/hw/perception/hazard_detections', self.on_hazard, 10)
        self.cmd_pub = self.create_publisher(Twist, '/hw/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/hw/nav/state', 10)

        # 控制心跳必须使用 steady clock。仿真低于实时速率时，仿真时钟 10 Hz
        # 可能对应超过 0.5 s 墙钟间隔，与平台零速看门狗冲突并造成走走停停。
        self._control_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            0.1, self.on_timer, clock=self._control_clock)

        self.get_logger().info(
            f'Frontier explorer ready. Home=({self.start_x:.1f}, {self.start_y:.1f})')

    # ---- 回调 ----

    def on_map(self, msg: OccupancyGrid):
        self.latest_map = msg
        try:
            self.grid = occupancy_grid_to_array(msg)
        except Exception:
            self.grid = None

    def on_hazard(self, msg: String):
        """解析感知检测结果，判断是否需要进入重观察状态。"""
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if self.state == 'REOBSERVING':
            self._update_reobservation_feedback(payload)
            return
        request = parse_reobservation_request(payload)
        if not reobservation_request_is_eligible(
                request,
                self.state,
                self._reobserve_attempts,
                self.get_parameter('reobserve_max_attempts_per_target').value):
            return
        self._trigger_reobservation(request)

    def on_scan(self, msg: LaserScan):
        """保存最近一帧公开激光，用作运动前的局部安全门禁。"""

        self.latest_scan = msg
        stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        # 仅收到时间戳真正推进的扫描才刷新新鲜度；重复转发一帧冻结扫描时
        # 必须在超时后停止，而不能因为回调仍触发就继续运动。
        if stamp != (0, 0) and stamp != self._last_scan_stamp:
            self._last_scan_stamp = stamp
            self._last_scan_monotonic = time.monotonic()

    def _trigger_reobservation(self, request: dict):
        """执行感知侧已经判定的明确复查动作。"""

        now = self._ros_time_sec()
        action = str(request['action'])
        duration_parameter = (
            'reobserve_lateral_motion_duration_s'
            if action in ('move_left', 'move_right')
            else 'reobserve_motion_duration_s'
        )
        motion_duration = max(
            0.0, float(self.get_parameter(duration_parameter).value),
        )
        settle_duration = max(
            0.0, float(self.get_parameter('reobserve_settle_duration_s').value),
        )
        observe_duration = max(
            0.1, float(self.get_parameter('reobserve_observe_duration_s').value),
        )
        self.reobserve_action = action
        self.reobserve_target_id = str(request['target_id'])
        self.reobserve_motion_end_time = now + motion_duration
        self.reobserve_end_time = (
            self.reobserve_motion_end_time + settle_duration + observe_duration
        )
        self.reobserve_settle_duration_s = settle_duration
        self.reobserve_observe_duration_s = observe_duration
        self.reobserve_baseline_bearing_deg = request.get('view_bearing_deg')
        self.reobserve_required_bearing_change_deg = max(
            1.0,
            float(request.get('required_bearing_change_deg', 25.0)),
        )
        self._reobserve_bearing_goal_met = False
        self._reobserve_attempts[self.reobserve_target_id] = (
            int(self._reobserve_attempts.get(self.reobserve_target_id, 0)) + 1
        )
        self._transition('REOBSERVING')
        self.get_logger().info(
            f'Entering REOBSERVING: target={self.reobserve_target_id} '
            f'action={self.reobserve_action} '
            f'attempt={self._reobserve_attempts[self.reobserve_target_id]} '
            f'reason={request.get("reason", "")}'
        )

    def _update_reobservation_feedback(self, payload: dict):
        """用实时感知反馈结束横移并保证停稳观察，而不是盲走固定时长。"""

        status = find_target_status(payload, self.reobserve_target_id)
        if status in ('confirmed', 'rejected', 'rejected_non_spherical'):
            now = self._ros_time_sec()
            self.reobserve_motion_end_time = now
            self.reobserve_end_time = now
            self.get_logger().info(
                f'Reobservation target {self.reobserve_target_id} resolved: {status}.'
            )
            return
        if self.reobserve_action not in ('move_left', 'move_right'):
            return
        detection = find_target_detection(payload, self.reobserve_target_id)
        if detection is None:
            return
        try:
            current_bearing_deg = float(detection.get('view_bearing_deg'))
        except (TypeError, ValueError):
            return
        if not math.isfinite(current_bearing_deg):
            return
        if self.reobserve_baseline_bearing_deg is None:
            self.reobserve_baseline_bearing_deg = current_bearing_deg
            return
        achieved = bearing_change_deg(
            self.reobserve_baseline_bearing_deg, current_bearing_deg,
        )
        if (achieved is None
                or achieved < self.reobserve_required_bearing_change_deg
                or self._reobserve_bearing_goal_met):
            return
        now = self._ros_time_sec()
        self.reobserve_motion_end_time = now
        self.reobserve_end_time = (
            now
            + self.reobserve_settle_duration_s
            + self.reobserve_observe_duration_s
        )
        self._reobserve_bearing_goal_met = True
        self.get_logger().info(
            'Reobservation bearing goal reached: '
            f'{achieved:.1f}° >= {self.reobserve_required_bearing_change_deg:.1f}°; '
            'stopping for stable RGB-D evidence.'
        )

    def _ros_time_sec(self) -> float:
        """复查动作按仿真时钟计时，避免低实时率时没有足够传感器帧。"""

        return self.get_clock().now().nanoseconds / 1e9

    def _exploration_time_limit_s(self) -> float:
        """按 600 秒总预算和当前返航距离计算动态探索截止时间。"""

        linear_speed = abs(float(self.get_parameter('linear_speed').value))
        minimum_speed = abs(
            float(self.get_parameter('minimum_linear_speed').value)
        )
        positive_speeds = [
            speed for speed in (linear_speed, minimum_speed) if speed > 0.0
        ]
        conservative_speed = (
            min(positive_speeds) if positive_speeds else 0.05
        )
        return compute_exploration_time_limit_s(
            configured_timeout_s=float(
                self.get_parameter('exploration_timeout_s').value),
            mission_budget_s=float(
                self.get_parameter('mission_time_budget_s').value),
            distance_home_m=math.hypot(
                self.robot_x - self.start_x,
                self.robot_y - self.start_y,
            ),
            return_speed_mps=conservative_speed,
            minimum_return_reserve_s=float(
                self.get_parameter('minimum_return_reserve_s').value),
            return_safety_factor=float(
                self.get_parameter('return_time_safety_factor').value),
            return_fixed_overhead_s=float(
                self.get_parameter('return_fixed_overhead_s').value),
        )

    def _return_deadline_reached(self) -> bool:
        """探索和复查都不得侵占动态返航预留。"""

        now_ros = self._ros_time_sec()
        if self._mission_start_ros_sec is None or now_ros <= 0.0:
            return False
        mission_elapsed = max(0.0, now_ros - self._mission_start_ros_sec)
        return mission_elapsed >= self._exploration_time_limit_s()

    # ---- 控制循环 ----

    def on_timer(self):
        """10Hz 主循环。"""
        now_ros = self._ros_time_sec()
        if self._mission_start_ros_sec is None and now_ros > 0.0:
            # 从官方 /clock 第一条有效消息开始计总预算，INIT 建图/开门耗时也
            # 必须计入 600 秒，不能到 EXPLORING 才重新起表。
            self._mission_start_ros_sec = now_ros
        self._update_pose()

        # 状态持久化发布
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        cmd = Twist()
        if not self._has_fresh_pose():
            # TF 缺失时默认的 (0,0,0) 不能当作真实位置，更不能据此记录 home 或发控制。
            self._update_stuck_detection(cmd)
            self.cmd_pub.publish(cmd)
            return

        if (self.state in ('EXPLORING', 'REOBSERVING')
                and self._return_deadline_reached()):
            self.get_logger().warn(
                'Exploration budget reached with protected return reserve; '
                'returning home.'
            )
            self._transition('RETURNING')

        if self.state == 'INIT':
            cmd = self._handle_init()
        elif self.state == 'EXPLORING':
            cmd = self._handle_exploring()
        elif self.state == 'REOBSERVING':
            cmd = self._handle_reobserving()
        elif self.state == 'RETURNING':
            cmd = self._handle_returning()
        elif self.state == 'FINISHED':
            cmd = Twist()  # 停止

        self._update_stuck_detection(cmd)
        self.cmd_pub.publish(cmd)

    # ---- 状态处理 ----

    def _handle_init(self) -> Twist:
        """INIT: 等待地图并原地慢转，让 SLAM 初始化。"""
        cmd = Twist()

        # 超时也进入探索
        elapsed = time.monotonic() - self._state_entry_time
        if self.latest_map is not None and self.grid is not None:
            # 检查地图是否已有足够数据
            free_cells = (
                (self.grid >= 0) & (self.grid <= FREE_MAX)
            ).sum()
            if free_cells > 100 or elapsed > 10.0:
                # 记录初始位姿作为家
                self.start_x = self.robot_x
                self.start_y = self.robot_y
                self.get_logger().info(
                    f'Map ready ({free_cells} free cells). '
                    f'Starting exploration from ({self.start_x:.2f}, {self.start_y:.2f})')
                self._transition('EXPLORING')
                return cmd

        # 原地慢转，积累初始扫描。没有新鲜且满足转向净空的激光时保持停车。
        if self._scan_allows_action(
                'turn_left',
                float(self.get_parameter('rotation_min_clearance_m').value)):
            cmd.angular.z = 0.5
        return cmd

    def _handle_exploring(self) -> Twist:
        """EXPLORING: 前沿检测 → 路径规划 → 速度控制。"""
        cmd = Twist()

        if self.grid is None or self.latest_map is None:
            return cmd

        # 定期重规划
        now = time.monotonic()
        now_ros = self._ros_time_sec()
        replan_interval = float(self.get_parameter('replan_interval_s').value)

        # 无目标时也遵守重规划间隔；否则 steady-clock 10 Hz 控制会每帧重复
        # 聚类整张地图并刷屏“All frontiers visited”，挤占传感器与控制回调。
        if now - self._last_replan_time > replan_interval:
            self._replan()
            self._last_replan_time = now

        # 无前沿 → 探索完成，返航
        if self.current_target is None and len(self.current_path) == 0:
            if self._entry_axis is None:
                # INIT 为建图做过原地旋转，深度相机可能停在入口侧面，严格
                # 入楼锥自然还没有前沿。先主动回正到公开入口朝向并持续采集，
                # 不能用墙钟宽限把“尚未看向入口”误判为探索完成。
                self._no_reachable_frontier_since = None
                entry_error = normalize_angle(
                    self._entry_heading() - self.robot_yaw
                )
                if abs(entry_error) > float(
                        self.get_parameter('heading_tolerance_rad').value):
                    action = 'turn_left' if entry_error > 0.0 else 'turn_right'
                    if self._scan_allows_action(
                            action,
                            float(self.get_parameter(
                                'rotation_min_clearance_m').value)):
                        cmd.angular.z = math.copysign(
                            min(
                                float(self.get_parameter(
                                    'frontier_recovery_turn_speed').value),
                                float(self.get_parameter(
                                    'angular_speed').value),
                            ),
                            entry_error,
                        )
                return cmd
            if any(
                    record[0] > now_ros
                    for record in self._unreachable_frontiers.values()):
                # 仍有失败盆地处于仿真时间退避期时不能用固定 completion
                # grace 提前宣称“探索完成”；保持扫描，等待盆地到期后换图重试。
                # 总任务返航截止由 on_timer 的硬门禁独立保证。
                self._no_reachable_frontier_since = None
                if self._scan_allows_action(
                        'turn_left',
                        float(self.get_parameter(
                            'rotation_min_clearance_m').value)):
                    cmd.angular.z = min(
                        float(self.get_parameter(
                            'frontier_recovery_turn_speed').value),
                        float(self.get_parameter('angular_speed').value),
                    )
                return cmd
            if self._no_reachable_frontier_since is None:
                self._no_reachable_frontier_since = now_ros
            elif now_ros < self._no_reachable_frontier_since:
                self._no_reachable_frontier_since = now_ros
            grace = float(
                self.get_parameter('frontier_completion_grace_s').value)
            if (now_ros - self._no_reachable_frontier_since
                    < max(0.0, grace)):
                # 地图刚扩展时前沿与机器人栅格可能短暂不可规划；先原地收集
                # 更多扫描，不能一次规划失败就把整栋楼误判为探索完成。
                if self._scan_allows_action(
                        'turn_left',
                        float(self.get_parameter(
                            'rotation_min_clearance_m').value)):
                    cmd.angular.z = min(
                        float(self.get_parameter(
                            'frontier_recovery_turn_speed').value),
                        float(self.get_parameter('angular_speed').value),
                    )
                return cmd
            self.get_logger().info('No frontiers remaining. Returning home.')
            self._transition('RETURNING')
            return cmd
        self._no_reachable_frontier_since = None

        if self._frontier_net_progress_expired(now_ros):
            # 机器人可能持续转动，从而绕过普通“位姿不变”卡死检测；但只要
            # 到目标的净距离长期不下降，就不能继续耗尽整个探索预算。
            self.get_logger().warn(
                'Frontier net-progress watchdog expired; suppressing '
                'the current target and replanning.'
            )
            if self.current_target is not None:
                self._mark_frontier_unreachable(self.current_target)
            self.current_target = None
            self.current_path = []
            self.path_index = 0
            self._current_target_selected_ros_sec = None
            self._reset_frontier_progress_watchdog()
            return cmd

        # 沿路径前进
        cmd = self._follow_path()
        return cmd

    def _handle_reobserving(self) -> Twist:
        """REOBSERVING: 执行感知请求的重观察机动。"""
        cmd = Twist()
        now = self._ros_time_sec()

        if now >= self.reobserve_end_time:
            self.get_logger().info('Reobservation complete, resuming exploration.')
            self.reobserve_action = None
            self.reobserve_target_id = ''
            self.reobserve_baseline_bearing_deg = None
            self._reobserve_bearing_goal_met = False
            # 强制重规划
            self.current_target = None
            self.current_path = []
            self._transition('EXPLORING')
            return cmd

        # 机动结束后必须停车等待机体稳定并采集确认帧；若持续运动到状态结束，
        # 感知的 camera_stable 门禁永远不会累计独立视角证据。
        if now >= self.reobserve_motion_end_time:
            return cmd

        # 根据感知建议生成短时机动 cmd_vel。
        action = self.reobserve_action or 'hold_observation'
        clearance_parameter = (
            'rotation_min_clearance_m'
            if action in ('turn_left', 'turn_right')
            else 'reobserve_min_clearance_m'
        )
        if not self._scan_allows_action(
                action,
                float(self.get_parameter(clearance_parameter).value)):
            return cmd

        if action == 'move_forward':
            cmd.linear.x = float(self.get_parameter('reobserve_forward_speed').value)
        elif action == 'turn_left':
            cmd.angular.z = float(self.get_parameter('reobserve_turn_speed').value)
        elif action == 'turn_right':
            cmd.angular.z = -float(self.get_parameter('reobserve_turn_speed').value)
        elif action == 'move_left':
            cmd.linear.y = float(self.get_parameter('reobserve_lateral_speed').value)
        elif action == 'move_right':
            cmd.linear.y = -float(self.get_parameter('reobserve_lateral_speed').value)
        elif action == 'hold_observation':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        return cmd

    def _handle_returning(self) -> Twist:
        """RETURNING: A* 返航到起点。"""
        cmd = Twist()
        goal_tol = float(self.get_parameter('goal_tolerance_m').value)
        now = self._ros_time_sec()

        dist_home = math.hypot(self.robot_x - self.start_x,
                               self.robot_y - self.start_y)

        if dist_home <= goal_tol:
            self.get_logger().info(
                f'Arrived home. Distance={dist_home:.2f}m')
            self._transition('FINISHED')
            return cmd

        force_replan = self._return_progress_watchdog_expired(
            now, dist_home,
        )
        if self.grid is None:
            # 无地图时直线盲返在复杂楼宇中不可接受；等待地图恢复。
            return cmd

        # 返航路径按 ROS 仿真时间周期刷新；地图继续变化时旧路径不能永久沿用。
        replan_interval = max(
            0.1, float(self.get_parameter('replan_interval_s').value),
        )
        should_replan = (
            force_replan
            or self._last_return_plan_time is None
            or now < self._last_return_plan_time
            or now - self._last_return_plan_time >= replan_interval
        )
        if should_replan:
            self._last_return_plan_time = now
            self.current_path = a_star_path(
                self.grid, self.latest_map,
                self.robot_x, self.robot_y,
                self.start_x, self.start_y,
                start_search_radius_m=0.50,
                # 返航终点必须是包含真实 home 的原始自由栅格，不能先吸附
                # 0.25 m 再叠加 0.25 m 路径容差，造成距 home 0.5 m 假完成。
                goal_search_radius_m=0.0,
                append_exact_goal=True,
            )
            self.path_index = 0
            if len(self.current_path) == 0:
                self.get_logger().warn(
                    'No safe path home found; stopping and waiting for a map update.')
                return cmd
        elif len(self.current_path) == 0:
            return cmd

        cmd = self._follow_path()
        return cmd

    def _return_progress_watchdog_expired(
            self, now_ros: float, dist_home: float) -> bool:
        """返航无位置进展时清路径；不依赖被 scan 门禁归零后的 cmd_vel。"""

        progress_distance = max(
            0.01,
            float(self.get_parameter('return_progress_distance_m').value),
        )
        timeout = max(
            0.1,
            float(self.get_parameter('return_progress_timeout_s').value),
        )
        net_progress_distance = max(
            progress_distance,
            float(self.get_parameter(
                'return_net_progress_distance_m').value),
        )
        net_timeout = max(
            timeout,
            float(self.get_parameter('return_net_progress_timeout_s').value),
        )
        if (self._return_last_progress_time is None
                or self._return_last_progress_pose is None
                or self._return_best_distance_home is None
                or self._return_last_net_progress_time is None
                or self._return_net_progress_reference_distance is None
                or now_ros < self._return_last_progress_time):
            self._return_best_distance_home = dist_home
            self._return_last_progress_time = now_ros
            self._return_last_progress_pose = (self.robot_x, self.robot_y)
            self._return_last_net_progress_time = now_ros
            self._return_net_progress_reference_distance = dist_home
            return False
        self._return_best_distance_home = min(
            self._return_best_distance_home, dist_home,
        )
        if (dist_home <= self._return_net_progress_reference_distance
                - net_progress_distance):
            self._return_net_progress_reference_distance = dist_home
            self._return_last_net_progress_time = now_ros
        if return_pose_has_progress(
                self._return_last_progress_pose[0],
                self._return_last_progress_pose[1],
                self.robot_x,
                self.robot_y,
                progress_distance):
            self._return_last_progress_time = now_ros
            self._return_last_progress_pose = (self.robot_x, self.robot_y)
        stationary_expired = (
            now_ros - self._return_last_progress_time >= timeout
        )
        net_progress_expired = (
            now_ros - self._return_last_net_progress_time >= net_timeout
        )
        if not stationary_expired and not net_progress_expired:
            return False

        self.get_logger().warn(
            'Return progress watchdog expired '
            f'(stationary={stationary_expired}, '
            f'net_progress={net_progress_expired}); '
            'clearing stale path and replanning.'
        )
        self.current_path = []
        self.path_index = 0
        self._return_last_progress_time = now_ros
        self._return_last_progress_pose = (self.robot_x, self.robot_y)
        self._return_last_net_progress_time = now_ros
        self._return_net_progress_reference_distance = dist_home
        return True

    # ---- 辅助方法 ----

    def _update_pose(self):
        """通过 tf2 获取合法 SLAM 的 map → base 变换。"""
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter('map_frame').value),
                str(self.get_parameter('base_frame').value),
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5),
            )
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            q = transform.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            if self._initial_heading_yaw is None:
                self._initial_heading_yaw = self.robot_yaw
            stamp = (
                int(transform.header.stamp.sec),
                int(transform.header.stamp.nanosec),
            )
            # lookup_transform(Time()) 会反复返回同一条冻结 TF。只有合法动态
            # 时间戳推进时才刷新新鲜度，从而让控制在 TF 停止后自动失效。
            if stamp != (0, 0) and stamp != self._last_pose_stamp:
                self._last_pose_stamp = stamp
                self._last_pose_monotonic = time.monotonic()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f'tf lookup failed: {e}')

    def _has_fresh_pose(self) -> bool:
        """只有新鲜合法 TF 才允许状态推进和控制。"""

        if self._last_pose_monotonic is None:
            return False
        timeout = float(self.get_parameter('pose_fresh_timeout_s').value)
        return time.monotonic() - self._last_pose_monotonic <= max(0.1, timeout)

    def _entry_heading(self) -> float:
        """返回合法入楼朝向：优先官方 profile 参数，其次第一条动态 TF。"""

        configured = float(self.get_parameter('entry_heading_yaw').value)
        if math.isfinite(configured):
            return configured
        if self._initial_heading_yaw is not None:
            return self._initial_heading_yaw
        return self.robot_yaw

    def _scan_allows_action(self, action: str, clearance_m: float) -> bool:
        """检查扫描新鲜度与动作对应扇区的净空。"""

        if action == 'hold_observation':
            return True
        if self.latest_scan is None or self._last_scan_monotonic is None:
            return False
        timeout = float(self.get_parameter('scan_fresh_timeout_s').value)
        if time.monotonic() - self._last_scan_monotonic > max(0.1, timeout):
            return False
        return action_has_scan_clearance(
            action,
            self.latest_scan.ranges,
            self.latest_scan.angle_min,
            self.latest_scan.angle_increment,
            clearance_m,
        )

    def _replan(self):
        """重新规划；周期刷新同一目标，只有到达/不可达后才切换。"""
        if self.grid is None:
            return
        now = time.monotonic()
        now_ros = self._ros_time_sec()
        stale_horizon = max(
            0.1,
            float(self.get_parameter(
                'unreachable_frontier_max_ttl_s').value),
        )
        self._unreachable_frontiers = {
            key: record
            for key, record in self._unreachable_frontiers.items()
            if record[0] >= now_ros - stale_horizon
        }

        frontier_mask = find_frontiers(self.grid)
        if frontier_mask.sum() == 0:
            self.current_target = None
            self.current_path = []
            return

        self.frontiers = cluster_frontiers(frontier_mask, self.grid, self.latest_map)

        # 过滤已访问的前沿
        min_size = int(self.get_parameter('min_frontier_size').value)
        unvisited_frontiers = []
        for f in self.frontiers:
            key = self._frontier_key(f)
            if (key not in self._visited_frontiers
                    and not self._frontier_is_unreachable(f, now_ros)):
                unvisited_frontiers.append(f)

        if not unvisited_frontiers:
            # 全部访问过 → 探索完成
            self.current_target = None
            self.current_path = []
            self.get_logger().info('All frontiers visited.')
            return

        entry_heading = self._entry_heading()

        def select_candidate(candidate_pool):
            return select_best_frontier(
                candidate_pool, self.robot_x, self.robot_y,
                last_target=self.last_target_world,
                min_frontier_size=min_size,
                locality_slack_m=float(
                    self.get_parameter('frontier_locality_slack_m').value),
                # 首次用当前朝向选中入楼前沿；随后用首段路径固定“楼内半平面”，
                # 允许左右房间参与评分，同时拒绝入口背后的巨大楼外前沿。
                robot_yaw=entry_heading if self._entry_axis is None else None,
                robot_yaw_half_angle_rad=math.radians(max(
                    5.0,
                    min(
                        90.0,
                        float(self.get_parameter(
                            'entry_forward_half_angle_deg').value),
                    ),
                )),
                require_robot_yaw_candidate=self._entry_axis is None,
                entry_origin=self._entry_origin,
                entry_axis=self._entry_axis,
                entry_lateral_limit_m=float(
                    self.get_parameter('entry_lateral_limit_m').value),
            )

        if self.current_target is not None:
            refreshed_path = a_star_path(
                self.grid, self.latest_map,
                self.robot_x, self.robot_y,
                self.current_target.centroid[0],
                self.current_target.centroid[1],
            )
            if not refreshed_path:
                self._mark_frontier_unreachable(self.current_target)
                self.current_target = None
                self.current_path = []
                self._current_target_selected_ros_sec = None
                self._reset_frontier_progress_watchdog()
            else:
                challenger = select_candidate(unvisited_frontiers)
                current_distance = math.hypot(
                    self.current_target.centroid[0] - self.robot_x,
                    self.current_target.centroid[1] - self.robot_y,
                )
                challenger_distance = (
                    float('inf') if challenger is None
                    else math.hypot(
                        challenger.centroid[0] - self.robot_x,
                        challenger.centroid[1] - self.robot_y,
                    )
                )
                held_duration = (
                    0.0 if self._current_target_selected_ros_sec is None
                    else max(
                        0.0,
                        now_ros - self._current_target_selected_ros_sec,
                    )
                )
                if not should_switch_frontier(
                        current_distance,
                        challenger_distance,
                        held_duration,
                        float(self.get_parameter(
                            'frontier_switch_margin_m').value),
                        float(self.get_parameter(
                            'frontier_minimum_hold_s').value)):
                    self.current_path = refreshed_path
                    self.path_index = 0
                    return
                self.get_logger().info(
                    'Switching frontier to nearer coverage target: '
                    f'{current_distance:.2f}m -> {challenger_distance:.2f}m.'
                )
                # 旧目标既未失败也未到达，不能错误加入 visited/unreachable。
                self.current_target = None
                self.current_path = []
                self._current_target_selected_ros_sec = None

        # 评分最高的前沿不一定能在“只走已知自由区”的安全地图上到达；
        # 逐个尝试，规划失败的目标本轮不再反复选择。
        candidates = list(unvisited_frontiers)
        while candidates:
            best = select_candidate(candidates)
            if best is None:
                break
            path = a_star_path(
                self.grid, self.latest_map,
                self.robot_x, self.robot_y,
                best.centroid[0], best.centroid[1],
            )
            if path:
                if self._entry_axis is None:
                    self._entry_origin = (self.start_x, self.start_y)
                    # 有官方公开朝向时固定使用该轴，而不是首个前沿质心的偏角；
                    # 这样入口较宽时也不会把侧向大厅误当成整栋楼纵深方向。
                    self._entry_axis = (
                        math.cos(entry_heading),
                        math.sin(entry_heading),
                    )
                self.current_target = best
                self._current_target_selected_ros_sec = now_ros
                self._frontier_last_net_progress_ros = now_ros
                self._frontier_progress_reference_distance = math.hypot(
                    best.centroid[0] - self.robot_x,
                    best.centroid[1] - self.robot_y,
                )
                self.last_target_world = best.centroid
                self.current_path = path
                self.path_index = 0
                self.get_logger().info(
                    f'New frontier: ({best.centroid[0]:.2f}, '
                    f'{best.centroid[1]:.2f}), size={best.size}, '
                    f'path={len(path)} steps'
                )
                return
            self._mark_frontier_unreachable(best)
            candidates.remove(best)

        self.current_target = None
        self.current_path = []
        self.get_logger().warn('No safely reachable frontier in the current map.')

    def _frontier_net_progress_expired(self, now_ros: float) -> bool:
        """目标净距离长期不下降时返回 True，持续原地转向也不能刷新门禁。"""

        if self.current_target is None:
            self._reset_frontier_progress_watchdog()
            return False
        distance = math.hypot(
            self.current_target.centroid[0] - self.robot_x,
            self.current_target.centroid[1] - self.robot_y,
        )
        threshold = max(
            0.01,
            float(self.get_parameter(
                'frontier_net_progress_distance_m').value),
        )
        if (self._frontier_last_net_progress_ros is None
                or self._frontier_progress_reference_distance is None
                or now_ros < self._frontier_last_net_progress_ros):
            self._frontier_last_net_progress_ros = now_ros
            self._frontier_progress_reference_distance = distance
            return False
        if distance <= self._frontier_progress_reference_distance - threshold:
            self._frontier_last_net_progress_ros = now_ros
            self._frontier_progress_reference_distance = distance
            return False
        timeout = max(
            0.1,
            float(self.get_parameter(
                'frontier_net_progress_timeout_s').value),
        )
        return now_ros - self._frontier_last_net_progress_ros >= timeout

    def _reset_frontier_progress_watchdog(self):
        self._frontier_last_net_progress_ros = None
        self._frontier_progress_reference_distance = None

    @staticmethod
    def _frontier_key(frontier: Frontier):
        return (round(frontier.centroid[0], 1), round(frontier.centroid[1], 1))

    def _mark_frontier_unreachable(self, frontier: Frontier):
        """按空间盆地抑制失败目标，随连续失败指数退避。"""

        now_ros = self._ros_time_sec()
        radius = float(
            self.get_parameter('unreachable_frontier_radius_m').value,
        )
        basin_key = nearest_frontier_basin_key(
            self._unreachable_frontiers.keys(),
            frontier.centroid[0],
            frontier.centroid[1],
            radius,
        )
        if basin_key is None:
            basin_key = self._frontier_key(frontier)
            previous_failures = 0
        else:
            previous_failures = int(
                self._unreachable_frontiers[basin_key][1]
            )
        failure_count = previous_failures + 1
        ttl = compute_frontier_backoff_ttl_s(
            float(self.get_parameter('unreachable_frontier_ttl_s').value),
            float(self.get_parameter(
                'unreachable_frontier_max_ttl_s').value),
            failure_count,
        )
        self._unreachable_frontiers[basin_key] = (
            now_ros + ttl,
            failure_count,
        )
        self.get_logger().warn(
            'Suppressing unreachable frontier basin '
            f'({basin_key[0]:.2f}, {basin_key[1]:.2f}) '
            f'within {max(0.0, radius):.2f}m for {ttl:.1f} sim seconds '
            f'(failure #{failure_count}).'
        )

    def _frontier_is_unreachable(
            self, frontier: Frontier, now_ros: Optional[float] = None) -> bool:
        """判断候选是否落在仍处于退避期的失败空间盆地中。"""

        if now_ros is None:
            now_ros = self._ros_time_sec()
        active_keys = [
            key for key, record in self._unreachable_frontiers.items()
            if record[0] > now_ros
        ]
        return nearest_frontier_basin_key(
            active_keys,
            frontier.centroid[0],
            frontier.centroid[1],
            float(self.get_parameter(
                'unreachable_frontier_radius_m').value),
        ) is not None

    def _clear_unreachable_frontier_basin(self, frontier: Frontier):
        """真正到达前沿后清除其邻域失败记录。"""

        basin_key = nearest_frontier_basin_key(
            self._unreachable_frontiers.keys(),
            frontier.centroid[0],
            frontier.centroid[1],
            float(self.get_parameter(
                'unreachable_frontier_radius_m').value),
        )
        if basin_key is not None:
            self._unreachable_frontiers.pop(basin_key, None)

    def _follow_path(self) -> Twist:
        """沿当前路径前进，返回 cmd_vel。"""
        cmd = Twist()
        goal_tol = float(self.get_parameter('goal_tolerance_m').value)
        linear_speed = float(self.get_parameter('linear_speed').value)
        minimum_linear_speed = min(
            abs(linear_speed),
            float(self.get_parameter('minimum_linear_speed').value),
        )
        angular_speed = float(self.get_parameter('angular_speed').value)
        minimum_turn_speed = min(
            abs(angular_speed),
            float(self.get_parameter('minimum_turn_speed').value),
        )
        heading_tol = float(self.get_parameter('heading_tolerance_rad').value)

        if len(self.current_path) == 0:
            return cmd

        # 跳过已到达的路径点
        while self.path_index < len(self.current_path):
            wx, wy = self.current_path[self.path_index]
            dist = math.hypot(wx - self.robot_x, wy - self.robot_y)
            if dist <= goal_tol:
                self.path_index += 1
            else:
                break

        if self.path_index >= len(self.current_path):
            # 路径走完
            if self.current_target is not None:
                key = self._frontier_key(self.current_target)
                self._visited_frontiers.add(key)
                self._clear_unreachable_frontier_basin(
                    self.current_target,
                )
                self.get_logger().info(
                    f'Frontier reached and marked visited: {key}'
                )
            self.current_path = []
            self.current_target = None
            self._reset_frontier_progress_watchdog()
            return cmd

        goal_x, goal_y = self.current_path[self.path_index]

        # 若距离太远，跳到最后一个可达点
        if self.path_index < len(self.current_path) - 1:
            last_x, last_y = self.current_path[-1]
        else:
            last_x, last_y = goal_x, goal_y

        # 朝向目标
        target_yaw = math.atan2(goal_y - self.robot_y, goal_x - self.robot_x)
        heading_error = normalize_angle(target_yaw - self.robot_yaw)
        cmd.angular.z = max(-angular_speed, min(angular_speed, heading_error))
        if 0.0 < abs(cmd.angular.z) < minimum_turn_speed:
            cmd.angular.z = math.copysign(
                minimum_turn_speed, cmd.angular.z,
            )

        if abs(heading_error) <= heading_tol:
            dist = math.hypot(goal_x - self.robot_x, goal_y - self.robot_y)
            cmd.linear.x = min(linear_speed, dist)
            if 0.0 < cmd.linear.x < minimum_linear_speed:
                cmd.linear.x = minimum_linear_speed
        else:
            cmd.linear.x = 0.0

        requested_linear = cmd.linear.x
        requested_angular = cmd.angular.z
        navigation_clearance = float(
            self.get_parameter('navigation_min_clearance_m').value)
        if cmd.linear.x > 0.0 and not self._scan_allows_action(
                'move_forward', navigation_clearance):
            cmd.linear.x = 0.0
        if cmd.angular.z != 0.0:
            turn_action = 'turn_left' if cmd.angular.z > 0.0 else 'turn_right'
            rotation_clearance = float(
                self.get_parameter('rotation_min_clearance_m').value)
            if not self._scan_allows_action(turn_action, rotation_clearance):
                cmd.angular.z = 0.0

        motion_requested = (
            abs(requested_linear) > 0.02
            or abs(requested_angular) > 0.05
        )
        motion_blocked = (
            motion_requested
            and abs(cmd.linear.x) <= 0.02
            and abs(cmd.angular.z) <= 0.05
        )
        if motion_blocked and self.state == 'EXPLORING':
            now_ros = self._ros_time_sec()
            if (self._safety_blocked_since_ros is None
                    or now_ros < self._safety_blocked_since_ros):
                self._safety_blocked_since_ros = now_ros
            elif now_ros - self._safety_blocked_since_ros >= max(
                    0.1,
                    float(self.get_parameter(
                        'safety_blocked_timeout_s').value)):
                self.get_logger().warn(
                    'Safety gate blocked all requested motion; '
                    'suppressing the current frontier instead of waiting '
                    'indefinitely.'
                )
                if self.current_target is not None:
                    self._mark_frontier_unreachable(self.current_target)
                self.current_target = None
                self.current_path = []
                self.path_index = 0
                self._current_target_selected_ros_sec = None
                self._reset_frontier_progress_watchdog()
                self._safety_blocked_since_ros = None
        else:
            self._safety_blocked_since_ros = None

        return cmd

    def _update_stuck_detection(self, cmd: Twist):
        """卡死检测：记录位姿历史，发现卡死时触发恢复。"""
        if self.state in ('INIT', 'FINISHED', 'REOBSERVING'):
            self._stuck_since = None
            self._safety_blocked_since_ros = None
            self._pose_history.clear()
            return

        linear_requested = abs(cmd.linear.x) > 0.02
        angular_requested = abs(cmd.angular.z) > 0.05
        if not linear_requested and not angular_requested:
            self._stuck_since = None
            self._pose_history.clear()
            return
        self._pose_history.append((self.robot_x, self.robot_y, self.robot_yaw))

        if len(self._pose_history) < self._pose_history.maxlen:
            return

        # 检查运动是否停滞
        first_x, first_y, first_yaw = self._pose_history[0]
        last_x, last_y, last_yaw = self._pose_history[-1]
        moved = math.hypot(last_x - first_x, last_y - first_y)
        rotated = abs(normalize_angle(last_yaw - first_yaw))
        made_progress = (
            (linear_requested and moved >= 0.08)
            or (angular_requested and rotated >= 0.10)
        )
        if made_progress:
            self._stuck_since = None
            return

        now = time.monotonic()
        if self._stuck_since is None:
            self._stuck_since = now
            return
        stuck_timeout = float(self.get_parameter('stuck_timeout_s').value)
        if (now - self._stuck_since > stuck_timeout
                and self.state == 'EXPLORING'):
            self.get_logger().warn(
                f'Stuck detected (moved {moved:.3f}m, rotated '
                f'{rotated:.3f}rad). Recovery: clearing path.')
            # 先短时标记失败目标再清理，避免下一帧立即选回同一前沿。
            if self.current_target is not None:
                self._mark_frontier_unreachable(self.current_target)
            self.current_path = []
            self.current_target = None
            self._current_target_selected_ros_sec = None
            self._reset_frontier_progress_watchdog()
            self._pose_history.clear()
            self._stuck_since = None

    def _transition(self, new_state: str):
        """状态转移并记录日志。"""
        self.prev_state = self.state
        self.state = new_state
        self._state_entry_time = time.monotonic()
        if new_state == 'EXPLORING' and self.prev_state == 'INIT':
            # 保留探索阶段起点供诊断；正式总预算已从 /clock 首个有效值计时，
            # 不会在 INIT 完成后重新起表。
            self.start_time = self.get_clock().now()
        if new_state == 'RETURNING':
            self.current_target = None
            self.current_path = []
            self.path_index = 0
            self._current_target_selected_ros_sec = None
            self._reset_frontier_progress_watchdog()
            self._last_return_plan_time = None
            self._return_best_distance_home = math.hypot(
                self.robot_x - self.start_x,
                self.robot_y - self.start_y,
            )
            self._return_last_progress_time = self._ros_time_sec()
            self._return_last_progress_pose = (
                self.robot_x, self.robot_y,
            )
            self._return_last_net_progress_time = (
                self._return_last_progress_time
            )
            self._return_net_progress_reference_distance = (
                self._return_best_distance_home
            )
        self.get_logger().info(f'State: {self.prev_state} → {new_state}')


def main():
    rclpy.init()
    node = FrontierExplorerNode()
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
