"""自主探索 ROS 节点：Frontier 驱动的楼层覆盖、避障、重观察与返航。

所属组：导航组。
功能：
- 订阅 SLAM 地图 (OccupancyGrid) 和感知检测结果。
- 使用 tf2 获取机器人位姿（map 帧），不依赖 /hw/Odometry_gazebo。
- 前沿检测 → A* 路径规划 → cmd_vel 控制。
- 接收感知重观察请求，执行靠近、横移、侧视复查。
- 探索完成后返航，到达起点时发布 FINISHED。

状态机: INIT → EXPLORING → REOBSERVING → RETURNING → FINISHED

多楼层扩展（target_floors 非空时）:
  INIT → EXPLORING(floor N) → FLOOR_COMPLETE → FLOOR_TRANSITION → EXPLORING(floor N+1) ...
  → RETURNING → FINISHED
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
from std_msgs.msg import Int32, String

from hazardwalker_nav.frontier_detector import (
    Frontier,
    a_star_path,
    cluster_frontiers,
    compute_frontier_backoff_ttl_s,
    compute_exploration_time_limit_s,
    entry_axis_progress_m,
    entry_ingress_constraint_active,
    entry_ingress_half_angles_deg,
    find_frontiers,
    frontier_route_is_excessive_detour,
    grid_to_world,
    nearest_frontier_basin_key,
    occupancy_grid_to_array,
    return_recovery_turn_command,
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
    reobservation_actions_conflict,
    reobservation_request_is_eligible,
    target_centered_in_image,
)
from hazardwalker_nav.coverage_tracker import CoverageGrid
from hazardwalker_nav.elevator_controller import (
    ElevatorResult,
    call_elevator,
    elevator_approach_position,
    elevator_door_id,
    set_door_state,
)
from hazardwalker_nav.nav_recorder import NavRecorder
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
        # 只要当前目标仍持续缩短合法 SLAM 距离，就不被地图刷新产生的近场
        # 新前沿抢占；否则机器人会在长走廊两端反复掉头，始终到不了房间入口。
        self.declare_parameter('frontier_recent_progress_protection_s', 12.0)
        self.declare_parameter(
            'frontier_progress_protection_max_hold_s', 45.0)
        self.declare_parameter('frontier_net_progress_timeout_s', 30.0)
        self.declare_parameter('frontier_net_progress_distance_m', 0.25)
        # 欧氏距离很近但 A* 必须绕墙十余米的目标会吞掉大量搜索预算。仅当
        # 路径比例和绝对绕行量同时超限、且仍有其他候选时，才短时延后该盆地。
        self.declare_parameter('frontier_max_detour_ratio', 2.8)
        self.declare_parameter('frontier_min_detour_excess_m', 5.0)
        self.declare_parameter('frontier_detour_defer_ttl_s', 30.0)
        self.declare_parameter('frontier_detour_defer_radius_m', 0.60)
        self.declare_parameter('frontier_detour_evaluation_limit', 2)
        # 非官方环境默认从第一条合法 TF 推断。官方 profile 会显式传入公开
        # 起点在 map 帧中的朝向，避免 INIT 建图旋转污染入楼方向。
        self.declare_parameter('entry_heading_yaw', float('nan'))
        self.declare_parameter('entry_forward_half_angle_deg', 35.0)
        # 第一次选到入口路径后仍保持向楼内推进，不能立刻退化成宽半平面并被
        # 楼外南北边界吸走。官方 profile 会按公开建筑尺度显式启用。
        self.declare_parameter('entry_ingress_depth_m', 0.0)
        self.declare_parameter('entry_ingress_relaxed_half_angle_deg', 55.0)
        self.declare_parameter('entry_ingress_max_half_angle_deg', 90.0)
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
        # 横移采用短分段并由合法 SLAM 实际位移限幅。一次最多约 0.8 m，
        # 停稳采帧后再决定下一段，避免目标被墙柱遮挡后仍盲走 10 秒。
        self.declare_parameter('reobserve_lateral_motion_duration_s', 3.0)
        self.declare_parameter('reobserve_lateral_max_distance_m', 0.80)
        self.declare_parameter('reobserve_target_loss_timeout_s', 0.40)
        self.declare_parameter('reobserve_center_tolerance_ratio', 0.18)
        self.declare_parameter('reobserve_settle_duration_s', 1.0)
        self.declare_parameter('reobserve_observe_duration_s', 1.5)
        # 官方 A1 RL 控制器实测会完整接收 0.15 m/s 横移指令却不产生可测位移；
        # 平移复查使用与正式导航相同量级的有效指令，并继续由激光门禁及 25°
        # 视线变化反馈提前停车，避免盲走完整 10 秒。
        self.declare_parameter('reobserve_lateral_speed', 0.45)
        self.declare_parameter('reobserve_forward_speed', 0.30)
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
        self.declare_parameter('return_progress_timeout_s', 8.0)
        self.declare_parameter('return_progress_distance_m', 0.10)
        self.declare_parameter('return_net_progress_timeout_s', 20.0)
        self.declare_parameter('return_net_progress_distance_m', 0.25)
        # 正常路径跟随的小角速度不全局抬高；只有返航静止看门狗触发时，
        # 才发送短时 0.8 rad/s 交替转向脉冲改变物理接触状态。
        self.declare_parameter('return_recovery_turn_speed', 0.80)
        self.declare_parameter('return_recovery_turn_duration_s', 2.0)
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
        # 到达房间入口或局部前沿后主动完成一圈 RGB-D 环视，避免相机只沿
        # 路径切线匆匆经过；感知候选仍可随时抢占进入严格 REOBSERVING。
        self.declare_parameter(
            'frontier_observation_sweep_rad', 2.0 * math.pi)
        self.declare_parameter('frontier_observation_sweep_speed', 0.60)
        self.declare_parameter('frontier_observation_sweep_timeout_s', 18.0)
        self.declare_parameter('unreachable_frontier_ttl_s', 45.0)
        self.declare_parameter('unreachable_frontier_max_ttl_s', 180.0)
        self.declare_parameter('unreachable_frontier_radius_m', 0.45)
        # A* 的空结果既可能表示单个目标不连通，也可能是当前起点吸附失败、
        # 地图瞬时断裂或搜索预算耗尽等整轮共享故障。单次重规划最多封禁少量
        # 盆地，避免一次系统性故障把整层几十个前沿同时判死并阻塞 ROS 回调。
        self.declare_parameter('max_frontier_plan_failures_per_replan', 4)
        # 必须长于基础 unreachable TTL + 一次重规划周期，否则目标刚过期前
        # 就会误判完成，永远没有机会用扩展后的地图重试。
        self.declare_parameter('frontier_completion_grace_s', 60.0)
        # 导航数据记录
        self.declare_parameter('nav_record_enabled', True)
        self.declare_parameter('nav_record_dir', '')
        # 多楼层参数（默认单层，向后兼容）
        self.declare_parameter('target_floors', [])  # 空列表=单层模式
        self.declare_parameter('current_floor_index', 0)
        self.declare_parameter('floor_coverage_threshold', 0.90)
        self.declare_parameter('elevator_id', 'elevator_main')
        self.declare_parameter('elevator_entry_floor', 0)
        self.declare_parameter('stair_detection_enabled', False)
        self.declare_parameter('simenv_container', 'simenv_ros1_hazard_platform')

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
        # 可达但路径效率很低的前沿不等于“不可达”；单独短时延后，避免污染
        # 失败次数与指数退避，并在其他候选耗尽后自动恢复探索完备性。
        self._detour_deferred_frontiers: dict = {}
        self._no_reachable_frontier_since: Optional[float] = None
        self._frontier_observation_remaining_rad: float = 0.0
        self._frontier_observation_last_yaw: Optional[float] = None
        self._frontier_observation_started_ros: Optional[float] = None

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
        self._reobserve_motion_stop_latched = False
        self._reobserve_start_pose: Optional[Tuple[float, float]] = None
        self._reobserve_last_target_seen_ros: Optional[float] = None
        self._reobserve_allow_untracked_upgrade = False
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
        self._return_recovery_attempts = 0
        self._return_recovery_turn_start_ros: Optional[float] = None
        self._return_recovery_turn_end_ros: Optional[float] = None
        self._return_recovery_turn_command = 0.0
        self._return_recovery_start_yaw: Optional[float] = None
        self._return_recovery_scan_blocked_logged = False

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

        # ---- 导航数据记录器 ----
        self.recorder = NavRecorder(
            output_dir=str(self.get_parameter('nav_record_dir').value),
            enabled=bool(self.get_parameter('nav_record_enabled').value),
        )

        # ---- 多楼层 ----
        self._target_floors: list = []
        self._current_floor: int = 0
        self._coverage: Optional[CoverageGrid] = None
        self._elevator_initiated: bool = False
        self._elevator_floor_reached: bool = False
        self._floor_complete_since_ros: Optional[float] = None
        self._floor_transition_phase: str = ''  # navigating | calling | waiting | entering | exiting
        self._floor_transition_start_ros: Optional[float] = None

        # floor_index 发布器（发布 Int32，触发 scan_imu_localizer 重置匹配地图）
        self.floor_index_pub = self.create_publisher(
            Int32, '/hazardwalker/navigation/floor_index', 10)

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
        self._reobserve_motion_stop_latched = False
        self._reobserve_start_pose = (self.robot_x, self.robot_y)
        self._reobserve_last_target_seen_ros = now
        self._reobserve_allow_untracked_upgrade = bool(
            request.get('target_was_untracked', False)
        )
        self._reobserve_attempts[self.reobserve_target_id] = (
            int(self._reobserve_attempts.get(self.reobserve_target_id, 0)) + 1
        )
        self.recorder.record_reobservation(
            now, self.reobserve_target_id, self.reobserve_action,
            'started', reason=str(request.get('reason', '')),
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

        now = self._ros_time_sec()
        status = find_target_status(payload, self.reobserve_target_id)
        if status in ('confirmed', 'rejected', 'rejected_non_spherical'):
            self.reobserve_motion_end_time = now
            self.reobserve_end_time = now
            self.recorder.record_reobservation(
                now, self.reobserve_target_id, self.reobserve_action or '',
                'aborted', reason=f'target_resolved:{status}',
            )
            self.get_logger().info(
                f'Reobservation target {self.reobserve_target_id} resolved: {status}.'
            )
            return
        if self._reobserve_motion_stop_latched:
            return
        detection = find_target_detection(
            payload,
            self.reobserve_target_id,
            allow_untracked_upgrade=self._reobserve_allow_untracked_upgrade,
        )
        if detection is None:
            loss_timeout = max(
                0.1,
                float(self.get_parameter(
                    'reobserve_target_loss_timeout_s').value),
            )
            if (self._reobserve_last_target_seen_ros is not None
                    and now - self._reobserve_last_target_seen_ros
                    >= loss_timeout):
                self._stop_reobservation_motion(
                    now,
                    f'target lost for {loss_timeout:.2f}s',
                )
            return
        self._reobserve_last_target_seen_ros = now
        detection_track_id = str(detection.get('track_id') or '').strip()
        if (detection_track_id
                and not detection_track_id.startswith('untracked:')):
            # 首帧未跟踪候选一旦升级为正式轨迹，后续只能消费精确 track_id。
            self._reobserve_allow_untracked_upgrade = False

        live_request = parse_reobservation_request(payload)
        if (live_request is not None
                and str(live_request.get('target_id', ''))
                == self.reobserve_target_id
                and reobservation_actions_conflict(
                    self.reobserve_action,
                    live_request.get('action'),
                )):
            self._stop_reobservation_motion(
                now,
                'live perception recommendation reversed direction',
            )
            return

        if self.reobserve_action in ('turn_left', 'turn_right'):
            if target_centered_in_image(
                    detection,
                    payload.get('image_width'),
                    float(self.get_parameter(
                        'reobserve_center_tolerance_ratio').value)):
                self._stop_reobservation_motion(
                    now,
                    'target entered image center band',
                )
            return
        if self.reobserve_action not in ('move_left', 'move_right'):
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
        self._stop_reobservation_motion(
            now,
            (
                f'bearing change {achieved:.1f}deg >= '
                f'{self.reobserve_required_bearing_change_deg:.1f}deg'
            ),
        )
        self._reobserve_bearing_goal_met = True
        self.get_logger().info(
            'Reobservation bearing goal reached: '
            f'{achieved:.1f}° >= {self.reobserve_required_bearing_change_deg:.1f}°; '
            'stopping for stable RGB-D evidence.'
        )

    def _stop_reobservation_motion(self, now_ros: float, reason: str):
        """锁存停车并保留停稳观察窗口，避免后续逐帧反馈重新延长动作。"""

        if self._reobserve_motion_stop_latched:
            return
        self.reobserve_motion_end_time = now_ros
        self.reobserve_end_time = (
            now_ros
            + self.reobserve_settle_duration_s
            + self.reobserve_observe_duration_s
        )
        self._reobserve_motion_stop_latched = True
        self.recorder.record_reobservation(
            now_ros, self.reobserve_target_id, self.reobserve_action or '',
            'aborted', reason=reason,
        )
        self.get_logger().info(
            f'Reobservation motion stopped: {reason}; '
            'settling for stable RGB-D evidence.'
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
            self._update_coverage()  # 多楼层：更新覆盖网格
        elif self.state == 'REOBSERVING':
            cmd = self._handle_reobserving()
        elif self.state == 'FLOOR_COMPLETE':
            cmd = self._handle_floor_complete()
        elif self.state == 'FLOOR_TRANSITION':
            cmd = self._handle_floor_transition()
        elif self.state == 'RETURNING':
            cmd = self._handle_returning()
        elif self.state == 'FINISHED':
            cmd = Twist()  # 停止

        self._update_stuck_detection(cmd)
        self.cmd_pub.publish(cmd)

        # ---- 记录位姿与速度指令 ----
        if self._has_fresh_pose():
            target = None
            if self.current_target is not None:
                target = self.current_target.centroid
            self.recorder.record_pose(
                now_ros, self.robot_x, self.robot_y, self.robot_yaw,
                self.state, target,
            )
        self.recorder.record_cmd_vel(
            now_ros, cmd.linear.x, cmd.angular.z, cmd.linear.y,
        )

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
                # 多楼层初始化
                self._init_multi_floor()
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
        if self._frontier_observation_remaining_rad > 0.0:
            return self._handle_frontier_observation_sweep(now_ros)
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
            self.get_logger().info('No frontiers remaining.')
            if self._target_floors and self._next_floor() is not None:
                self._transition('FLOOR_COMPLETE')
            else:
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

    def _handle_frontier_observation_sweep(self, now_ros: float) -> Twist:
        """到达前沿后原地环视，让 RGB-D 覆盖房间而不是只看路径方向。"""

        cmd = Twist()
        if self._frontier_observation_last_yaw is None:
            self._frontier_observation_last_yaw = self.robot_yaw
        else:
            delta = abs(normalize_angle(
                self.robot_yaw - self._frontier_observation_last_yaw
            ))
            self._frontier_observation_remaining_rad = max(
                0.0,
                self._frontier_observation_remaining_rad - delta,
            )
            self._frontier_observation_last_yaw = self.robot_yaw

        timed_out = (
            self._frontier_observation_started_ros is not None
            and now_ros >= self._frontier_observation_started_ros
            and now_ros - self._frontier_observation_started_ros >= max(
                0.1,
                float(self.get_parameter(
                    'frontier_observation_sweep_timeout_s').value),
            )
        )
        if self._frontier_observation_remaining_rad <= 0.05 or timed_out:
            reason = 'timeout' if timed_out else 'completed'
            self.get_logger().info(
                f'Frontier RGB-D observation sweep {reason}; '
                'resuming exploration.'
            )
            self._frontier_observation_remaining_rad = 0.0
            self._frontier_observation_last_yaw = None
            self._frontier_observation_started_ros = None
            return cmd

        if self._scan_allows_action(
                'turn_left',
                float(self.get_parameter(
                    'rotation_min_clearance_m').value)):
            cmd.angular.z = min(
                abs(float(self.get_parameter(
                    'frontier_observation_sweep_speed').value)),
                abs(float(self.get_parameter('angular_speed').value)),
            )
        return cmd

    def _handle_reobserving(self) -> Twist:
        """REOBSERVING: 执行感知请求的重观察机动。"""
        cmd = Twist()
        now = self._ros_time_sec()

        if now >= self.reobserve_end_time:
            self.get_logger().info('Reobservation complete, resuming exploration.')
            self.recorder.record_reobservation(
                now, self.reobserve_target_id or '', self.reobserve_action or '',
                'completed',
                bearing_change_deg=(
                    None if self.reobserve_baseline_bearing_deg is None
                    else None  # bearing_change recorded live by _update_reobservation_feedback
                ),
            )
            self.reobserve_action = None
            self.reobserve_target_id = ''
            self.reobserve_baseline_bearing_deg = None
            self._reobserve_bearing_goal_met = False
            self._reobserve_motion_stop_latched = False
            self._reobserve_start_pose = None
            self._reobserve_last_target_seen_ros = None
            self._reobserve_allow_untracked_upgrade = False
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
        if (action in ('move_left', 'move_right')
                and self._reobserve_start_pose is not None):
            lateral_distance = math.hypot(
                self.robot_x - self._reobserve_start_pose[0],
                self.robot_y - self._reobserve_start_pose[1],
            )
            maximum_distance = max(
                0.1,
                float(self.get_parameter(
                    'reobserve_lateral_max_distance_m').value),
            )
            if lateral_distance >= maximum_distance:
                self._stop_reobservation_motion(
                    now,
                    (
                        f'lateral displacement {lateral_distance:.2f}m '
                        f'>= {maximum_distance:.2f}m'
                    ),
                )
                return cmd
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

        recovery_cmd = self._return_recovery_command_for_now(now)
        if recovery_cmd is not None:
            return recovery_cmd

        force_replan = self._return_progress_watchdog_expired(
            now, dist_home,
        )
        # 即使 watchdog 未触发，也要检测是否在持续远离起点。
        # 门未开或路径规划异常时，A* 可能生成先远离再折返的路径，
        # 导致机器人在 RETURNING 中越走越远。
        if (self._return_best_distance_home is not None
                and dist_home > self._return_best_distance_home + 2.0):
            self.get_logger().warn(
                f'Returning but drifting away from home: '
                f'current={dist_home:.2f}m, best='
                f'{self._return_best_distance_home:.2f}m; '
                'force clearing path and replanning.'
            )
            force_replan = True
        recovery_cmd = self._return_recovery_command_for_now(now)
        if recovery_cmd is not None:
            return recovery_cmd
        if self.grid is None:
            # 无地图时直线盲返在复杂楼宇中不可接受；等待地图恢复。
            return cmd

        # 有效路径存在时保持路线承诺；第 22 轮实测每 3 秒重算会让动态地图
        # 两条近似等价路线反复翻转。新障碍仍由 scan 门禁停车，再由看门狗重算。
        should_replan = (
            force_replan
            or self._last_return_plan_time is None
            or len(self.current_path) == 0
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

    def _return_recovery_command_for_now(
            self, now_ros: float) -> Optional[Twist]:
        """返航静止后执行短时、安全门禁下的交替原地转向。"""

        if self._return_recovery_turn_end_ros is None:
            return None
        cmd = Twist()
        if (self._return_recovery_turn_start_ros is not None
                and now_ros < self._return_recovery_turn_start_ros):
            self.get_logger().warn(
                'Simulation clock moved backward during return recovery; '
                'clearing the recovery window.'
            )
            self._return_recovery_turn_start_ros = None
            self._return_recovery_turn_end_ros = None
            self._return_recovery_turn_command = 0.0
            self._return_recovery_start_yaw = None
            self._return_recovery_scan_blocked_logged = False
            self.current_path = []
            self.path_index = 0
            self._last_return_plan_time = None
            return cmd
        if now_ros < self._return_recovery_turn_end_ros:
            action = (
                'turn_left'
                if self._return_recovery_turn_command > 0.0
                else 'turn_right'
            )
            scan_allowed = self._scan_allows_action(
                action,
                float(self.get_parameter(
                    'rotation_min_clearance_m').value),
            )
            if scan_allowed:
                cmd.angular.z = self._return_recovery_turn_command
            elif not self._return_recovery_scan_blocked_logged:
                self._return_recovery_scan_blocked_logged = True
                self.get_logger().warn(
                    'Return recovery turn blocked by the full-circle '
                    'scan safety gate; holding zero velocity.'
                )
            return cmd

        yaw_delta = (
            0.0
            if self._return_recovery_start_yaw is None
            else abs(normalize_angle(
                self.robot_yaw - self._return_recovery_start_yaw,
            ))
        )
        self.get_logger().info(
            'Return recovery turn finished: '
            f'attempt={self._return_recovery_attempts}, '
            f'yaw_delta={yaw_delta:.3f}rad; replanning.'
        )
        self._return_recovery_turn_start_ros = None
        self._return_recovery_turn_end_ros = None
        self._return_recovery_turn_command = 0.0
        self._return_recovery_start_yaw = None
        self._return_recovery_scan_blocked_logged = False
        self.current_path = []
        self.path_index = 0
        self._last_return_plan_time = None
        return None

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
        if stationary_expired:
            self._return_recovery_attempts += 1
            maximum_speed = abs(
                float(self.get_parameter('angular_speed').value),
            )
            configured_speed = min(
                maximum_speed,
                abs(float(self.get_parameter(
                    'return_recovery_turn_speed').value)),
            )
            self._return_recovery_turn_command = (
                return_recovery_turn_command(
                    self._return_recovery_attempts,
                    configured_speed,
                )
            )
            duration = max(
                0.1,
                float(self.get_parameter(
                    'return_recovery_turn_duration_s').value),
            )
            self._return_recovery_turn_start_ros = now_ros
            self._return_recovery_turn_end_ros = now_ros + duration
            self._return_recovery_start_yaw = self.robot_yaw
            self._return_recovery_scan_blocked_logged = False
            direction = (
                'left'
                if self._return_recovery_turn_command > 0.0
                else 'right'
            )
            self.get_logger().warn(
                'Starting bounded return recovery turn: '
                f'attempt={self._return_recovery_attempts}, '
                f'direction={direction}, '
                f'speed={abs(self._return_recovery_turn_command):.2f}rad/s, '
                f'duration={duration:.2f}s.'
            )
        self._return_last_progress_time = now_ros
        self._return_last_progress_pose = (self.robot_x, self.robot_y)
        self._return_last_net_progress_time = now_ros
        self._return_net_progress_reference_distance = dist_home
        return True

    # ---- 多楼层处理 ----

    def _init_multi_floor(self):
        """读取 target_floors 参数，初始化多楼层探索。空列表则跳过（单层模式）。"""
        try:
            target_floors = list(self.get_parameter('target_floors').value)
        except (TypeError, ValueError):
            target_floors = []
        if not target_floors:
            return
        self._target_floors = sorted(target_floors)
        self._current_floor = int(
            self.get_parameter('current_floor_index').value)
        if self.grid is not None:
            h, w = self.grid.shape
            self._coverage = CoverageGrid(h, w)
        self._publish_floor_index(self._current_floor)
        self.recorder.record_floor_change(
            self._ros_time_sec(), -1, self._current_floor, 'initial')
        self.get_logger().info(
            f'Multi-floor exploration enabled: '
            f'floors={self._target_floors}, '
            f'current={self._current_floor}')

    def _update_coverage(self):
        """以 2 Hz 降采样更新覆盖网格。"""
        if self._coverage is None or self.grid is None:
            return
        now = time.monotonic()
        if (getattr(self, '_last_coverage_update', None) is not None
                and now - self._last_coverage_update < 0.5):
            return
        self._last_coverage_update = now
        gx, gy = world_to_grid(
            self.robot_x, self.robot_y, self.latest_map)
        if 0 <= gx < self.grid.shape[1] and 0 <= gy < self.grid.shape[0]:
            self._coverage.update(gx, gy, self.grid)

    def _floor_is_covered(self) -> bool:
        """判断当前楼层是否覆盖达标。"""
        if self._target_floors is None or len(self._target_floors) == 0:
            return False
        threshold = float(
            self.get_parameter('floor_coverage_threshold').value)
        no_frontiers = (
            self.current_target is None
            and len(self.current_path) == 0
            and len(self._visited_frontiers) > 0
        )
        coverage_ok = False
        if self._coverage is not None and self.grid is not None:
            ratio = self._coverage.floor_coverage_ratio(self.grid)
            coverage_ok = ratio >= threshold
        return no_frontiers and coverage_ok

    def _handle_floor_complete(self) -> Twist:
        """FLOOR_COMPLETE: 当前层探索完毕，准备跨层。"""
        cmd = Twist()
        now_ros = self._ros_time_sec()
        if self._floor_complete_since_ros is None:
            self._floor_complete_since_ros = now_ros
        if now_ros - self._floor_complete_since_ros < 2.0:
            return cmd
        next_floor = self._next_floor()
        if next_floor is None:
            self.get_logger().info(
                'All target floors explored. Preparing to return home.')
            self._transition('RETURNING')
            return cmd
        self._current_floor = next_floor
        self._floor_complete_since_ros = None
        self._elevator_initiated = False
        self._elevator_floor_reached = False
        self._floor_transition_phase = 'navigating'
        self._floor_transition_start_ros = now_ros
        self.recorder.record_floor_change(
            now_ros, self._current_floor, next_floor, 'elevator')
        self.get_logger().info(
            f'Floor {self._current_floor} complete. '
            f'Transitioning to floor {next_floor}.')
        self._transition('FLOOR_TRANSITION')
        return cmd

    def _handle_floor_transition(self) -> Twist:
        """FLOOR_TRANSITION: 导航到电梯 → 呼叫电梯 → 跨层 → 新层探索。"""
        cmd = Twist()
        now_ros = self._ros_time_sec()
        container = str(self.get_parameter('simenv_container').value)
        elevator_id = str(self.get_parameter('elevator_id').value)
        tol = float(self.get_parameter('goal_tolerance_m').value)
        if self._floor_transition_phase == 'navigating':
            elevator_pos = elevator_approach_position(self._current_floor)
            dist = math.hypot(
                self.robot_x - elevator_pos[0],
                self.robot_y - elevator_pos[1])
            if dist > tol:
                if self.grid is not None:
                    self.current_path = []
                    try:
                        path = a_star_path(
                            self.grid, self.latest_map,
                            self.robot_x, self.robot_y,
                            elevator_pos[0], elevator_pos[1],
                            start_search_radius_m=0.50,
                        )
                        self.current_path = path
                    except Exception:
                        pass
                self.path_index = 0
                if len(self.current_path) > 0:
                    cmd = self._follow_path()
                return cmd
            self._floor_transition_phase = 'calling'
            self._floor_transition_start_ros = now_ros
            self.get_logger().info('Arrived at elevator. Calling...')
        if self._floor_transition_phase == 'calling':
            if not self._elevator_initiated:
                try:
                    entry_floor = int(
                        self.get_parameter('elevator_entry_floor').value)
                    result = call_elevator(
                        container, elevator_id, entry_floor,
                        open_doors=True, timeout_s=30.0,
                    )
                    self.recorder.record_elevator_call(
                        now_ros, elevator_id, entry_floor,
                        'called', result.state)
                    if result.accepted:
                        self._elevator_initiated = True
                        self.get_logger().info(
                            f'Elevator called to floor {entry_floor}: '
                            f'{result.state}')
                    else:
                        self.get_logger().warn(
                            f'Elevator call rejected: {result.message}')
                except Exception as exc:
                    self.get_logger().error(
                        f'Elevator call failed: {exc}')
            if (self._elevator_initiated
                    and now_ros - (self._floor_transition_start_ros or now_ros) > 5.0):
                self._floor_transition_phase = 'entering'
                self._floor_transition_start_ros = now_ros
        if self._floor_transition_phase == 'entering':
            if not self._elevator_floor_reached:
                try:
                    result = call_elevator(
                        container, elevator_id, self._current_floor,
                        open_doors=True, timeout_s=30.0,
                    )
                    self.recorder.record_elevator_call(
                        now_ros, elevator_id, self._current_floor,
                        'entered', result.state)
                    if result.accepted and result.current_floor == self._current_floor:
                        self._elevator_floor_reached = True
                        self.get_logger().info(
                            f'Arrived at floor {self._current_floor}')
                        self._publish_floor_index(self._current_floor)
                except Exception as exc:
                    self.get_logger().error(
                        f'Floor transition failed: {exc}')
        if self._elevator_floor_reached or (
                self._floor_transition_start_ros is not None
                and now_ros - self._floor_transition_start_ros > 60.0):
            self.get_logger().info(
                f'Beginning exploration on floor {self._current_floor}')
            self.current_target = None
            self.current_path = []
            self._visited_frontiers.clear()
            self._unreachable_frontiers.clear()
            self._detour_deferred_frontiers.clear()
            self._reset_frontier_progress_watchdog()
            if self.grid is not None:
                h, w = self.grid.shape
                self._coverage = CoverageGrid(h, w)
            self._floor_transition_phase = ''
            self._transition('EXPLORING')
        return cmd

    def _next_floor(self) -> Optional[int]:
        """返回下一个待探索楼层，若全部完成则返回 None。"""
        if not self._target_floors:
            return None
        try:
            idx = self._target_floors.index(self._current_floor)
        except ValueError:
            return None
        if idx + 1 >= len(self._target_floors):
            return None
        return self._target_floors[idx + 1]

    def _publish_floor_index(self, index: int):
        """发布 /hazardwalker/navigation/floor_index，触发 SLAM 地图重置。"""
        msg = Int32()
        msg.data = index
        self.floor_index_pub.publish(msg)
        self.get_logger().info(
            f'Published floor_index={index} → SLAM map will reset.')

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
        plan_failure_budget = max(
            1,
            int(self.get_parameter(
                'max_frontier_plan_failures_per_replan').value),
        )
        plan_failures_this_cycle = 0
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
        self._detour_deferred_frontiers = {
            key: expiry
            for key, expiry in self._detour_deferred_frontiers.items()
            if expiry > now_ros
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
        entry_progress = entry_axis_progress_m(
            self.robot_x,
            self.robot_y,
            self._entry_origin,
            self._entry_axis,
        )
        ingress_depth = max(
            0.0,
            float(self.get_parameter('entry_ingress_depth_m').value),
        )
        ingress_constraint_active = entry_ingress_constraint_active(
            self._entry_axis,
            entry_progress,
            ingress_depth,
        )
        selected_ingress_half_angle: Optional[float] = None

        def select_candidate(candidate_pool):
            nonlocal selected_ingress_half_angle
            selected_ingress_half_angle = None
            half_angles = entry_ingress_half_angles_deg(
                float(self.get_parameter(
                    'entry_forward_half_angle_deg').value),
                float(self.get_parameter(
                    'entry_ingress_relaxed_half_angle_deg').value),
                float(self.get_parameter(
                    'entry_ingress_max_half_angle_deg').value),
                ingress_constraint_active,
            )
            for half_angle in half_angles:
                selected = select_best_frontier(
                    candidate_pool, self.robot_x, self.robot_y,
                    last_target=self.last_target_world,
                    min_frontier_size=min_size,
                    locality_slack_m=float(
                        self.get_parameter(
                            'frontier_locality_slack_m').value),
                    # 在达到最小入楼纵深前持续使用公开起点轴；若当前地图
                    # 暂无窄锥候选，再分级放宽到 55°/90°，避免死锁。
                    robot_yaw=(
                        entry_heading
                        if half_angle is not None
                        else None
                    ),
                    robot_yaw_half_angle_rad=math.radians(
                        90.0 if half_angle is None else half_angle,
                    ),
                    require_robot_yaw_candidate=half_angle is not None,
                    entry_origin=self._entry_origin,
                    entry_axis=self._entry_axis,
                    entry_lateral_limit_m=float(
                        self.get_parameter('entry_lateral_limit_m').value),
                )
                if selected is not None:
                    selected_ingress_half_angle = half_angle
                    return selected
            return None

        preferred_frontiers = [
            frontier for frontier in unvisited_frontiers
            if not self._frontier_is_detour_deferred(frontier, now_ros)
        ]
        # TTL 只在存在替代目标时降低长绕行盆地的优先级。若所有合法前沿都
        # 处于延后状态则立即回退，不能为了优化路径而原地等待 30 秒。
        selection_frontiers = preferred_frontiers or unvisited_frontiers

        if self.current_target is not None:
            refreshed_path = a_star_path(
                self.grid, self.latest_map,
                self.robot_x, self.robot_y,
                self.current_target.centroid[0],
                self.current_target.centroid[1],
            )
            if not refreshed_path:
                self._mark_frontier_unreachable(self.current_target)
                plan_failures_this_cycle += 1
                self.current_target = None
                self.current_path = []
                self._current_target_selected_ros_sec = None
                self._reset_frontier_progress_watchdog()
            else:
                challenger = select_candidate(selection_frontiers)
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
                recent_progress_age = (
                    None if self._frontier_last_net_progress_ros is None
                    else max(
                        0.0,
                        now_ros - self._frontier_last_net_progress_ros,
                    )
                )
                if not should_switch_frontier(
                        current_distance,
                        challenger_distance,
                        held_duration,
                        float(self.get_parameter(
                            'frontier_switch_margin_m').value),
                        float(self.get_parameter(
                            'frontier_minimum_hold_s').value),
                        recent_progress_age_s=recent_progress_age,
                        progress_protection_s=float(self.get_parameter(
                            'frontier_recent_progress_protection_s').value),
                        progress_protection_max_hold_s=float(
                            self.get_parameter(
                                'frontier_progress_protection_max_hold_s'
                            ).value)):
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

        # 当前目标或本轮规划失败后，立即从候选池移除整个活动失败盆地。
        # 否则同一轮仍会再次选择已标记目标，把一次失败错误升级成两次退避。
        unvisited_frontiers = [
            frontier for frontier in unvisited_frontiers
            if not self._frontier_is_unreachable(frontier, now_ros)
        ]
        preferred_frontiers = [
            frontier for frontier in unvisited_frontiers
            if not self._frontier_is_detour_deferred(frontier, now_ros)
        ]

        # 评分最高的前沿不一定能在“只走已知自由区”的安全地图上到达；
        # 逐个尝试，规划失败的目标本轮不再反复选择。失败预算用于区分少量
        # 真实不可达目标和整张地图共享的瞬时规划故障，剩余目标留给更新后的
        # 地图再次判断，不能在一个 ROS 回调里全部封禁。
        candidates = list(unvisited_frontiers)
        detour_evaluation_limit = max(
            1,
            int(self.get_parameter(
                'frontier_detour_evaluation_limit').value),
        )
        detour_evaluations = 0
        detour_fallback = None
        while (candidates
               and plan_failures_this_cycle < plan_failure_budget):
            # 先试没有处于路径效率 TTL 的目标；若它们均失败或不存在，则在
            # 同一规划周期立即回退全部合法候选，绝不原地等待 TTL。
            preferred_candidates = [
                candidate for candidate in candidates
                if not self._frontier_is_detour_deferred(
                    candidate, now_ros,
                )
            ]
            best = select_candidate(preferred_candidates or candidates)
            if best is None:
                break
            path = a_star_path(
                self.grid, self.latest_map,
                self.robot_x, self.robot_y,
                best.centroid[0], best.centroid[1],
            )
            if path:
                path_distance = sum(
                    math.hypot(
                        path[index][0] - path[index - 1][0],
                        path[index][1] - path[index - 1][1],
                    )
                    for index in range(1, len(path))
                )
                straight_distance = math.hypot(
                    best.centroid[0] - self.robot_x,
                    best.centroid[1] - self.robot_y,
                )
                if (
                    not ingress_constraint_active
                    and len(candidates) > 1
                    and detour_evaluations < detour_evaluation_limit
                    and frontier_route_is_excessive_detour(
                        path_distance,
                        straight_distance,
                        float(self.get_parameter(
                            'frontier_max_detour_ratio').value),
                        float(self.get_parameter(
                            'frontier_min_detour_excess_m').value),
                    )
                ):
                    if detour_fallback is None:
                        detour_fallback = (
                            best,
                            path,
                            selected_ingress_half_angle,
                            path_distance,
                            straight_distance,
                        )
                    self._defer_frontier_detour(
                        best,
                        path_distance,
                        straight_distance,
                    )
                    detour_evaluations += 1
                    candidates = [
                        candidate for candidate in candidates
                        if not self._frontier_is_detour_deferred(
                            candidate, now_ros,
                        )
                    ]
                    continue
                self._accept_frontier_plan(
                    best,
                    path,
                    now_ros,
                    entry_heading,
                    selected_ingress_half_angle,
                )
                return
            self._mark_frontier_unreachable(best)
            plan_failures_this_cycle += 1
            candidates = [
                candidate for candidate in candidates
                if not self._frontier_is_unreachable(candidate, now_ros)
            ]

        if detour_fallback is not None:
            best, path, half_angle, path_distance, straight_distance = (
                detour_fallback
            )
            self._clear_detour_deferred_frontier(best)
            self.get_logger().info(
                'All sampled frontier alternatives were excessive detours '
                'or unavailable; accepting the first safe fallback now: '
                f'path={path_distance:.2f}m, '
                f'straight={straight_distance:.2f}m.'
            )
            self._accept_frontier_plan(
                best,
                path,
                now_ros,
                entry_heading,
                half_angle,
            )
            return

        self.current_target = None
        self.current_path = []
        if (candidates
                and plan_failures_this_cycle >= plan_failure_budget):
            self.get_logger().warn(
                'Frontier planning failure budget exhausted: '
                f'{plan_failures_this_cycle} failures; '
                f'{len(candidates)} remaining candidates preserved for '
                'a later map update.'
            )
        self.get_logger().warn('No safely reachable frontier in the current map.')

    def _accept_frontier_plan(
            self,
            frontier: Frontier,
            path,
            now_ros: float,
            entry_heading: float,
            selected_ingress_half_angle: Optional[float]):
        """提交已验证安全的前沿路径，并统一初始化进度监视状态。"""

        if self._entry_axis is None:
            self._entry_origin = (self.start_x, self.start_y)
            # 有官方公开朝向时固定使用该轴，而不是首个前沿质心的偏角；
            # 这样入口较宽时也不会把侧向大厅误当成整栋楼纵深方向。
            self._entry_axis = (
                math.cos(entry_heading),
                math.sin(entry_heading),
            )
        self.current_target = frontier
        self._current_target_selected_ros_sec = now_ros
        self._frontier_last_net_progress_ros = now_ros
        self._frontier_progress_reference_distance = math.hypot(
            frontier.centroid[0] - self.robot_x,
            frontier.centroid[1] - self.robot_y,
        )
        self.last_target_world = frontier.centroid
        self.current_path = path
        self.path_index = 0
        selection_mode = (
            'all-directions'
            if selected_ingress_half_angle is None
            else (
                'ingress-cone='
                f'{selected_ingress_half_angle:.0f}deg'
            )
        )
        self.get_logger().info(
            f'New frontier: ({frontier.centroid[0]:.2f}, '
            f'{frontier.centroid[1]:.2f}), size={frontier.size}, '
            f'path={len(path)} steps, mode={selection_mode}'
        )

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

    def _defer_frontier_detour(
            self,
            frontier: Frontier,
            path_distance_m: float,
            straight_distance_m: float):
        """短时延后隔墙长绕行前沿，不把它伪装成不可达失败。"""

        now_ros = self._ros_time_sec()
        radius = max(
            0.0,
            float(self.get_parameter(
                'frontier_detour_defer_radius_m').value),
        )
        basin_key = nearest_frontier_basin_key(
            self._detour_deferred_frontiers.keys(),
            frontier.centroid[0],
            frontier.centroid[1],
            radius,
        )
        if basin_key is None:
            basin_key = self._frontier_key(frontier)
        ttl = max(
            0.1,
            float(self.get_parameter(
                'frontier_detour_defer_ttl_s').value),
        )
        self._detour_deferred_frontiers[basin_key] = now_ros + ttl
        ratio = path_distance_m / max(0.25, straight_distance_m)
        self.get_logger().info(
            'Deferring inefficient frontier basin '
            f'({basin_key[0]:.2f}, {basin_key[1]:.2f}) for {ttl:.1f} '
            f'sim seconds: path={path_distance_m:.2f}m, '
            f'straight={straight_distance_m:.2f}m, ratio={ratio:.2f}.'
        )

    def _frontier_is_detour_deferred(
            self, frontier: Frontier, now_ros: Optional[float] = None) -> bool:
        """判断候选是否落在仍处于短时路径效率延后的空间盆地中。"""

        if now_ros is None:
            now_ros = self._ros_time_sec()
        active_keys = [
            key for key, expiry in self._detour_deferred_frontiers.items()
            if expiry > now_ros
        ]
        return nearest_frontier_basin_key(
            active_keys,
            frontier.centroid[0],
            frontier.centroid[1],
            float(self.get_parameter(
                'frontier_detour_defer_radius_m').value),
        ) is not None

    def _clear_detour_deferred_frontier(self, frontier: Frontier):
        """回退使用唯一安全长绕行路线时，立即清除对应临时延后盆地。"""

        basin_key = nearest_frontier_basin_key(
            self._detour_deferred_frontiers.keys(),
            frontier.centroid[0],
            frontier.centroid[1],
            float(self.get_parameter(
                'frontier_detour_defer_radius_m').value),
        )
        if basin_key is not None:
            self._detour_deferred_frontiers.pop(basin_key, None)

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
                sweep_rad = max(
                    0.0,
                    float(self.get_parameter(
                        'frontier_observation_sweep_rad').value),
                )
                if sweep_rad > 0.0:
                    self._frontier_observation_remaining_rad = sweep_rad
                    self._frontier_observation_last_yaw = self.robot_yaw
                    self._frontier_observation_started_ros = (
                        self._ros_time_sec()
                    )
                    self.get_logger().info(
                        'Starting frontier RGB-D observation sweep: '
                        f'{math.degrees(sweep_rad):.0f} deg.'
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
                self.recorder.record_failure(
                    now_ros, 'safety_blocked',
                    self.robot_x, self.robot_y,
                    'scan clearance gate blocked all motion for '
                    f'{now_ros - self._safety_blocked_since_ros:.1f}s',
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
            self.recorder.record_failure(
                self._ros_time_sec(), 'stuck',
                self.robot_x, self.robot_y,
                f'moved={moved:.3f}m rotated={rotated:.3f}rad',
            )
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
        self.recorder.record_state_transition(
            self._ros_time_sec(), self.prev_state, new_state,
        )
        # 多楼层：退避跨层状态下的卡死检测
        if new_state in ('FLOOR_COMPLETE', 'FLOOR_TRANSITION'):
            self._stuck_since = None
            self._pose_history.clear()
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
            self._return_recovery_attempts = 0
            self._return_recovery_turn_start_ros = None
            self._return_recovery_turn_end_ros = None
            self._return_recovery_turn_command = 0.0
            self._return_recovery_start_yaw = None
            self._return_recovery_scan_blocked_logged = False
        elif new_state == 'FINISHED':
            self._return_recovery_turn_start_ros = None
            self._return_recovery_turn_end_ros = None
            self._return_recovery_turn_command = 0.0
            self._return_recovery_start_yaw = None
            self._return_recovery_scan_blocked_logged = False
            # 保存地图并关闭记录器
            if self.grid is not None:
                self.recorder.save_map(
                    self.grid, self.latest_map, self._ros_time_sec(),
                )
            self.recorder.close(
                self._ros_time_sec(),
                final_state='FINISHED',
                total_frontiers_visited=len(self._visited_frontiers),
            )
        self.get_logger().info(f'State: {self.prev_state} → {new_state}')


def main():
    rclpy.init()
    node = FrontierExplorerNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        # 异常退出时也保存已有数据
        try:
            if node.recorder._enabled:
                node.recorder.close(
                    node._ros_time_sec(),
                    final_state=node.state,
                    total_frontiers_visited=len(node._visited_frontiers),
                )
        except Exception:
            pass
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
