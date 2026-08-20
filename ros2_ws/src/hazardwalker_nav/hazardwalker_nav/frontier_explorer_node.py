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
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
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
from hazardwalker_nav.elevator_controller import call_elevator
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
        # 探索区域约束：世界系（odom=gazebo world）轴对齐矩形。楼外/门外
        # 前沿一律抑制不选为目标；机器人自身若已出区则强制导引回楼内。
        # 默认全 0 表示不启用（通用/未知场景向后兼容）。
        self.declare_parameter('region_x_min', 0.0)
        self.declare_parameter('region_x_max', 0.0)
        self.declare_parameter('region_y_min', 0.0)
        self.declare_parameter('region_y_max', 0.0)
        # 官方场景前沿通常距离较近；过大的容差会把首个目标直接误判为“已到达”。
        self.declare_parameter('goal_tolerance_m', 0.25)
        self.declare_parameter('linear_speed', 0.45)
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
        # 返航进入 home 附近（起点大厅）后，A* 的起点吸附会被近场回波/
        # SLAM 漂移标占用而引向身后幽灵栅格，造成在 home 旁原地绕圈。
        # dist_home 小于该距离时改为直接对准 home 直线前进（仍受 scan
        # 门禁），静止看门狗兜底回退 A* 绕行，避免绕圈几十轮不 FINISHED。
        self.declare_parameter('return_straight_distance_m', 3.0)
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
            'frontier_observation_sweep_rad', 0.0)  # 2026-08-19: 走廊/墙沿到达不再原地转圈，环视仅保留在房间探索内
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
        # 房间/走廊分类参数
        self.declare_parameter('room_boost_score', 2.0)
        self.declare_parameter('corridor_penalty_score', 0.3)
        self.declare_parameter('room_coverage_threshold', 0.85)
        # Phase 3: 房间完整探索 enter -> explore(走深+覆盖) -> exit
        # explore 阶段覆盖 >= room_coverage_threshold 才认为探索完整
        self.declare_parameter('room_entry_depth_m', 2.0)
        self.declare_parameter('room_half_extent_m', 4.0)
        self.declare_parameter('room_explore_timeout_s', 60.0)
        self.declare_parameter('room_enter_fail_s', 3.0)
        self.declare_parameter('room_sweep_timeout_s', 30.0)
        self.declare_parameter('room_scan_turn_speed', 0.8)
        # 门口候选触发：corridor 型前沿窄边 >= room_trigger_min_door_m 且
        # aspect <= room_trigger_max_aspect 时也尝试进入（enter 有快失败兜底）
        self.declare_parameter('room_trigger_max_aspect', 4.0)
        self.declare_parameter('room_trigger_min_door_m', 1.0)
        # 每层电梯入口在 map 帧的坐标 {floor: [x, y]}。map 帧每层 SLAM 重建
        # 后原点会变，无法用固定世界坐标推导，必须真机标定后注入；空字符串时
        # 跨层 navigating 阶段会告警并跳过，避免导航到地图原点。ROS2 参数无
        # dict 类型，故以 YAML 字符串传递、在 _init_multi_floor 内解析。
        self.declare_parameter('elevator_positions', '')
        # 电梯服务调用失败后的最小重试间隔，防止在 10 Hz 控制循环内高频
        # 同步阻塞 docker exec 拖死控制心跳。
        self.declare_parameter('elevator_retry_interval_s', 2.0)
        # 控制输出话题。平台组引入 hazardwalker_command_mux 仲裁器后，导航命令
        # 必须发布到 /hw/control/navigation_cmd_vel，由仲裁器统一裁决后转发到
        # /hw/cmd_vel；直接抢占 /hw/cmd_vel 会被仲裁器输出的零速度稀释/覆盖。
        self.declare_parameter(
            'cmd_vel_topic', '/hw/control/navigation_cmd_vel')
        # command_mux 默认 default_mode=keyboard，需显式请求 navigation 才会
        # 转发导航命令。启动期按 1 Hz 重试 3 次覆盖 DDS 匹配窗口，之后依赖
        # 模式一次锁存不再发；若实测中途切回 keyboard 再改为周期心跳。
        self.declare_parameter(
            'control_mode_request_topic', '/hw/control/mode_request')
        self.declare_parameter('control_mode_value', 'navigation')

        # ---- 状态机 ----
        self.state = 'INIT'
        self.prev_state = ''
        self.start_time = self.get_clock().now()
        self._mission_start_ros_sec: Optional[float] = None
        self._state_entry_time = time.monotonic()
        self._mode_request_sent_count = 0
        self._last_mode_request_time = 0.0

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

        # ---- 世界系参考（区域约束） ----
        self._world_x: Optional[float] = None
        self._world_y: Optional[float] = None
        self._world_yaw: Optional[float] = None
        # map↔world 变换跳变门禁：map 重锚/localizer yaw 漂移导致变换瞬跳时
        # hold 区域过滤（本周期视为区内），靠机器人侧 region-return 兜底防出区。
        self._region_last_transform = None  # last-good: (yaw_diff, s, c, dx, dy)
        self._region_transform_hold_until = None

        # ---- 返航 ----
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        # world 帧家：INIT 时用 /hw/odom 捕获物理出发点。map 帧 start_x/start_y
        # 会随 SLAM 重锚漂移失效，RETURNING 必须以世界帧家为准、规划时用当前
        # map↔world 变换重投影回 map 帧，避免返航目标被重锚作废（2026-08-20
        # run_20260820_171917 实锤：RETURNING 卡在 map(0,0) 假目标上打转）。
        self._home_world: Optional[Tuple[float, float]] = None
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
        # 世界真值里程计：adapter 把 /Odometry_gazebo（gazebo world 系）转发
        # 为 /hw/odom。区域约束用它判断机器人/前沿在世界系中的位置。若环境
        # 没有该话题（非官方/离线），区域约束自动失效，不影响通用探索。
        self.odom_sub = self.create_subscription(
            Odometry, '/hw/odom', self.on_odom, 10)
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self.state_pub = self.create_publisher(String, '/hw/nav/state', 10)
        self.mode_request_pub = self.create_publisher(
            String,
            str(self.get_parameter('control_mode_request_topic').value),
            10,
        )

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
        self._floor_transition_phase: str = ''  # navigating | calling | entering
        self._floor_transition_start_ros: Optional[float] = None
        self._floor_transition_from_floor: Optional[int] = None
        self._elevator_positions: dict = {}
        self._last_elevator_call_ros: Optional[float] = None

        # ---- Phase 3: 房间完整探索 enter | explore | exit ----
        self._room_sweep_active: bool = False
        self._room_sweep_frontier: Optional[Frontier] = None
        self._room_sweep_phase: str = ''  # enter | explore | exit
        self._room_sweep_start_time: Optional[float] = None
        self._room_sweep_entry_pose: Optional[Tuple[float, float]] = None
        self._room_sweep_yaw_start: Optional[float] = None
        self._room_enter_blocked_since_ros: Optional[float] = None
        self._room_internal_target: Optional[Frontier] = None
        self._room_internal_path: list = []
        self._room_internal_path_i: int = 0
        self._room_exit_started_ros: Optional[float] = None
        self._room_exit_path: list = []
        self._room_exit_path_i: int = 0
        self._room_spin_last_yaw: Optional[float] = None
        self._room_spin_accum: float = 0.0
        self._room_no_target_spins: int = 0

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

    def on_odom(self, msg: Odometry):
        """保存 gazebo 世界系真值位姿（区域约束参考）。"""
        self._world_x = msg.pose.pose.position.x
        self._world_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._world_yaw = math.atan2(siny_cosp, cosy_cosp)

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
            distance_home_m=self._distance_home_m(),
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
        self._ensure_control_mode()

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
                # 记录初始位姿作为家；world 帧家保证 map 重锚后仍指向物理出发点。
                self.start_x = self.robot_x
                self.start_y = self.robot_y
                if self._world_x is not None and self._world_y is not None:
                    self._home_world = (self._world_x, self._world_y)
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

    def _handle_region_return(self) -> Twist:
        """机器人已走出探索区域：计算区域内最近可达点并导引回去。

        不参与常规前沿选择；返回区域后才恢复正常探索。目标标记为
        synthetic（frontier_type='region_return'），到达后不记 visited、
        不触发房间扫描。
        """

        cmd = Twist()
        if self.grid is None or self.latest_map is None:
            return cmd
        if self._world_x is None:
            return cmd
        # transform 在跳变 hold 期可能返回 None；world→map 用 last-good 兜底，
        # 安全网不得因 hold 而失效。

        # 区域内最近点（世界系，向内收 0.6 m，避免目标贴墙/贴门板）
        x_min, x_max, y_min, y_max = self._region_bounds()
        inset_x_min, inset_x_max = x_min + 0.6, x_max - 0.6
        inset_y_min, inset_y_max = y_min + 0.6, y_max - 0.6
        gx = min(max(self._world_x, inset_x_min), inset_x_max)
        gy = min(max(self._world_y, inset_y_min), inset_y_max)
        target_map = self._region_world_to_map(gx, gy)
        # last-good 变换可能过期（map 重锚大幅改动帧）：若目标距机器人现 map
        # 位过远则原地扫描等变换恢复，避免朝过期目标乱走。
        if math.hypot(target_map[0] - self.robot_x,
                      target_map[1] - self.robot_y) > 12.0:
            self.current_target = None
            self.current_path = []
            self.get_logger().warn(
                'Region return target stale (map frame moved); '
                'holding for map update.')
            return self._hold_scan_rotation(cmd)

        # 已在回区路径中且有进展：继续当前的，避免每帧重算打断转向。
        if self.current_target is not None and len(self.current_path) > 0:
            if (getattr(self.current_target, 'frontier_type', '')
                    == 'region_return'
                    and math.hypot(
                        target_map[0] - self.current_target.centroid[0],
                        target_map[1] - self.current_target.centroid[1],
                    ) < 1.5):
                return self._follow_path()

        path = a_star_path(
            self.grid, self.latest_map,
            self.robot_x, self.robot_y,
            target_map[0], target_map[1],
        )
        if not path or len(path) == 0:
            # 没有可行路径（目标格被占/地图瞬时断裂）：原地扫描等下一次。
            self.current_target = None
            self.current_path = []
            self.get_logger().warn(
                'Region return: no path to nearest in-region point '
                f'({gx:.2f}, {gy:.2f}) world; holding for map update.'
            )
            return self._hold_scan_rotation(cmd)

        ret = Frontier(
            centroid=target_map, size=1, points=[], info_gain=0.0,
            frontier_type='region_return',
        )
        self.current_target = ret
        self.current_path = path
        self.path_index = 0
        self._current_target_selected_ros_sec = self._ros_time_sec()
        self._reset_frontier_progress_watchdog()
        self.get_logger().warn(
            f'Outside exploration region at world ({self._world_x:.2f}, '
            f'{self._world_y:.2f}); returning to ({gx:.2f}, {gy:.2f}).'
        )
        return self._follow_path()

    def _hold_scan_rotation(self, cmd: Twist) -> Twist:
        """区域内/区域外等待时缓慢原地扫描，积累地图更新。"""

        if self._scan_allows_action(
                'turn_left',
                float(self.get_parameter('rotation_min_clearance_m').value)):
            cmd.angular.z = min(
                float(self.get_parameter('frontier_recovery_turn_speed').value),
                float(self.get_parameter('angular_speed').value),
            )
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
        if self._room_sweep_active:
            return self._handle_room_sweep(now_ros)

        # 机器人已出探索区域：优先导引回楼内，跳过常规前沿选择。
        if self._region_enabled() and not self._robot_in_region_world():
            return self._handle_region_return()

        replan_interval = float(self.get_parameter('replan_interval_s').value)

        # 无目标时也遵守重规划间隔；否则 steady-clock 10 Hz 控制会每帧重复
        # 聚类整张地图并刷屏“All frontiers visited”，挤占传感器与控制回调。
        if now - self._last_replan_time > replan_interval:
            self._replan()
            self._last_replan_time = now

        # 无前沿 → 探索完成，返航
        if self.current_target is None and len(self.current_path) == 0:
            # 只剩区域外前沿时不做完成判定：replan 已在内部短路 keep，这里
            # 只是兜底再确认一次（map 更新期间 replan 可能因 grid 尚未刷新
            # 而提前清目标，此时应等待而不是把门外前沿误判成探索完成）。
            if self._all_frontiers_outside_region_only():
                self._no_reachable_frontier_since = None
                return self._hold_scan_rotation(cmd)
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
                # 多层模式：本层完成必须覆盖达标，否则说明只是前沿暂时
                # 不可达，应重置前沿搜索继续探索未覆盖区域，而非提前跨层。
                if self._floor_is_covered():
                    self._transition('FLOOR_COMPLETE')
                else:
                    self.get_logger().warn(
                        'Frontiers exhausted but floor coverage below '
                        'threshold; resetting frontier search to continue.')
                    self._unreachable_frontiers.clear()
                    self._detour_deferred_frontiers.clear()
                    self._no_reachable_frontier_since = None
                    self._reset_frontier_progress_watchdog()
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

    def _should_enter_room(self, frontier: Frontier) -> bool:
        """判定前沿是否值得尝试进入房间探索。

        room 型前沿直接进入；corridor 型但窄边 >= room_trigger_min_door_m、
        aspect <= room_trigger_max_aspect 的（很可能是被 bbox 纵横比判低的
        房间门口条带）也尝试进入——enter 阶段用连续受阻快速失败兜底。
        """
        ftype = getattr(frontier, 'frontier_type', '')
        if ftype == 'room':
            return True
        if (self.latest_map is None
                or self.latest_map.info.resolution <= 0.0):
            return False
        aspect = getattr(frontier, 'aspect_ratio', 0.0) or 0.0
        narrow_cells = getattr(frontier, 'door_width_cells', 0) or 0
        narrow_m = narrow_cells * self.latest_map.info.resolution
        return (0.0 < aspect <= float(self.get_parameter(
                    'room_trigger_max_aspect').value)
                and narrow_m >= float(self.get_parameter(
                    'room_trigger_min_door_m').value))

    def _start_room_sweep(self, frontier: Frontier):
        """启动房间完整探索 enter -> explore -> exit 序列。"""
        self._room_sweep_active = True
        self._room_sweep_frontier = frontier
        self._room_sweep_phase = 'enter'
        self._room_sweep_start_time = self._ros_time_sec()
        self._room_sweep_entry_pose = (self.robot_x, self.robot_y)
        self._room_sweep_yaw_start = self.robot_yaw
        self._room_enter_blocked_since_ros = None
        self._room_internal_target = None
        self._room_internal_path = []
        self._room_internal_path_i = 0
        self._room_exit_started_ros = None
        self._room_exit_path = []
        self._room_exit_path_i = 0
        self._room_spin_last_yaw = None
        self._room_spin_accum = 0.0
        self._room_no_target_spins = 0
        key = self._frontier_key(frontier)
        self.get_logger().info(
            f'Room sweep START for frontier {key} '
            f'at ({frontier.centroid[0]:.1f},{frontier.centroid[1]:.1f}) '
            f'type={frontier.frontier_type} aspect={frontier.aspect_ratio}'
        )

    def _room_refresh_frontiers(self, wall_now: float):
        """房间探索期间刷新前沿。

        _handle_exploring 因提前 return 不刷新 self.frontiers，房间内部走深后
        会产生新的内部前沿，必须在本状态机内按 replan_interval_s 主动重聚类。
        """
        if self.grid is None or self.latest_map is None:
            return
        replan_interval = max(1.0, float(self.get_parameter(
            'replan_interval_s').value))
        if wall_now - self._last_replan_time <= replan_interval:
            return
        frontier_mask = find_frontiers(self.grid)
        self.frontiers = cluster_frontiers(
            frontier_mask, self.grid, self.latest_map,
            classify=True, resolution_m=0.05)
        self._last_replan_time = wall_now

    def _room_coverage_ratio(self) -> Optional[float]:
        """房间半幅矩形内 已访问自由格 / 已知自由格 比例。

        以入口为中心 room_half_extent_m 方形区域，自由格与 robot 走过半径
        (CoverageGrid) 求交集；数据不足（无地图/入口）返回 None。
        """
        if (self._coverage is None or self.grid is None
                or self.latest_map is None
                or self._room_sweep_entry_pose is None):
            return None
        half = max(1.0, float(self.get_parameter(
            'room_half_extent_m').value))
        res = self.latest_map.info.resolution
        if res <= 0.0:
            return None
        origin = self.latest_map.info.origin
        gx = int((self._room_sweep_entry_pose[0]
                  - origin.position.x) / res)
        gy = int((self._room_sweep_entry_pose[1]
                  - origin.position.y) / res)
        r = max(1, int(round(half / res)))
        h, w = self.grid.shape
        x0, x1 = max(0, gx - r), min(w, gx + r + 1)
        y0, y1 = max(0, gy - r), min(h, gy + r + 1)
        if x1 <= x0 or y1 <= y0:
            return None
        patch = self.grid[y0:y1, x0:x1]
        patch_free = (patch >= 0) & (patch <= FREE_MAX)
        total = int(patch_free.sum())
        if total == 0:
            return None
        covered = int((self._coverage.grid[y0:y1, x0:x1]
                       & patch_free).sum())
        return covered / float(total)

    def _room_internal_frontiers(self) -> List[Frontier]:
        """房间半幅范围内 未访问、未不可达 的前沿，按到机器人距离排序。"""
        if self._room_sweep_entry_pose is None:
            return []
        half = max(1.0, float(self.get_parameter(
            'room_half_extent_m').value))
        radius_sq = half * half
        now_ros = self._ros_time_sec()
        ex, ey = self._room_sweep_entry_pose
        candidates = []
        for f in self.frontiers:
            key = self._frontier_key(f)
            if key in self._visited_frontiers:
                continue
            if self._frontier_is_unreachable(f, now_ros):
                continue
            dx = f.centroid[0] - ex
            dy = f.centroid[1] - ey
            if dx * dx + dy * dy <= radius_sq:
                candidates.append(f)
        candidates.sort(key=lambda f: (
            (f.centroid[0] - self.robot_x) ** 2
            + (f.centroid[1] - self.robot_y) ** 2))
        return candidates

    def _room_drive_to(self, wx: float, wy: float,
                       cmd: Twist, tol_m: float) -> bool:
        """车头引导移动至世界点，受激光门禁约束。返回 True 表示已到达。"""
        dist = math.hypot(wx - self.robot_x, wy - self.robot_y)
        if dist <= tol_m:
            return True
        target_yaw = math.atan2(wy - self.robot_y, wx - self.robot_x)
        heading_error = normalize_angle(target_yaw - self.robot_yaw)
        if abs(heading_error) > float(self.get_parameter(
                'heading_tolerance_rad').value):
            action = ('turn_left' if heading_error > 0.0
                      else 'turn_right')
            if self._scan_allows_action(
                    action,
                    float(self.get_parameter(
                        'rotation_min_clearance_m').value)):
                cmd.angular.z = max(
                    -float(self.get_parameter('angular_speed').value),
                    min(float(self.get_parameter('angular_speed').value),
                        heading_error))
            return False
        if self._scan_allows_action(
                'move_forward',
                float(self.get_parameter(
                    'navigation_min_clearance_m').value)):
            cmd.linear.x = min(
                float(self.get_parameter('linear_speed').value), dist)
        return False

    def _room_spin_in_place(self, cmd: Twist, turn_speed: float,
                            wall_now: float) -> bool:
        """原地左转一圈（累计角增量归一 340°）。转完返回 True。"""
        prev = getattr(self, '_room_spin_last_yaw', None)
        if prev is None:
            self._room_spin_last_yaw = self.robot_yaw
            self._room_spin_accum = 0.0
        else:
            self._room_spin_accum = getattr(
                self, '_room_spin_accum', 0.0) + abs(normalize_angle(
                    self.robot_yaw - prev))
        self._room_spin_last_yaw = self.robot_yaw
        if self._room_spin_accum >= math.radians(340):
            self._room_spin_last_yaw = None
            self._room_spin_accum = 0.0
            return True
        if self._scan_allows_action(
                'turn_left',
                float(self.get_parameter(
                    'rotation_min_clearance_m').value)):
            cmd.angular.z = turn_speed
        return False

    def _room_reached_internal_target(self):
        """当前房内目标已环视完毕：标记 visited、清空回到选目标环节。"""
        target = self._room_internal_target
        if target is not None:
            key = self._frontier_key(target)
            self._visited_frontiers.add(key)
            self._clear_unreachable_frontier_basin(target)
            self.get_logger().info(
                'In-room frontier reached & swept, marked visited: '
                f'{key}')
        self._room_internal_target = None
        self._room_internal_path = []
        self._room_internal_path_i = 0
        self._room_spin_last_yaw = None
        self._room_spin_accum = 0.0

    def _room_begin_exit(self):
        """切入 exit 阶段（先清空内部推进状态）。"""
        if self._room_sweep_phase != 'exit':
            self._room_sweep_phase = 'exit'
            self._room_exit_path = []
            self._room_exit_path_i = 0
            self._room_internal_target = None
            self._room_internal_path = []
            self._room_internal_path_i = 0
            self._room_spin_last_yaw = None
            self._room_spin_accum = 0.0
            self.get_logger().info('Room explore complete; exiting room.')

    def _handle_room_sweep(self, now_ros: float) -> Twist:
        """Phase 3: 房间完整探索 enter -> explore(走深+覆盖) -> exit。

        enter:   沿进入朝向直行 room_entry_depth_m，走满视为进入房间；
                 期间连续受阻 room_enter_fail_s 则快速失败回到正常探索。
        explore: 主动刷新房内前沿，A*+车头引导走入房间内部并原地环视，
                 直到覆盖 >= room_coverage_threshold 或 explore 超时。
        exit:    A* 回入口附近（无路则直视逼近，激光门禁兜底），
                 完成并清扫房内前沿。
        """
        cmd = Twist()
        if not self._room_sweep_active:
            return cmd
        wall_now = time.monotonic()
        elapsed = now_ros - (self._room_sweep_start_time or now_ros)
        entry_depth = max(0.5, float(self.get_parameter(
            'room_entry_depth_m').value))
        sweep_timeout = max(5.0, float(self.get_parameter(
            'room_sweep_timeout_s').value))
        explore_timeout = max(5.0, float(self.get_parameter(
            'room_explore_timeout_s').value))
        cover_threshold = max(
            0.1, min(0.99, float(self.get_parameter(
                'room_coverage_threshold').value)))
        turn_speed = min(
            abs(float(self.get_parameter('room_scan_turn_speed').value)),
            abs(float(self.get_parameter('angular_speed').value)),
        )
        entry_tol = max(0.3, float(self.get_parameter(
            'goal_tolerance_m').value))

        # ---- enter ----
        if self._room_sweep_phase == 'enter':
            dist_entered = math.hypot(
                self.robot_x - self._room_sweep_entry_pose[0],
                self.robot_y - self._room_sweep_entry_pose[1])
            if dist_entered < entry_depth:
                if self._scan_allows_action(
                        'move_forward',
                        float(self.get_parameter(
                            'rotation_min_clearance_m').value)):
                    cmd.linear.x = float(self.get_parameter(
                        'linear_speed').value)
                    self._room_enter_blocked_since_ros = None
                elif self._room_enter_blocked_since_ros is None:
                    self._room_enter_blocked_since_ros = now_ros
                elif (now_ros - self._room_enter_blocked_since_ros
                      >= float(self.get_parameter(
                          'room_enter_fail_s').value)):
                    # 连续受阻：多半是走廊尽头 / 墙边条带被误判为房间门口，
                    # 快速失败，作为普通前沿处理回到正常探索。
                    self.get_logger().info(
                        'Room entry blocked; treating frontier as '
                        'non-room, back to normal exploration.')
                    self._finish_room_sweep()
                return cmd
            # 走满 entry_depth -> 判定进入房间
            self._room_sweep_phase = 'explore'
            self._room_internal_target = None
            self._room_internal_path = []
            self._room_internal_path_i = 0
            self.get_logger().info(
                f'Room ENTER done ({dist_entered:.1f}m); '
                'exploring interior.')
            return cmd

        # ---- explore ----
        if self._room_sweep_phase == 'explore':
            self._room_refresh_frontiers(wall_now)
            ratio = self._room_coverage_ratio()
            if ratio is not None and ratio >= cover_threshold:
                self.get_logger().info(
                    f'Room coverage {ratio * 100:.0f}% >= '
                    f'{cover_threshold * 100:.0f}%; exiting.')
                self._room_begin_exit()
                return cmd
            if elapsed >= explore_timeout:
                self.get_logger().warn(
                    f'Room explore timeout after {elapsed:.1f}s; exiting.')
                self._room_begin_exit()
                return cmd

            # 选房内目标
            if self._room_internal_target is None:
                targets = self._room_internal_frontiers()
                if targets:
                    self._room_internal_target = targets[0]
                    self._room_internal_path = []
                    self._room_internal_path_i = 0
                    t = self._room_internal_target
                    self.get_logger().info(
                        f'In-room target '
                        f'({t.centroid[0]:.1f},{t.centroid[1]:.1f}) '
                        f'type={t.frontier_type} aspect={t.aspect_ratio}')
                    return cmd
                # 暂无内部前沿：原地环视（可能发现新内部前沿）；两圈仍无 ->
                # 覆盖判定与超时在阶段顶兜底，这里也给出强制退出保险。
                if self._room_spin_in_place(cmd, turn_speed, wall_now):
                    spins = getattr(self, '_room_no_target_spins', 0) + 1
                    self._room_no_target_spins = spins
                    if spins >= 2:
                        self.get_logger().info(
                            'No in-room frontiers after 2 spins; '
                            'exiting.')
                        self._room_begin_exit()
                return cmd

            # 前往房内目标（A* 路径 + 车头引导 + 到点环视）
            target = self._room_internal_target
            if not self._room_internal_path:
                path = a_star_path(
                    self.grid, self.latest_map,
                    self.robot_x, self.robot_y,
                    target.centroid[0], target.centroid[1],
                    inflation_radius_m=0.45)
                if path:
                    self._room_internal_path = path
                    self._room_internal_path_i = 0
            if self._room_internal_path:
                while (self._room_internal_path_i
                       < len(self._room_internal_path)):
                    wx, wy = self._room_internal_path[
                        self._room_internal_path_i]
                    if self._room_drive_to(wx, wy, cmd, 0.3):
                        self._room_internal_path_i += 1
                    else:
                        break
            dist_target = math.hypot(
                target.centroid[0] - self.robot_x,
                target.centroid[1] - self.robot_y)
            if dist_target <= entry_tol * 2.5:
                # 贴近目标：原地环视一圈采集房间，转完标记 visited
                if self._room_spin_in_place(cmd, turn_speed, wall_now):
                    self._room_reached_internal_target()
                return cmd
            if not self._room_internal_path:
                # A* 无路（目标在未知深处）：直视逼近，激光门禁兜底
                self._room_drive_to(
                    target.centroid[0], target.centroid[1],
                    cmd, entry_tol * 2.5)
            return cmd

        # ---- exit ----
        if self._room_sweep_phase == 'exit':
            if self._room_exit_started_ros is None:
                self._room_exit_started_ros = now_ros
            dist_entry = math.hypot(
                self.robot_x - self._room_sweep_entry_pose[0],
                self.robot_y - self._room_sweep_entry_pose[1])
            if dist_entry <= entry_tol:
                self._finish_room_sweep()
                return cmd
            if not self._room_exit_path:
                path = a_star_path(
                    self.grid, self.latest_map,
                    self.robot_x, self.robot_y,
                    self._room_sweep_entry_pose[0],
                    self._room_sweep_entry_pose[1],
                    inflation_radius_m=0.45)
                if path:
                    self._room_exit_path = path
                    self._room_exit_path_i = 0
            if self._room_exit_path:
                while (self._room_exit_path_i
                       < len(self._room_exit_path)):
                    wx, wy = self._room_exit_path[
                        self._room_exit_path_i]
                    if self._room_drive_to(wx, wy, cmd, 0.3):
                        self._room_exit_path_i += 1
                    else:
                        break
                if self._room_exit_path_i >= len(self._room_exit_path):
                    self._room_exit_path = []
            else:
                # 无路可回：直视入口逼近，激光门禁兜底
                self._room_drive_to(
                    self._room_sweep_entry_pose[0],
                    self._room_sweep_entry_pose[1], cmd, entry_tol)
            if (now_ros - self._room_exit_started_ros
                    > sweep_timeout):
                self.get_logger().warn(
                    'Room exit timeout; forcing completion.')
                self._finish_room_sweep()
            return cmd

        # 兜底
        if elapsed > sweep_timeout:
            self.get_logger().warn(
                f'Room sweep overall timeout ({elapsed:.1f}s); '
                'forcing completion.')
            self._finish_room_sweep()
        return cmd

    def _finish_room_sweep(self):
        """完成房间探索：清扫房内（半幅矩形内）前沿为 visited，重置状态。"""
        if self._room_sweep_entry_pose is not None:
            half = max(1.0, float(self.get_parameter(
                'room_half_extent_m').value))
            radius_sq = half * half
            ex, ey = self._room_sweep_entry_pose
            swept = 0
            for f in self.frontiers:
                dx = f.centroid[0] - ex
                dy = f.centroid[1] - ey
                if dx * dx + dy * dy <= radius_sq:
                    key = self._frontier_key(f)
                    self._visited_frontiers.add(key)
                    self._clear_unreachable_frontier_basin(f)
                    swept += 1
            self.get_logger().info(
                f'Room sweep COMPLETE; {swept} in-room frontiers '
                'marked visited.')
        if self._room_sweep_frontier is not None:
            key = self._frontier_key(self._room_sweep_frontier)
            self._visited_frontiers.add(key)
            self._clear_unreachable_frontier_basin(
                self._room_sweep_frontier,
            )
            self.get_logger().info(
                f'Room sweep COMPLETE: {key} marked visited')
        self._room_sweep_active = False
        self._room_sweep_frontier = None
        self._room_sweep_phase = ''
        self._room_sweep_start_time = None
        self._room_sweep_entry_pose = None
        self._room_sweep_yaw_start = None
        self._room_enter_blocked_since_ros = None
        self._room_internal_target = None
        self._room_internal_path = []
        self._room_internal_path_i = 0
        self._room_exit_started_ros = None
        self._room_exit_path = []
        self._room_exit_path_i = 0
        self._room_spin_last_yaw = None
        self._room_spin_accum = 0.0
        self._room_no_target_spins = 0

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

    def _distance_home_m(self) -> float:
        """返航物理距离：优先 world 帧（/hw/odom 真值与 INIT 捕获的 world 家）。

        map 帧距离随 SLAM 重锚漂移失真（run_20260820_171917 实锤：返航预算
        与 drifting-away 看门狗都被 MAP 帧 17m 假距离带偏）。world 帧两点的
        差值不受重锚影响；无 world 参考时回退 map 帧，行为同旧版。
        """

        if (self._home_world is not None
                and self._world_x is not None and self._world_y is not None):
            return math.hypot(
                self._world_x - self._home_world[0],
                self._world_y - self._home_world[1],
            )
        return math.hypot(self.robot_x - self.start_x,
                          self.robot_y - self.start_y)

    def _handle_returning(self) -> Twist:
        """RETURNING: A* 返航到起点。"""
        cmd = Twist()
        goal_tol = float(self.get_parameter('goal_tolerance_m').value)
        now = self._ros_time_sec()

        # 返航目标以 world 帧家为准：map 重锚会让 INIT 捕获的 map 帧 start 漂移
        # 失效（run_20260820_171917 实锤：RETURNING 卡死在失效目标上打转）。
        # 物理距离用 world 帧（/hw/odom），A* 目标用当前 map↔world 变换把
        # world 家重投影回当前 map 帧，每次规划重取，跟随物理出发点而非旧帧坐标。
        if self._home_world is not None:
            hx, hy = self._region_world_to_map(
                self._home_world[0], self._home_world[1])
        else:
            hx, hy = self.start_x, self.start_y
        dist_home = self._distance_home_m()

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

        # 近距离直线返航：home 附近（起点大厅）空旷，A* 的起点吸附会被
        # 近场回波/SLAM 漂移引向身后幽灵栅格，机器人在 home 旁绕圈却迟迟
        # 不 FINISHED。dist_home 小于阈值时直接对准 home 直线前进（复用
        # _follow_path 的朝向 + scan 门禁）；若直线方向被真实障碍挡住，
        # 静止看门狗超时会把 force_replan 置位，自然回退到下方 A* 绕行。
        straight_dist = float(
            self.get_parameter('return_straight_distance_m').value)
        if not force_replan and dist_home <= straight_dist:
            self.current_path = [(hx, hy)]
            self.path_index = 0
            return self._follow_path()

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
                hx, hy,
                start_search_radius_m=0.50,
                # home 栅格会被机身近场回波或门口门板标成占用；若吸附半径为
                # 0，_nearest_traversable_cell 会直接返回 None，A* 空路径导致
                # 返航永久卡死。给一个 goal_tol 量级的小吸附半径（0.30 m）让
                # 端点吸附到 home 附近最近自由格，再由 append_exact_goal 追加
                # 真实 home 坐标、FINISHED 仍用真实 dist_home ≤ goal_tol 判定，
                # 既不会卡死也不会假完成。
                goal_search_radius_m=0.30,
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
        # 电梯坐标注入：YAML dict 字符串，键为楼层 int，值为 [x, y] map 帧
        # 坐标。ROS2 参数无 dict 类型，故字符串传递、此处解析成 dict。
        try:
            raw_positions = self.get_parameter('elevator_positions').value
            if isinstance(raw_positions, str) and raw_positions.strip():
                parsed = yaml.safe_load(raw_positions)
                if isinstance(parsed, dict):
                    self._elevator_positions = {
                        int(k): (float(v[0]), float(v[1]))
                        for k, v in parsed.items()
                    }
                else:
                    self._elevator_positions = {}
            else:
                self._elevator_positions = {}
        except (TypeError, ValueError, IndexError, yaml.YAMLError):
            self._elevator_positions = {}
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
        """判断当前楼层覆盖是否达标。

        覆盖网格不可用（CoverageGrid 未建成或地图未就绪）时降级为 True，
        只按前沿耗尽判定完成，避免多层模式因覆盖网格缺失而永久卡死。
        """
        if self._target_floors is None or len(self._target_floors) == 0:
            return False
        if self._coverage is None or self.grid is None:
            return True
        threshold = float(
            self.get_parameter('floor_coverage_threshold').value)
        ratio = self._coverage.floor_coverage_ratio(self.grid)
        return ratio >= threshold

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
        # 保存刚完成的楼层。跨层 navigating/calling 阶段机器人仍停留在旧楼层，
        # 必须基于旧楼层取电梯入口坐标、把电梯叫到旧楼层，不能提前覆盖成
        # 目标楼层（否则会导航到错误楼层的电梯坐标）。
        from_floor = self._current_floor
        self._floor_transition_from_floor = from_floor
        self._current_floor = next_floor
        self._floor_complete_since_ros = None
        self._elevator_initiated = False
        self._elevator_floor_reached = False
        self._floor_transition_phase = 'navigating'
        self._floor_transition_start_ros = now_ros
        self.recorder.record_floor_change(
            now_ros, from_floor, next_floor, 'elevator')
        self.get_logger().info(
            f'Floor {from_floor} complete. '
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
        retry_interval = float(
            self.get_parameter('elevator_retry_interval_s').value)
        # 旧楼层：navigating/calling 阶段机器人仍停留在该层，必须用旧楼层取
        # 电梯入口坐标、把电梯叫到旧楼层。目标楼层是 _current_floor（已被
        # _handle_floor_complete 更新为 next_floor），只用于 entering 阶段。
        from_floor = (
            self._floor_transition_from_floor
            if self._floor_transition_from_floor is not None
            else self._current_floor
        )
        target_floor = self._current_floor

        if self._floor_transition_phase == 'navigating':
            elevator_pos = self._elevator_positions.get(from_floor)
            if elevator_pos is None:
                self.get_logger().error(
                    f'No elevator position configured for floor '
                    f'{from_floor} (map frame); cannot navigate to '
                    f'elevator. Aborting multi-floor transition.')
                self._transition('RETURNING')
                return cmd
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
            self._last_elevator_call_ros = None
            self.get_logger().info('Arrived at elevator. Calling...')

        if self._floor_transition_phase == 'calling':
            if not self._elevator_initiated:
                # 退避：失败后至少隔 retry_interval 秒再重试，避免在 10 Hz
                # 控制循环内高频同步阻塞 docker exec 拖死控制心跳。
                if (self._last_elevator_call_ros is None
                        or now_ros - self._last_elevator_call_ros
                        >= retry_interval):
                    self._last_elevator_call_ros = now_ros
                    try:
                        result = call_elevator(
                            container, elevator_id, from_floor,
                            open_doors=True, timeout_s=30.0,
                        )
                        self.recorder.record_elevator_call(
                            now_ros, elevator_id, from_floor,
                            'called', result.state)
                        if result.accepted:
                            self._elevator_initiated = True
                            self.get_logger().info(
                                f'Elevator called to floor {from_floor}: '
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
                self._last_elevator_call_ros = None

        if self._floor_transition_phase == 'entering':
            if not self._elevator_floor_reached:
                if (self._last_elevator_call_ros is None
                        or now_ros - self._last_elevator_call_ros
                        >= retry_interval):
                    self._last_elevator_call_ros = now_ros
                    try:
                        result = call_elevator(
                            container, elevator_id, target_floor,
                            open_doors=True, timeout_s=30.0,
                        )
                        self.recorder.record_elevator_call(
                            now_ros, elevator_id, target_floor,
                            'entered', result.state)
                        if (result.accepted
                                and result.current_floor == target_floor):
                            self._elevator_floor_reached = True
                            self.get_logger().info(
                                f'Arrived at floor {target_floor}')
                    except Exception as exc:
                        self.get_logger().error(
                            f'Floor transition failed: {exc}')

        if self._elevator_floor_reached or (
                self._floor_transition_start_ros is not None
                and now_ros - self._floor_transition_start_ros > 60.0):
            # 进入新层探索前必须重置 SLAM 地图（发布 floor_index）。正常到达时
            # 也在此统一发布；60s 超时逃生路径同样兜底发布，避免用旧楼层地图
            # 探索新楼层。
            self._publish_floor_index(target_floor)
            self.get_logger().info(
                f'Beginning exploration on floor {target_floor}')
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
            self._floor_transition_from_floor = None
            self._last_elevator_call_ros = None
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

    # ---- 探索区域约束（世界系矩形） ----

    def _region_enabled(self) -> bool:
        """世界系矩形已配置（四角非全 0）时启用；无世界真值参考时
        退化为不限制（保持通用环境向后兼容）。"""

        configured = (
            float(self.get_parameter('region_x_min').value) != 0.0
            or float(self.get_parameter('region_x_max').value) != 0.0
            or float(self.get_parameter('region_y_min').value) != 0.0
            or float(self.get_parameter('region_y_max').value) != 0.0
        )
        if not configured:
            return False
        return self._world_x is not None

    def _region_bounds(self) -> Tuple[float, float, float, float]:
        return (
            float(self.get_parameter('region_x_min').value),
            float(self.get_parameter('region_x_max').value),
            float(self.get_parameter('region_y_min').value),
            float(self.get_parameter('region_y_max').value),
        )

    def _point_in_region_world(self, wx: float, wy: float) -> bool:
        x_min, x_max, y_min, y_max = self._region_bounds()
        return x_min <= wx <= x_max and y_min <= wy <= y_max

    def _robot_in_region_world(self) -> bool:
        if self._world_x is None:
            return True  # 无世界参考时不限制
        return self._point_in_region_world(self._world_x, self._world_y)

    def _frontier_in_region(self, frontier: Frontier) -> bool:
        """把 frontier 质心从 map 帧转到世界系后做矩形检查。

        frontier 质心在 map 帧（grid_to_world 产自 /map origin）。世界真值
        /hw/odom 给出 robot 在世界系位姿，而 tf map→base 给出同一时刻
        robot 在 map 帧位姿。两者的差 + 朝向差构成 map↔world 的 2D 刚性
        变换，把质心转到世界系再判断。map 帧回环/漂移只影响该变换的瞬时
        精度（亚米级），对矩形边界判断可接受。
        """

        if self._world_x is None:
            return True  # 无世界参考时不限制
        x, y = frontier.centroid
        if self._region_map_to_world_transform() is None:
            return True
        wx, wy = self._region_map_to_world(x, y)
        return self._point_in_region_world(wx, wy)

    def _all_frontiers_outside_region_only(self) -> bool:
        """region 启用且当前地图只剩区域外前沿 → True。

        用于 completion 块区分“真正全部访问完”与“仅剩门外前沿（应保持
        扫描等地图更新，而非宣告探索完成返回）”。前沿列表为空时不在此
        列：那是真正扫完了。区域未启用时恒 False，保持旧行为。
        """

        if not self._region_enabled() or not self.frontiers:
            return False
        result = True
        for f in self.frontiers:
            if self._frontier_in_region(f):
                result = False
                break
        return result

    def _region_map_to_world_transform(self):
        """返回 (yaw_diff, sin, cos, dx, dy) 组成的 map→world 变换。

        map 帧随 SLAM 重锚/重定位会瞬间大幅旋转平移（一次可跳 90°+），
        localizer odom→base 在停车时也会缓慢漂移。若当前变换相对上一次
        last-good 变化过大（yaw 跳 >0.6 rad 或平移跳 >2.0 m），说明 map↔world
        此刻不一致（tf 树与 /hw/odom 来自不同漂移状态），进入 10s hold：期内
        返回 None，调用方视为“区内/不抑制”，靠机器人侧 region-return 兜底
        防出区；hold 结束后重新评估，map 稳定后自动接受新变换并继续过滤。
        """

        if self._world_x is None:
            return None
        # robot map 帧位姿与机器人世界系位姿是同一物理点，两者差构成了
        # 平移；世界朝向与 map 朝向之差构成旋转。
        yaw_diff = self._world_yaw - self.robot_yaw
        s = math.sin(yaw_diff)
        c = math.cos(yaw_diff)
        dx = self._world_x - (c * self.robot_x - s * self.robot_y)
        dy = self._world_y - (s * self.robot_x + c * self.robot_y)

        if self._region_transform_hold_until is not None:
            if time.monotonic() < self._region_transform_hold_until:
                return None  # hold 期内：视为区内，不抑制前沿
            self._region_transform_hold_until = None  # hold 结束，重新评估

        last = self._region_last_transform
        if last is not None:
            yaw_jump = abs(
                ((yaw_diff - last[0] + math.pi) % (2 * math.pi)) - math.pi)
            dpos_jump = math.hypot(dx - last[3], dy - last[4])
            if yaw_jump > 0.6 or dpos_jump > 2.0:
                self._region_transform_hold_until = time.monotonic() + 10.0
                self.get_logger().warn(
                    'Region map->world transform unstable '
                    f'(yaw_delta={math.degrees(yaw_jump):.1f} deg, '
                    f'pos_delta={dpos_jump:.2f} m); holding region filter '
                    '10s, treating frontiers as in-region.')
                return None

        self._region_transform_hold_until = None
        self._region_last_transform = (yaw_diff, s, c, dx, dy)
        return (yaw_diff, s, c, dx, dy)

    def _region_map_to_world(self, mx: float, my: float):
        t = self._region_map_to_world_transform()
        if t is None:
            lt = self._region_last_transform
            if lt is None:
                return (mx, my)
            _, s, c, dx, dy = lt
        else:
            _, s, c, dx, dy = t
        return (c * mx - s * my + dx, s * mx + c * my + dy)

    def _region_world_to_map(self, wx: float, wy: float):
        """世界系点转 map 帧（区域外回归目标的逆变换）。

        transform 在跳变 hold 期可能为 None；此时用 last-good 兜底（最多
        是短暂的过时对齐，目标仍大致在机器人附近，距离门禁会兜住）。
        """

        t = self._region_map_to_world_transform()
        if t is None:
            lt = self._region_last_transform
            if lt is None:
                return (wx, wy)
            _, s, c, dx, dy = lt
        else:
            _, s, c, dx, dy = t
        # 逆旋转并减平移
        rx = wx - dx
        ry = wy - dy
        return (c * rx + s * ry, -s * rx + c * ry)

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

        self.frontiers = cluster_frontiers(frontier_mask, self.grid, self.latest_map, classify=True, resolution_m=0.05)

        # 过滤已访问的前沿（区域之外的前沿视为不可选择，但不解成完成）
        min_size = int(self.get_parameter('min_frontier_size').value)
        region_enabled = self._region_enabled()
        suppressed_outside = 0
        unvisited_frontiers = []
        for f in self.frontiers:
            key = self._frontier_key(f)
            if (key not in self._visited_frontiers
                    and not self._frontier_is_unreachable(f, now_ros)):
                if region_enabled and not self._frontier_in_region(f):
                    suppressed_outside += 1
                    continue
                unvisited_frontiers.append(f)

        if not unvisited_frontiers:
            if region_enabled and suppressed_outside > 0:
                # 本层只剩区域外前沿：不宣告完成。若已在朝向一个合法目标的
                # 路径上则保持，否则原地等待下一次地图更新继续扫描。
                self.get_logger().info(
                    f'All {suppressed_outside} remaining frontiers are '
                    'outside the exploration region; holding for map update.'
                )
                if self.current_target is not None and len(self.current_path) > 0:
                    return
                self.current_target = None
                self.current_path = []
                return
            # 全部访问过 → 探索完成
            self.current_target = None
            self.current_path = []
            self.get_logger().info('All frontiers visited.')
            return
        # 房间/走廊分类统计        room_frontiers = [f for f in unvisited_frontiers if getattr(f, "frontier_type", "") == "room"]        corridor_frontiers = [f for f in unvisited_frontiers if getattr(f, "frontier_type", "") == "corridor"]        self.get_logger().info(            f"Frontiers: {len(unvisited_frontiers)} total "            f"({len(room_frontiers)} room, {len(corridor_frontiers)} corridor)")

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
                    room_boost=float(self.get_parameter("room_boost_score").value),
                    corridor_penalty=float(self.get_parameter("corridor_penalty_score").value),
                )
                selected_ingress_half_angle = half_angle
                if selected is not None:
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
                # 区域回区合成目标：到达即完成，不视为真实前沿。
                if (getattr(self.current_target, 'frontier_type', '')
                        == 'region_return'):
                    self.current_path = []
                    self.current_target = None
                    self._reset_frontier_progress_watchdog()
                    self.get_logger().info('Region return target reached.')
                    return cmd
                key = self._frontier_key(self.current_target)
                # Phase 3: 门口候选不直接标记已访问，进入 enter->explore->exit；
                # 宽触发（room 型 + 窄边>=1m 的 corridor 型），走廊/墙边误判由
                # enter 阶段连续受阻快速失败兜底，成本仅数秒。
                if (self._should_enter_room(self.current_target)
                        and not self._room_sweep_active):
                    self._start_room_sweep(self.current_target)
                    self.current_path = []
                    self.current_target = None
                    self._reset_frontier_progress_watchdog()
                    return cmd
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
            self._return_best_distance_home = self._distance_home_m()
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
