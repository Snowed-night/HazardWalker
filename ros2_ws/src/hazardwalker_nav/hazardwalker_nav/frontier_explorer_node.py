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
import heapq
import math
import time
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from typing import List, Optional, Tuple

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from hazardwalker_nav.frontier_detector import (
    Frontier,
    a_star_path,
    append_loop_erased_history,
    body_tilt_degrees_from_quaternion,
    build_reverse_history_path,
    build_counterclockwise_room_loop,
    cluster_frontiers,
    compute_frontier_backoff_ttl_s,
    compute_exploration_time_limit_s,
    detect_opened_door_from_scans,
    entry_axis_progress_m,
    entry_ingress_constraint_active,
    entry_ingress_half_angles_deg,
    find_frontiers,
    frontier_route_is_excessive_detour,
    grid_to_world,
    interpolate_path_lookahead,
    nearest_frontier_basin_key,
    occupancy_grid_to_array,
    polygon_signed_area,
    physical_room_loop_is_valid,
    return_recovery_turn_command,
    return_pose_has_progress,
    corridor_room_sector,
    prioritize_unvisited_room_frontiers,
    select_best_frontier,
    select_cached_missing_room_doorway,
    select_symmetric_doorway_stations,
    scaled_room_waypoint_candidates,
    simulation_period_elapsed,
    should_switch_frontier,
    transform_planar_point,
    transform_planar_goal_to_robot_frame,
    world_to_grid,
    OCCUPIED,
    FREE_MAX,
)
from hazardwalker_nav.reobservation_contract import (
    action_has_scan_clearance,
    bearing_change_deg,
    bounded_planar_pose_increment,
    find_target_detection,
    find_target_status,
    lateral_centering_angular_velocity,
    live_reobservation_action_update_allowed,
    parse_reobservation_request,
    reobservation_actions_conflict,
    reobservation_request_is_eligible,
    select_live_reobservation_update,
    target_centered_in_image,
    target_horizontal_error_ratio,
)
from hazardwalker_nav.coverage_tracker import CoverageGrid
from hazardwalker_nav.elevator_controller import (
    ElevatorResult,
    call_elevator,
    elevator_approach_position,
    elevator_door_id,
    set_door_state,
    set_robot_floor,
)
from hazardwalker_nav.nav_recorder import NavRecorder
from hazardwalker_nav.room_inspection_planner import (
    RoomInspectionExecution,
    build_strict_room_inspection_plan,
)


# 首层门洞位置只允许通过公开里程计近似；平台载荷拒绝后按由近到远的
# 锯齿序列沿墙搜索门洞，避免把单个固定坐标当成环境真值。
ELEVATOR_ENTRY_ALIGNMENT_OFFSETS = (
    0.0, -0.45, 0.45, -0.90, 0.90,
    -1.35, 1.35, -1.80, 1.80, 2.25,
)
from hazardwalker_nav.waypoint_controller import normalize_angle


class FrontierExplorerNode(Node):
    """Frontier 探索节点——自主覆盖楼层、复查候选、返航。"""

    def __init__(self):
        super().__init__('frontier_explorer_node')

        # ---- 参数 ----
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        # 导航只能写入独立输入源，由 command_mux 唯一发布 /hw/cmd_vel。
        self.declare_parameter(
            'cmd_vel_topic', '/hw/control/navigation_cmd_vel')
        # command_mux 默认键盘模式；启动期显式请求 navigation 并短时重试，
        # 防止 DDS 尚未完成匹配时第一条模式请求丢失。
        self.declare_parameter(
            'control_mode_request_topic', '/hw/control/mode_request')
        self.declare_parameter('control_mode_value', 'navigation')
        # 局部运动默认保持既有直接控制。官方 SimEnv profile 显式选择
        # unitree_move_base 后，Frontier 只发布 odom 目标，门框/障碍规避由赛事
        # 仓库随附的宇树 move_base + TrajectoryPlannerROS(DWA) 完成。
        self.declare_parameter('local_planner_backend', 'direct')
        self.declare_parameter(
            'unitree_move_base_goal_topic',
            '/hw/navigation/unitree_move_base_goal')
        self.declare_parameter(
            'unitree_move_base_control_topic',
            '/hw/navigation/unitree_move_base_control')
        self.declare_parameter(
            'unitree_move_base_cmd_topic',
            '/hw/control/unitree_move_base_cmd_vel')
        self.declare_parameter('unitree_move_base_lookahead_m', 1.25)
        self.declare_parameter('unitree_move_base_corridor_lookahead_m', 3.0)
        self.declare_parameter('unitree_move_base_goal_change_m', 0.40)
        self.declare_parameter('unitree_move_base_corridor_goal_change_m', 0.80)
        # 保留旧参数名以兼容已有 launch；目标由可靠 DDS/rosbridge 发送一次，
        # 相同目标不得按墙钟重发，否则 ROS1 simple action server 会不断取消
        # 自己的规划。只有前视点实际移动超过 change_m 才更新。
        self.declare_parameter('unitree_move_base_goal_refresh_s', 15.0)
        # ROS1 10 Hz 按仿真时间运行；复杂楼宇实时倍率约 0.08--0.16 时，
        # 墙钟相邻速度可间隔 0.6--1.3 秒。保留约三个低倍率周期，失联后归零。
        self.declare_parameter('unitree_move_base_cmd_timeout_s', 3.00)
        # 赛事公开 Gazebo odom 只用于 ROS1 DWA 的局部走廊居中目标；SLAM、
        # 房间判定、返航和危险源坐标继续使用 scan+IMU 合法位姿。
        self.declare_parameter('use_official_odom_for_corridor_control', False)
        self.declare_parameter('official_odom_topic', '/hw/odom')
        self.declare_parameter('official_odom_fresh_timeout_s', 2.0)
        self.declare_parameter('official_corridor_center_x_m', 0.0)
        self.declare_parameter('official_corridor_forward_lookahead_m', 3.0)
        self.declare_parameter('official_corridor_end_y_m', 35.0)
        self.declare_parameter('official_corridor_yaw_rad', math.pi / 2.0)
        self.declare_parameter('use_official_odom_for_room_control', False)
        self.declare_parameter('official_near_room_y_m', 14.865)
        self.declare_parameter('official_far_room_y_m', 28.895)
        self.declare_parameter('official_room_goal_tolerance_m', 0.55)
        self.declare_parameter('official_room_loop_goal_tolerance_m', 0.50)
        self.declare_parameter('official_room_exit_goal_tolerance_m', 0.80)
        self.declare_parameter('use_official_odom_for_return_control', False)
        self.declare_parameter('official_home_x_m', 0.0)
        self.declare_parameter('official_home_y_m', -2.2)
        self.declare_parameter('official_home_yaw_rad', math.pi / 2.0)
        self.declare_parameter('official_home_tolerance_m', 0.60)
        self.declare_parameter('official_return_floor_index', 0)
        self.declare_parameter('use_official_odom_for_elevator_control', False)
        self.declare_parameter('official_elevator_lobby_x_m', 0.80)
        self.declare_parameter('official_elevator_cabin_x_m', 2.70)
        self.declare_parameter('official_elevator_y_m', 2.60)
        self.declare_parameter('official_elevator_goal_tolerance_m', 0.40)
        self.declare_parameter('official_elevator_door_timeout_s', 25.0)
        self.declare_parameter(
            'official_elevator_minimum_linear_command', 0.35)
        self.declare_parameter('official_robot_model_name', 'a1_gazebo')
        self.declare_parameter('official_robot_ground_z_m', 0.313)
        self.declare_parameter('official_floor_height_m', 2.6)
        # 赛事 DWA 若因门框/起步姿态无法生成速度，短时后退回现有 A* 路径
        # 跟踪和同一套激光净空门禁；DWA 一旦恢复新鲜速度立即重新接管。
        self.declare_parameter('unitree_move_base_direct_fallback_s', 12.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('min_frontier_size', 10)
        # 三维分层地图的远处门口会被墙柱切成多个小簇；入楼轴约束仍有效时
        # 允许 3 格前沿，深入建筑后恢复常规阈值，避免追逐楼外稀疏噪声。
        self.declare_parameter('entry_min_frontier_size', 3)
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
        # 仅根据合法 SLAM 轨迹密度降低重复区域前沿分数；首次经过不处罚，
        # 多次折返或长时间停留才逐步生效。
        self.declare_parameter('revisit_penalty_enabled', True)
        self.declare_parameter('revisit_penalty_radius_m', 1.2)
        self.declare_parameter('revisit_penalty_strength', 0.70)
        self.declare_parameter('revisit_penalty_free_samples', 4)
        self.declare_parameter('revisit_penalty_full_samples', 12)
        # 非官方环境默认从第一条合法 TF 推断。官方 profile 会显式传入公开
        # 起点在 map 帧中的朝向，避免 INIT 建图旋转污染入楼方向。
        self.declare_parameter('entry_heading_yaw', float('nan'))
        self.declare_parameter('entry_forward_half_angle_deg', 35.0)
        # 第一次选到入口路径后仍保持向楼内推进，不能立刻退化成宽半平面并被
        # 楼外南北边界吸走。官方 profile 会按公开建筑尺度显式启用。
        self.declare_parameter('entry_ingress_depth_m', 0.0)
        self.declare_parameter('entry_ingress_progress_slack_m', 2.0)
        self.declare_parameter('entry_ingress_time_limit_s', 0.0)
        self.declare_parameter('room_sector_split_depth_m', 14.0)
        self.declare_parameter('room_sector_visit_lateral_m', 1.5)
        self.declare_parameter('room_sector_candidate_lateral_m', 1.2)
        self.declare_parameter('room_sector_candidate_max_lateral_m', 4.0)
        self.declare_parameter('room_sector_far_depth_margin_m', 2.0)
        self.declare_parameter('room_entry_inflation_radius_m', 0.45)
        self.declare_parameter('room_entry_navigation_clearance_m', 0.35)
        self.declare_parameter('room_sector_early_finish_min_s', 60.0)
        # 跨过门口只代表房间任务开始，不能直接算完成。房间内继续追踪
        # 前沿并累计轨迹；回到门口附近形成闭环，或环视后确认前沿耗尽，
        # 才把该扇区记为已覆盖。
        self.declare_parameter('room_perimeter_min_path_m', 6.0)
        self.declare_parameter('room_perimeter_min_duration_s', 6.0)
        self.declare_parameter('room_perimeter_loop_radius_m', 1.8)
        self.declare_parameter('room_perimeter_linear_speed', 1.80)
        self.declare_parameter('room_perimeter_minimum_linear_speed', 1.10)
        self.declare_parameter('room_perimeter_probe_count', 4)
        self.declare_parameter('room_perimeter_probe_spacing_m', 1.5)
        self.declare_parameter('room_mirrored_doorway_extra_lateral_m', 0.8)
        # 官方楼宇确定性覆盖：先沿走廊到最远门带，再返程逐房执行逆时针
        # 单圈。默认关闭以保持通用 Frontier 行为，官方 profile 显式开启。
        self.declare_parameter('deterministic_room_route_enabled', False)
        self.declare_parameter('deterministic_corridor_end_extra_m', 2.0)
        self.declare_parameter('deterministic_corridor_end_tolerance_m', 1.0)
        # 人工标定确认：机器人从楼外起点到真实远端房门约 33m；入口两侧
        # 空地在 0--10m 也会形成成对 Frontier。未走过 30m 前禁止把这些
        # 入口空地当作两排房门并提前返程。
        self.declare_parameter('deterministic_corridor_min_progress_m', 30.0)
        # 走廊骨架只接受中心线附近前沿；入口两侧开放区属于后续房间候选，
        # 不能在中心线短暂没有前沿时把机器人横向拉走。
        self.declare_parameter('deterministic_corridor_max_lateral_m', 1.0)
        # 固定楼宇出生点不在大门几何中心；官方 profile 以人工路线标定值
        # 修正走廊轴心。通用环境默认 0，不改变原有坐标系。
        self.declare_parameter('deterministic_corridor_center_lateral_m', 0.0)
        # 仅用于中心线骨架 A*。官方主入口宽 2.0m，0.45m 通用膨胀会因
        # SLAM 墙厚/栅格离散把门封死；底层 Unitree DWA 仍执行实时避障。
        self.declare_parameter('deterministic_corridor_inflation_radius_m', 0.20)
        # 大门已由平台在 move_base 启动前自动打开，且扫描自回波已修复；
        # 正式任务从出生点起全程使用赛事 DWA，避免直控步态横向漂移。
        self.declare_parameter('deterministic_entry_direct_until_progress_m', 0.0)
        # -1 仅用于已知开启且与出生点同轴的主门直行段。官方水平雷达的
        # 正前方射线全部命中机身并被过滤为 NaN，无法提供前向净空；转向
        # 仍使用全周有效射线门禁，穿门后立即恢复 Unitree DWA。
        self.declare_parameter('deterministic_entry_clearance_m', -1.0)
        self.declare_parameter('deterministic_entry_rotation_clearance_m', 0.30)
        # 官方三层楼的四个房间在每层同构。人工完整路线与生成场景几何共同
        # 标定出近/远两排门；正式 profile 直接使用这两排，不再依赖会随
        # 地图展开而消失的门口 Frontier。
        self.declare_parameter('deterministic_calibrated_doorways_enabled', False)
        self.declare_parameter('deterministic_near_door_progress_m', 18.9)
        self.declare_parameter('deterministic_far_door_progress_m', 32.7)
        # 1/2 层从电梯大厅重新建立 SLAM 子图，map 轴长度短于 0 层楼外
        # 出生点基准。单独保存上层标定，避免走到物理尽头仍等待旧 32.7m。
        self.declare_parameter(
            'deterministic_upper_near_door_progress_m', 10.0)
        self.declare_parameter(
            'deterministic_upper_far_door_progress_m', 21.4)
        self.declare_parameter(
            'deterministic_upper_corridor_min_progress_m', 20.0)
        self.declare_parameter(
            'deterministic_upper_corridor_hard_limit_m', 27.0)
        self.declare_parameter('deterministic_door_lateral_m', 1.2)
        self.declare_parameter('deterministic_corridor_hard_limit_m', 35.5)
        self.declare_parameter('deterministic_corridor_waypoint_tolerance_m', 0.8)
        self.declare_parameter('deterministic_waypoint_tolerance_m', 0.55)
        self.declare_parameter('deterministic_room_cross_depth_m', 1.2)
        self.declare_parameter('deterministic_room_loop_shallow_m', 1.2)
        self.declare_parameter('deterministic_room_loop_deep_m', 2.5)
        self.declare_parameter('deterministic_room_loop_half_length_m', 1.5)
        self.declare_parameter('deterministic_room_loop_corner_radius_m', 0.35)
        self.declare_parameter('deterministic_waypoint_stall_s', 30.0)
        self.declare_parameter('official_room_waypoint_stall_s', 8.0)
        self.declare_parameter('deterministic_room_min_physical_path_m', 4.0)
        self.declare_parameter('deterministic_room_min_loop_area_m2', 0.8)
        self.declare_parameter(
            'deterministic_room_hold_heading_during_loop', False)
        # 房内逐障碍巡检默认关闭，保持已验收三层路线不变。集成测试显式开启
        # 后，固定环线只负责展开地图，随后由独立严格规划器生成观察目标。
        self.declare_parameter('strict_room_inspection_enabled', False)
        self.declare_parameter('strict_room_min_free_cells', 120)
        self.declare_parameter('strict_room_min_obstacle_area_m2', 0.15)
        self.declare_parameter('strict_room_wall_margin_m', 0.90)
        self.declare_parameter('strict_room_seed_offset_m', 0.50)
        self.declare_parameter('strict_room_door_width_m', 1.80)
        self.declare_parameter('strict_room_viewpoint_count', 8)
        self.declare_parameter('strict_room_required_views_per_obstacle', 3)
        self.declare_parameter('strict_room_viewpoint_standoff_m', 0.50)
        self.declare_parameter('strict_room_viewpoint_clearance_m', 0.30)
        self.declare_parameter('strict_room_path_inflation_radius_m', 0.25)
        self.declare_parameter('strict_room_heading_tolerance_rad', 0.20)
        self.declare_parameter('strict_room_capture_timeout_s', 15.0)
        self.declare_parameter(
            'inspection_request_topic',
            '/hw/perception/inspection_request')
        self.declare_parameter(
            'inspection_result_topic',
            '/hw/perception/inspection_result')
        self.declare_parameter('deterministic_min_scale_tolerance_m', 1.5)
        self.declare_parameter('deterministic_blocked_corner_accept_m', 1.0)
        # 官方四房楼层由两组左右同排门组成；不使用固定 14m 分界，而是
        # 走到实际尽头后对在线 SLAM 门带观测做纵向聚类，并取最远两簇。
        self.declare_parameter('deterministic_door_pair_progress_gap_m', 2.0)
        self.declare_parameter('deterministic_door_station_cluster_m', 2.0)
        self.declare_parameter(
            'deterministic_door_station_min_separation_m', 6.0)
        self.declare_parameter('entry_ingress_relaxed_half_angle_deg', 55.0)
        self.declare_parameter('entry_ingress_max_half_angle_deg', 90.0)
        # 0 表示通用环境不限制；官方 profile 按公开 20 m 楼宽上限加安全裕量。
        self.declare_parameter('entry_lateral_limit_m', 0.0)
        # 官方场景前沿通常距离较近；过大的容差会把首个目标直接误判为“已到达”。
        self.declare_parameter('goal_tolerance_m', 0.25)
        self.declare_parameter('return_goal_tolerance_m', 0.50)
        self.declare_parameter('linear_speed', 0.60)
        self.declare_parameter('minimum_linear_speed', 0.45)
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
        # map->base 受 SLAM 闭环影响可能瞬时跳变。10 Hz 控制下单帧真实位移
        # 不可能达到 0.30 m，超过该值只重置累计锚点，不作为横移距离。
        self.declare_parameter('reobserve_pose_jump_reject_m', 0.30)
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
        # 侧移时同步转向把目标保持在图像中央，形成绕目标的弧线视差；否则
        # 0.8 m 纯横移会把远球推到反侧边缘，重复复查又反向走回原视角。
        self.declare_parameter('reobserve_lateral_centering_gain', 0.80)
        self.declare_parameter(
            'reobserve_lateral_centering_max_turn_speed', 0.60,
        )
        self.declare_parameter(
            'reobserve_lateral_centering_deadband_ratio', 0.05,
        )
        self.declare_parameter('reobserve_max_attempts_per_target', 4)
        # 返航途中若目标首次从画面边缘出现，完全忽略会损失识别率；仅允许
        # 两次受激光安全门约束的短复查，随后必须恢复 RETURNING。
        self.declare_parameter('reobserve_during_returning', False)
        self.declare_parameter(
            'reobserve_returning_max_attempts_per_target', 2,
        )
        self.declare_parameter('stuck_timeout_s', 15.0)
        self.declare_parameter('imu_topic', '/hw/trunk_imu')
        self.declare_parameter('require_upright_imu', True)
        self.declare_parameter('imu_fresh_timeout_s', 1.0)
        self.declare_parameter('upright_arm_tilt_deg', 25.0)
        self.declare_parameter('upright_arm_samples', 3)
        self.declare_parameter('fall_tilt_threshold_deg', 55.0)
        self.declare_parameter('fall_confirm_samples', 3)
        self.declare_parameter('fall_completion_invalidation_window_s', 5.0)
        self.declare_parameter('exploration_recovery_turn_duration_s', 2.0)
        self.declare_parameter('exploration_recovery_turn_speed', 1.0)
        self.declare_parameter('exploration_recovery_turn_clearance_m', 0.20)
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
        # 已建图轨迹返航使用独立高速档；最后两米自动降速，电梯门槛仍继续
        # 使用专用低速参数。所有档位均受实时激光净空和航向门禁约束。
        self.declare_parameter('return_linear_speed', 2.00)
        self.declare_parameter('return_minimum_linear_speed', 1.20)
        self.declare_parameter('return_final_linear_speed', 0.80)
        self.declare_parameter('return_final_minimum_linear_speed', 0.30)
        self.declare_parameter('return_waypoint_spacing_m', 1.50)
        self.declare_parameter('return_angular_speed', 1.80)
        self.declare_parameter('return_minimum_turn_speed', 0.60)
        self.declare_parameter('return_min_clearance_m', 0.45)
        self.declare_parameter('return_rotation_min_clearance_m', 0.30)
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
        # 到达房间/支路端点后快速完成 360° 检查；1.0 rad/s 在 10 Hz RGB
        # 下仍有约 60 帧，足以确认空房间，同时避免旧版每处等待 18 秒。
        self.declare_parameter('frontier_observation_sweep_speed', 1.00)
        self.declare_parameter('frontier_observation_sweep_timeout_s', 10.0)
        self.declare_parameter('unreachable_frontier_ttl_s', 45.0)
        self.declare_parameter('unreachable_frontier_max_ttl_s', 180.0)
        self.declare_parameter('unreachable_frontier_radius_m', 0.45)
        # A* 的空结果既可能表示单个目标不连通，也可能是当前起点吸附失败、
        # 地图瞬时断裂或搜索预算耗尽等整轮共享故障。单次重规划最多封禁少量
        # 盆地，避免一次系统性故障把整层几十个前沿同时判死并阻塞 ROS 回调。
        self.declare_parameter('max_frontier_plan_failures_per_replan', 12)
        # 必须长于基础 unreachable TTL + 一次重规划周期，否则目标刚过期前
        # 就会误判完成，永远没有机会用扩展后的地图重试。
        self.declare_parameter('frontier_completion_grace_s', 60.0)
        # 导航数据记录
        self.declare_parameter('nav_record_enabled', True)
        self.declare_parameter('nav_record_dir', '')
        # 多楼层参数（默认单层，向后兼容）
        # 空 Python list 会被 rclpy 推断成 BYTE_ARRAY，之后 [0,1,2] 覆盖值会
        # 因类型不符拒绝启动；显式声明整数数组兼容单层未设置和多层列表。
        self.declare_parameter(
            'target_floors', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('current_floor_index', 0)
        self.declare_parameter('floor_coverage_threshold', 0.90)
        self.declare_parameter('elevator_id', 'elevator_main')
        self.declare_parameter('elevator_entry_floor', 0)
        self.declare_parameter('stair_detection_enabled', False)
        self.declare_parameter('simenv_container', 'simenv_ros1_hazard_platform')
        self.declare_parameter('manual_elevator_assist', True)
        self.declare_parameter('automatic_elevator_entry', True)
        self.declare_parameter('platform_unloads_elevator_payload', True)
        # 入口以 0.45 m/s 精确驶入后，电梯门中心相对 SLAM 起点约在
        # 前方 2.10 m；先到门正前方，再横向驶过门槛，避免斜撞门框。
        self.declare_parameter('elevator_entry_forward_offset_m', 2.10)
        self.declare_parameter('elevator_entry_right_offset_m', 2.80)
        self.declare_parameter('elevator_exit_right_offset_m', 1.45)
        # A1 在门槛碰撞边界内已进入平台 1.5 m 载荷捕获范围，但合法 odom
        # 到理论轿厢中心仍相差约 0.68 m；0.75 m 可避免安全进梯后反向退出。
        self.declare_parameter('elevator_entry_tolerance_m', 1.50)
        # 楼上从安全大厅返回轿厢的距离较短，必须使用更严格的确认阈值，
        # 防止机器人尚在门外就提前调用电梯而产生“状态换层、本体未换层”。
        self.declare_parameter(
            'elevator_upper_floor_entry_tolerance_m', 1.50)
        self.declare_parameter('elevator_entry_stall_timeout_s', 20.0)
        # 入梯仍保留雷达门禁以防撞墙；出梯时轿厢内可能没有有效前向量程，
        # 仅对目标层门已打开后的精确反向短路径使用负值跳过门禁。
        self.declare_parameter('elevator_entry_min_clearance_m', 0.28)
        self.declare_parameter('elevator_entry_rotation_clearance_m', 0.22)
        self.declare_parameter('elevator_exit_min_clearance_m', -1.0)
        self.declare_parameter('elevator_exit_rotation_clearance_m', -1.0)
        # 电梯门宽有限，精确进梯阶段必须降低速度，避免控制链延迟造成越过门洞。
        self.declare_parameter('elevator_entry_linear_speed', 0.30)
        self.declare_parameter('elevator_exit_linear_speed', 0.45)
        self.declare_parameter('elevator_minimum_linear_speed', 0.18)
        self.declare_parameter('elevator_angular_speed', 1.00)
        self.declare_parameter('elevator_minimum_turn_speed', 0.30)
        self.declare_parameter('per_floor_exploration_s', 120.0)
        self.declare_parameter('floor_height_m', 2.6)
        self.declare_parameter('elevator_height_confirm_ratio', 0.50)
        self.declare_parameter('elevator_transport_timeout_s', 30.0)

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
        self.robot_z = 0.0
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
        self._latest_body_tilt_deg: Optional[float] = None
        self._last_imu_monotonic: Optional[float] = None
        self._upright_imu_armed = False
        self._upright_imu_sample_count = 0
        self._fall_violation_count = 0
        self._fall_pending_tilt_deg: Optional[float] = None
        self._fall_latched = False

        # ---- 宇树官方 move_base 局部规划 ----
        self._local_planner_backend = str(
            self.get_parameter('local_planner_backend').value
        ).strip().lower()
        if self._local_planner_backend not in ('direct', 'unitree_move_base'):
            raise ValueError(
                'local_planner_backend must be direct or unitree_move_base')
        self._unitree_move_base_cmd = Twist()
        self._unitree_move_base_cmd_wall: Optional[float] = None
        self._unitree_move_base_goal_active = False
        self._unitree_move_base_cmd_after_goal = False
        self._unitree_move_base_last_goal_map: Optional[
            Tuple[float, float]] = None
        self._unitree_move_base_last_goal_wall: Optional[float] = None
        self._unitree_move_base_selected_this_cycle = False

        # ---- 探索 ----
        self.frontiers: list = []
        self.current_target: Optional[Frontier] = None
        self.last_target_world: Optional[Tuple[float, float]] = None
        self.current_path: List[Tuple[float, float]] = []
        self.path_index: int = 0
        self._current_target_selected_ros_sec: Optional[float] = None
        self._frontier_last_net_progress_ros: Optional[float] = None
        self._frontier_progress_reference_distance: Optional[float] = None
        self._current_target_was_ingress = False
        self._visited_room_sectors: set[str] = set()
        self._attempted_room_sectors: set[str] = set()
        self._room_sector_doorway_poses: dict[str, Tuple[float, float]] = {}
        self._room_sector_candidate_poses: dict[str, Tuple[float, float]] = {}
        self._last_completed_room_sector: Optional[str] = None
        self._last_completed_room_floor: Optional[int] = None
        self._last_room_completion_ros: Optional[float] = None
        self._room_selection_mode = 'unconstrained'
        self._active_room_sector: Optional[str] = None
        self._active_room_entry_pose: Optional[Tuple[float, float]] = None
        self._active_room_started_ros: Optional[float] = None
        self._active_room_last_pose: Optional[Tuple[float, float]] = None
        self._active_room_path_m = 0.0
        self._active_room_scan_completed = False
        self._active_room_probe_points: List[Tuple[float, float]] = []
        self._active_room_probe_exhausted = False
        self._active_room_return_attempted = False
        self._deterministic_route_phase = 'corridor_outbound'
        self._deterministic_room_queue: List[str] = []
        self._deterministic_room_sector: Optional[str] = None
        self._deterministic_room_waypoints: List[Tuple[float, float]] = []
        self._deterministic_room_waypoint_index = 0
        self._deterministic_waypoint_label = ''
        self._deterministic_corridor_goal_progress = 0.0
        self._deterministic_corridor_observed_max = 0.0
        self._deterministic_corridor_last_plan_progress: Optional[float] = None
        self._deterministic_corridor_stall_count = 0
        self._deterministic_corridor_no_forward_count = 0
        self._deterministic_room_path_m = 0.0
        self._deterministic_room_last_pose: Optional[Tuple[float, float]] = None
        self._deterministic_room_physical_path_m = 0.0
        self._deterministic_room_last_official_pose: Optional[
            Tuple[float, float]] = None
        self._deterministic_official_loop_samples: List[
            Tuple[float, float]] = []
        self._deterministic_room_hold_yaw: Optional[float] = None
        self._room_inspection_execution: Optional[
            RoomInspectionExecution] = None
        self._room_inspection_request_goal_id = ''
        self._room_inspection_result_goal_id = ''
        self._room_inspection_capture_started_ros: Optional[float] = None
        self._official_room_door_label = ''
        self._official_room_door_stage = ''
        self._official_room_door_center_y: Optional[float] = None
        self._deterministic_room_started_ros: Optional[float] = None
        self._deterministic_loop_reached = 0
        self._deterministic_waypoint_scales = {}
        self._deterministic_waypoint_started_ros = None
        self._deterministic_waypoint_best_distance = None
        self._deterministic_doorway_observations: List[
            Tuple[float, float]] = []
        self._last_replan_ros_sec: Optional[float] = None
        self._last_coverage_update_ros_sec: Optional[float] = None
        self._last_return_plan_time: Optional[float] = None
        self._visited_frontiers: set = set()  # 真正到达的前沿质心
        self._visited_pose_history = deque(maxlen=4000)
        self._visited_odom_history = deque(maxlen=4000)
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
        self._reobserve_last_pose: Optional[Tuple[float, float]] = None
        self._reobserve_lateral_distance_m: float = 0.0
        self._reobserve_target_center_error_ratio: Optional[float] = None
        self._reobserve_last_target_seen_ros: Optional[float] = None
        self._reobserve_allow_untracked_upgrade = False
        self._reobserve_attempts: dict = {}
        self._reobserve_resume_state: str = 'EXPLORING'

        # ---- 卡死检测 ----
        self._pose_history: deque = deque(maxlen=30)  # 3秒位置与朝向历史 (10Hz)
        self._stuck_since: Optional[float] = None
        self._safety_blocked_since_ros: Optional[float] = None
        self._exploration_recovery_end_ros: Optional[float] = None
        self._exploration_recovery_direction = 1.0
        self._exploration_recovery_attempts = 0

        # ---- 返航 ----
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        # 家点保存在团队合法 odom 帧；Cartographer 回环改变 map→odom 后，
        # 返航目标按当前变换重投影，禁止读取 /hw/odom Gazebo 真值。
        self._home_odom: Optional[Tuple[float, float]] = None
        self._elevator_anchor_odom: Optional[Tuple[float, float]] = None
        self._robot_odom: Optional[Tuple[float, float]] = None
        self._robot_odom_yaw: Optional[float] = None
        self._official_control_odom: Optional[
            Tuple[float, float, float, float]] = None
        self._home_map_last: Optional[Tuple[float, float]] = None
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
        self.imu_sub = self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value), self.on_imu, 20)
        self.official_odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter('official_odom_topic').value),
            self.on_official_control_odom,
            10,
        )
        self.hazard_sub = self.create_subscription(
            String, '/hw/perception/hazard_detections', self.on_hazard, 10)
        self.inspection_result_sub = self.create_subscription(
            String,
            str(self.get_parameter('inspection_result_topic').value),
            self.on_inspection_result,
            10,
        )
        self.inspection_request_pub = self.create_publisher(
            String,
            str(self.get_parameter('inspection_request_topic').value),
            10,
        )
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self.unitree_move_base_goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('unitree_move_base_goal_topic').value),
            10,
        )
        self.unitree_move_base_control_pub = self.create_publisher(
            String,
            str(self.get_parameter('unitree_move_base_control_topic').value),
            10,
        )
        self.unitree_move_base_cmd_sub = self.create_subscription(
            Twist,
            str(self.get_parameter('unitree_move_base_cmd_topic').value),
            self.on_unitree_move_base_cmd,
            10,
        )
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
        self._floor_transition_phase: str = ''  # navigating | calling | waiting | entering | exiting
        self._floor_transition_start_ros: Optional[float] = None
        # Docker/ROS1 电梯服务可能等待数十秒，不能在 10 Hz 控制定时器中
        # 同步调用，否则会中断速度心跳。单线程执行器保证请求仍按顺序执行。
        self._elevator_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix='hazardwalker-elevator',
        )
        self._elevator_future: Optional[Future] = None
        self._elevator_request_stage: str = ''
        self._elevator_request_floor: Optional[int] = None
        self._elevator_next_retry_ros: float = 0.0
        self._manual_elevator_ready: bool = False
        self._auto_elevator_entry_attempts: int = 0
        self._auto_entry_alignment_index: int = 0
        self._auto_entry_best_distance: Optional[float] = None
        self._auto_entry_last_progress_ros: Optional[float] = None
        self._auto_entry_last_pose: Optional[Tuple[float, float]] = None
        self._auto_entry_backoff_until_ros: Optional[float] = None
        self._door_closed_scan: Optional[Tuple[List[float], float, float]] = None
        self._door_scan_wait_stamp: Optional[Tuple[int, int]] = None
        self._door_target_odom: Optional[Tuple[float, float]] = None
        # 第一次平台确认载荷已在轿厢内时，保存该合法 odom 平面位置。
        # 电梯各层平面位置一致，后续楼层优先复用，避免重复门差分误定位。
        self._validated_elevator_cabin_odom: Optional[
            Tuple[float, float]] = None
        self._door_search_attempts: int = 0
        self._door_search_target_yaw: Optional[float] = None
        self._door_validation_rejections: int = 0
        self._door_approach_started_ros: Optional[float] = None
        self._door_rotation_started_ros: Optional[float] = None
        self._transition_from_floor: Optional[int] = None
        self._floor_transition_start_z: Optional[float] = None
        self._floor_exit_target: Optional[Tuple[float, float]] = None
        self._floor_exit_target_odom: Optional[Tuple[float, float]] = None
        self._elevator_cabin_odom_by_floor: dict[
            int, Tuple[float, float]] = {}
        self._elevator_odom_path: List[Tuple[float, float]] = []
        self._elevator_odom_path_index: int = 0
        self._floor_started_ros: Optional[float] = None
        self._manual_mode_last_request_wall: float = 0.0

        # floor_index 发布器（发布 Int32，触发 scan_imu_localizer 重置匹配地图）
        self.floor_index_pub = self.create_publisher(
            Int32, '/hazardwalker/navigation/floor_index', 10)
        self.create_service(
            Trigger, '/hazardwalker/navigation/elevator_ready',
            self.on_manual_elevator_ready)

        self.get_logger().info(
            f'Frontier explorer ready. Home=({self.start_x:.1f}, '
            f'{self.start_y:.1f}), local_planner={self._local_planner_backend}')

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
        allow_returning = bool(
            self.get_parameter('reobserve_during_returning').value
        )
        maximum_attempts = (
            self.get_parameter(
                'reobserve_returning_max_attempts_per_target'
            ).value
            if self.state == 'RETURNING'
            else self.get_parameter('reobserve_max_attempts_per_target').value
        )
        if not reobservation_request_is_eligible(
                request,
                self.state,
                self._reobserve_attempts,
                maximum_attempts,
                allow_returning=allow_returning):
            return
        self._trigger_reobservation(request)

    def on_inspection_result(self, msg: String):
        """接收感知对当前观察目标的采帧确认。

        ``success`` 仅表示该视角完成了有效图像/深度处理，不代表一定检出
        红球。过期目标、失败采帧和格式错误都不会推进房间巡检。
        """

        execution = self._room_inspection_execution
        if execution is None or execution.phase != execution.CAPTURE:
            return
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        goal = execution.current_goal
        goal_id = str(payload.get('goal_id', ''))
        if goal is None or goal_id != goal.goal_id:
            return
        succeeded = bool(payload.get('success', False))
        if execution.mark_capture(succeeded):
            self._room_inspection_result_goal_id = goal_id
            self._room_inspection_request_goal_id = ''
            self._room_inspection_capture_started_ros = None
            self.get_logger().info(
                f'Inspection capture confirmed: {goal_id} '
                f'({execution.progress.completed_goal_count}/'
                f'{execution.progress.required_goal_count}).')
        elif not succeeded:
            self.get_logger().warning(
                f'Inspection capture failed and remains pending: {goal_id}.')

    def on_unitree_move_base_cmd(self, msg: Twist):
        """接收赛事仓库宇树 DWA 输出；仅在活动目标期间允许进入仲裁。"""

        self._unitree_move_base_cmd = msg
        self._unitree_move_base_cmd_wall = time.monotonic()
        if self._unitree_move_base_goal_active:
            self._unitree_move_base_cmd_after_goal = True

    def on_official_control_odom(self, message: Odometry):
        """缓存赛事公开 odom，仅供 ROS1 DWA 走廊居中，不发布/消费 TF。"""

        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (
            float(position.x), float(position.y),
            float(orientation.x), float(orientation.y),
            float(orientation.z), float(orientation.w),
        )
        if not all(math.isfinite(value) for value in values):
            return
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y)
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z)
        self._official_control_odom = (
            float(position.x),
            float(position.y),
            math.atan2(sin_yaw, cos_yaw),
            time.monotonic(),
        )

    def _has_fresh_official_control_odom(self) -> bool:
        return (
            self._official_control_odom is not None
            and time.monotonic() - self._official_control_odom[3] <= max(
                0.1,
                float(self.get_parameter(
                    'official_odom_fresh_timeout_s').value),
            )
        )

    def _deterministic_door_progresses(self) -> Tuple[float, float]:
        """返回当前楼层 SLAM 子图中的近/远门轴向标定。"""

        prefix = 'deterministic_upper_' if self._current_floor > 0 else (
            'deterministic_')
        near = float(self.get_parameter(
            f'{prefix}near_door_progress_m').value)
        far = float(self.get_parameter(
            f'{prefix}far_door_progress_m').value)
        return max(0.0, near), max(max(0.0, near), far)

    def _deterministic_corridor_limits(self) -> Tuple[float, float]:
        """返回当前楼层的最小走廊进度和地图硬上限。"""

        if self._current_floor > 0:
            return (
                max(0.0, float(self.get_parameter(
                    'deterministic_upper_corridor_min_progress_m').value)),
                max(0.0, float(self.get_parameter(
                    'deterministic_upper_corridor_hard_limit_m').value)),
            )
        return (
            max(0.0, float(self.get_parameter(
                'deterministic_corridor_min_progress_m').value)),
            max(0.0, float(self.get_parameter(
                'deterministic_corridor_hard_limit_m').value)),
        )

    def _official_room_goal_from_map(
            self, map_point: Tuple[float, float]) -> Optional[
                Tuple[float, float, float]]:
        """把固定楼层房间目标从 SLAM轴坐标映射到 ROS1物理 odom。"""

        if not self._has_fresh_official_control_odom():
            return None
        progress, lateral = self._deterministic_axis_coordinates(map_point)
        near_progress, far_progress = (
            self._deterministic_door_progresses())
        near_y = float(self.get_parameter('official_near_room_y_m').value)
        far_y = float(self.get_parameter('official_far_room_y_m').value)
        denominator = far_progress - near_progress
        if abs(denominator) <= 1e-6:
            return None
        physical_y = near_y + (
            progress - near_progress) * (far_y - near_y) / denominator
        # map 轴左侧为正 lateral；官方 world/odom 的左房在负 x。
        physical_x = -lateral
        official_x, official_y, _yaw, _stamp = self._official_control_odom
        physical_yaw = math.atan2(
            physical_y - official_y,
            physical_x - official_x,
        )
        if self._deterministic_waypoint_label.startswith(
                'room_inspect_orient:'):
            execution = self._room_inspection_execution
            goal = execution.current_goal if execution is not None else None
            if goal is not None and self._entry_axis is not None:
                map_axis_yaw = math.atan2(
                    self._entry_axis[1], self._entry_axis[0])
                relative_yaw = normalize_angle(
                    goal.face_yaw_rad - map_axis_yaw)
                official_axis_yaw = float(self.get_parameter(
                    'official_corridor_yaw_rad').value)
                physical_yaw = normalize_angle(
                    official_axis_yaw + relative_yaw)
        if (self._deterministic_waypoint_label.startswith('room_loop:')
                and self._deterministic_room_hold_yaw is not None
                and bool(self.get_parameter(
                    'deterministic_room_hold_heading_during_loop').value)):
            # A1/TrajectoryPlannerROS 是全向底盘。进入房间后固定朝向内部，
            # 用横移完成矩形四边，避免每个角点都原地转90°甚至绕远260°。
            physical_yaw = self._deterministic_room_hold_yaw
        return physical_x, physical_y, physical_yaw

    def on_scan(self, msg: LaserScan):
        """保存最近一帧公开激光，用作运动前的局部安全门禁。"""

        self.latest_scan = msg
        stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        # 仅收到时间戳真正推进的扫描才刷新新鲜度；重复转发一帧冻结扫描时
        # 必须在超时后停止，而不能因为回调仍触发就继续运动。
        if stamp != (0, 0) and stamp != self._last_scan_stamp:
            self._last_scan_stamp = stamp
            self._last_scan_monotonic = time.monotonic()

    def on_imu(self, msg: Imu):
        """用公开机体 IMU 锁存倒地事件，不读取 Gazebo 模型真值。"""

        orientation = msg.orientation
        tilt = body_tilt_degrees_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if tilt is None:
            return
        self._latest_body_tilt_deg = tilt
        self._last_imu_monotonic = time.monotonic()

        if not self._upright_imu_armed:
            if tilt <= max(
                    0.0,
                    float(self.get_parameter('upright_arm_tilt_deg').value)):
                self._upright_imu_sample_count += 1
            else:
                self._upright_imu_sample_count = 0
            if self._upright_imu_sample_count >= max(
                    1, int(self.get_parameter('upright_arm_samples').value)):
                self._upright_imu_armed = True
                self.get_logger().info(
                    f'Upright IMU guard armed: tilt={tilt:.1f}deg.'
                )
            return

        if tilt >= max(
                1.0,
                float(self.get_parameter('fall_tilt_threshold_deg').value)):
            self._fall_violation_count += 1
        else:
            self._fall_violation_count = 0
        if self._fall_violation_count >= max(
                1, int(self.get_parameter('fall_confirm_samples').value)):
            self._fall_pending_tilt_deg = tilt

    def on_manual_elevator_ready(self, request, response):
        """人工把 A1 停入轿厢后，确认本次跨层可以开始。"""

        del request
        if (self.state != 'FLOOR_TRANSITION'
                or self._floor_transition_phase != 'manual_waiting'):
            response.success = False
            response.message = '当前不在等待电梯人工确认状态'
            return response
        self._manual_elevator_ready = True
        response.success = True
        response.message = (
            f'已确认进入轿厢，准备前往 floor={self._current_floor}')
        self.get_logger().info(response.message)
        return response

    def _trigger_reobservation(self, request: dict):
        """执行感知侧已经判定的明确复查动作。"""

        now = self._ros_time_sec()
        self._reobserve_resume_state = (
            'RETURNING' if self.state == 'RETURNING' else 'EXPLORING'
        )
        action = str(request['action'])
        motion_duration = self._reobservation_motion_duration(action)
        maximum_motion_duration = max(
            self._reobservation_motion_duration('turn_left'),
            self._reobservation_motion_duration('move_left'),
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
            now + maximum_motion_duration + settle_duration + observe_duration
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
        self._reobserve_target_center_error_ratio = None
        self._reset_reobservation_lateral_tracking()
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
            self._reobserve_target_center_error_ratio = None
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
        self._reobserve_target_center_error_ratio = target_horizontal_error_ratio(
            detection, payload.get('image_width'),
        )
        detection_track_id = str(detection.get('track_id') or '').strip()
        if (detection_track_id
                and not detection_track_id.startswith('untracked:')):
            # 首帧未跟踪候选一旦升级为正式轨迹，后续只能消费精确 track_id。
            self._reobserve_allow_untracked_upgrade = False

        live_request = select_live_reobservation_update(
            payload, self.reobserve_target_id, self.reobserve_action,
        )
        if live_request is not None:
            if reobservation_actions_conflict(
                    self.reobserve_action, live_request.get('action')):
                self._stop_reobservation_motion(
                    now,
                    'live perception recommendation reversed direction',
                )
                return
            if live_reobservation_action_update_allowed(
                    self.reobserve_action, live_request.get('action')):
                self._update_active_reobservation_action(live_request, now)
                if self._reobserve_motion_stop_latched:
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

    def _reobservation_motion_duration(self, action: str) -> float:
        """返回单段复查动作时长；总复查截止时间不会被逐帧建议延长。"""

        parameter = (
            'reobserve_lateral_motion_duration_s'
            if action in ('move_left', 'move_right')
            else 'reobserve_motion_duration_s'
        )
        return max(0.0, float(self.get_parameter(parameter).value))

    def _reset_reobservation_lateral_tracking(self):
        """从当前合法 SLAM 位姿开始累计本段横移，不跨动作复用旧基准。"""

        self._reobserve_last_pose = (self.robot_x, self.robot_y)
        self._reobserve_lateral_distance_m = 0.0

    def _update_active_reobservation_action(
            self, request: dict, now_ros: float):
        """让同一候选跟随最新感知建议，同时保留原复查硬截止时间。"""

        old_action = self.reobserve_action or 'hold_observation'
        new_action = str(request['action'])
        if new_action == 'hold_observation':
            self._stop_reobservation_motion(
                now_ros, 'live perception requested stable observation',
            )
            return

        # 动作切换只使用本次复查尚余的机动窗口，不能通过逐帧改变建议无限续期。
        latest_motion_end = max(
            now_ros,
            self.reobserve_end_time
            - self.reobserve_settle_duration_s
            - self.reobserve_observe_duration_s,
        )
        remaining_duration = min(
            self._reobservation_motion_duration(new_action),
            max(0.0, latest_motion_end - now_ros),
        )
        if remaining_duration <= 0.0:
            self._stop_reobservation_motion(
                now_ros, 'no bounded motion window remains for live action update',
            )
            return

        self.reobserve_action = new_action
        self.reobserve_motion_end_time = now_ros + remaining_duration
        self._reset_reobservation_lateral_tracking()
        self.reobserve_baseline_bearing_deg = request.get('view_bearing_deg')
        self.reobserve_required_bearing_change_deg = max(
            1.0,
            float(request.get(
                'required_bearing_change_deg',
                self.reobserve_required_bearing_change_deg,
            )),
        )
        self._reobserve_bearing_goal_met = False
        self.get_logger().info(
            f'Reobservation action updated for target={self.reobserve_target_id}: '
            f'{old_action} -> {new_action}; '
            f'bounded_motion={remaining_duration:.2f}s'
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
        """启动期请求导航控制权；最多三次，不持续抢回人工控制。"""

        if self._mode_request_sent_count >= 3:
            return
        now = time.monotonic()
        if now - self._last_mode_request_time < 1.0:
            return
        self._last_mode_request_time = now
        self._mode_request_sent_count += 1
        message = String()
        message.data = str(self.get_parameter('control_mode_value').value)
        self.mode_request_pub.publish(message)

    def _request_control_mode(self, mode: str) -> None:
        """跨层期间限频切换人工/导航控制，避免两套速度源同时生效。"""

        now = time.monotonic()
        if now - self._manual_mode_last_request_wall < 1.0:
            return
        self._manual_mode_last_request_wall = now
        message = String()
        message.data = str(mode).strip().lower()
        self.mode_request_pub.publish(message)

    def on_timer(self):
        """10Hz 主循环。"""
        self._unitree_move_base_selected_this_cycle = False
        now_ros = self._ros_time_sec()
        if self._mission_start_ros_sec is None and now_ros > 0.0:
            # 从官方 /clock 第一条有效消息开始计总预算，INIT 建图/开门耗时也
            # 必须计入 600 秒，不能到 EXPLORING 才重新起表。
            self._mission_start_ros_sec = now_ros
        self._update_pose()

        if (self._fall_pending_tilt_deg is not None
                and self.state not in ('FINISHED', 'FAILED')):
            self._handle_fall_detected(now_ros)

        # 状态持久化发布
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        self._ensure_control_mode()

        cmd = Twist()
        if self.state == 'FAILED':
            self._cancel_unitree_move_base()
            self.cmd_pub.publish(cmd)
            return
        if (self.state in (
                'EXPLORING', 'REOBSERVING', 'RETURNING',
                'FLOOR_COMPLETE', 'FLOOR_TRANSITION')
                and not self._has_fresh_imu()):
            self.get_logger().warning(
                'Navigation held at zero: upright IMU is missing or stale.',
                throttle_duration_sec=3.0,
            )
            self._cancel_unitree_move_base()
            self._update_stuck_detection(cmd)
            self.cmd_pub.publish(cmd)
            return
        if not self._has_fresh_pose():
            # TF 缺失时默认的 (0,0,0) 不能当作真实位置，更不能据此记录 home 或发控制。
            self._cancel_unitree_move_base()
            self._update_stuck_detection(cmd)
            self.cmd_pub.publish(cmd)
            return

        if (self.state in ('EXPLORING', 'REOBSERVING')
                and not (
                    self.state == 'REOBSERVING'
                    and self._reobserve_resume_state == 'RETURNING'
                )
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
        elif self.state in ('FINISHED', 'FAILED'):
            cmd = Twist()  # 停止

        # unitree_move_base 只在本周期确实执行路径跟踪时保有目标。INIT、环视、
        # 感知复查和电梯精确机动均由原状态机直接控制，必须先撤销 ROS1 目标，
        # 避免局部规划器在后台继续产生上一条路径的速度。
        if (self._local_planner_backend == 'unitree_move_base'
                and not self._unitree_move_base_selected_this_cycle):
            self._cancel_unitree_move_base()

        self._update_stuck_detection(cmd)
        self.cmd_pub.publish(cmd)

        # ---- 记录位姿与速度指令 ----
        if self._has_fresh_pose():
            target = None
            if self.current_target is not None:
                target = self.current_target.centroid
            official_pose = (
                self._official_control_odom[:3]
                if self._has_fresh_official_control_odom()
                else None
            )
            self.recorder.record_pose(
                now_ros, self.robot_x, self.robot_y, self.robot_yaw,
                self.state, target,
                odom_pose=self._robot_odom,
                official_pose=official_pose,
                home_distance_m=self._distance_home_m(),
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
                if self._robot_odom is not None:
                    self._home_odom = self._robot_odom
                    if self._elevator_anchor_odom is None:
                        self._elevator_anchor_odom = self._robot_odom
                self._home_map_last = (self.start_x, self.start_y)
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

    def _deterministic_route_enabled(self) -> bool:
        return bool(self.get_parameter(
            'deterministic_room_route_enabled').value)

    def _reset_deterministic_room_route(self) -> None:
        """每层重置走廊到底、返程逐房和逆时针单圈状态。"""

        self._deterministic_route_phase = 'corridor_outbound'
        self._deterministic_room_queue = []
        self._deterministic_room_sector = None
        self._deterministic_room_waypoints = []
        self._deterministic_room_waypoint_index = 0
        self._deterministic_waypoint_label = ''
        self._deterministic_corridor_goal_progress = 0.0
        self._deterministic_corridor_observed_max = 0.0
        self._deterministic_corridor_last_plan_progress = None
        self._deterministic_corridor_stall_count = 0
        self._deterministic_corridor_no_forward_count = 0
        self._deterministic_room_path_m = 0.0
        self._deterministic_room_last_pose = None
        self._deterministic_room_physical_path_m = 0.0
        self._deterministic_room_last_official_pose = None
        self._deterministic_official_loop_samples = []
        self._deterministic_room_hold_yaw = None
        self._room_inspection_execution = None
        self._room_inspection_request_goal_id = ''
        self._room_inspection_result_goal_id = ''
        self._room_inspection_capture_started_ros = None
        self._official_room_door_label = ''
        self._official_room_door_stage = ''
        self._official_room_door_center_y = None
        self._deterministic_room_started_ros = None
        self._deterministic_loop_reached = 0
        self._deterministic_waypoint_scales = {}
        self._deterministic_waypoint_started_ros = None
        self._deterministic_waypoint_best_distance = None
        self._deterministic_doorway_observations = []

    def _deterministic_axis_coordinates(
            self, point: Tuple[float, float]) -> Tuple[float, float]:
        if self._entry_origin is None or self._entry_axis is None:
            return 0.0, 0.0
        axis_x, axis_y = self._entry_axis
        norm = max(1e-6, math.hypot(axis_x, axis_y))
        axis_x, axis_y = axis_x / norm, axis_y / norm
        dx = float(point[0]) - self._entry_origin[0]
        dy = float(point[1]) - self._entry_origin[1]
        return (
            dx * axis_x + dy * axis_y,
            -dx * axis_y + dy * axis_x,
        )

    def _deterministic_world_point(
            self, progress: float, lateral: float) -> Tuple[float, float]:
        axis_x, axis_y = self._entry_axis or (1.0, 0.0)
        norm = max(1e-6, math.hypot(axis_x, axis_y))
        axis_x, axis_y = axis_x / norm, axis_y / norm
        normal_x, normal_y = -axis_y, axis_x
        origin = self._entry_origin or (self.robot_x, self.robot_y)
        return (
            origin[0] + axis_x * float(progress) + normal_x * float(lateral),
            origin[1] + axis_y * float(progress) + normal_y * float(lateral),
        )

    def _seed_calibrated_doorways(self) -> None:
        """按固定楼宇标定写入两排四个真实房门，覆盖易漂移的前沿候选。"""

        if not bool(self.get_parameter(
                'deterministic_calibrated_doorways_enabled').value):
            return
        near_progress, far_progress = (
            self._deterministic_door_progresses())
        lateral = max(
            0.2,
            abs(float(self.get_parameter(
                'deterministic_door_lateral_m').value)),
        )
        self._room_sector_candidate_poses = {
            'far_left': self._deterministic_world_point(
                far_progress, lateral),
            'far_right': self._deterministic_world_point(
                far_progress, -lateral),
            'near_left': self._deterministic_world_point(
                near_progress, lateral),
            'near_right': self._deterministic_world_point(
                near_progress, -lateral),
        }

    def _refresh_deterministic_doorways(self) -> None:
        """从当前合法占用图缓存门带前沿，并更新走廊最远已知纵深。"""

        if self.grid is None or self.latest_map is None:
            return
        mask = find_frontiers(self.grid)
        if int(mask.sum()) <= 0:
            return
        self.frontiers = cluster_frontiers(mask, self.grid, self.latest_map)
        self._cache_room_doorway_candidates(self.frontiers)
        # 固定赛场以标定门位为最终权威；在线 Frontier 仅用于通用环境。
        self._seed_calibrated_doorways()
        center_limit = max(
            0.2,
            float(self.get_parameter(
                'room_sector_candidate_lateral_m').value),
        )
        progresses = []
        for frontier in self.frontiers:
            progress, lateral = self._deterministic_axis_coordinates(
                frontier.centroid)
            if progress >= 0.0 and abs(lateral) <= center_limit:
                progresses.append(progress)
        for pose in self._room_sector_candidate_poses.values():
            progress, _ = self._deterministic_axis_coordinates(pose)
            if progress >= 0.0:
                progresses.append(progress)
        if progresses:
            self._deterministic_corridor_observed_max = max(
                self._deterministic_corridor_observed_max,
                max(progresses),
            )

    def _complete_deterministic_doorway_cache(self) -> None:
        """只用已观测门带的轴向/横向对称补齐偶发消失的门口。"""

        poses = self._room_sector_candidate_poses
        opposite = {
            'far_left': 'far_right', 'far_right': 'far_left',
            'near_left': 'near_right', 'near_right': 'near_left',
        }
        for sector, source_sector in opposite.items():
            if sector in poses or source_sector not in poses:
                continue
            progress, lateral = self._deterministic_axis_coordinates(
                poses[source_sector])
            poses[sector] = self._deterministic_world_point(
                progress, -lateral)
        # 不再用“远端纵深一半”伪造近端门。人工标定证明入口空地也会形成
        # 强前沿，只有两组真实左右配对都被 SLAM 观测到才允许开始逐房覆盖。

    def _build_deterministic_room_queue(self) -> List[str]:
        self._complete_deterministic_doorway_cache()
        sectors = [
            key for key in (
                'far_left', 'far_right', 'near_left', 'near_right')
            if key in self._room_sector_candidate_poses
            and key not in self._visited_room_sectors
        ]
        # 固定楼宇已经人工确认两排四门同构。禁止按浮点 SLAM纵深再次排序，
        # 否则同排轻微漂移会改变访问次序并产生跨走廊折返。
        return sectors

    def _set_deterministic_goal(
            self, goal: Tuple[float, float], label: str,
            now_ros: float, path_override=None) -> bool:
        if (self._deterministic_waypoint_label == label
                and self.current_target is not None and self.current_path):
            return True
        official_room_mode = (
            label.startswith('room_')
            and bool(self.get_parameter(
                'use_official_odom_for_room_control').value)
        )
        resolved_goal = (float(goal[0]), float(goal[1]))
        if ((label.startswith('room_cross:')
                or label.startswith('room_loop:'))
                and self._deterministic_room_sector is not None
                and not official_room_mode):
            scale = float(self._deterministic_waypoint_scales.get(label, 1.0))
            doorway = self._room_sector_candidate_poses.get(
                self._deterministic_room_sector)
            if doorway is not None and scale < 0.999:
                resolved_goal = (
                    doorway[0] + (resolved_goal[0] - doorway[0]) * scale,
                    doorway[1] + (resolved_goal[1] - doorway[1]) * scale,
                )

        def plan_to(candidate):
            return a_star_path(
                self.grid, self.latest_map,
                self.robot_x, self.robot_y,
                float(candidate[0]), float(candidate[1]),
                inflation_radius_m=max(
                    0.25,
                    float(self.get_parameter(
                        'room_entry_inflation_radius_m').value),
                ),
                start_search_radius_m=0.60,
                goal_search_radius_m=0.80,
            )
        path = list(path_override or [])
        if not path:
            path = plan_to(resolved_goal)
        scaled_candidates = []
        if (not path and label.startswith('room_loop:')
                and not official_room_mode
                and self._deterministic_room_sector is not None):
            doorway = self._room_sector_candidate_poses.get(
                self._deterministic_room_sector)
            if doorway is not None:
                scaled_candidates = scaled_room_waypoint_candidates(
                    doorway, resolved_goal)
                for candidate in scaled_candidates:
                    candidate_path = plan_to(candidate)
                    if not candidate_path:
                        continue
                    resolved_goal = candidate
                    path = candidate_path
                    if (0 <= self._deterministic_room_waypoint_index
                            < len(self._deterministic_room_waypoints)):
                        self._deterministic_room_waypoints[
                            self._deterministic_room_waypoint_index] = candidate
                    self.get_logger().info(
                        f'Adjusted blocked room loop waypoint {label} to '
                        f'({candidate[0]:.2f}, {candidate[1]:.2f}).')
                    break
        if not path and scaled_candidates and not official_room_mode:
            nearest = min(
                scaled_candidates,
                key=lambda candidate: math.hypot(
                    candidate[0] - self.robot_x,
                    candidate[1] - self.robot_y),
            )
            nearest_distance = math.hypot(
                nearest[0] - self.robot_x,
                nearest[1] - self.robot_y,
            )
            accept_m = max(
                0.2,
                float(self.get_parameter(
                    'deterministic_blocked_corner_accept_m').value),
            )
            if nearest_distance <= accept_m:
                resolved_goal = (self.robot_x, self.robot_y)
                path = [resolved_goal]
                if (0 <= self._deterministic_room_waypoint_index
                        < len(self._deterministic_room_waypoints)):
                    self._deterministic_room_waypoints[
                        self._deterministic_room_waypoint_index] = resolved_goal
                self.get_logger().info(
                    f'Blocked room loop corner {label} accepted at current '
                    f'pose; nearest scaled corner distance='
                    f'{nearest_distance:.2f}m.')
        if not path and official_room_mode:
            # 固定房间的物理 odom 目标由赛事 DWA 再规划；SLAM栅格尚未
            # 展开到角点时不能阻止机器人进入房间采样。
            path = [resolved_goal]
        if not path:
            self.get_logger().warning(
                f'Deterministic waypoint is not reachable: {label} '
                f'at ({goal[0]:.2f}, {goal[1]:.2f}).',
                throttle_duration_sec=3.0,
            )
            return False
        if (label == 'corridor_outbound'
                and math.hypot(
                    path[-1][0] - resolved_goal[0],
                    path[-1][1] - resolved_goal[1],
                ) > 0.10):
            # 未知边界目标会被 A*吸附到最近自由格。若直接采用该终点，
            # 路径会在脚边立即“完成”并每周期重建。保留真实走廊终点，
            # 由已过滤机身自回波的官方 Unitree DWA 做最后局部可达性判断。
            path.append(resolved_goal)
        self.current_target = Frontier(
            centroid=resolved_goal,
            size=1, points=[], info_gain=0.0)
        self.current_path = path
        self.path_index = 0
        self._deterministic_waypoint_label = str(label)
        self._current_target_selected_ros_sec = now_ros
        self._frontier_last_net_progress_ros = now_ros
        self._frontier_progress_reference_distance = math.hypot(
            resolved_goal[0] - self.robot_x,
            resolved_goal[1] - self.robot_y)
        self._deterministic_waypoint_started_ros = now_ros
        self._deterministic_waypoint_best_distance = (
            self._frontier_progress_reference_distance)
        self.get_logger().info(
            f'Deterministic waypoint: {label} -> '
            f'({resolved_goal[0]:.2f}, {resolved_goal[1]:.2f}), '
            f'path={len(path)}.')
        return True

    def _recover_stalled_deterministic_waypoint(self, now_ros: float) -> bool:
        """收缩卡住的门内目标并重规划；只有实际到达后才推进房间阶段。"""

        label = self._deterministic_waypoint_label
        if (self.current_target is None
                or not (label.startswith('room_cross:')
                        or label.startswith('room_loop:')
                        or label.startswith('room_exit:'))):
            return False
        if (bool(self.get_parameter(
                'use_official_odom_for_room_control').value)
                and self._has_fresh_official_control_odom()):
            official_goal = self._official_room_goal_from_map(
                self.current_target.centroid)
        else:
            official_goal = None
        if official_goal is not None:
            official_x, official_y, _yaw, _stamp = (
                self._official_control_odom)
            distance = math.hypot(
                official_goal[0] - official_x,
                official_goal[1] - official_y,
            )
        else:
            distance = math.hypot(
                self.current_target.centroid[0] - self.robot_x,
                self.current_target.centroid[1] - self.robot_y,
            )
        if (self._deterministic_waypoint_best_distance is None
                or distance
                <= self._deterministic_waypoint_best_distance - 0.12):
            self._deterministic_waypoint_best_distance = distance
            self._deterministic_waypoint_started_ros = now_ros
            return False
        if self._deterministic_waypoint_started_ros is None:
            self._deterministic_waypoint_started_ros = now_ros
            return False
        stall_parameter = (
            'official_room_waypoint_stall_s'
            if official_goal is not None
            else 'deterministic_waypoint_stall_s'
        )
        stall_s = max(
            5.0,
            float(self.get_parameter(stall_parameter).value),
        )
        if now_ros - self._deterministic_waypoint_started_ros < stall_s:
            return False
        if official_goal is not None:
            # 固定赛场的物理角点是验收目标。旧逻辑把角点向门口缩小，甚至
            # 在当前位置直接“接受”，会让未进房也累计 probe_count。卡住时
            # 只撤销并重发同一个 DWA目标，绝不改变物理覆盖范围。
            self.get_logger().warning(
                f'Official room waypoint stalled: {label}; reissuing the '
                'same physical goal without shrinking or accepting it.')
            self.recorder.record_failure(
                now_ros,
                'official_room_waypoint_stall',
                self.robot_x,
                self.robot_y,
                detail=f'label={label}; distance={distance:.3f}',
            )
            self._cancel_unitree_move_base()
            self.current_target = None
            self.current_path = []
            self.path_index = 0
            self._deterministic_waypoint_label = ''
            self._deterministic_waypoint_started_ros = now_ros
            self._deterministic_waypoint_best_distance = None
            return True
        previous_scale = float(
            self._deterministic_waypoint_scales.get(label, 1.0))
        next_scale = max(0.40, previous_scale * 0.75)
        if next_scale >= previous_scale - 0.01:
            self._deterministic_waypoint_started_ros = now_ros
            return False
        self._deterministic_waypoint_scales[label] = next_scale
        self.get_logger().warning(
            f'Deterministic waypoint stalled: {label}; shrinking relative '
            f'room target to {next_scale:.2f} and replanning.')
        self._cancel_unitree_move_base()
        self.current_target = None
        self.current_path = []
        self.path_index = 0
        self._deterministic_waypoint_label = ''
        self._deterministic_waypoint_started_ros = now_ros
        self._deterministic_waypoint_best_distance = None
        return True

    def _plan_reachable_corridor_goal(self, now_ros: float) -> bool:
        """选择走廊方向最远的当前可达前沿，逐段推进到尽头。"""

        robot_progress, _ = self._deterministic_axis_coordinates(
            (self.robot_x, self.robot_y))
        if self._deterministic_corridor_last_plan_progress is not None:
            if robot_progress - self._deterministic_corridor_last_plan_progress < 0.20:
                self._deterministic_corridor_stall_count += 1
            else:
                self._deterministic_corridor_stall_count = 0
        self._deterministic_corridor_last_plan_progress = robot_progress
        _minimum_progress, configured_hard_limit = (
            self._deterministic_corridor_limits())
        hard_limit = max(robot_progress, configured_hard_limit)
        corridor_limit = max(
            0.2,
            float(self.get_parameter(
                'deterministic_corridor_max_lateral_m').value),
        )
        minimum_goal_gap = max(
            0.40,
            float(self.get_parameter(
                'deterministic_corridor_waypoint_tolerance_m').value) + 0.20,
        )
        annotated = []
        for frontier in self.frontiers:
            progress, lateral = self._deterministic_axis_coordinates(
                frontier.centroid)
            if (progress > robot_progress + minimum_goal_gap
                    and abs(lateral) <= corridor_limit):
                annotated.append((progress, abs(lateral), frontier))
        candidates = sorted(
            annotated,
            key=lambda item: item[0],
            reverse=True,
        )
        for progress, _lateral, frontier in candidates[:24]:
            goal_progress = min(progress, hard_limit)
            center_goal = self._deterministic_world_point(
                goal_progress, 0.0)
            path = a_star_path(
                self.grid, self.latest_map,
                self.robot_x, self.robot_y,
                center_goal[0], center_goal[1],
                inflation_radius_m=max(
                    0.10,
                    float(self.get_parameter(
                        'deterministic_corridor_inflation_radius_m').value),
                ),
                start_search_radius_m=0.60,
                goal_search_radius_m=0.80,
            )
            if not path:
                continue
            self._deterministic_corridor_goal_progress = goal_progress
            planned = self._set_deterministic_goal(
                center_goal,
                'corridor_outbound',
                now_ros,
                path_override=path,
            )
            if planned:
                self._deterministic_corridor_no_forward_count = 0
            return planned
        # 地图初始前沿尚未连成远端时，仅尝试短距离中心线目标。
        if robot_progress >= hard_limit - 0.05:
            self._deterministic_corridor_no_forward_count += 1
            return False
        fallback_progress = min(robot_progress + 1.50, hard_limit)
        fallback = self._deterministic_world_point(fallback_progress, 0.0)
        # 固定赛场主走廊是连续直线。未知边界不应先交给 A*吸附到脚边，
        # 而应按 1.5m 中心线短目标交给已接入标准 laser_filters 的赛事 DWA；
        # DWA 负责最终局部可达性与避障。
        fallback_path = [fallback]
        if fallback_path and self._set_deterministic_goal(
                fallback, 'corridor_outbound', now_ros,
                path_override=fallback_path):
            self._deterministic_corridor_goal_progress = fallback_progress
            self._deterministic_corridor_no_forward_count = 0
            return True
        self._deterministic_corridor_no_forward_count += 1
        return False

    def _update_deterministic_room_distance(self) -> None:
        if self._deterministic_room_started_ros is None:
            return
        pose = (self.robot_x, self.robot_y)
        if self._deterministic_room_last_pose is not None:
            step = math.hypot(
                pose[0] - self._deterministic_room_last_pose[0],
                pose[1] - self._deterministic_room_last_pose[1])
            if 0.01 <= step <= 0.50:
                self._deterministic_room_path_m += step
        self._deterministic_room_last_pose = pose
        if self._has_fresh_official_control_odom():
            official_pose = (
                self._official_control_odom[0],
                self._official_control_odom[1],
            )
            if self._deterministic_room_last_official_pose is not None:
                physical_step = math.hypot(
                    official_pose[0]
                    - self._deterministic_room_last_official_pose[0],
                    official_pose[1]
                    - self._deterministic_room_last_official_pose[1],
                )
                if 0.01 <= physical_step <= 0.50:
                    self._deterministic_room_physical_path_m += physical_step
            self._deterministic_room_last_official_pose = official_pose

    def _prepare_strict_room_inspection(self, now_ros: float) -> bool:
        """在基础环线展开地图后建立严格逐障碍观察计划。"""

        sector = self._deterministic_room_sector
        doorway = self._room_sector_candidate_poses.get(sector or '')
        if (sector is None or doorway is None or self.grid is None
                or self.latest_map is None or self._entry_axis is None):
            return False
        axis_x, axis_y = self._entry_axis
        axis_norm = math.hypot(axis_x, axis_y)
        if axis_norm <= 1e-6:
            return False
        axis_x, axis_y = axis_x / axis_norm, axis_y / axis_norm
        normal_x, normal_y = -axis_y, axis_x
        side_sign = 1.0 if sector.endswith('left') else -1.0
        entry_yaw = math.atan2(
            side_sign * normal_y,
            side_sign * normal_x,
        )
        plan = build_strict_room_inspection_plan(
            self.grid,
            self.latest_map,
            entry_world=doorway,
            entry_yaw_rad=entry_yaw,
            start_world=(self.robot_x, self.robot_y),
            door_width_m=float(self.get_parameter(
                'strict_room_door_width_m').value),
            seed_offset_m=float(self.get_parameter(
                'strict_room_seed_offset_m').value),
            minimum_room_free_cells=int(self.get_parameter(
                'strict_room_min_free_cells').value),
            minimum_obstacle_area_m2=float(self.get_parameter(
                'strict_room_min_obstacle_area_m2').value),
            wall_margin_m=float(self.get_parameter(
                'strict_room_wall_margin_m').value),
            viewpoint_count=int(self.get_parameter(
                'strict_room_viewpoint_count').value),
            required_views_per_obstacle=int(self.get_parameter(
                'strict_room_required_views_per_obstacle').value),
            viewpoint_standoff_m=float(self.get_parameter(
                'strict_room_viewpoint_standoff_m').value),
            viewpoint_clearance_m=float(self.get_parameter(
                'strict_room_viewpoint_clearance_m').value),
            path_inflation_radius_m=float(self.get_parameter(
                'strict_room_path_inflation_radius_m').value),
        )
        self._room_inspection_execution = RoomInspectionExecution(plan)
        self._room_inspection_request_goal_id = ''
        self._room_inspection_result_goal_id = ''
        self._room_inspection_capture_started_ros = None
        if not plan.executable:
            details = ','.join(
                f'{item.obstacle_id}:'
                f'{item.reachable_direction_count}/'
                f'{item.required_direction_count}'
                for item in plan.uncovered_obstacles
            ) or 'room_mask_unavailable'
            self.recorder.record_failure(
                now_ros,
                'strict_room_inspection_plan_failed',
                self.robot_x,
                self.robot_y,
                detail=f'sector={sector}; {details}',
            )
            self.get_logger().error(
                f'Strict room inspection plan failed for {sector}: '
                f'{details}. Room will not be counted complete.')
            return False
        self.get_logger().info(
            f'Strict room inspection plan ready for {sector}: '
            f'obstacles={plan.obstacle_count}, goals={len(plan.goals)}, '
            f'room_cells={plan.room_mask_cell_count}.')
        return True

    def _finish_deterministic_room(self, now_ros: float) -> bool:
        sector = self._deterministic_room_sector
        if sector is None:
            return False
        elapsed = max(
            0.0,
            now_ros - (self._deterministic_room_started_ros or now_ros),
        )
        loop_area = abs(polygon_signed_area(
            self._deterministic_official_loop_samples))
        loop_perimeter = 0.0
        for index, point in enumerate(
                self._deterministic_official_loop_samples):
            next_point = self._deterministic_official_loop_samples[
                (index + 1)
                % len(self._deterministic_official_loop_samples)]
            loop_perimeter += math.hypot(
                next_point[0] - point[0],
                next_point[1] - point[1],
            )
        physical_path = self._deterministic_room_physical_path_m
        strict_physical_validation = bool(self.get_parameter(
            'use_official_odom_for_room_control').value)
        valid_loop = (
            not strict_physical_validation
            or physical_room_loop_is_valid(
                self._deterministic_official_loop_samples,
                self._deterministic_loop_reached,
                len(self._deterministic_room_waypoints),
                physical_path,
                float(self.get_parameter(
                    'deterministic_room_min_physical_path_m').value),
                float(self.get_parameter(
                    'deterministic_room_min_loop_area_m2').value),
            )
        )
        strict_inspection_enabled = bool(self.get_parameter(
            'strict_room_inspection_enabled').value)
        inspection_valid = (
            not strict_inspection_enabled
            or (
                self._room_inspection_execution is not None
                and self._room_inspection_execution.complete
            )
        )
        valid_loop = valid_loop and inspection_valid
        official_pose = (
            self._official_control_odom[:3]
            if self._has_fresh_official_control_odom()
            else None
        )
        inspection = self._room_inspection_execution
        obstacle_count = (
            inspection.plan.obstacle_count if inspection is not None else None)
        inspection_goal_count = (
            inspection.progress.required_goal_count
            if inspection is not None else None)
        inspection_completed_count = (
            inspection.progress.completed_goal_count
            if inspection is not None else None)
        if not valid_loop:
            self.recorder.record_room_coverage(
                now_ros, self._current_floor, sector, 'validation_failed',
                self.robot_x, self.robot_y,
                path_m=self._deterministic_room_path_m,
                duration_s=elapsed,
                probe_count=self._deterministic_loop_reached,
                reason=(
                    'strict_room_inspection_incomplete'
                    if not inspection_valid
                    else 'insufficient_physical_counterclockwise_loop'),
                official_pose=official_pose,
                physical_path_m=physical_path,
                loop_area_m2=loop_area,
                loop_perimeter_m=loop_perimeter,
                official_loop_samples=(
                    self._deterministic_official_loop_samples),
                obstacle_count=obstacle_count,
                inspection_goal_count=inspection_goal_count,
                inspection_completed_count=inspection_completed_count,
            )
            self.get_logger().error(
                f'Deterministic room validation failed: {sector}, '
                f'loop_points={self._deterministic_loop_reached}, '
                f'physical_path={physical_path:.2f}m, '
                f'loop_area={loop_area:.2f}m2; repeating the same room.')
            # 已回到门口，从穿门阶段重新执行同一固定逆时针环线；不弹出
            # 队列、不标记 visited，保证“4/4”只代表四个真实房间。
            self._deterministic_route_phase = 'room_cross'
            self._deterministic_room_waypoints = []
            self._deterministic_room_waypoint_index = 0
            self._deterministic_room_started_ros = None
            self._deterministic_room_last_pose = None
            self._deterministic_room_path_m = 0.0
            self._deterministic_room_last_official_pose = None
            self._deterministic_room_physical_path_m = 0.0
            self._deterministic_official_loop_samples = []
            self._deterministic_room_hold_yaw = None
            self._room_inspection_execution = None
            self._room_inspection_request_goal_id = ''
            self._room_inspection_result_goal_id = ''
            self._room_inspection_capture_started_ros = None
            self._deterministic_loop_reached = 0
            return False
        self._visited_room_sectors.add(sector)
        self.recorder.record_room_coverage(
            now_ros, self._current_floor, sector, 'completed',
            self.robot_x, self.robot_y,
            path_m=self._deterministic_room_path_m,
            duration_s=elapsed,
            probe_count=self._deterministic_loop_reached,
            reason=(
                'deterministic_loop_and_strict_inspection'
                if strict_inspection_enabled
                else 'deterministic_counterclockwise_loop'),
            official_pose=official_pose,
            physical_path_m=physical_path,
            loop_area_m2=loop_area,
            loop_perimeter_m=loop_perimeter,
            official_loop_samples=(
                self._deterministic_official_loop_samples),
            obstacle_count=obstacle_count,
            inspection_goal_count=inspection_goal_count,
            inspection_completed_count=inspection_completed_count,
        )
        self.get_logger().info(
            f'Deterministic room complete: {sector}, '
            f'loop_points={self._deterministic_loop_reached}, '
            f'path={self._deterministic_room_path_m:.2f}m, '
            f'physical_path={physical_path:.2f}m, '
            f'loop_perimeter={loop_perimeter:.2f}m, '
            f'loop_area={loop_area:.2f}m2, duration={elapsed:.1f}s.')
        if self._deterministic_room_queue \
                and self._deterministic_room_queue[0] == sector:
            self._deterministic_room_queue.pop(0)
        self._deterministic_room_sector = None
        self._deterministic_room_waypoints = []
        self._deterministic_room_waypoint_index = 0
        self._deterministic_room_started_ros = None
        self._deterministic_room_last_pose = None
        self._deterministic_room_path_m = 0.0
        self._deterministic_room_last_official_pose = None
        self._deterministic_room_physical_path_m = 0.0
        self._deterministic_official_loop_samples = []
        self._deterministic_room_hold_yaw = None
        self._room_inspection_execution = None
        self._room_inspection_request_goal_id = ''
        self._room_inspection_result_goal_id = ''
        self._room_inspection_capture_started_ros = None
        self._deterministic_loop_reached = 0
        self._deterministic_route_phase = 'room_approach'
        return True

    def _on_deterministic_waypoint_reached(
            self, label: str, now_ros: float) -> None:
        if label == 'corridor_outbound':
            return
        sector = self._deterministic_room_sector
        if label.startswith('room_door:') and sector is not None:
            self._deterministic_route_phase = 'room_cross'
            return
        if label.startswith('room_cross:') and sector is not None:
            self._deterministic_route_phase = 'room_loop'
            self._room_inspection_execution = None
            self._room_inspection_request_goal_id = ''
            self._room_inspection_result_goal_id = ''
            self._room_inspection_capture_started_ros = None
            self._deterministic_room_started_ros = now_ros
            self._deterministic_room_last_pose = (
                self.robot_x, self.robot_y)
            self._deterministic_room_physical_path_m = 0.0
            self._deterministic_official_loop_samples = []
            official_pose = None
            if self._has_fresh_official_control_odom():
                official_pose = self._official_control_odom[:3]
                self._deterministic_room_last_official_pose = (
                    official_pose[0], official_pose[1])
                self._deterministic_room_hold_yaw = official_pose[2]
            else:
                self._deterministic_room_last_official_pose = None
                self._deterministic_room_hold_yaw = None
            doorway = self._room_sector_candidate_poses[sector]
            side = 'left' if sector.endswith('left') else 'right'
            self._deterministic_room_waypoints = (
                build_counterclockwise_room_loop(
                    doorway,
                    self._entry_axis or (1.0, 0.0),
                    side,
                    float(self.get_parameter(
                        'deterministic_room_loop_shallow_m').value),
                    float(self.get_parameter(
                        'deterministic_room_loop_deep_m').value),
                    float(self.get_parameter(
                        'deterministic_room_loop_half_length_m').value),
                    float(self.get_parameter(
                        'deterministic_room_loop_corner_radius_m').value),
                )
            )
            self._deterministic_room_waypoint_index = 0
            self.recorder.record_room_coverage(
                now_ros, self._current_floor, sector, 'entered',
                self.robot_x, self.robot_y,
                official_pose=official_pose)
            self.get_logger().info(
                f'Deterministic room entered: {sector}; '
                'starting counterclockwise loop.')
            return
        if label.startswith('room_inspect_move:') and sector is not None:
            execution = self._room_inspection_execution
            if execution is not None:
                execution.mark_position_reached()
            return
        if label.startswith('room_inspect_orient:') and sector is not None:
            execution = self._room_inspection_execution
            if execution is not None:
                execution.mark_orientation_reached()
            return
        if label.startswith('room_loop:') and sector is not None:
            if self._has_fresh_official_control_odom():
                self._deterministic_official_loop_samples.append((
                    self._official_control_odom[0],
                    self._official_control_odom[1],
                ))
            self._deterministic_loop_reached += 1
            self._deterministic_room_waypoint_index += 1
            if self._deterministic_room_waypoint_index >= len(
                    self._deterministic_room_waypoints):
                if bool(self.get_parameter(
                        'strict_room_inspection_enabled').value):
                    if self._prepare_strict_room_inspection(now_ros):
                        execution = self._room_inspection_execution
                        self._deterministic_route_phase = (
                            'room_exit'
                            if execution is not None and execution.complete
                            else 'room_inspection'
                        )
                    else:
                        self._deterministic_route_phase = (
                            'room_inspection_failed')
                else:
                    self._deterministic_route_phase = 'room_exit'
            return
        if label.startswith('room_exit:'):
            self._finish_deterministic_room(now_ros)

    def _handle_deterministic_room_route(self, now_ros: float) -> Twist:
        """确定性走廊到底、返程逐房、房内逆时针单圈。"""

        cmd = Twist()
        required_sectors = {
            'far_left', 'far_right', 'near_left', 'near_right'}
        if required_sectors.issubset(self._visited_room_sectors):
            self.get_logger().info(
                f'Deterministic floor {self._current_floor} completed all '
                'four counterclockwise room loops.')
            if self._target_floors and self._next_floor() is not None:
                self._transition('FLOOR_COMPLETE')
            else:
                self._transition('RETURNING')
            return cmd
        if self._entry_axis is None:
            heading = self._entry_heading()
            self._entry_axis = (math.cos(heading), math.sin(heading))
            lateral_offset = float(self.get_parameter(
                'deterministic_corridor_center_lateral_m').value)
            self._entry_origin = (
                self.start_x - self._entry_axis[1] * lateral_offset,
                self.start_y + self._entry_axis[0] * lateral_offset,
            )
            self.get_logger().info(
                'Deterministic corridor axis calibrated: '
                f'origin=({self._entry_origin[0]:.2f}, '
                f'{self._entry_origin[1]:.2f}), '
                f'lateral_offset={lateral_offset:.2f}m.')
        self._seed_calibrated_doorways()
        self._update_deterministic_room_distance()
        # 门口候选只在“沿走廊走到底”阶段更新；开始返程逐房后坐标冻结。
        # 否则房内新 Frontier 会移动门口基准，表现为目标前后跳变和回摆。
        if (self._deterministic_route_phase == 'corridor_outbound'
                and simulation_period_elapsed(
                now_ros, self._last_replan_ros_sec,
                float(self.get_parameter('replan_interval_s').value))):
            self._refresh_deterministic_doorways()
            self._last_replan_ros_sec = now_ros

        phase = self._deterministic_route_phase
        if self.current_target is not None and self.current_path:
            if self._recover_stalled_deterministic_waypoint(now_ros):
                return cmd
            if phase == 'corridor_outbound':
                progress, _ = self._deterministic_axis_coordinates(
                    (self.robot_x, self.robot_y))
                direct_until = max(
                    0.0,
                    float(self.get_parameter(
                        'deterministic_entry_direct_until_progress_m').value),
                )
                if progress < direct_until:
                    clearance = float(self.get_parameter(
                        'deterministic_entry_clearance_m').value)
                    rotation_clearance = max(
                        0.0,
                        float(self.get_parameter(
                            'deterministic_entry_rotation_clearance_m').value),
                    )
                    return self._follow_path_with_direct_backend(
                        navigation_clearance_override=clearance,
                        rotation_clearance_override=rotation_clearance,
                    )
            if phase in (
                    'room_approach', 'room_cross', 'room_loop',
                    'room_inspection', 'room_exit'):
                return self._follow_path()
            return self._follow_path()

        if phase == 'corridor_outbound':
            progress, _ = self._deterministic_axis_coordinates(
                (self.robot_x, self.robot_y))
            far_progresses = [
                self._deterministic_axis_coordinates(
                    self._room_sector_candidate_poses[key])[0]
                for key in ('far_left', 'far_right')
                if key in self._room_sector_candidate_poses
            ]
            tolerance = max(
                0.2,
                float(self.get_parameter(
                    'deterministic_corridor_end_tolerance_m').value),
            )
            extra = max(
                0.0,
                float(self.get_parameter(
                    'deterministic_corridor_end_extra_m').value),
            )
            farthest_door_progress = (
                max(far_progresses) if far_progresses else None)
            minimum_corridor_progress, _hard_limit = (
                self._deterministic_corridor_limits())
            minimum_outbound_reached = (
                progress >= minimum_corridor_progress - tolerance)
            reached_beyond_farthest_doors = (
                farthest_door_progress is not None
                and minimum_outbound_reached
                and progress >= farthest_door_progress + extra - tolerance
            )
            corridor_stalled_at_end = (
                farthest_door_progress is not None
                and minimum_outbound_reached
                and progress >= farthest_door_progress - tolerance
                and self._deterministic_corridor_no_forward_count >= 2
                and len(self._room_sector_candidate_poses) == 4
            )
            if reached_beyond_farthest_doors or corridor_stalled_at_end:
                self._deterministic_room_queue = (
                    self._build_deterministic_room_queue())
                if len(self._deterministic_room_queue) != 4:
                    self.get_logger().warning(
                        'Corridor end observed but two paired doorway '
                        'stations are incomplete; continuing centerline '
                        'observation.',
                        throttle_duration_sec=5.0,
                    )
                    self._plan_reachable_corridor_goal(now_ros)
                    return cmd
                self._deterministic_route_phase = 'room_approach'
                self.current_target = None
                self.current_path = []
                self.path_index = 0
                self._deterministic_waypoint_label = ''
                reason = (
                    'no reachable centerline frontier past paired far doors'
                    if corridor_stalled_at_end
                    else 'progress beyond paired far doors')
                self.get_logger().info(
                    f'Corridor end reached by {reason}; '
                    'returning through rooms in order: '
                    f'{self._deterministic_room_queue}.')
                return cmd
            self._plan_reachable_corridor_goal(now_ros)
            return cmd if self.current_path == [] else self._follow_path()

        if phase == 'room_approach':
            if not self._deterministic_room_queue:
                self._deterministic_room_queue = (
                    self._build_deterministic_room_queue())
            if not self._deterministic_room_queue:
                self.get_logger().warning(
                    'Deterministic route lacks four doorway observations; '
                    'holding floor for more map updates.',
                    throttle_duration_sec=5.0)
                return cmd
            sector = self._deterministic_room_queue[0]
            self._deterministic_room_sector = sector
            doorway = self._room_sector_candidate_poses[sector]
            self._set_deterministic_goal(
                doorway, f'room_door:{sector}', now_ros)
            # 房间阶段继续使用接入过滤扫描的赛事 Unitree DWA，避免直控链路
            # 因官方水平雷达正前方自回波盲区而持续零速。
            return (
                cmd if self.current_path == []
                else self._follow_path()
            )

        sector = self._deterministic_room_sector
        if sector is None:
            self._deterministic_route_phase = 'room_approach'
            return cmd
        doorway = self._room_sector_candidate_poses[sector]
        progress, lateral = self._deterministic_axis_coordinates(doorway)
        side_sign = 1.0 if sector.endswith('left') else -1.0

        if phase == 'room_cross':
            depth = max(
                0.8,
                float(self.get_parameter(
                    'deterministic_room_cross_depth_m').value),
            )
            goal = self._deterministic_world_point(
                progress, lateral + side_sign * depth)
            self._set_deterministic_goal(
                goal, f'room_cross:{sector}', now_ros)
        elif phase == 'room_loop':
            if self._deterministic_room_waypoint_index >= len(
                    self._deterministic_room_waypoints):
                self._deterministic_route_phase = 'room_exit'
                return cmd
            index = self._deterministic_room_waypoint_index
            self._set_deterministic_goal(
                self._deterministic_room_waypoints[index],
                f'room_loop:{sector}:{index}', now_ros)
        elif phase == 'room_inspection':
            execution = self._room_inspection_execution
            if execution is None:
                self._deterministic_route_phase = 'room_inspection_failed'
                return cmd
            goal = execution.current_goal
            if execution.phase == execution.COMPLETE:
                self._deterministic_route_phase = 'room_exit'
                return cmd
            if execution.phase == execution.FAILED or goal is None:
                self._deterministic_route_phase = 'room_inspection_failed'
                return cmd
            if execution.phase == execution.MOVE:
                self._room_inspection_capture_started_ros = None
                self._set_deterministic_goal(
                    (goal.x_m, goal.y_m),
                    f'room_inspect_move:{goal.goal_id}',
                    now_ros,
                    path_override=list(goal.path),
                )
                return cmd if self.current_path == [] else self._follow_path()
            if execution.phase == execution.ORIENT:
                self._room_inspection_capture_started_ros = None
                self._set_deterministic_goal(
                    (goal.x_m, goal.y_m),
                    f'room_inspect_orient:{goal.goal_id}',
                    now_ros,
                    path_override=[(goal.x_m, goal.y_m)],
                )
                return cmd if self.current_path == [] else self._follow_path()
            if execution.phase == execution.CAPTURE:
                if self._room_inspection_capture_started_ros is None:
                    self._room_inspection_capture_started_ros = now_ros
                capture_timeout = max(
                    1.0,
                    float(self.get_parameter(
                        'strict_room_capture_timeout_s').value),
                )
                if (now_ros - self._room_inspection_capture_started_ros
                        >= capture_timeout):
                    execution.mark_motion_failure(
                        'inspection_capture_timeout')
                    self.recorder.record_failure(
                        now_ros,
                        'inspection_capture_timeout',
                        self.robot_x,
                        self.robot_y,
                        detail=f'goal_id={goal.goal_id}',
                    )
                    return cmd
                if self._room_inspection_request_goal_id != goal.goal_id:
                    request = String()
                    request.data = json.dumps({
                        'goal_id': goal.goal_id,
                        'obstacle_id': goal.obstacle_id,
                        'floor': int(self._current_floor),
                        'sector': str(sector),
                        'pose': {
                            'x': float(self.robot_x),
                            'y': float(self.robot_y),
                            'yaw': float(self.robot_yaw),
                        },
                    }, ensure_ascii=False)
                    self.inspection_request_pub.publish(request)
                    self._room_inspection_request_goal_id = goal.goal_id
                    self.get_logger().info(
                        f'Inspection capture requested: {goal.goal_id}.')
                return cmd
        elif phase == 'room_inspection_failed':
            self.get_logger().error(
                f'Room inspection failed on floor {self._current_floor} '
                f'sector {sector}; returning without counting the room.')
            self._transition('RETURNING')
            return cmd
        elif phase == 'room_exit':
            self._set_deterministic_goal(
                doorway, f'room_exit:{sector}', now_ros)
        return (
            cmd if self.current_path == []
            else self._follow_path()
        )

    def _handle_exploring(self) -> Twist:
        """EXPLORING: 前沿检测 → 路径规划 → 速度控制。"""
        cmd = Twist()

        if self.grid is None or self.latest_map is None:
            return cmd

        # 定期重规划
        now_ros = self._ros_time_sec()
        if not self._deterministic_route_enabled():
            self._update_room_sector_coverage()
        recovery_cmd = self._handle_exploration_recovery(now_ros)
        if recovery_cmd is not None:
            return recovery_cmd
        if (self._target_floors and self._next_floor() is not None
                and self._floor_started_ros is not None
                and now_ros >= self._floor_started_ros):
            floor_elapsed = now_ros - self._floor_started_ros
            floor_limit = max(
                1.0,
                float(self.get_parameter('per_floor_exploration_s').value),
            )
            early_finish_min = max(
                0.0,
                float(self.get_parameter(
                    'room_sector_early_finish_min_s').value),
            )
            # 逐房间闭环后才允许提前换层。走廊上看到轮廓不等于进入房间，
            # 四个扇区缺一项都继续探索，直到达到本层硬时限。
            required_sectors = {
                'far_left', 'far_right', 'near_left', 'near_right',
            }
            room_complete = required_sectors.issubset(
                self._visited_room_sectors)
            if ((room_complete and floor_elapsed >= early_finish_min)
                    or floor_elapsed >= floor_limit):
                if room_complete:
                    self.get_logger().info(
                        f'Floor {self._current_floor} all four room sectors '
                        f'completed in {floor_elapsed:.1f}s; requesting '
                        'elevator transition.'
                    )
                else:
                    missing = sorted(
                        required_sectors - self._visited_room_sectors)
                    self.get_logger().warn(
                        f'Floor {self._current_floor} hard time limit '
                        f'reached with incomplete room sectors: {missing}; '
                        'requesting elevator transition.'
                    )
                self._transition('FLOOR_COMPLETE')
                return cmd
        if self._deterministic_route_enabled():
            return self._handle_deterministic_room_route(now_ros)
        if self._frontier_observation_remaining_rad > 0.0:
            return self._handle_frontier_observation_sweep(now_ros)
        replan_interval = float(self.get_parameter('replan_interval_s').value)

        # 无目标时也遵守重规划间隔；否则 steady-clock 10 Hz 控制会每帧重复
        # 聚类整张地图并刷屏“All frontiers visited”，挤占传感器与控制回调。
        if simulation_period_elapsed(
                now_ros, self._last_replan_ros_sec, replan_interval):
            self._replan()
            self._last_replan_ros_sec = now_ros

        # 无前沿 → 探索完成，返航
        if self.current_target is None and len(self.current_path) == 0:
            if self._active_room_sector is not None:
                # 房间闭环任务未完成时绝不能把暂时没有前沿误判为整层完成。
                # 重规划会启动 360° 耗尽检查；两次地图更新之间保持停车。
                return cmd
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
            if self._active_room_sector is not None:
                # 即使受低实时率影响超时，环视期间也持续接收了完整 RGB-D/
                # 激光帧；下一次规划必须先再次检查房间前沿，不能直接离开。
                self._active_room_scan_completed = True
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
            resume_state = self._reobserve_resume_state
            self.get_logger().info(
                f'Reobservation complete, resuming {resume_state}.'
            )
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
            self._reobserve_last_pose = None
            self._reobserve_lateral_distance_m = 0.0
            self._reobserve_target_center_error_ratio = None
            self._reobserve_last_target_seen_ros = None
            self._reobserve_allow_untracked_upgrade = False
            # 强制重规划
            self.current_target = None
            self.current_path = []
            self._transition(resume_state)
            return cmd

        # 机动结束后必须停车等待机体稳定并采集确认帧；若持续运动到状态结束，
        # 感知的 camera_stable 门禁永远不会累计独立视角证据。
        if now >= self.reobserve_motion_end_time:
            return cmd

        # 根据感知建议生成短时机动 cmd_vel。
        action = self.reobserve_action or 'hold_observation'
        if (action in ('move_left', 'move_right')
                and self._reobserve_last_pose is not None):
            current_pose = (self.robot_x, self.robot_y)
            maximum_increment = max(
                0.05,
                float(self.get_parameter(
                    'reobserve_pose_jump_reject_m').value),
            )
            increment = bounded_planar_pose_increment(
                self._reobserve_last_pose,
                current_pose,
                maximum_increment,
            )
            # 即使本帧是地图坐标跳变，也要把它作为新锚点；否则下一帧会继续
            # 相对旧地图坐标产生同一个假大位移。总时长与激光门禁仍持续生效。
            self._reobserve_last_pose = current_pose
            if increment is None:
                self.get_logger().warning(
                    'Ignored implausible SLAM pose jump during reobservation; '
                    f'threshold={maximum_increment:.2f}m.'
                )
            else:
                self._reobserve_lateral_distance_m += increment
            lateral_distance = self._reobserve_lateral_distance_m
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

        if action in ('move_left', 'move_right'):
            error_ratio = self._reobserve_target_center_error_ratio
            deadband = max(
                0.0,
                float(self.get_parameter(
                    'reobserve_lateral_centering_deadband_ratio').value),
            )
            if error_ratio is not None and abs(error_ratio) > deadband:
                gain = max(
                    0.0,
                    float(self.get_parameter(
                        'reobserve_lateral_centering_gain').value),
                )
                maximum_turn = max(
                    0.0,
                    float(self.get_parameter(
                        'reobserve_lateral_centering_max_turn_speed').value),
                )
                angular = lateral_centering_angular_velocity(
                    error_ratio,
                    gain,
                    maximum_turn,
                    deadband,
                )
                turn_action = 'turn_left' if angular > 0.0 else 'turn_right'
                if self._scan_allows_action(
                        turn_action,
                        float(self.get_parameter(
                            'rotation_min_clearance_m').value)):
                    cmd.angular.z = angular

        return cmd

    def _official_return_control_active(self) -> bool:
        """最终楼层也允许用官方 odom 返到电梯大厅。"""

        configured_floor = int(self.get_parameter(
            'official_return_floor_index').value)
        final_target_floor = (
            self._target_floors[-1] if self._target_floors else None)
        return (
            bool(self.get_parameter(
                'use_official_odom_for_return_control').value)
            and self._has_fresh_official_control_odom()
            and (
                self._current_floor == configured_floor
                or self._current_floor == final_target_floor
            )
        )

    def _official_return_target(self) -> Tuple[float, float, float]:
        """多层任务最后一层返回电梯大厅；0 层仍返回楼外出生点。"""

        if (self._target_floors
                and self._current_floor == self._target_floors[-1]
                and self._current_floor > 0):
            final_goal = (
                float(self.get_parameter(
                    'official_elevator_lobby_x_m').value),
                float(self.get_parameter('official_elevator_y_m').value),
                math.pi,
            )
        else:
            final_goal = (
                float(self.get_parameter('official_home_x_m').value),
                float(self.get_parameter('official_home_y_m').value),
                float(self.get_parameter('official_home_yaw_rad').value),
            )
        if self._has_fresh_official_control_odom():
            official_x, official_y, _yaw, _stamp = (
                self._official_control_odom)
            if (abs(official_y - final_goal[1]) > 1.0
                    and abs(official_x) > 0.55):
                # 房间内不能向楼外/电梯画对角线穿墙。先退回当前门排中心
                # 线，再沿直走廊高速返程。
                return (
                    0.0,
                    official_y,
                    math.atan2(0.0, -official_x),
                )
        return final_goal

    def _handle_returning(self) -> Twist:
        """RETURNING: 优先逆序回放实走轨迹返航到起点。"""
        cmd = Twist()
        goal_tol = max(
            float(self.get_parameter('goal_tolerance_m').value),
            float(self.get_parameter('return_goal_tolerance_m').value),
        )
        now = self._ros_time_sec()

        use_official_return = self._official_return_control_active()
        if use_official_return:
            official_x, official_y, _yaw, _stamp = (
                self._official_control_odom)
            home_x, home_y, _home_yaw = self._official_return_target()
            official_distance = math.hypot(
                home_x - official_x, home_y - official_y)
            if official_distance <= max(
                    0.1,
                    float(self.get_parameter(
                        'official_home_tolerance_m').value)):
                self.get_logger().info(
                    'Arrived at official physical home. '
                    f'Distance={official_distance:.2f}m')
                self._transition('FINISHED')
                return cmd
            # 物理返程由 ROS1 DWA规划；dummy map点只用于保持路径跟踪调用，
            # 实际发布目标在 _publish_unitree_move_base_goal 中被替换为 odom家点。
            self.current_path = [(self.robot_x + 3.0, self.robot_y)]
            self.path_index = 0
            return self._follow_path_with_unitree_move_base()

        dist_home = self._distance_home_m()

        if dist_home <= goal_tol:
            self.get_logger().info(
                f'Arrived home. Distance={dist_home:.2f}m')
            self._transition('FINISHED')
            return cmd

        if dist_home <= 2.0:
            direct_command = self._direct_home_odom_command()
            if direct_command is not None:
                return direct_command

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
        odom_return = self._verified_return_odom_command()
        if odom_return is not None:
            return odom_return
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
            home_x, home_y = self._home_map_position()
            self.current_path = self._hybrid_return_path(
                home_x, home_y, dist_home)
            self.path_index = 0
            if len(self.current_path) == 0:
                self.get_logger().warn(
                    'No safe path home found; stopping and waiting for a map update.')
                return cmd
        elif len(self.current_path) == 0:
            return cmd

        cmd = self._follow_return_path()
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
        if self._elevator_odom_path:
            self._elevator_odom_path_index = min(
                len(self._elevator_odom_path),
                self._elevator_odom_path_index + 2,
            )
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
        except (ParameterUninitializedException, TypeError, ValueError):
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
        self._floor_started_ros = self._ros_time_sec()
        self.recorder.record_floor_change(
            self._ros_time_sec(), -1, self._current_floor, 'initial')
        self.get_logger().info(
            f'Multi-floor exploration enabled: '
            f'floors={self._target_floors}, '
            f'current={self._current_floor}')

    def _update_coverage(self):
        """以 2 Hz 降采样更新覆盖网格。"""
        if self.grid is None or self.latest_map is None:
            return
        now_ros = self._ros_time_sec()
        if not simulation_period_elapsed(
                now_ros, self._last_coverage_update_ros_sec, 0.5):
            return
        self._last_coverage_update_ros_sec = now_ros
        current_pose = (self.robot_x, self.robot_y)
        record_replay_history = self.state in ('EXPLORING', 'REOBSERVING')
        if record_replay_history:
            append_loop_erased_history(
                self._visited_pose_history, current_pose,
                spacing_m=0.10, loop_radius_m=0.75,
                min_index_gap=10)
        if record_replay_history and self._robot_odom is not None:
            append_loop_erased_history(
                self._visited_odom_history, self._robot_odom,
                spacing_m=0.10, loop_radius_m=0.75,
                min_index_gap=10)
        if self._coverage is None:
            return
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
        previous_floor = self._current_floor
        self._current_floor = next_floor
        self._transition_from_floor = previous_floor
        self._floor_complete_since_ros = None
        self._elevator_initiated = False
        self._elevator_floor_reached = False
        self._floor_transition_phase = (
            'manual_returning_lobby'
            if bool(self.get_parameter('manual_elevator_assist').value)
            else 'navigating'
        )
        self.current_target = None
        self.current_path = []
        self.path_index = 0
        self._last_return_plan_time = None
        # 换层前返梯与任务结束返航共用同一套进展看门狗。必须在每层
        # 切换开始时清零，否则上一层留下的距离基准会立即误触发恢复转向。
        self._return_best_distance_home = None
        self._return_last_progress_time = None
        self._return_last_progress_pose = None
        self._return_last_net_progress_time = None
        self._return_net_progress_reference_distance = None
        self._return_recovery_attempts = 0
        self._return_recovery_turn_start_ros = None
        self._return_recovery_turn_end_ros = None
        self._return_recovery_turn_command = 0.0
        self._return_recovery_start_yaw = None
        self._return_recovery_scan_blocked_logged = False
        self._manual_elevator_ready = False
        self._auto_elevator_entry_attempts = 0
        self._auto_entry_alignment_index = 0
        self._auto_entry_best_distance = None
        self._auto_entry_last_progress_ros = None
        self._auto_entry_last_pose = None
        self._auto_entry_backoff_until_ros = None
        self._door_closed_scan = None
        self._door_scan_wait_stamp = None
        self._door_target_odom = None
        self._door_search_attempts = 0
        self._door_search_target_yaw = None
        self._door_validation_rejections = 0
        self._door_approach_started_ros = None
        self._door_rotation_started_ros = None
        self._floor_transition_start_z = self.robot_z
        self._floor_transition_start_ros = now_ros
        self._floor_exit_target = None
        self._floor_exit_target_odom = None
        self._elevator_odom_path = []
        self._elevator_odom_path_index = 0
        self.recorder.record_floor_change(
            now_ros, previous_floor, next_floor, 'elevator')
        self.get_logger().info(
            f'Floor {previous_floor} complete. '
            f'Transitioning to floor {next_floor}.')
        self._transition('FLOOR_TRANSITION')
        return cmd

    def _start_elevator_request(
        self,
        now_ros: float,
        stage: str,
        target_floor: int,
        container: str,
        elevator_id: str,
        open_doors: bool = True,
    ) -> None:
        """异步提交一次电梯请求，保持导航控制心跳不中断。"""
        if self._elevator_future is not None:
            return
        if now_ros < self._elevator_next_retry_ros:
            return
        self._elevator_request_stage = stage
        self._elevator_request_floor = target_floor
        official_mode = bool(self.get_parameter(
            'use_official_odom_for_elevator_control').value)
        official_pose = self._official_control_odom

        def execute_request() -> ElevatorResult:
            result = call_elevator(
                container,
                elevator_id,
                target_floor,
                open_doors,
                30.0,
            )
            if not official_mode or not result.accepted:
                return result

            if stage == 'riding' and official_pose is not None:
                moved = set_robot_floor(
                    container,
                    target_floor,
                    official_pose[0],
                    official_pose[1],
                    official_pose[2],
                    model_name=str(self.get_parameter(
                        'official_robot_model_name').value),
                    ground_robot_z_m=float(self.get_parameter(
                        'official_robot_ground_z_m').value),
                    floor_height_m=float(self.get_parameter(
                        'official_floor_height_m').value),
                    timeout_s=10.0,
                )
                if not moved:
                    return ElevatorResult(
                        False, result.current_floor, 'robot_transfer_failed',
                        'elevator moved but robot floor transfer failed')

            if open_doors:
                door = set_door_state(
                    container,
                    elevator_door_id(target_floor),
                    True,
                    timeout_s=float(self.get_parameter(
                        'official_elevator_door_timeout_s').value),
                )
                if not door.accepted:
                    return ElevatorResult(
                        False, result.current_floor, 'door_open_failed',
                        f'physical floor door rejected: {door.state}')
            return result

        self._elevator_future = self._elevator_executor.submit(
            execute_request,
        )

    def _take_elevator_result(
        self,
        now_ros: float,
    ) -> Optional[Tuple[str, int, ElevatorResult]]:
        """非阻塞获取已完成请求；失败后限频重试，避免刷爆 Docker。"""
        future = self._elevator_future
        if future is None or not future.done():
            return None
        stage = self._elevator_request_stage
        target_floor = self._elevator_request_floor
        self._elevator_future = None
        self._elevator_request_stage = ''
        self._elevator_request_floor = None
        try:
            result = future.result()
        except Exception as exc:
            self._elevator_next_retry_ros = now_ros + 2.0
            self.get_logger().error(f'Elevator {stage} request failed: {exc}')
            return None
        if target_floor is None:
            return None
        return stage, target_floor, result

    def _begin_new_floor_exploration(self) -> None:
        """切换分层地图并清理上一层前沿状态。"""

        self.get_logger().info(
            f'Beginning exploration on floor {self._current_floor}')
        # 每层以电梯落点建立独立返梯锚点。跨层后的map→odom会因高度和
        # 新子图调整，继续使用0层原点会把本层返航目标投到错误位置。
        self.start_x = self.robot_x
        self.start_y = self.robot_y
        self.home_x = self.robot_x
        self.home_y = self.robot_y
        if self._robot_odom is not None:
            self._home_odom = self._robot_odom
        self._home_map_last = (self.robot_x, self.robot_y)
        self._entry_origin = (self.robot_x, self.robot_y)
        self._entry_axis = None
        self._visited_pose_history.clear()
        self._visited_odom_history.clear()
        self._publish_floor_index(self._current_floor)
        self.current_target = None
        self.current_path = []
        self._visited_frontiers.clear()
        self._unreachable_frontiers.clear()
        self._detour_deferred_frontiers.clear()
        self._reset_frontier_progress_watchdog()
        self._current_target_was_ingress = False
        self._visited_room_sectors.clear()
        self._attempted_room_sectors.clear()
        self._room_sector_doorway_poses.clear()
        self._room_sector_candidate_poses.clear()
        self._room_selection_mode = 'unconstrained'
        self._reset_active_room_coverage()
        self._reset_deterministic_room_route()
        if self.grid is not None:
            height, width = self.grid.shape
            self._coverage = CoverageGrid(height, width)
        self._floor_transition_phase = ''
        self._floor_started_ros = self._ros_time_sec()
        self._manual_elevator_ready = False
        self._transition_from_floor = None
        self._floor_transition_start_z = None
        self._transition('EXPLORING')

    def _handle_manual_floor_transition(self) -> Twist:
        """自动或人工入梯，运输成功均由合法三维SLAM高度变化验收。"""

        cmd = Twist()
        now_ros = self._ros_time_sec()
        container = str(self.get_parameter('simenv_container').value)
        elevator_id = str(self.get_parameter('elevator_id').value)
        if self._floor_transition_phase == 'manual_returning_lobby':
            self._request_control_mode('navigation')
            home_x, home_y = self._home_map_position()
            distance = math.hypot(
                self.robot_x - home_x, self.robot_y - home_y)
            if (distance <= max(
                    0.40,
                    float(self.get_parameter('goal_tolerance_m').value))
                    or self._distance_home_m() <= 1.50):
                self.current_path = []
                self.path_index = 0
                if bool(self.get_parameter(
                        'automatic_elevator_entry').value):
                    source_floor = (
                        self._transition_from_floor
                        if self._transition_from_floor is not None else 0)
                    if self._validated_elevator_cabin_odom is not None:
                        self._door_target_odom = (
                            self._validated_elevator_cabin_odom[0],
                            self._validated_elevator_cabin_odom[1],
                        )
                        self._elevator_odom_path = [self._door_target_odom]
                        self._elevator_odom_path_index = 0
                        self._auto_entry_best_distance = (
                            None if self._robot_odom is None
                            else math.hypot(
                                self._robot_odom[0] - self._door_target_odom[0],
                                self._robot_odom[1] - self._door_target_odom[1],
                            )
                        )
                        self._auto_entry_last_progress_ros = now_ros
                        self._door_approach_started_ros = now_ros
                        self._door_validation_rejections = 0
                        self._floor_transition_phase = 'auto_door_approaching'
                        self.get_logger().info(
                            f'Elevator lobby reached on floor {source_floor}; '
                            'reusing the previously platform-validated cabin '
                            'odom anchor before transport.')
                    else:
                        self._floor_transition_phase = (
                            'auto_door_localizing_close')
                        self.get_logger().info(
                            f'Elevator lobby reached on floor {source_floor}; '
                            'starting local door relocalization before '
                            'autonomous transport.')
                else:
                    self._floor_transition_phase = 'manual_waiting'
                    self.get_logger().info(
                        'Elevator lobby reached. Keyboard control enabled; '
                        'drive A1 into the cabin and call elevator_ready.')
                return cmd
            if distance <= 2.0:
                direct_command = self._direct_home_odom_command()
                if direct_command is not None:
                    return direct_command
            recovery_cmd = self._return_recovery_command_for_now(now_ros)
            if recovery_cmd is not None:
                return recovery_cmd
            force_replan = self._return_progress_watchdog_expired(
                now_ros, distance,
            )
            recovery_cmd = self._return_recovery_command_for_now(now_ros)
            if recovery_cmd is not None:
                return recovery_cmd
            odom_return = self._verified_return_odom_command()
            if odom_return is not None:
                return odom_return

            # 换层返梯优先逆序回放本层真实走过的合法 odom 轨迹。对称房间
            # 回环后，A* 可能在两条外观相似的走廊间选出错误捷径；实走轨迹
            # 虽略长，但已通过碰撞与门宽验证，可靠性优先于几何最短路。
            if (force_replan
                    or not self.current_path
                    or self._last_return_plan_time is None
                    or now_ros < self._last_return_plan_time):
                self.current_path = self._hybrid_return_path(
                    home_x, home_y, distance)
                self.path_index = 0
                self._last_return_plan_time = now_ros
            if self.current_path:
                return self._follow_return_path()
            self.get_logger().warning(
                'No safe SLAM path to elevator lobby; waiting for a later '
                'map update.', throttle_duration_sec=3.0)
            return cmd

        if self._floor_transition_phase == 'auto_door_localizing_close':
            self._request_control_mode('navigation')
            source_floor = (
                self._transition_from_floor
                if self._transition_from_floor is not None else 0)
            self._start_elevator_request(
                now_ros, 'door_localize_close', source_floor,
                container, elevator_id, open_doors=False)
            completed = self._take_elevator_result(now_ros)
            if completed is None:
                return cmd
            stage, request_floor, result = completed
            self.recorder.record_elevator_call(
                now_ros, elevator_id, request_floor, stage, result.state)
            if not result.accepted:
                self.get_logger().error(
                    f'Unable to close elevator door for scan localization: '
                    f'{result.message}')
                return cmd
            self._door_scan_wait_stamp = self._last_scan_stamp
            self._floor_transition_phase = 'auto_door_capture_closed'
            return cmd

        if self._floor_transition_phase == 'auto_door_capture_closed':
            if (self.latest_scan is None
                    or self._last_scan_stamp == self._door_scan_wait_stamp):
                return cmd
            self._door_closed_scan = (
                [float(value) for value in self.latest_scan.ranges],
                float(self.latest_scan.angle_min),
                float(self.latest_scan.angle_increment),
            )
            self._floor_transition_phase = 'auto_door_localizing_open'
            return cmd

        if self._floor_transition_phase == 'auto_door_localizing_open':
            source_floor = (
                self._transition_from_floor
                if self._transition_from_floor is not None else 0)
            self._start_elevator_request(
                now_ros, 'door_localize_open', source_floor,
                container, elevator_id, open_doors=True)
            completed = self._take_elevator_result(now_ros)
            if completed is None:
                return cmd
            stage, request_floor, result = completed
            self.recorder.record_elevator_call(
                now_ros, elevator_id, request_floor, stage, result.state)
            if not result.accepted:
                self.get_logger().error(
                    f'Unable to open elevator door for scan localization: '
                    f'{result.message}')
                return cmd
            self._door_scan_wait_stamp = self._last_scan_stamp
            self._floor_transition_phase = 'auto_door_capture_open'
            return cmd

        if self._floor_transition_phase == 'auto_door_capture_open':
            if (self.latest_scan is None
                    or self._last_scan_stamp == self._door_scan_wait_stamp
                    or self._door_closed_scan is None):
                return cmd
            closed, angle_min, angle_increment = self._door_closed_scan
            opening = detect_opened_door_from_scans(
                closed, self.latest_scan.ranges,
                angle_min, angle_increment,
            )
            if opening is None:
                self._door_search_attempts += 1
                if (self._door_search_attempts >= 4
                        or self._robot_odom_yaw is None):
                    self.get_logger().warning(
                        'Elevator door scan difference unavailable after '
                        'active search; using bounded odom candidate fallback.')
                    self._floor_transition_phase = 'auto_opening_entry'
                    return cmd
                self._door_search_target_yaw = normalize_angle(
                    self._robot_odom_yaw + math.pi / 2.0)
                self._door_rotation_started_ros = now_ros
                self._floor_transition_phase = 'auto_door_search_rotating'
                return cmd
            bearing, closed_distance, changed_bins = opening
            if self._robot_odom is None or self._robot_odom_yaw is None:
                return cmd
            target_distance = max(
                0.40, min(5.0, float(closed_distance) + 0.60))
            target_yaw = self._robot_odom_yaw + float(bearing)
            self._door_target_odom = (
                self._robot_odom[0] + target_distance * math.cos(target_yaw),
                self._robot_odom[1] + target_distance * math.sin(target_yaw),
            )
            self._elevator_odom_path = [self._door_target_odom]
            self._elevator_odom_path_index = 0
            self._auto_entry_best_distance = target_distance
            self._auto_entry_last_progress_ros = now_ros
            self._door_validation_rejections = 0
            self._floor_transition_start_z = self.robot_z
            self._floor_transition_start_ros = now_ros
            self._floor_transition_phase = 'manual_calling'
            self.get_logger().info(
                f'Elevator door localized from scan difference: '
                f'bearing={math.degrees(bearing):.1f}deg, '
                f'range={closed_distance:.2f}m, bins={changed_bins}; '
                'requesting platform validation before approach.')
            return cmd

        if self._floor_transition_phase == 'auto_door_search_rotating':
            if (self._robot_odom_yaw is None
                    or self._door_search_target_yaw is None):
                return cmd
            heading_error = normalize_angle(
                self._door_search_target_yaw - self._robot_odom_yaw)
            if (abs(heading_error) <= 0.15
                    or (self._door_rotation_started_ros is not None
                        and now_ros - self._door_rotation_started_ros
                        >= 15.0)):
                self._door_closed_scan = None
                self._door_scan_wait_stamp = None
                self._door_rotation_started_ros = None
                self._floor_transition_phase = 'auto_door_localizing_close'
                return cmd
            rotation_command = Twist()
            rotation_command.angular.z = max(
                -0.60, min(0.60, heading_error))
            if 0.0 < abs(rotation_command.angular.z) < 0.30:
                rotation_command.angular.z = math.copysign(
                    0.30, rotation_command.angular.z)
            return rotation_command

        if self._floor_transition_phase == 'auto_door_approaching':
            if self._door_target_odom is None or self._robot_odom is None:
                self._floor_transition_phase = 'auto_door_localizing_close'
                return cmd
            distance = math.hypot(
                self._robot_odom[0] - self._door_target_odom[0],
                self._robot_odom[1] - self._door_target_odom[1],
            )
            if distance <= 0.50:
                self._elevator_odom_path = []
                self._elevator_odom_path_index = 0
                self._floor_transition_start_z = self.robot_z
                self._floor_transition_start_ros = now_ros
                self._floor_transition_phase = 'manual_calling'
                self.get_logger().info(
                    'Localized elevator doorway reached; requesting '
                    'platform payload validation.')
                return cmd
            if (self._auto_entry_best_distance is None
                    or distance <= self._auto_entry_best_distance - 0.08):
                self._auto_entry_best_distance = distance
                self._auto_entry_last_progress_ros = now_ros
            stall_timeout = max(
                1.0, float(self.get_parameter(
                    'elevator_entry_stall_timeout_s').value))
            if (self._auto_entry_last_progress_ros is not None
                    and (now_ros - self._auto_entry_last_progress_ros
                         >= stall_timeout
                         or (self._door_approach_started_ros is not None
                             and now_ros - self._door_approach_started_ros
                             >= 25.0))):
                self._elevator_odom_path = []
                self._elevator_odom_path_index = 0
                self._floor_transition_start_z = self.robot_z
                self._floor_transition_start_ros = now_ros
                self._floor_transition_phase = 'manual_calling'
                self.get_logger().info(
                    'Door approach reached the physical threshold; '
                    'requesting platform payload validation.')
                return cmd
            if not self._elevator_odom_path:
                self._elevator_odom_path = [self._door_target_odom]
                self._elevator_odom_path_index = 0
            return self._follow_elevator_odom_path(entering=True)

        if self._floor_transition_phase == 'auto_opening_entry':
            self._request_control_mode('navigation')
            source_floor = (
                self._transition_from_floor
                if self._transition_from_floor is not None
                else self._current_floor)
            self._start_elevator_request(
                now_ros, 'auto_entry_open', source_floor,
                container, elevator_id, open_doors=True)
            completed = self._take_elevator_result(now_ros)
            if completed is None:
                return cmd
            stage, request_floor, result = completed
            self.recorder.record_elevator_call(
                now_ros, elevator_id, request_floor, stage, result.state)
            if not result.accepted:
                self.get_logger().error(
                    f'Unable to open elevator for autonomous entry: '
                    f'{result.message}')
                self._floor_transition_phase = 'manual_waiting'
                return cmd
            self.current_path = []
            self.path_index = 0
            self._auto_entry_best_distance = None
            self._auto_entry_last_progress_ros = now_ros
            self._auto_entry_last_pose = self._robot_odom
            self._floor_transition_phase = 'auto_entering'

        if self._floor_transition_phase == 'auto_entering':
            self._request_control_mode('navigation')
            if self._auto_entry_backoff_until_ros is not None:
                if now_ros < self._auto_entry_backoff_until_ros:
                    # 门框卡住时先沿机体后方退出到大厅净空，再重规划。
                    return self._elevator_backoff_command()
                self._auto_entry_backoff_until_ros = None
            if (self._home_odom is None
                    or self._elevator_anchor_odom is None
                    or self._robot_odom is None
                    or self._robot_odom_yaw is None):
                self.get_logger().warning(
                    'Waiting for legal odom before autonomous elevator entry.',
                    throttle_duration_sec=3.0)
                return cmd
            # 室内SLAM在入口朝向上建立+x；建筑公共布局中电梯位于
            # 入门右侧，因此右向为-y。失败复试时逐次向轿厢深处多走20cm。
            source_floor = (
                self._transition_from_floor
                if self._transition_from_floor is not None else 0)
            upper_floor_entry = source_floor > 0
            alignment_offsets = (
                (0.0,) if upper_floor_entry
                else ELEVATOR_ENTRY_ALIGNMENT_OFFSETS)
            alignment_index = min(
                self._auto_entry_alignment_index,
                len(alignment_offsets) - 1)
            entry_right_offset = float(self.get_parameter(
                'elevator_entry_right_offset_m').value)
            exit_right_offset = float(self.get_parameter(
                'elevator_exit_right_offset_m').value)
            known_upper_cabin = (
                self._elevator_cabin_odom_by_floor.get(source_floor)
                if upper_floor_entry else None)
            if known_upper_cabin is not None:
                # 直接逆向回到该层成功出梯前记录的轿厢点；这与平台为
                # 动态载荷做的横向居中无关，完全使用合法 odom 轨迹。
                target_forward = 0.0
                target_left = 0.0
                target_anchor = known_upper_cabin
            elif upper_floor_entry:
                target_forward = 0.0
                target_left = -(
                    entry_right_offset - exit_right_offset
                    + 0.20 * self._auto_elevator_entry_attempts)
                target_anchor = self._home_odom
            else:
                target_forward = (
                    float(self.get_parameter(
                        'elevator_entry_forward_offset_m').value)
                    + alignment_offsets[alignment_index])
                target_left = -(
                    entry_right_offset
                    + 0.20 * self._auto_elevator_entry_attempts)
                target_anchor = self._elevator_anchor_odom
            doorway_left = -float(self.get_parameter(
                'elevator_exit_right_offset_m').value)
            doorway_odom = (
                target_anchor[0] + target_forward,
                target_anchor[1] + (0.0 if upper_floor_entry else doorway_left),
            )
            target_odom = (
                target_anchor[0] + target_forward,
                target_anchor[1] + target_left,
            )
            entry_distance = math.hypot(
                self._robot_odom[0] - target_odom[0],
                self._robot_odom[1] - target_odom[1])
            entry_tolerance = max(
                0.10, float(self.get_parameter(
                    'elevator_entry_tolerance_m').value))
            if upper_floor_entry:
                entry_tolerance = max(
                    0.10, float(self.get_parameter(
                        'elevator_upper_floor_entry_tolerance_m').value))
            if (self._auto_entry_best_distance is None
                    or entry_distance
                    <= self._auto_entry_best_distance - 0.08):
                self._auto_entry_best_distance = entry_distance
                self._auto_entry_last_progress_ros = now_ros
            if (self._auto_entry_last_pose is None
                    or math.hypot(
                        self._robot_odom[0] - self._auto_entry_last_pose[0],
                        self._robot_odom[1] - self._auto_entry_last_pose[1])
                    >= 0.12):
                # 对门重试会先远离最终目标退回走廊；只要机体确实在移动，
                # 就不能误判为卡死并提前切换下一个候选。
                self._auto_entry_last_pose = self._robot_odom
                self._auto_entry_last_progress_ros = now_ros
            stall_timeout = max(
                1.0, float(self.get_parameter(
                    'elevator_entry_stall_timeout_s').value))
            if (self._auto_entry_last_progress_ros is not None
                    and now_ros >= self._auto_entry_last_progress_ros
                    and now_ros - self._auto_entry_last_progress_ros
                    >= stall_timeout):
                if self._auto_entry_alignment_index + 1 >= len(
                        alignment_offsets):
                    # 一轮候选均被门框阻挡时从近端重新扫描，并略微增加
                    # 横向入梯深度；全流程保持自动，不退回人工确认。
                    self._auto_entry_alignment_index = 0
                    self._auto_elevator_entry_attempts += 1
                else:
                    self._auto_entry_alignment_index += 1
                self._auto_entry_best_distance = None
                self._auto_entry_last_progress_ros = now_ros
                self._auto_entry_last_pose = self._robot_odom
                self._auto_entry_backoff_until_ros = now_ros + 4.0
                adjusted_forward = (
                    float(self.get_parameter(
                        'elevator_entry_forward_offset_m').value)
                    + alignment_offsets[self._auto_entry_alignment_index])
                adjusted_doorway = (
                    target_anchor[0] + adjusted_forward,
                    target_anchor[1] + doorway_left,
                )
                adjusted_target = (
                    target_anchor[0] + adjusted_forward,
                    target_anchor[1] + target_left,
                )
                # 先退回本层大厅，再纵向换一个门口候选，最后横向入梯。
                self._elevator_odom_path = [
                    self._home_odom,
                    adjusted_doorway,
                    adjusted_target,
                ]
                self._elevator_odom_path_index = 0
                self.get_logger().warning(
                    'Elevator entry stalled; retrying doorway alignment '
                    f'with forward offset '
                    f'{adjusted_forward:.2f}m.')
                return cmd
            if entry_distance <= entry_tolerance:
                self._elevator_odom_path = []
                self._elevator_odom_path_index = 0
                self._floor_transition_start_z = self.robot_z
                self._floor_transition_start_ros = now_ros
                self._floor_transition_phase = 'manual_calling'
                self.get_logger().info(
                    f'Autonomous elevator entry reached: '
                    f'distance={entry_distance:.2f}m; starting transport.')
                return cmd
            if not self._elevator_odom_path:
                # 先沿主走廊到达电梯门正前方，再横向进入轿厢；大厅墙角会
                # 挡住直达对角线。轿厢属于未知空间，因此仍由360°激光门禁
                # 保护这两段短路径，不依赖膨胀栅格把门内判成已知自由区。
                self._elevator_odom_path = [
                    doorway_odom,
                    target_odom,
                ]
                self._elevator_odom_path_index = 0
            return self._follow_elevator_odom_path(entering=True)

        if self._floor_transition_phase == 'manual_waiting':
            self._request_control_mode('keyboard')
            if not self._manual_elevator_ready:
                return cmd
            self._request_control_mode('navigation')
            self._floor_transition_start_z = self.robot_z
            self._floor_transition_start_ros = now_ros
            self._floor_transition_phase = 'manual_calling'

        if self._floor_transition_phase == 'manual_calling':
            self._request_control_mode('navigation')
            self._start_elevator_request(
                now_ros, 'manual_transport', self._current_floor,
                container, elevator_id, open_doors=True)
            completed = self._take_elevator_result(now_ros)
            if completed is None:
                return cmd
            stage, request_floor, result = completed
            self.recorder.record_elevator_call(
                now_ros, elevator_id, request_floor, stage, result.state)
            if not result.accepted:
                self.get_logger().error(
                    f'Elevator transport rejected: {result.message}')
                self._manual_elevator_ready = False
                source_floor = (
                    self._transition_from_floor
                    if self._transition_from_floor is not None else 0)
                if (bool(self.get_parameter(
                        'automatic_elevator_entry').value)
                        and self._door_target_odom is not None
                        and self._door_validation_rejections == 0
                        and self._robot_odom is not None):
                    self._door_validation_rejections = 1
                    self._elevator_odom_path = [self._door_target_odom]
                    self._elevator_odom_path_index = 0
                    self._auto_entry_best_distance = math.hypot(
                        self._robot_odom[0] - self._door_target_odom[0],
                        self._robot_odom[1] - self._door_target_odom[1],
                    )
                    self._auto_entry_last_progress_ros = now_ros
                    self._door_approach_started_ros = now_ros
                    self._floor_transition_phase = 'auto_door_approaching'
                    self.get_logger().info(
                        'Platform rejected lobby pose; approaching the '
                        'localized elevator doorway before retry.')
                    return cmd
                if source_floor <= 0:
                    self._auto_entry_alignment_index += 1
                    if self._auto_entry_alignment_index >= len(
                            ELEVATOR_ENTRY_ALIGNMENT_OFFSETS):
                        self._auto_entry_alignment_index = 0
                        self._auto_elevator_entry_attempts += 1
                else:
                    self._auto_elevator_entry_attempts += 1
                self._auto_entry_best_distance = None
                self._auto_entry_last_progress_ros = now_ros
                self._auto_entry_last_pose = self._robot_odom
                self._auto_entry_backoff_until_ros = now_ros + 4.0
                self._elevator_odom_path = []
                self._elevator_odom_path_index = 0
                if bool(self.get_parameter(
                        'automatic_elevator_entry').value):
                    self._floor_transition_phase = (
                        'auto_door_localizing_close')
                    self._door_closed_scan = None
                    self._door_scan_wait_stamp = None
                    self._door_target_odom = None
                else:
                    self._floor_transition_phase = 'manual_waiting'
                return cmd
            # 电梯公开动作已确认目标楼层。先发布合法楼层动作，使相对里程计
            # 更新z并隔离旧层扫描；随后仍要求Cartographer高度实际变化，
            # 服务成功本身不能直接宣告跨层完成。
            if self._robot_odom is not None:
                self._validated_elevator_cabin_odom = (
                    self._robot_odom[0],
                    self._robot_odom[1],
                )
                self.get_logger().info(
                    'Stored platform-validated elevator cabin odom anchor '
                    f'at ({self._robot_odom[0]:.2f}, '
                    f'{self._robot_odom[1]:.2f}).'
                )
            self._publish_floor_index(self._current_floor)
            self._floor_transition_phase = 'manual_waiting_height'
            self._floor_transition_start_ros = now_ros

        if self._floor_transition_phase == 'manual_waiting_height':
            self._request_control_mode('navigation')
            source_floor = (
                self._transition_from_floor
                if self._transition_from_floor is not None
                else self._current_floor)
            floor_delta = abs(self._current_floor - source_floor)
            expected_height = (
                floor_delta
                * float(self.get_parameter('floor_height_m').value))
            required_height = max(
                0.8,
                expected_height * float(self.get_parameter(
                    'elevator_height_confirm_ratio').value),
            )
            start_z = self._floor_transition_start_z
            if (start_z is not None
                    and abs(self.robot_z - start_z) >= required_height):
                self.get_logger().info(
                    f'Elevator height confirmed by legal 3D SLAM: '
                    f'{start_z:.2f} -> {self.robot_z:.2f} m.')
                if (bool(self.get_parameter(
                        'automatic_elevator_entry').value)
                        and bool(self.get_parameter(
                            'platform_unloads_elevator_payload').value)):
                    if self._robot_odom is not None:
                        self._elevator_cabin_odom_by_floor[
                            self._current_floor] = self._robot_odom
                    self.get_logger().info(
                        'Platform payload transport completed at the '
                        'target-floor safe lobby; starting Frontier.')
                    self._begin_new_floor_exploration()
                    return cmd
                if bool(self.get_parameter(
                        'automatic_elevator_entry').value):
                    if self._robot_odom is None:
                        self.get_logger().error(
                            'Legal odom pose is missing for elevator exit.')
                        self._floor_transition_phase = 'manual_waiting'
                        return cmd
                    entry_right_offset = float(self.get_parameter(
                        'elevator_entry_right_offset_m').value)
                    exit_right_offset = float(self.get_parameter(
                        'elevator_exit_right_offset_m').value)
                    self._elevator_cabin_odom_by_floor[
                        self._current_floor] = self._robot_odom
                    exit_lateral_distance = max(
                        0.2, entry_right_offset - exit_right_offset)
                    self._floor_exit_target_odom = (
                        self._robot_odom[0],
                        self._robot_odom[1] + exit_lateral_distance,
                    )
                    # 只横向离开轿厢到门外安全大厅点，避免越过楼板边界；
                    # 该点随后成为本层返航锚点。
                    self._elevator_odom_path = [
                        self._floor_exit_target_odom,
                    ]
                    self._elevator_odom_path_index = 0
                    self._floor_transition_phase = 'auto_exiting_floor'
                    self.get_logger().info(
                        'Elevator transport confirmed; autonomously exiting '
                        'to the new-floor lobby before Frontier starts.')
                else:
                    self._begin_new_floor_exploration()
                return cmd
            timeout = float(self.get_parameter(
                'elevator_transport_timeout_s').value)
            if (self._floor_transition_start_ros is not None
                    and now_ros - self._floor_transition_start_ros
                    >= max(1.0, timeout)):
                self.get_logger().error(
                    'Elevator service returned but 3D SLAM height did not '
                    'change; robot was not inside the cabin.')
                self.recorder.record_failure(
                    now_ros, 'elevator_payload_not_transported',
                    self.robot_x, self.robot_y,
                    f'start_z={start_z} current_z={self.robot_z}')
                self._manual_elevator_ready = False
                if bool(self.get_parameter(
                        'automatic_elevator_entry').value):
                    self._auto_elevator_entry_attempts += 1
                    self._floor_transition_phase = 'auto_opening_entry'
                else:
                    self._floor_transition_phase = 'manual_waiting'
            return cmd

        if self._floor_transition_phase == 'auto_exiting_floor':
            self._request_control_mode('navigation')
            if (self._floor_exit_target_odom is None
                    or self._robot_odom is None):
                self.get_logger().error(
                    'New-floor elevator exit target is missing.')
                self._floor_transition_phase = 'manual_waiting'
                return cmd
            exit_distance = math.hypot(
                self._robot_odom[0] - self._floor_exit_target_odom[0],
                self._robot_odom[1] - self._floor_exit_target_odom[1])
            if exit_distance <= max(
                    0.30,
                    float(self.get_parameter('goal_tolerance_m').value)):
                self._elevator_odom_path = []
                self._elevator_odom_path_index = 0
                self._floor_exit_target = None
                self._floor_exit_target_odom = None
                self.get_logger().info(
                    'New-floor elevator exit completed; starting Frontier.')
                self._begin_new_floor_exploration()
                return cmd
            if self._elevator_odom_path:
                return self._follow_elevator_odom_path(entering=False)
            self.get_logger().warning(
                'Elevator exit path ended before the lobby target; '
                'holding for diagnosis.', throttle_duration_sec=3.0)
            return cmd

        return cmd

    def _official_elevator_distance(self, cabin: bool) -> float:
        if not self._has_fresh_official_control_odom():
            return math.inf
        target_x = float(self.get_parameter(
            'official_elevator_cabin_x_m' if cabin
            else 'official_elevator_lobby_x_m').value)
        target_y = float(self.get_parameter('official_elevator_y_m').value)
        official_x, official_y, _yaw, _stamp = self._official_control_odom
        return math.hypot(target_x - official_x, target_y - official_y)

    def _follow_official_elevator_target(self) -> Twist:
        self.current_path = [(self.robot_x + 1.0, self.robot_y)]
        self.path_index = 0
        return self._follow_path_with_unitree_move_base()

    def _handle_official_floor_transition(self) -> Twist:
        """按公开楼宇坐标完成大厅→轿厢→目标层→大厅四阶段。"""

        cmd = Twist()
        now_ros = self._ros_time_sec()
        container = str(self.get_parameter('simenv_container').value)
        elevator_id = str(self.get_parameter('elevator_id').value)
        tolerance = max(
            0.1,
            float(self.get_parameter(
                'official_elevator_goal_tolerance_m').value),
        )

        completed = self._take_elevator_result(now_ros)
        if completed is not None:
            stage, request_floor, result = completed
            self.recorder.record_elevator_call(
                now_ros, elevator_id, request_floor, stage, result.state)
            if stage == 'called' and result.accepted:
                self._elevator_initiated = True
                self._floor_transition_phase = 'entering'
                self._cancel_unitree_move_base()
                self.get_logger().info(
                    f'Official elevator open on floor {request_floor}; '
                    'entering cabin.')
            elif (stage == 'riding' and result.accepted
                    and result.current_floor == request_floor):
                self._elevator_floor_reached = True
                self._floor_transition_phase = 'exiting'
                self._publish_floor_index(request_floor)
                self._cancel_unitree_move_base()
                self.get_logger().info(
                    f'Official elevator arrived at floor {request_floor}; '
                    'exiting to lobby.')
            else:
                self._elevator_next_retry_ros = now_ros + 2.0
                self.get_logger().warning(
                    f'Official elevator {stage} request rejected: '
                    f'{result.message}')

        phase = self._floor_transition_phase
        if phase == 'navigating':
            if self._official_elevator_distance(cabin=False) > tolerance:
                return self._follow_official_elevator_target()
            self._cancel_unitree_move_base()
            self._floor_transition_phase = 'calling'
            phase = 'calling'
            self.get_logger().info(
                'Official elevator lobby reached; opening current-floor door.')

        if phase == 'calling':
            if self._elevator_future is None and not self._elevator_initiated:
                entry_floor = (
                    self._transition_from_floor
                    if self._transition_from_floor is not None
                    else max(0, self._current_floor - 1)
                )
                self._start_elevator_request(
                    now_ros, 'called', int(entry_floor),
                    container, elevator_id)
            return cmd

        if phase == 'entering':
            if self._official_elevator_distance(cabin=True) > tolerance:
                return self._follow_official_elevator_target()
            self._cancel_unitree_move_base()
            if self._elevator_future is None:
                self._start_elevator_request(
                    now_ros, 'riding', self._current_floor,
                    container, elevator_id)
                self._floor_transition_phase = 'riding'
                self.get_logger().info(
                    f'Cabin entered; requesting floor {self._current_floor}.')
            return cmd

        if phase == 'riding':
            return cmd

        if phase == 'exiting':
            if self._official_elevator_distance(cabin=False) > tolerance:
                return self._follow_official_elevator_target()
            self._cancel_unitree_move_base()
            self.get_logger().info(
                'Official elevator exit completed; starting new floor.')
            self._begin_new_floor_exploration()
            return cmd
        return cmd

    def _handle_floor_transition(self) -> Twist:
        """FLOOR_TRANSITION: 导航到电梯 → 呼叫电梯 → 跨层 → 新层探索。"""
        if bool(self.get_parameter('manual_elevator_assist').value):
            return self._handle_manual_floor_transition()
        if bool(self.get_parameter(
                'use_official_odom_for_elevator_control').value):
            return self._handle_official_floor_transition()
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
                    cmd = self._follow_return_path()
                return cmd
            self._floor_transition_phase = 'calling'
            self._floor_transition_start_ros = now_ros
            self.get_logger().info('Arrived at elevator. Calling...')
        if self._floor_transition_phase == 'calling':
            if not self._elevator_initiated:
                entry_floor = int(
                    self.get_parameter('elevator_entry_floor').value)
                self._start_elevator_request(
                    now_ros, 'called', entry_floor, container, elevator_id)
                completed = self._take_elevator_result(now_ros)
                if completed is not None:
                    stage, request_floor, result = completed
                    self.recorder.record_elevator_call(
                        now_ros, elevator_id, request_floor,
                        stage, result.state)
                    if stage == 'called' and result.accepted:
                        self._elevator_initiated = True
                        self.get_logger().info(
                            f'Elevator called to floor {request_floor}: '
                            f'{result.state}')
                    else:
                        self.get_logger().warn(
                            f'Elevator call rejected: {result.message}')
            if (self._elevator_initiated
                    and now_ros - (self._floor_transition_start_ros or now_ros) > 5.0):
                self._floor_transition_phase = 'entering'
                self._floor_transition_start_ros = now_ros
        if self._floor_transition_phase == 'entering':
            if not self._elevator_floor_reached:
                self._start_elevator_request(
                    now_ros,
                    'entered',
                    self._current_floor,
                    container,
                    elevator_id,
                )
                completed = self._take_elevator_result(now_ros)
                if completed is not None:
                    stage, request_floor, result = completed
                    self.recorder.record_elevator_call(
                        now_ros, elevator_id, request_floor,
                        stage, result.state)
                    if (stage == 'entered' and result.accepted
                            and result.current_floor == request_floor):
                        self._elevator_floor_reached = True
                        self.get_logger().info(
                            f'Arrived at floor {request_floor}')
                        self._publish_floor_index(request_floor)
        if self._elevator_floor_reached or (
                self._floor_transition_start_ros is not None
                and now_ros - self._floor_transition_start_ros > 60.0):
            self._begin_new_floor_exploration()
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
            self.robot_z = transform.transform.translation.z
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

        try:
            odom_transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter('odom_frame').value),
                str(self.get_parameter('base_frame').value),
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.2),
            )
            self._robot_odom = (
                float(odom_transform.transform.translation.x),
                float(odom_transform.transform.translation.y),
            )
            odom_q = odom_transform.transform.rotation
            self._robot_odom_yaw = math.atan2(
                2.0 * (odom_q.w * odom_q.z + odom_q.x * odom_q.y),
                1.0 - 2.0 * (
                    odom_q.y * odom_q.y + odom_q.z * odom_q.z),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

    def _distance_home_m(self) -> float:
        """优先用合法 odom 相对家点计算距离，缺失时回退当前 map。"""

        if self._home_odom is not None and self._robot_odom is not None:
            return math.hypot(
                self._robot_odom[0] - self._home_odom[0],
                self._robot_odom[1] - self._home_odom[1],
            )
        return math.hypot(
            self.robot_x - self.start_x,
            self.robot_y - self.start_y,
        )

    def _home_map_position(self) -> Tuple[float, float]:
        """把合法 odom 家点按当前 map→odom TF 重投影为规划目标。"""

        if self._home_odom is None:
            return self.start_x, self.start_y
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter('map_frame').value),
                str(self.get_parameter('odom_frame').value),
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.2),
            )
            q = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            self._home_map_last = transform_planar_point(
                self._home_odom[0],
                self._home_odom[1],
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                yaw,
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, ValueError):
            pass
        return self._home_map_last or (self.start_x, self.start_y)

    def _home_relative_map_position(
            self, forward_m: float, left_m: float) -> Tuple[float, float]:
        """在稳定odom帧计算相对大厅目标，再整体变换到当前map帧。"""

        if self._home_odom is None:
            home_x, home_y = self._home_map_position()
            heading = self._entry_heading()
            return (
                home_x + math.cos(heading) * float(forward_m)
                - math.sin(heading) * float(left_m),
                home_y + math.sin(heading) * float(forward_m)
                + math.cos(heading) * float(left_m),
            )
        target_odom_x = self._home_odom[0] + float(forward_m)
        target_odom_y = self._home_odom[1] + float(left_m)
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter('map_frame').value),
                str(self.get_parameter('odom_frame').value),
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.2),
            )
            q = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return transform_planar_point(
                target_odom_x, target_odom_y,
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                yaw,
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, ValueError):
            home_x, home_y = self._home_map_position()
            return home_x + float(forward_m), home_y + float(left_m)

    def _hybrid_return_path(
            self, home_x: float, home_y: float,
            distance_home_m: float) -> List[Tuple[float, float]]:
        """近距离走 A*，远距离逆序回放；任一不可用时回退另一方案。"""

        def a_star_candidate():
            if self.grid is None:
                return []
            return a_star_path(
                self.grid, self.latest_map,
                self.robot_x, self.robot_y, home_x, home_y,
                start_search_radius_m=0.50,
                goal_search_radius_m=0.0,
                append_exact_goal=True,
            )

        path = self._verified_reverse_return_path(home_x, home_y)
        return path if path else a_star_candidate()

    def _direct_home_odom_command(self) -> Optional[Twist]:
        """在大厅最后 2 米内直接跟踪合法 odom home，并保留激光门禁。"""

        if self._home_odom is None or self._robot_odom is None:
            return None
        self._elevator_odom_path = [self._home_odom]
        self._elevator_odom_path_index = 0
        return self._follow_elevator_odom_path(
            entering=True,
            return_mode=True,
            final_approach=True,
        )

    def _verified_return_odom_command(self) -> Optional[Twist]:
        """直接在 odom 帧倒放实走轨迹，隔离 map/odom 回环漂移。"""

        if (self._robot_odom is None or self._home_odom is None
                or not self._visited_odom_history):
            return None
        if not self._elevator_odom_path:
            self._elevator_odom_path = build_reverse_history_path(
                self._visited_odom_history,
                self._robot_odom[0], self._robot_odom[1],
                self._home_odom[0], self._home_odom[1],
                spacing_m=float(self.get_parameter(
                    'return_waypoint_spacing_m').value),
            )
            self._elevator_odom_path_index = 0
        if not self._elevator_odom_path:
            return None
        return self._follow_elevator_odom_path(
            entering=True,
            return_mode=True,
        )

    def _verified_reverse_return_path(
            self, home_map_x: float, home_map_y: float):
        """优先倒放稳定odom轨迹，并一次性投影到当前map帧。"""

        if (self._robot_odom is not None
                and self._home_odom is not None
                and self._visited_odom_history):
            odom_path = build_reverse_history_path(
                self._visited_odom_history,
                self._robot_odom[0], self._robot_odom[1],
                self._home_odom[0], self._home_odom[1],
                spacing_m=float(self.get_parameter(
                    'return_waypoint_spacing_m').value),
            )
            try:
                transform = self.tf_buffer.lookup_transform(
                    str(self.get_parameter('map_frame').value),
                    str(self.get_parameter('odom_frame').value),
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.2),
                )
                q = transform.transform.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
                return [
                    transform_planar_point(
                        point[0], point[1],
                        float(transform.transform.translation.x),
                        float(transform.transform.translation.y),
                        yaw,
                    )
                    for point in odom_path
                ]
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException, ValueError):
                pass
        return build_reverse_history_path(
            self._visited_pose_history,
            self.robot_x, self.robot_y,
            home_map_x, home_map_y,
            spacing_m=float(self.get_parameter(
                'return_waypoint_spacing_m').value),
        )

    def _has_fresh_pose(self) -> bool:
        """只有新鲜合法 TF 才允许状态推进和控制。"""

        if self._last_pose_monotonic is None:
            return False
        timeout = float(self.get_parameter('pose_fresh_timeout_s').value)
        return time.monotonic() - self._last_pose_monotonic <= max(0.1, timeout)

    def _has_fresh_imu(self) -> bool:
        """正式运动必须有新鲜且曾确认直立的公开机体 IMU。"""

        if not bool(self.get_parameter('require_upright_imu').value):
            return True
        if not self._upright_imu_armed or self._last_imu_monotonic is None:
            return False
        timeout = max(
            0.1,
            float(self.get_parameter('imu_fresh_timeout_s').value),
        )
        return time.monotonic() - self._last_imu_monotonic <= timeout

    def _handle_fall_detected(self, now_ros: float) -> None:
        """撤销倒地前后的无效房间证据，并进入显式失败终态。"""

        if self._fall_latched or self._fall_pending_tilt_deg is None:
            return
        tilt = float(self._fall_pending_tilt_deg)
        self._fall_pending_tilt_deg = None
        self._fall_latched = True

        if self._active_room_sector is not None:
            sector = self._active_room_sector
            elapsed = (
                0.0 if self._active_room_started_ros is None
                else max(0.0, now_ros - self._active_room_started_ros)
            )
            self.recorder.record_room_coverage(
                now_ros,
                self._current_floor,
                sector,
                'aborted',
                self.robot_x,
                self.robot_y,
                path_m=self._active_room_path_m,
                duration_s=elapsed,
                probe_count=len(self._active_room_probe_points),
                reason=f'fall_detected_tilt_{tilt:.1f}deg',
            )
            self._reset_active_room_coverage()
        elif (
            self._last_completed_room_sector is not None
            and self._last_room_completion_ros is not None
            and now_ros >= self._last_room_completion_ros
            and now_ros - self._last_room_completion_ros <= max(
                0.0,
                float(self.get_parameter(
                    'fall_completion_invalidation_window_s').value),
            )
        ):
            sector = self._last_completed_room_sector
            self._visited_room_sectors.discard(sector)
            self.recorder.record_room_coverage(
                now_ros,
                self._last_completed_room_floor
                if self._last_completed_room_floor is not None
                else self._current_floor,
                sector,
                'invalidated',
                self.robot_x,
                self.robot_y,
                reason=f'fall_detected_tilt_{tilt:.1f}deg',
            )

        self.recorder.record_failure(
            now_ros,
            'fall_detected',
            self.robot_x,
            self.robot_y,
            f'body_tilt_deg={tilt:.2f}; navigation stopped',
        )
        self.current_target = None
        self.current_path = []
        self.path_index = 0
        self._current_target_selected_ros_sec = None
        self._reset_frontier_progress_watchdog()
        self._transition('FAILED')

    def _entry_heading(self) -> float:
        """返回合法入楼朝向：优先官方 profile 参数，其次第一条动态 TF。"""

        configured = float(self.get_parameter('entry_heading_yaw').value)
        if math.isfinite(configured):
            return configured
        if self._initial_heading_yaw is not None:
            return self._initial_heading_yaw
        return self.robot_yaw

    def _entry_backbone_active(self) -> bool:
        """是否仍处于入口主轴的快速走廊骨架阶段。"""

        time_limit = max(
            0.0,
            float(self.get_parameter('entry_ingress_time_limit_s').value),
        )
        now_ros = self._ros_time_sec()
        if (time_limit > 0.0
                and self._floor_started_ros is not None
                and now_ros >= self._floor_started_ros
                and now_ros - self._floor_started_ros >= time_limit):
            return False
        progress = entry_axis_progress_m(
            self.robot_x,
            self.robot_y,
            self._entry_origin,
            self._entry_axis,
        )
        return entry_ingress_constraint_active(
            self._entry_axis,
            progress,
            max(
                0.0,
                float(self.get_parameter('entry_ingress_depth_m').value),
            ),
        )

    def _update_room_sector_coverage(self) -> None:
        """进入房间后累计闭环轨迹，门口观望或浅入不算完成。"""

        if self._entry_backbone_active():
            return
        now_ros = self._ros_time_sec()
        if self._active_room_sector is not None:
            current_pose = (self.robot_x, self.robot_y)
            if self._active_room_last_pose is not None:
                step = math.hypot(
                    current_pose[0] - self._active_room_last_pose[0],
                    current_pose[1] - self._active_room_last_pose[1],
                )
                # SLAM 闭环跳变不属于真实覆盖距离。
                if 0.02 <= step <= 0.50:
                    self._active_room_path_m += step
            self._active_room_last_pose = current_pose

            elapsed = (
                0.0 if self._active_room_started_ros is None
                else max(0.0, now_ros - self._active_room_started_ros)
            )
            loop_distance = (
                float('inf') if self._active_room_entry_pose is None
                else math.hypot(
                    self.robot_x - self._active_room_entry_pose[0],
                    self.robot_y - self._active_room_entry_pose[1],
                )
            )
            minimum_room_path = max(
                0.0,
                float(self.get_parameter(
                    'room_perimeter_min_path_m').value),
            )
            minimum_room_duration = max(
                0.0,
                float(self.get_parameter(
                    'room_perimeter_min_duration_s').value),
            )
            coverage_distance_met = (
                self._active_room_path_m >= minimum_room_path
                and elapsed >= minimum_room_duration
            )
            if (
                coverage_distance_met
                and loop_distance <= max(
                    0.1,
                    float(self.get_parameter(
                        'room_perimeter_loop_radius_m').value),
                )
            ):
                self._finish_active_room_coverage('trajectory_loop_closed')
            elif (
                coverage_distance_met
                and not self._active_room_return_attempted
            ):
                # 已取得足够房间内轨迹后立即回门闭环。继续追逐不断刷新的
                # 小前沿只会增加重复路线，挤占其余房间与感知搜索时间。
                self.current_target = None
                self.current_path = []
                self.path_index = 0
                self._current_target_selected_ros_sec = None
                self._reset_frontier_progress_watchdog()
                self._plan_active_room_return_to_entry(now_ros)
            return

        sector = corridor_room_sector(
            self.robot_x,
            self.robot_y,
            self._entry_origin,
            self._entry_axis,
            split_depth_m=float(self.get_parameter(
                'room_sector_split_depth_m').value),
            lateral_entry_m=float(self.get_parameter(
                'room_sector_visit_lateral_m').value),
        )
        if sector is None or sector in self._visited_room_sectors:
            return
        self._active_room_sector = sector
        self._active_room_entry_pose = (self.robot_x, self.robot_y)
        self._active_room_started_ros = now_ros
        self._active_room_last_pose = (self.robot_x, self.robot_y)
        self._active_room_path_m = 0.0
        self._active_room_scan_completed = False
        self._active_room_probe_points = []
        self._active_room_probe_exhausted = False
        self._active_room_return_attempted = False
        self._room_sector_doorway_poses[sector] = (
            self.robot_x, self.robot_y,
        )
        self.recorder.record_room_coverage(
            now_ros,
            self._current_floor,
            sector,
            'entered',
            self.robot_x,
            self.robot_y,
        )
        self.get_logger().info(
            'Room sector entered; starting perimeter coverage before it can '
            f'be marked complete: {sector}'
        )
        # 当前目标通常只是门口或房间首个前沿。立即释放它，下一次规划会
        # 锁定同一扇区更深处的前沿，直到闭环或确认前沿耗尽。
        self.current_target = None
        self.current_path = []
        self.path_index = 0
        self._current_target_selected_ros_sec = None
        self._current_target_was_ingress = False
        self._reset_frontier_progress_watchdog()
        self.get_logger().info(
            f'Room sector {sector} is locked as the active coverage task.'
        )

    def _reset_active_room_coverage(self) -> None:
        """清空当前房间闭环状态；换层和完成房间时统一调用。"""

        self._active_room_sector = None
        self._active_room_entry_pose = None
        self._active_room_started_ros = None
        self._active_room_last_pose = None
        self._active_room_path_m = 0.0
        self._active_room_scan_completed = False
        self._active_room_probe_points = []
        self._active_room_probe_exhausted = False
        self._active_room_return_attempted = False

    def _finish_active_room_coverage(self, reason: str) -> None:
        """以可审计的轨迹/前沿证据完成当前房间，并切换下一房间。"""

        sector = self._active_room_sector
        if sector is None:
            return
        elapsed = (
            0.0 if self._active_room_started_ros is None
            else max(0.0, self._ros_time_sec() - self._active_room_started_ros)
        )
        now_ros = self._ros_time_sec()
        self._visited_room_sectors.add(sector)
        self._last_completed_room_sector = sector
        self._last_completed_room_floor = self._current_floor
        self._last_room_completion_ros = now_ros
        self.get_logger().info(
            'Room perimeter coverage complete: '
            f'sector={sector}, reason={reason}, '
            f'path={self._active_room_path_m:.2f}m, '
            f'duration={elapsed:.1f}s, '
            f'completed={sorted(self._visited_room_sectors)}'
        )
        self.recorder.record_room_coverage(
            now_ros,
            self._current_floor,
            sector,
            'completed',
            self.robot_x,
            self.robot_y,
            path_m=self._active_room_path_m,
            duration_s=elapsed,
            probe_count=len(self._active_room_probe_points),
            reason=reason,
        )
        self._reset_active_room_coverage()
        self.current_target = None
        self.current_path = []
        self.path_index = 0
        self._current_target_selected_ros_sec = None
        self._current_target_was_ingress = False
        self._reset_frontier_progress_watchdog()

    def _active_room_can_finish_after_scan(self) -> bool:
        """房间前沿耗尽后，要求足够停留并完成环视再接受覆盖完成。"""

        if (
            self._active_room_sector is None
            or not self._active_room_scan_completed
            or self._active_room_started_ros is None
        ):
            return False
        elapsed = max(0.0, self._ros_time_sec() - self._active_room_started_ros)
        return (
            elapsed >= max(
                0.0,
                float(self.get_parameter(
                    'room_perimeter_min_duration_s').value),
            )
            and (
                self._active_room_path_m >= max(
                    0.0,
                    float(self.get_parameter(
                        'room_perimeter_min_path_m').value),
                )
                or self._active_room_probe_exhausted
            )
            and self._active_room_return_attempted
        )

    def _start_active_room_exhaustion_sweep(self) -> None:
        """房间暂时无前沿时做完整环视，验证不是相机盲区导致的假耗尽。"""

        if (
            self._active_room_sector is None
            or self._active_room_scan_completed
            or self._frontier_observation_remaining_rad > 0.0
        ):
            return
        sweep_rad = 2.0 * math.pi
        self._frontier_observation_remaining_rad = sweep_rad
        self._frontier_observation_last_yaw = self.robot_yaw
        self._frontier_observation_started_ros = self._ros_time_sec()
        self.get_logger().info(
            f'Active room {self._active_room_sector} has no remaining '
            'frontier; starting a 360 deg exhaustion sweep.'
        )

    def _plan_active_room_probe(self, now_ros: float) -> bool:
        """从合法占用图中选取分散自由点，补齐房间边界覆盖而不读真值。"""

        if (
            self._active_room_sector is None
            or self._active_room_entry_pose is None
            or self._entry_origin is None
            or self._entry_axis is None
            or self.grid is None
            or self.latest_map is None
        ):
            return False
        maximum_probes = max(
            0,
            int(self.get_parameter('room_perimeter_probe_count').value),
        )
        if len(self._active_room_probe_points) >= maximum_probes:
            return False

        resolution = max(0.01, float(self.latest_map.info.resolution))
        stride = max(1, int(round(0.40 / resolution)))
        minimum_spacing = max(
            0.5,
            float(self.get_parameter('room_perimeter_probe_spacing_m').value),
        )
        lateral_limit = max(
            3.0,
            float(self.get_parameter('entry_lateral_limit_m').value),
        )
        maximum_progress = max(
            float(self.get_parameter('room_sector_split_depth_m').value) + 4.0,
            float(self.get_parameter('entry_ingress_depth_m').value) + 6.0,
        )
        anchors = [self._active_room_entry_pose]
        anchors.extend(self._active_room_probe_points)
        scored = []
        height, width = self.grid.shape
        for gy in range(0, height, stride):
            for gx in range(0, width, stride):
                value = int(self.grid[gy, gx])
                if value < 0 or value > FREE_MAX:
                    continue
                wx, wy = grid_to_world(gx, gy, self.latest_map)
                if corridor_room_sector(
                    wx, wy,
                    self._entry_origin,
                    self._entry_axis,
                    split_depth_m=float(self.get_parameter(
                        'room_sector_split_depth_m').value),
                    lateral_entry_m=float(self.get_parameter(
                        'room_sector_visit_lateral_m').value),
                ) != self._active_room_sector:
                    continue
                progress = entry_axis_progress_m(
                    wx, wy, self._entry_origin, self._entry_axis,
                )
                if progress is None or progress > maximum_progress:
                    continue
                axis_x, axis_y = self._entry_axis
                axis_norm = max(1e-6, math.hypot(axis_x, axis_y))
                delta_x = wx - self._entry_origin[0]
                delta_y = wy - self._entry_origin[1]
                lateral = (
                    -delta_x * axis_y + delta_y * axis_x
                ) / axis_norm
                if abs(lateral) > lateral_limit:
                    continue
                spacing = min(
                    math.hypot(wx - anchor[0], wy - anchor[1])
                    for anchor in anchors
                )
                if spacing < minimum_spacing:
                    continue
                # 最大化与既有路径锚点的分离，同时略偏向房间外沿，依次
                # 形成几个分散探针，而不是反复走同一条中心线。
                score = spacing + 0.12 * abs(lateral)
                scored.append((score, wx, wy))

        for score, wx, wy in heapq.nlargest(40, scored):
            path = a_star_path(
                self.grid,
                self.latest_map,
                self.robot_x,
                self.robot_y,
                wx,
                wy,
                inflation_radius_m=float(self.get_parameter(
                    'room_entry_inflation_radius_m').value),
            )
            if not path:
                continue
            probe = Frontier(
                centroid=(wx, wy),
                size=1,
                points=[],
                info_gain=float(score),
            )
            self._active_room_probe_points.append((wx, wy))
            self._accept_frontier_plan(
                probe, path, now_ros, self._entry_heading(), None,
            )
            self._room_selection_mode = 'active_room_free_space_probe'
            self.get_logger().info(
                f'Room perimeter probe {len(self._active_room_probe_points)}/'
                f'{maximum_probes}: sector={self._active_room_sector}, '
                f'target=({wx:.2f}, {wy:.2f}).'
            )
            return True
        return False

    def _plan_active_room_return_to_entry(self, now_ros: float) -> bool:
        """房间采样完成后沿已知自由区回到门口，形成可验证闭环。"""

        if (
            self._active_room_entry_pose is None
            or self.grid is None
            or self.latest_map is None
            or self._active_room_return_attempted
        ):
            return False
        self._active_room_return_attempted = True
        target_x, target_y = self._active_room_entry_pose
        path = a_star_path(
            self.grid,
            self.latest_map,
            self.robot_x,
            self.robot_y,
            target_x,
            target_y,
            inflation_radius_m=float(self.get_parameter(
                'room_entry_inflation_radius_m').value),
        )
        if not path:
            self.get_logger().warn(
                f'Room {self._active_room_sector} perimeter return path is '
                'temporarily unavailable; validating coverage by scan.'
            )
            return False
        target = Frontier(
            centroid=(target_x, target_y),
            size=1,
            points=[],
            info_gain=0.0,
        )
        self._accept_frontier_plan(
            target, path, now_ros, self._entry_heading(), None,
        )
        self._room_selection_mode = 'active_room_loop_return'
        self.get_logger().info(
            f'Room {self._active_room_sector} coverage distance reached; returning '
            'to the recorded doorway to close the coverage loop.'
        )
        return True

    def _handle_active_room_frontiers_exhausted(self) -> bool:
        """处理房间内无剩余前沿；返回是否拦截普通整层完成逻辑。"""

        if self._active_room_sector is None:
            return False
        if self._active_room_can_finish_after_scan():
            self._finish_active_room_coverage(
                'frontiers_exhausted_after_360_scan',
            )
            return True
        if self._active_room_return_attempted:
            # 回门路径已经生成时必须保持它为唯一活动目标；普通地图前沿
            # 可能暂时为零，不能因此清空返回路径或重新生成房间探针。
            if self.current_target is not None and self.current_path:
                return True
            self._start_active_room_exhaustion_sweep()
            return True
        now_ros = self._ros_time_sec()
        maximum_probes = max(
            0,
            int(self.get_parameter('room_perimeter_probe_count').value),
        )
        if (
            len(self._active_room_probe_points) < maximum_probes
            and self._plan_active_room_probe(now_ros)
        ):
            return True
        if len(self._active_room_probe_points) < maximum_probes:
            self._active_room_probe_exhausted = True
        if self._plan_active_room_return_to_entry(now_ros):
            return True
        self._start_active_room_exhaustion_sweep()
        return True

    def _frontier_room_sector(self, frontier: Optional[Frontier]) -> Optional[str]:
        """返回候选所属房间扇区，供窄门规划和实时净空选择。"""

        if frontier is None:
            return None
        return corridor_room_sector(
            frontier.centroid[0],
            frontier.centroid[1],
            self._entry_origin,
            self._entry_axis,
            split_depth_m=float(self.get_parameter(
                'room_sector_split_depth_m').value),
            lateral_entry_m=float(self.get_parameter(
                'room_sector_candidate_lateral_m').value),
        )

    def _cache_room_doorway_candidates(self, frontiers) -> None:
        """缓存合法 SLAM 曾出现的门带前沿，防止走廊扫描后未知边界消失。"""

        if self._entry_origin is None or self._entry_axis is None:
            return
        axis_x, axis_y = self._entry_axis
        norm = math.hypot(axis_x, axis_y)
        if norm <= 1e-6:
            return
        axis_x /= norm
        axis_y /= norm
        candidate_min = max(
            0.05,
            float(self.get_parameter(
                'room_sector_candidate_lateral_m').value),
        )
        candidate_max = max(
            candidate_min,
            float(self.get_parameter(
                'room_sector_candidate_max_lateral_m').value),
        )
        preferred_lateral = max(
            candidate_min,
            float(self.get_parameter(
                'room_sector_visit_lateral_m').value),
        )
        observations = self._deterministic_doorway_observations
        for frontier in list(frontiers or []):
            delta_x = frontier.centroid[0] - self._entry_origin[0]
            delta_y = frontier.centroid[1] - self._entry_origin[1]
            progress = delta_x * axis_x + delta_y * axis_y
            lateral = -delta_x * axis_y + delta_y * axis_x
            magnitude = abs(lateral)
            if (progress < 0.0 or magnitude < candidate_min
                    or magnitude > candidate_max):
                continue
            pose = (float(frontier.centroid[0]), float(frontier.centroid[1]))
            if any(math.hypot(
                    pose[0] - existing[0], pose[1] - existing[1]) <= 0.35
                    for existing in observations):
                continue
            observations.append(pose)
        if len(observations) > 256:
            del observations[:-256]

        paired = select_symmetric_doorway_stations(
            observations,
            self._entry_origin,
            self._entry_axis,
            preferred_lateral_m=preferred_lateral,
            pair_progress_gap_m=float(self.get_parameter(
                'deterministic_door_pair_progress_gap_m').value),
            station_cluster_m=float(self.get_parameter(
                'deterministic_door_station_cluster_m').value),
            minimum_station_separation_m=float(self.get_parameter(
                'deterministic_door_station_min_separation_m').value),
            maximum_station_count=2,
        )
        if paired:
            self._room_sector_candidate_poses = paired

    def _plan_cached_missing_room_doorway(self, now_ros: float) -> bool:
        """返回曾观测但已无未知边界的房门，继续完成十二房间覆盖。"""

        if (
            self.current_target is not None
            or self._active_room_sector is not None
            or self.grid is None
            or self.latest_map is None
        ):
            return False
        selected = select_cached_missing_room_doorway(
            self._room_sector_candidate_poses,
            self._visited_room_sectors,
            self._attempted_room_sectors,
        )
        if selected is None:
            return False
        sector, pose = selected
        path = a_star_path(
            self.grid,
            self.latest_map,
            self.robot_x,
            self.robot_y,
            pose[0],
            pose[1],
            inflation_radius_m=max(
                0.20,
                float(self.get_parameter(
                    'room_entry_inflation_radius_m').value),
            ),
        )
        if not path:
            return False
        self._room_selection_mode = 'cached_room_doorway'
        frontier = Frontier(
            centroid=pose,
            size=1,
            points=[],
            info_gain=0.0,
        )
        self._accept_frontier_plan(
            frontier,
            path,
            now_ros,
            self._entry_heading(),
            None,
        )
        self.get_logger().info(
            f'Returning to cached SLAM doorway for missing room: {sector} '
            f'at ({pose[0]:.2f}, {pose[1]:.2f}).'
        )
        return True

    def _plan_mirrored_room_doorway(self, now_ros: float) -> bool:
        """用已观测门口关于入口轴的镜像，直接寻找同排对侧房间。"""

        if (
            self._active_room_sector is not None
            or self.current_target is not None
            or self._entry_origin is None
            or self._entry_axis is None
            or self.grid is None
            or self.latest_map is None
        ):
            return False
        axis_x, axis_y = self._entry_axis
        axis_norm = math.hypot(axis_x, axis_y)
        if axis_norm <= 1e-6:
            return False
        axis_x /= axis_norm
        axis_y /= axis_norm
        perpendicular_x = -axis_y
        perpendicular_y = axis_x
        opposite = {
            'far_left': 'far_right',
            'far_right': 'far_left',
            'near_left': 'near_right',
            'near_right': 'near_left',
        }
        # 仍按远到近访问；同排已有一个真实门口后，优先镜像到对侧，避免
        # Frontier 把机器人重新拉到走廊尽头或楼外开放边界。
        for target_sector in (
            'far_left', 'far_right', 'near_left', 'near_right',
        ):
            if target_sector in self._visited_room_sectors:
                continue
            source_sector = opposite[target_sector]
            source_pose = self._room_sector_doorway_poses.get(source_sector)
            if source_pose is None:
                continue
            delta_x = source_pose[0] - self._entry_origin[0]
            delta_y = source_pose[1] - self._entry_origin[1]
            progress = delta_x * axis_x + delta_y * axis_y
            lateral = (
                delta_x * perpendicular_x + delta_y * perpendicular_y
            )
            mirrored_lateral = -lateral
            mirrored_lateral += math.copysign(
                max(
                    0.0,
                    float(self.get_parameter(
                        'room_mirrored_doorway_extra_lateral_m').value),
                ),
                mirrored_lateral,
            )
            mirrored_x = (
                self._entry_origin[0]
                + progress * axis_x
                + mirrored_lateral * perpendicular_x
            )
            mirrored_y = (
                self._entry_origin[1]
                + progress * axis_y
                + mirrored_lateral * perpendicular_y
            )
            if corridor_room_sector(
                mirrored_x,
                mirrored_y,
                self._entry_origin,
                self._entry_axis,
                split_depth_m=float(self.get_parameter(
                    'room_sector_split_depth_m').value),
                lateral_entry_m=float(self.get_parameter(
                    'room_sector_candidate_lateral_m').value),
            ) != target_sector:
                continue
            path = a_star_path(
                self.grid,
                self.latest_map,
                self.robot_x,
                self.robot_y,
                mirrored_x,
                mirrored_y,
                inflation_radius_m=float(self.get_parameter(
                    'room_entry_inflation_radius_m').value),
                goal_search_radius_m=0.45,
            )
            if not path:
                continue
            target = Frontier(
                centroid=(mirrored_x, mirrored_y),
                size=1,
                points=[],
                info_gain=0.0,
            )
            self._accept_frontier_plan(
                target, path, now_ros, self._entry_heading(), None,
            )
            self._room_selection_mode = 'mirrored_room_doorway'
            self.get_logger().info(
                'Using observed doorway symmetry to visit the opposite room: '
                f'{source_sector} -> {target_sector}, '
                f'target=({mirrored_x:.2f}, {mirrored_y:.2f}).'
            )
            return True
        return False

    def _frontier_planning_inflation_m(self, frontier: Frontier) -> float:
        """未完成房间入口使用较小但仍大于半机宽的规划膨胀。"""

        sector = self._frontier_room_sector(frontier)
        if sector is not None and sector not in self._visited_room_sectors:
            return max(
                0.20,
                float(self.get_parameter(
                    'room_entry_inflation_radius_m').value),
            )
        return 0.45

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

        if not self._entry_backbone_active():
            if self._plan_cached_missing_room_doorway(now_ros):
                return
            if self._plan_mirrored_room_doorway(now_ros):
                return

        frontier_mask = find_frontiers(self.grid)
        if frontier_mask.sum() == 0:
            if self._handle_active_room_frontiers_exhausted():
                return
            self.current_target = None
            self.current_path = []
            return

        self.frontiers = cluster_frontiers(frontier_mask, self.grid, self.latest_map)
        self._cache_room_doorway_candidates(self.frontiers)

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
            if self._handle_active_room_frontiers_exhausted():
                return
            self.current_target = None
            self.current_path = []
            self.get_logger().info('All frontiers visited.')
            return

        entry_heading = self._entry_heading()
        ingress_constraint_active = self._entry_backbone_active()
        if ingress_constraint_active:
            min_size = max(
                1, int(self.get_parameter('entry_min_frontier_size').value))
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
                    entry_progress_priority_slack_m=(
                        float(self.get_parameter(
                            'entry_ingress_progress_slack_m').value)
                        if ingress_constraint_active else None
                    ),
                    visited_positions=(
                        self._visited_pose_history
                        if bool(self.get_parameter(
                            'revisit_penalty_enabled').value)
                        else None
                    ),
                    revisit_penalty_radius_m=float(self.get_parameter(
                        'revisit_penalty_radius_m').value),
                    revisit_penalty_strength=float(self.get_parameter(
                        'revisit_penalty_strength').value),
                    revisit_free_samples=int(self.get_parameter(
                        'revisit_penalty_free_samples').value),
                    revisit_full_penalty_samples=int(self.get_parameter(
                        'revisit_penalty_full_samples').value),
                )
                if selected is not None:
                    selected_ingress_half_angle = half_angle
                    return selected
            return None

        def apply_room_priority(candidate_pool):
            if ingress_constraint_active:
                self._room_selection_mode = 'corridor_backbone'
                return list(candidate_pool)
            if (
                self._active_room_sector is not None
                and self._active_room_return_attempted
            ):
                prioritized = []
                mode = 'active_room_frontiers_exhausted'
            else:
                prioritized, mode = prioritize_unvisited_room_frontiers(
                    candidate_pool,
                    self._entry_origin,
                    self._entry_axis,
                    self._visited_room_sectors,
                    self._attempted_room_sectors,
                    active_sector=self._active_room_sector,
                    split_depth_m=float(self.get_parameter(
                        'room_sector_split_depth_m').value),
                    candidate_lateral_m=float(self.get_parameter(
                        'room_sector_candidate_lateral_m').value),
                    candidate_max_lateral_m=float(self.get_parameter(
                        'room_sector_candidate_max_lateral_m').value),
                    far_depth_margin_m=float(self.get_parameter(
                        'room_sector_far_depth_margin_m').value),
                )
            if mode == 'active_room_frontiers_exhausted':
                active_finished = self._active_room_can_finish_after_scan()
                self._handle_active_room_frontiers_exhausted()
                if active_finished:
                    prioritized, mode = prioritize_unvisited_room_frontiers(
                        candidate_pool,
                        self._entry_origin,
                        self._entry_axis,
                        self._visited_room_sectors,
                        self._attempted_room_sectors,
                        active_sector=None,
                        split_depth_m=float(self.get_parameter(
                            'room_sector_split_depth_m').value),
                        candidate_lateral_m=float(self.get_parameter(
                            'room_sector_candidate_lateral_m').value),
                        candidate_max_lateral_m=float(self.get_parameter(
                            'room_sector_candidate_max_lateral_m').value),
                        far_depth_margin_m=float(self.get_parameter(
                            'room_sector_far_depth_margin_m').value),
                    )
            self._room_selection_mode = mode
            return prioritized

        preferred_frontiers = [
            frontier for frontier in unvisited_frontiers
            if not self._frontier_is_detour_deferred(frontier, now_ros)
        ]
        # TTL 只在存在替代目标时降低长绕行盆地的优先级。若所有合法前沿都
        # 处于延后状态则立即回退，不能为了优化路径而原地等待 30 秒。
        selection_frontiers = apply_room_priority(
            preferred_frontiers or unvisited_frontiers)

        if (self.current_target is not None
                and self._current_target_was_ingress
                and not ingress_constraint_active):
            self.get_logger().info(
                'Corridor backbone phase completed; releasing the active '
                'ingress target for side-room coverage.'
            )
            self.current_target = None
            self.current_path = []
            self.path_index = 0
            self._current_target_selected_ros_sec = None
            self._current_target_was_ingress = False
            self._reset_frontier_progress_watchdog()

        if self.current_target is not None:
            refreshed_path = a_star_path(
                self.grid, self.latest_map,
                self.robot_x, self.robot_y,
                self.current_target.centroid[0],
                self.current_target.centroid[1],
                inflation_radius_m=self._frontier_planning_inflation_m(
                    self.current_target),
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
        candidates = apply_room_priority(unvisited_frontiers)
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
                inflation_radius_m=self._frontier_planning_inflation_m(best),
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
        # 活动房间中的候选均不可规划时，必须进入既有的探针/回门/环视
        # 收尾链路；继续普通 Frontier 重试只会永久停在同一批不可达盆地。
        if self._handle_active_room_frontiers_exhausted():
            return
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
        room_sector = self._frontier_room_sector(frontier)
        if (self._active_room_sector is None
                and room_sector is not None
                and room_sector not in self._visited_room_sectors
                and room_sector not in self._attempted_room_sectors):
            self.get_logger().info(
                f'Room doorway approach selected: {room_sector}; deep room '
                'frontiers remain locked until this doorway target is reached.'
            )
        self._current_target_was_ingress = (
            selected_ingress_half_angle is not None)
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
            self._room_selection_mode
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

    def _follow_elevator_odom_path(
            self, entering: bool, *, return_mode: bool = False,
            final_approach: bool = False) -> Twist:
        """直接在合法odom帧跟踪电梯短路径，隔离map回环/楼层对称翻转。"""

        cmd = Twist()
        if self._robot_odom is None or self._robot_odom_yaw is None:
            return cmd
        goal_tolerance = float(self.get_parameter('goal_tolerance_m').value)
        while self._elevator_odom_path_index < len(self._elevator_odom_path):
            target = self._elevator_odom_path[self._elevator_odom_path_index]
            distance = math.hypot(
                target[0] - self._robot_odom[0],
                target[1] - self._robot_odom[1])
            if distance <= goal_tolerance:
                self._elevator_odom_path_index += 1
            else:
                break
        if self._elevator_odom_path_index >= len(self._elevator_odom_path):
            self._elevator_odom_path = []
            self._elevator_odom_path_index = 0
            return cmd

        target = self._elevator_odom_path[self._elevator_odom_path_index]
        distance = math.hypot(
            target[0] - self._robot_odom[0],
            target[1] - self._robot_odom[1])
        target_yaw = math.atan2(
            target[1] - self._robot_odom[1],
            target[0] - self._robot_odom[0])
        heading_error = normalize_angle(target_yaw - self._robot_odom_yaw)
        if return_mode:
            linear_speed = float(self.get_parameter(
                'return_final_linear_speed'
                if final_approach else 'return_linear_speed').value)
            minimum_linear_speed = min(
                abs(linear_speed), float(self.get_parameter(
                    'return_final_minimum_linear_speed'
                    if final_approach
                    else 'return_minimum_linear_speed').value))
            angular_speed = float(self.get_parameter(
                'return_angular_speed').value)
            minimum_turn_speed = min(
                abs(angular_speed), float(self.get_parameter(
                    'return_minimum_turn_speed').value))
        else:
            linear_speed = float(self.get_parameter(
                'elevator_entry_linear_speed'
                if entering else 'elevator_exit_linear_speed').value)
            minimum_linear_speed = min(
                abs(linear_speed), float(self.get_parameter(
                    'elevator_minimum_linear_speed').value))
            angular_speed = float(self.get_parameter(
                'elevator_angular_speed').value)
            minimum_turn_speed = min(
                abs(angular_speed), float(self.get_parameter(
                    'elevator_minimum_turn_speed').value))
        cmd.angular.z = max(
            -angular_speed, min(angular_speed, heading_error))
        heading_tolerance = float(self.get_parameter(
            'heading_tolerance_rad').value)
        # 电梯门槛限制原地转向；电梯短路径允许在约 57° 内边转边前进，
        # 用小半径弧线脱离门框碰撞边界。普通走廊导航仍使用严格航向阈值。
        heading_tolerance = (
            heading_tolerance if return_mode
            else max(heading_tolerance, 1.0)
        )
        if abs(heading_error) <= heading_tolerance:
            cmd.linear.x = min(linear_speed, distance)
            if 0.0 < cmd.linear.x < minimum_linear_speed:
                cmd.linear.x = minimum_linear_speed
        elif 0.0 < abs(cmd.angular.z) < minimum_turn_speed:
            cmd.angular.z = math.copysign(
                minimum_turn_speed, cmd.angular.z)

        if return_mode:
            clearance = float(self.get_parameter(
                'return_min_clearance_m').value)
            rotation_clearance = float(self.get_parameter(
                'return_rotation_min_clearance_m').value)
        else:
            clearance = float(self.get_parameter(
                'elevator_entry_min_clearance_m'
                if entering else 'elevator_exit_min_clearance_m').value)
            rotation_clearance = float(self.get_parameter(
                'elevator_entry_rotation_clearance_m'
                if entering else 'elevator_exit_rotation_clearance_m').value)
        if (cmd.linear.x > 0.0 and clearance >= 0.0
                and not self._scan_allows_action(
                    'move_forward', clearance)):
            cmd.linear.x = 0.0
        if cmd.angular.z != 0.0 and rotation_clearance >= 0.0:
            action = 'turn_left' if cmd.angular.z > 0.0 else 'turn_right'
            if not self._scan_allows_action(action, rotation_clearance):
                cmd.angular.z = 0.0
        return cmd

    @staticmethod
    def _elevator_backoff_command() -> Twist:
        """返回门框脱困用的低速后退命令。"""

        command = Twist()
        command.linear.x = -0.45
        return command

    def _follow_return_path(self) -> Twist:
        """在已验证地图/A*路径上使用高速返航档。"""

        return self._follow_path(
            navigation_clearance_override=float(self.get_parameter(
                'return_min_clearance_m').value),
            rotation_clearance_override=float(self.get_parameter(
                'return_rotation_min_clearance_m').value),
            linear_speed_override=float(self.get_parameter(
                'return_linear_speed').value),
            minimum_linear_speed_override=float(self.get_parameter(
                'return_minimum_linear_speed').value),
            angular_speed_override=float(self.get_parameter(
                'return_angular_speed').value),
            minimum_turn_speed_override=float(self.get_parameter(
                'return_minimum_turn_speed').value),
        )

    def _map_goal_to_unitree_base(
            self, map_x: float, map_y: float,
            map_yaw: float) -> Optional[Tuple[float, float, float]]:
        """把合法 map 目标转成 base 相对目标，隔离 ROS1/ROS2 odom 原点。"""

        try:
            return transform_planar_goal_to_robot_frame(
                map_x, map_y, map_yaw,
                self.robot_x, self.robot_y, self.robot_yaw,
            )
        except ValueError:
            return None

    def _unitree_move_base_lookahead_goal(
            self) -> Optional[Tuple[float, float, float]]:
        """从已验证 A* 路径选择有限前视目标，避免 DWA 穿过未知捷径。"""

        if self.path_index >= len(self.current_path):
            return None
        lookahead_parameter = (
            'unitree_move_base_corridor_lookahead_m'
            if self._deterministic_waypoint_label == 'corridor_outbound'
            else 'unitree_move_base_lookahead_m'
        )
        lookahead = max(
            0.25,
            float(self.get_parameter(lookahead_parameter).value),
        )
        return interpolate_path_lookahead(
            self.current_path,
            self.path_index,
            self.robot_x,
            self.robot_y,
            lookahead,
        )

    def _publish_unitree_move_base_goal(
            self, goal_map: Tuple[float, float, float]) -> bool:
        """按位移阈值更新宇树 move_base 目标；不在此实现任何避障逻辑。"""

        now_wall = time.monotonic()
        use_official_corridor_goal = (
            self._deterministic_waypoint_label == 'corridor_outbound'
            and bool(self.get_parameter(
                'use_official_odom_for_corridor_control').value)
            and self._has_fresh_official_control_odom()
        )
        use_official_room_goal = (
            self._deterministic_waypoint_label.startswith('room_')
            and bool(self.get_parameter(
                'use_official_odom_for_room_control').value)
            and self._has_fresh_official_control_odom()
        )
        use_official_return_goal = (
            self.state == 'RETURNING'
            and self._official_return_control_active()
        )
        use_official_elevator_goal = (
            self.state == 'FLOOR_TRANSITION'
            and self._floor_transition_phase in (
                'navigating', 'entering', 'exiting')
            and bool(self.get_parameter(
                'use_official_odom_for_elevator_control').value)
            and self._has_fresh_official_control_odom()
        )
        if use_official_corridor_goal:
            official_x, official_y, _official_yaw, _stamp = (
                self._official_control_odom)
            forward = max(
                0.5,
                float(self.get_parameter(
                    'official_corridor_forward_lookahead_m').value),
            )
            end_y = float(self.get_parameter(
                'official_corridor_end_y_m').value)
            target_y = min(end_y, official_y + forward)
            if target_y <= official_y + 0.10:
                target_y = official_y + 0.50
            outgoing_goal = (
                float(self.get_parameter(
                    'official_corridor_center_x_m').value),
                target_y,
                float(self.get_parameter(
                    'official_corridor_yaw_rad').value),
            )
            outgoing_frame = str(self.get_parameter('odom_frame').value)
            goal_key = (outgoing_goal[0], outgoing_goal[1])
        elif use_official_room_goal:
            map_target = (
                self.current_target.centroid
                if self.current_target is not None
                else (goal_map[0], goal_map[1])
            )
            official_room_goal = self._official_room_goal_from_map(map_target)
            if official_room_goal is None:
                return False
            outgoing_goal = official_room_goal
            if self._deterministic_waypoint_label.startswith('room_door:'):
                official_x, official_y, _yaw, _stamp = (
                    self._official_control_odom)
                final_x, final_y, _final_yaw = official_room_goal
                label = self._deterministic_waypoint_label
                if self._official_room_door_label != label:
                    self._official_room_door_label = label
                    self._official_room_door_center_y = official_y
                    self._official_room_door_stage = (
                        'center_lateral'
                        if (abs(official_y - final_y) > 1.0
                            and abs(official_x) > 0.45)
                        else 'center_longitudinal'
                        if abs(official_y - final_y) > 1.0
                        else 'enter_door'
                    )
                # 三阶段只允许向前推进，不能根据瞬时x在阈值两侧反复切换。
                if (self._official_room_door_stage == 'center_lateral'
                        and abs(official_x) <= 0.45):
                    self._official_room_door_stage = 'center_longitudinal'
                if (self._official_room_door_stage == 'center_longitudinal'
                        and abs(official_y - final_y) <= 0.60):
                    self._official_room_door_stage = 'enter_door'

                if self._official_room_door_stage == 'center_lateral':
                    staged_x = 0.0
                    staged_y = (
                        self._official_room_door_center_y
                        if self._official_room_door_center_y is not None
                        else official_y)
                    outgoing_goal = (
                        staged_x,
                        staged_y,
                        math.atan2(
                            staged_y - official_y,
                            staged_x - official_x),
                    )
                elif self._official_room_door_stage == 'center_longitudinal':
                    outgoing_goal = (
                        0.0,
                        final_y,
                        math.atan2(
                            final_y - official_y,
                            -official_x),
                    )
                else:
                    outgoing_goal = official_room_goal
            outgoing_frame = str(self.get_parameter('odom_frame').value)
            goal_key = (outgoing_goal[0], outgoing_goal[1])
        elif use_official_return_goal:
            outgoing_goal = self._official_return_target()
            outgoing_frame = str(self.get_parameter('odom_frame').value)
            goal_key = (outgoing_goal[0], outgoing_goal[1])
        elif use_official_elevator_goal:
            lobby_x = float(self.get_parameter(
                'official_elevator_lobby_x_m').value)
            cabin_x = float(self.get_parameter(
                'official_elevator_cabin_x_m').value)
            elevator_y = float(self.get_parameter(
                'official_elevator_y_m').value)
            target_x = (
                cabin_x
                if self._floor_transition_phase == 'entering'
                else lobby_x
            )
            official_x, official_y, _yaw, _stamp = (
                self._official_control_odom)
            target_y = elevator_y
            # 房间到电梯不能画一条斜线穿过隔墙。返梯时先在当前门排退到
            # x=0 的走廊中心，再沿中心线到大厅；进入/退出轿厢本来就在
            # 同一 y 轴线上，无需额外中间点。
            if (self._floor_transition_phase == 'navigating'
                    and abs(official_y - elevator_y) > 1.0
                    and abs(official_x) > 0.55):
                target_x = 0.0
                target_y = official_y
            target_yaw = math.atan2(
                target_y - official_y,
                target_x - official_x,
            )
            outgoing_goal = target_x, target_y, target_yaw
            outgoing_frame = str(self.get_parameter('odom_frame').value)
            goal_key = (target_x, target_y)
        else:
            goal_local = self._map_goal_to_unitree_base(*goal_map)
            if goal_local is None:
                self.get_logger().warning(
                    'Unitree move_base goal held: invalid map/base relative goal.',
                    throttle_duration_sec=3.0,
                )
                return False
            outgoing_goal = goal_local
            outgoing_frame = str(self.get_parameter('base_frame').value)
            goal_key = (float(goal_map[0]), float(goal_map[1]))
        goal_change_parameter = (
            'unitree_move_base_corridor_goal_change_m'
            if self._deterministic_waypoint_label == 'corridor_outbound'
            else 'unitree_move_base_goal_change_m'
        )
        goal_changed = (
            self._unitree_move_base_last_goal_map is None
            or math.hypot(
                goal_key[0] - self._unitree_move_base_last_goal_map[0],
                goal_key[1] - self._unitree_move_base_last_goal_map[1],
            ) >= max(
                0.05,
                float(self.get_parameter(goal_change_parameter).value),
            )
        )
        if (self._unitree_move_base_goal_active
                and not goal_changed):
            return True

        message = PoseStamped()
        message.header.frame_id = outgoing_frame
        message.pose.position.x = float(outgoing_goal[0])
        message.pose.position.y = float(outgoing_goal[1])
        message.pose.orientation.z = math.sin(outgoing_goal[2] * 0.5)
        message.pose.orientation.w = math.cos(outgoing_goal[2] * 0.5)
        self.unitree_move_base_goal_pub.publish(message)
        self._unitree_move_base_goal_active = True
        self._unitree_move_base_cmd_after_goal = False
        self._unitree_move_base_last_goal_map = goal_key
        self._unitree_move_base_last_goal_wall = now_wall
        self.get_logger().info(
            'Unitree move_base goal updated: '
            f'map=({goal_map[0]:.2f},{goal_map[1]:.2f},'
            f'{goal_map[2]:.2f}) -> {outgoing_frame}='
            f'({outgoing_goal[0]:.2f},{outgoing_goal[1]:.2f},'
            f'{outgoing_goal[2]:.2f}).'
        )
        return True

    def _cancel_unitree_move_base(self) -> None:
        """撤销 ROS1 move_base 活动目标，并立即让其速度输出失效。"""

        if self._local_planner_backend != 'unitree_move_base':
            return
        if self._unitree_move_base_goal_active:
            message = String()
            message.data = 'cancel'
            self.unitree_move_base_control_pub.publish(message)
        self._unitree_move_base_goal_active = False
        self._unitree_move_base_cmd_after_goal = False
        self._unitree_move_base_last_goal_map = None
        self._unitree_move_base_last_goal_wall = None

    def _follow_path_with_unitree_move_base(self) -> Twist:
        """把高层路径交给赛事仓库随附的宇树 DWA，并转发其新鲜速度。"""

        self._unitree_move_base_selected_this_cycle = True
        goal = self._unitree_move_base_lookahead_goal()
        if goal is None or not self._publish_unitree_move_base_goal(goal):
            return Twist()
        timeout = max(
            0.1,
            float(self.get_parameter(
                'unitree_move_base_cmd_timeout_s').value),
        )
        command_stale = (
            not self._unitree_move_base_cmd_after_goal
            or self._unitree_move_base_cmd_wall is None
            or time.monotonic() - self._unitree_move_base_cmd_wall > timeout
        )
        if command_stale:
            fallback_after = max(
                timeout,
                float(self.get_parameter(
                    'unitree_move_base_direct_fallback_s').value),
            )
            goal_age = (
                0.0 if self._unitree_move_base_last_goal_wall is None
                else time.monotonic() - self._unitree_move_base_last_goal_wall
            )
            if goal_age >= fallback_after:
                self.get_logger().warning(
                    'Unitree move_base produced no fresh command; using '
                    'existing A* path with lidar clearance fallback.',
                    throttle_duration_sec=5.0,
                )
                return self._follow_path_with_direct_backend()
            self.get_logger().warning(
                'Unitree move_base command unavailable or stale; holding zero.',
                throttle_duration_sec=3.0,
            )
            return Twist()
        command = self._unitree_move_base_cmd
        if self.state == 'FLOOR_TRANSITION':
            minimum = max(
                0.0,
                float(self.get_parameter(
                    'official_elevator_minimum_linear_command').value),
            )
            norm = math.hypot(command.linear.x, command.linear.y)
            if 1e-6 < norm < minimum:
                # A1 RL 步态对 0.1m/s 的 DWA近目标减速没有位移响应，会
                # 永久停在轿厢门槛。只放大 DWA 已判定安全的平移方向，角
                # 速度和避障决策不变。
                scale = minimum / norm
                scaled = Twist()
                scaled.linear.x = command.linear.x * scale
                scaled.linear.y = command.linear.y * scale
                scaled.linear.z = command.linear.z
                scaled.angular.x = command.angular.x
                scaled.angular.y = command.angular.y
                scaled.angular.z = command.angular.z
                return scaled
        return command

    def _follow_path_with_direct_backend(
            self,
            navigation_clearance_override: Optional[float] = None,
            rotation_clearance_override: Optional[float] = None) -> Twist:
        """复用既有 A* 跟踪与激光净空门禁，不进入 ROS1 DWA 重规划。"""

        saved_backend = self._local_planner_backend
        self._local_planner_backend = 'direct'
        try:
            return self._follow_path(
                navigation_clearance_override=navigation_clearance_override,
                rotation_clearance_override=rotation_clearance_override,
            )
        finally:
            self._local_planner_backend = saved_backend

    def _follow_path(
            self, navigation_clearance_override: Optional[float] = None,
            rotation_clearance_override: Optional[float] = None,
            linear_speed_override: Optional[float] = None,
            minimum_linear_speed_override: Optional[float] = None,
            angular_speed_override: Optional[float] = None,
            minimum_turn_speed_override: Optional[float] = None) -> Twist:
        """沿当前路径前进，返回 cmd_vel。"""
        cmd = Twist()
        goal_tol = float(self.get_parameter('goal_tolerance_m').value)
        if self._deterministic_route_enabled():
            label = self._deterministic_waypoint_label
            tolerance_parameter = (
                'deterministic_corridor_waypoint_tolerance_m'
                if label == 'corridor_outbound'
                else 'deterministic_waypoint_tolerance_m'
            )
            goal_tol = max(
                goal_tol,
                float(self.get_parameter(tolerance_parameter).value),
            )
            if ((label.startswith('room_cross:')
                    or label.startswith('room_loop:'))
                    and float(self._deterministic_waypoint_scales.get(
                        label, 1.0)) <= 0.41):
                goal_tol = max(
                    goal_tol,
                    float(self.get_parameter(
                        'deterministic_min_scale_tolerance_m').value),
                )
        room_coverage_active = (
            linear_speed_override is None
            and self.state == 'EXPLORING'
            and self._active_room_sector is not None
        )
        linear_speed = (
            float(linear_speed_override)
            if linear_speed_override is not None
            else float(self.get_parameter(
                'room_perimeter_linear_speed'
                if room_coverage_active else 'linear_speed').value))
        minimum_linear_speed = min(
            abs(linear_speed),
            float(minimum_linear_speed_override)
            if minimum_linear_speed_override is not None
            else float(self.get_parameter(
                'room_perimeter_minimum_linear_speed'
                if room_coverage_active
                else 'minimum_linear_speed').value),
        )
        angular_speed = (
            float(angular_speed_override)
            if angular_speed_override is not None
            else float(self.get_parameter('angular_speed').value))
        minimum_turn_speed = min(
            abs(angular_speed),
            float(minimum_turn_speed_override)
            if minimum_turn_speed_override is not None
            else float(self.get_parameter('minimum_turn_speed').value),
        )
        heading_tol = float(self.get_parameter('heading_tolerance_rad').value)

        if len(self.current_path) == 0:
            return cmd

        if (self._deterministic_waypoint_label == 'corridor_outbound'
                and bool(self.get_parameter(
                    'use_official_odom_for_corridor_control').value)
                and self._has_fresh_official_control_odom()):
            official_x, official_y, _yaw, _stamp = (
                self._official_control_odom)
            center_x = float(self.get_parameter(
                'official_corridor_center_x_m').value)
            end_y = float(self.get_parameter(
                'official_corridor_end_y_m').value)
            official_distance = math.hypot(
                center_x - official_x, end_y - official_y)
            if official_distance <= max(
                    0.2,
                    float(self.get_parameter(
                        'deterministic_corridor_waypoint_tolerance_m').value)):
                # 上层 map 原点在电梯大厅，旧 A* 锚点可能与真实楼宇终点
                # 不重合。官方里程计已经到终点时必须结算该路径，随后由
                # 确定性状态机按本层门位开始四房覆盖。
                self.path_index = len(self.current_path)

        official_room_pending = False
        if (self._deterministic_waypoint_label.startswith('room_')
                and self.current_target is not None
                and bool(self.get_parameter(
                    'use_official_odom_for_room_control').value)
                and self._has_fresh_official_control_odom()):
            official_goal = self._official_room_goal_from_map(
                self.current_target.centroid)
            if official_goal is not None:
                official_x, official_y, _yaw, _stamp = (
                    self._official_control_odom)
                official_distance = math.hypot(
                    official_goal[0] - official_x,
                    official_goal[1] - official_y,
                )
                if self._deterministic_waypoint_label.startswith(
                        'room_loop:'):
                    tolerance_parameter = (
                        'official_room_loop_goal_tolerance_m')
                elif self._deterministic_waypoint_label.startswith(
                        'room_exit:'):
                    tolerance_parameter = (
                        'official_room_exit_goal_tolerance_m')
                else:
                    tolerance_parameter = 'official_room_goal_tolerance_m'
                position_reached = official_distance <= max(
                        0.1,
                        float(self.get_parameter(
                            tolerance_parameter).value))
                orientation_reached = True
                if self._deterministic_waypoint_label.startswith(
                        'room_inspect_orient:'):
                    orientation_reached = abs(normalize_angle(
                        official_goal[2] - _yaw)) <= max(
                            0.05,
                            float(self.get_parameter(
                                'strict_room_heading_tolerance_rad').value),
                        )
                if position_reached and orientation_reached:
                    self.path_index = len(self.current_path)
                else:
                    # map/SLAM路径可能因漂移或 A*吸附先到终点。官方固定楼宇
                    # 模式必须等机器人真实到达物理门内/环线角点，禁止仅凭
                    # map 容差推进 probe_count。
                    official_room_pending = True
                    self.path_index = min(
                        self.path_index,
                        max(0, len(self.current_path) - 1),
                    )

        # DWA 可能已经越过稀疏 A* 的旧锚点。只按“距离旧点小于容差”推进
        # 会永远停在旧索引，前视目标随后落到身后并触发 180° 回转。
        if (not official_room_pending
                and self.path_index < len(self.current_path) - 1):
            nearest_index = min(
                range(self.path_index, len(self.current_path)),
                key=lambda index: math.hypot(
                    self.current_path[index][0] - self.robot_x,
                    self.current_path[index][1] - self.robot_y,
                ),
            )
            self.path_index = max(self.path_index, nearest_index)

        # 跳过已到达的路径点
        if not official_room_pending:
            while self.path_index < len(self.current_path):
                wx, wy = self.current_path[self.path_index]
                dist = math.hypot(wx - self.robot_x, wy - self.robot_y)
                if dist <= goal_tol:
                    self.path_index += 1
                else:
                    break

        if self.path_index >= len(self.current_path):
            # 路径走完
            self._cancel_unitree_move_base()
            if (self._deterministic_route_enabled()
                    and self._deterministic_waypoint_label):
                label = self._deterministic_waypoint_label
                self._deterministic_waypoint_label = ''
                self.current_path = []
                self.current_target = None
                self.path_index = 0
                self._current_target_selected_ros_sec = None
                self._reset_frontier_progress_watchdog()
                self._on_deterministic_waypoint_reached(
                    label, self._ros_time_sec())
                return cmd
            if self.current_target is not None:
                key = self._frontier_key(self.current_target)
                reached_room_sector = self._frontier_room_sector(
                    self.current_target,
                )
                self._visited_frontiers.add(key)
                self._clear_unreachable_frontier_basin(
                    self.current_target,
                )
                self.get_logger().info(
                    f'Frontier reached and marked visited: {key}'
                )
                if (
                    self._active_room_sector is None
                    and reached_room_sector is not None
                    and reached_room_sector not in self._visited_room_sectors
                    and reached_room_sector not in self._attempted_room_sectors
                ):
                    self._attempted_room_sectors.add(reached_room_sector)
                    self._room_sector_doorway_poses[reached_room_sector] = (
                        self.robot_x,
                        self.robot_y,
                    )
                    self.get_logger().info(
                        'Room doorway target physically reached; unlocking '
                        f'deeper frontiers for {reached_room_sector}.'
                    )
                sweep_rad = max(
                    0.0,
                    float(self.get_parameter(
                        'frontier_observation_sweep_rad').value),
                )
                if (
                    sweep_rad > 0.0
                    and not self._entry_backbone_active()
                    and self._active_room_sector is None
                ):
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
            self._current_target_was_ingress = False
            self._reset_frontier_progress_watchdog()
            return cmd

        goal_x, goal_y = self.current_path[self.path_index]

        if self._local_planner_backend == 'unitree_move_base':
            return self._follow_path_with_unitree_move_base()

        # 若距离太远，跳到最后一个可达点
        if self.path_index < len(self.current_path) - 1:
            last_x, last_y = self.current_path[-1]
        else:
            last_x, last_y = goal_x, goal_y

        # 朝向目标
        target_yaw = math.atan2(goal_y - self.robot_y, goal_x - self.robot_x)
        heading_error = normalize_angle(target_yaw - self.robot_yaw)
        cmd.angular.z = max(-angular_speed, min(angular_speed, heading_error))

        if abs(heading_error) <= heading_tol:
            # 行进中保留小角度比例修正；若强制抬到 0.45 rad/s，会在目标线
            # 两侧逐帧变号，形成蛇形并从门口/前沿旁边高速掠过。
            dist = math.hypot(goal_x - self.robot_x, goal_y - self.robot_y)
            cmd.linear.x = min(linear_speed, dist)
            if 0.0 < cmd.linear.x < minimum_linear_speed:
                cmd.linear.x = minimum_linear_speed
        else:
            cmd.linear.x = 0.0
            if 0.0 < abs(cmd.angular.z) < minimum_turn_speed:
                cmd.angular.z = math.copysign(
                    minimum_turn_speed, cmd.angular.z,
                )

        requested_linear = cmd.linear.x
        requested_angular = cmd.angular.z
        navigation_clearance = (
            float(navigation_clearance_override)
            if navigation_clearance_override is not None
            else float(self.get_parameter(
                'navigation_min_clearance_m').value))
        target_sector = self._frontier_room_sector(self.current_target)
        if (navigation_clearance_override is None
                and target_sector is not None
                and target_sector not in self._visited_room_sectors):
            navigation_clearance = min(
                navigation_clearance,
                float(self.get_parameter(
                    'room_entry_navigation_clearance_m').value),
            )
        if (cmd.linear.x > 0.0
                and navigation_clearance >= 0.0
                and not self._scan_allows_action(
                    'move_forward', navigation_clearance)):
            cmd.linear.x = 0.0
        if cmd.angular.z != 0.0:
            turn_action = 'turn_left' if cmd.angular.z > 0.0 else 'turn_right'
            rotation_clearance = (
                float(rotation_clearance_override)
                if rotation_clearance_override is not None
                else float(self.get_parameter(
                    'rotation_min_clearance_m').value))
            if (rotation_clearance >= 0.0
                    and not self._scan_allows_action(
                        turn_action, rotation_clearance)):
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
                self._start_exploration_recovery('safety_blocked')
                self._safety_blocked_since_ros = None
        else:
            self._safety_blocked_since_ros = None

        return cmd

    def _start_exploration_recovery(self, reason: str) -> None:
        """启动一次有界交替转向，摆脱门侧/障碍边缘的局部零速状态。"""

        now_ros = self._ros_time_sec()
        duration = max(
            0.1,
            float(self.get_parameter(
                'exploration_recovery_turn_duration_s').value),
        )
        self._exploration_recovery_attempts += 1
        self._exploration_recovery_direction = (
            1.0 if self._exploration_recovery_attempts % 2 else -1.0
        )
        self._exploration_recovery_end_ros = now_ros + duration
        self.get_logger().warn(
            'Starting bounded exploration recovery turn: '
            f'reason={reason}, attempt={self._exploration_recovery_attempts}, '
            f'direction={self._exploration_recovery_direction:+.0f}, '
            f'duration={duration:.1f}s.'
        )

    def _handle_exploration_recovery(
        self,
        now_ros: float,
    ) -> Optional[Twist]:
        """在实时激光门禁下执行恢复转向；结束后再由正常前沿规划接管。"""

        end_ros = self._exploration_recovery_end_ros
        if end_ros is None:
            return None
        if now_ros < 0.0 or now_ros >= end_ros:
            self._exploration_recovery_end_ros = None
            self._pose_history.clear()
            self._stuck_since = None
            return None

        command = Twist()
        clearance = max(
            0.0,
            float(self.get_parameter(
                'exploration_recovery_turn_clearance_m').value),
        )
        speed = min(
            abs(float(self.get_parameter(
                'exploration_recovery_turn_speed').value)),
            abs(float(self.get_parameter('angular_speed').value)),
        )
        direction = self._exploration_recovery_direction
        action = 'turn_left' if direction > 0.0 else 'turn_right'
        if self._scan_allows_action(action, clearance):
            command.angular.z = direction * speed
            return command

        # 首选方向被近场结构阻挡时，在同一周期尝试反向；两侧都不安全则
        # 保持停车，不能为了脱困绕过激光安全门。
        opposite_action = 'turn_right' if direction > 0.0 else 'turn_left'
        if self._scan_allows_action(opposite_action, clearance):
            self._exploration_recovery_direction = -direction
            command.angular.z = -direction * speed
        return command

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

        # Gazebo 低实时因子时，现实时间会比机器人实际运动/传感器推进快很多。
        # 卡住门限必须跟随 /clock；否则现实 15 秒可能只有约 1.5 秒仿真，
        # 会抢在确定性房间目标的 5 秒恢复器之前清空路径，形成原地循环。
        now = self._ros_time_sec()
        if now <= 0.0:
            now = time.monotonic()
        if self._stuck_since is None or now < self._stuck_since:
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
            if self._local_planner_backend == 'unitree_move_base':
                # 宇树模式的局部脱困由 move_base recovery_behaviors 负责；
                # 高层只撤销已超时目标并换一个 Frontier，不另写避障动作。
                self._cancel_unitree_move_base()
            else:
                self._start_exploration_recovery('stuck')
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
        elif new_state in ('FINISHED', 'FAILED'):
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
                final_state=new_state,
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
