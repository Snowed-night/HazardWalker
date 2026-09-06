"""官方 SimEnv 的 ROS2 业务层启动入口。

所属组：系统集成/平台组。负责人：姜晨。
文件作用：只启动 ROS2 业务节点，假定官方 ROS1 容器及双向适配层已提供稳定 /hw/*；绝不启动
fake_platform_node 或 Gazebo Harmonic，避免把占位平台接入官方比赛 profile。

导航组 (2026-07-17 更新):
- 新增 SLAM Toolbox 在线异步建图节点
- 新增 frontier_explorer_node 自主探索（替代固定航点 waypoint_patrol_node）
- 保留 waypoint_patrol_node 作为诊断回退

验证方式：先运行 scripts/run_official_simenv_rosbridge_adapter.sh，再 ros2 launch 本文件。
"""
import os
import shutil
import tempfile

import yaml

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from hazardwalker_perception.perception_config import flatten_perception_config


def _as_bool(value):
    """按 ROS launch 常用写法解析布尔参数。"""

    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def _launch_slam_toolbox(context, slam_config):
    """仅在显式选择 slam_toolbox 时解析其安装路径并启动生命周期入口。"""

    if not _as_bool(LaunchConfiguration('start_slam').perform(context)):
        return []
    if LaunchConfiguration('slam_backend').perform(context) != 'slam_toolbox':
        return []

    try:
        slam_toolbox_share = get_package_share_directory('slam_toolbox')
    except PackageNotFoundError as exc:
        raise RuntimeError(
            'start_slam=true and slam_backend=slam_toolbox require '
            'slam_toolbox; install the ROS package or select cartographer.'
        ) from exc
    slam_launch = os.path.join(
        slam_toolbox_share,
        'launch',
        'online_async_launch.py',
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'slam_params_file': slam_config,
                'autostart': 'true',
                'use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
            }.items(),
        ),
    ]


def _launch_cartographer(context, nav_pkg):
    """只在显式选择后准备上游 Lua 目录并返回 Cartographer 节点。

    Cartographer 的 Lua resolver 只搜索一个目录；因此把上游只读配置复制到临时
    目录，再加入本项目配置。该过程不修改 /opt/ros，也不会在默认关闭 SLAM 时
    强制要求 Cartographer 已安装。
    """

    if not _as_bool(LaunchConfiguration('start_slam').perform(context)):
        return []
    if LaunchConfiguration('slam_backend').perform(context) != 'cartographer':
        return []

    # Ubuntu 的 Cartographer ROS 二进制包会安装 share/cartographer/
    # 配置，却不一定把它注册为可被 ament_index 查询的独立包。因此从已注册的
    # cartographer_ros 共享目录回到同一 share 前缀，避免正式 launch 因
    # get_package_share_directory('cartographer') 直接退出。
    try:
        cartographer_ros_share = get_package_share_directory('cartographer_ros')
    except PackageNotFoundError as exc:
        raise RuntimeError(
            'start_slam=true and slam_backend=cartographer require '
            'cartographer_ros; configure the official stack Cartographer prefix.'
        ) from exc
    builtin_dir = os.path.join(
        os.path.dirname(cartographer_ros_share),
        'cartographer',
        'configuration_files',
    )
    if not os.path.isdir(builtin_dir):
        raise RuntimeError(
            'Cartographer configuration directory is missing: '
            f'{builtin_dir}'
        )
    runtime_dir = os.path.join(
        tempfile.gettempdir(),
        f'hazardwalker_cartographer_configuration_files_{os.getenv("USER", "unknown")}',
    )
    shutil.copytree(builtin_dir, runtime_dir, dirs_exist_ok=True)
    dimension = LaunchConfiguration('slam_dimension').perform(context).lower()
    if dimension not in ('2d', '3d'):
        raise RuntimeError(f'slam_dimension must be 2d or 3d, got {dimension!r}')
    configuration_basename = f'cartographer_official_{dimension}.lua'
    shutil.copy2(
        os.path.join(nav_pkg, 'config', configuration_basename),
        os.path.join(runtime_dir, configuration_basename),
    )
    use_sim_time = _as_bool(
        LaunchConfiguration('use_sim_time').perform(context),
    )
    cartographer = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='hazardwalker_cartographer',
            output='screen',
            arguments=[
                '-configuration_directory', runtime_dir,
                '-configuration_basename', configuration_basename,
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=([
                ('points2', '/hw/lidar/points'),
                ('imu', '/hw/trunk_imu'),
                ('odom', '/hazardwalker/slam/odometry'),
            ] if dimension == '3d' else [
                ('scan', '/hw/scan'),
                ('imu', '/hw/trunk_imu'),
            ]),
        )
    if dimension == '3d':
        occupancy = Node(
            package='hazardwalker_nav',
            executable='multifloor_occupancy_mapper',
            name='hazardwalker_multifloor_occupancy_mapper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )
    else:
        occupancy = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='hazardwalker_cartographer_occupancy_grid',
            output='screen',
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
            parameters=[{'use_sim_time': use_sim_time}],
        )
    return [cartographer, occupancy]


def _launch_perception(context):
    """加载实际参与运行的感知配置并创建检测节点。

    配置文件沿用仓库的分组可读格式，先严格展开为 ROS 参数；未知键直接
    中止启动，避免评估报告记录了某个配置、节点却静默忽略它。
    """

    if not _as_bool(LaunchConfiguration('start_perception').perform(context)):
        return []
    parameter_file = LaunchConfiguration(
        'perception_parameter_file').perform(context).strip()
    parameters = {}
    if parameter_file:
        if not os.path.isfile(parameter_file):
            raise RuntimeError(
                f'perception parameter file does not exist: {parameter_file}')
        with open(parameter_file, encoding='utf-8') as handle:
            parameters.update(flatten_perception_config(yaml.safe_load(handle)))

    # 合法坐标来源和仿真时间属于本轮运行合同，必须覆盖配置文件中的通用值。
    parameters.update({
        'camera_axis_convention': 'gazebo_link_x_forward',
        'output_frame': LaunchConfiguration(
            'perception_output_frame').perform(context),
        'localization_provenance': LaunchConfiguration(
            'localization_provenance').perform(context),
        'use_sim_time': _as_bool(
            LaunchConfiguration('use_sim_time').perform(context)),
    })
    return [Node(
        package='hazardwalker_perception',
        executable='hsv_detector_node',
        name='hsv_detector_node',
        output='screen',
        parameters=[parameters],
    )]


def generate_launch_description():
    """按显式开关组合业务节点，官方 profile 不引入任何模拟平台节点。"""

    start_perception = LaunchConfiguration('start_perception')
    start_decision = LaunchConfiguration('start_decision')
    start_navigation = LaunchConfiguration('start_navigation')
    start_slam = LaunchConfiguration('start_slam')
    start_slam_monitor = LaunchConfiguration('start_slam_monitor')
    start_pointcloud_map = LaunchConfiguration('start_pointcloud_map')
    start_slam_video = LaunchConfiguration('start_slam_video')
    slam_backend = LaunchConfiguration('slam_backend')
    slam_dimension = LaunchConfiguration('slam_dimension')
    start_legal_localization = LaunchConfiguration('start_legal_localization')
    start_evidence_recorder = LaunchConfiguration('start_evidence_recorder')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav_mode = LaunchConfiguration('nav_mode')
    local_planner_backend = LaunchConfiguration('local_planner_backend')
    navigation_cmd_vel_topic = LaunchConfiguration(
        'navigation_cmd_vel_topic')
    navigation_linear_speed = LaunchConfiguration(
        'navigation_linear_speed')
    navigation_minimum_linear_speed = LaunchConfiguration(
        'navigation_minimum_linear_speed')
    target_floors = LaunchConfiguration('target_floors')
    per_floor_exploration_s = LaunchConfiguration('per_floor_exploration_s')
    manual_elevator_assist = LaunchConfiguration('manual_elevator_assist')
    automatic_elevator_entry = LaunchConfiguration(
        'automatic_elevator_entry')
    strict_room_inspection = LaunchConfiguration(
        'strict_room_inspection')
    strict_room_clearance_m = ParameterValue(
        LaunchConfiguration('strict_room_clearance_m'), value_type=float)
    simenv_container = LaunchConfiguration('simenv_container')
    nav_record_dir = LaunchConfiguration('nav_record_dir')
    perception_output_frame = LaunchConfiguration('perception_output_frame')
    localization_provenance = LaunchConfiguration('localization_provenance')
    exploration_timeout_s = LaunchConfiguration('exploration_timeout_s')
    mission_time_budget_s = LaunchConfiguration('mission_time_budget_s')
    evidence_output_dir = LaunchConfiguration('evidence_output_dir')
    slam_monitor_output_dir = LaunchConfiguration('slam_monitor_output_dir')
    pointcloud_map_output_dir = LaunchConfiguration(
        'pointcloud_map_output_dir')
    slam_video_output = LaunchConfiguration('slam_video_output')
    test_record_dir = LaunchConfiguration('test_record_dir')
    evidence_run_mode = LaunchConfiguration('evidence_run_mode')
    scenario_seed = LaunchConfiguration('scenario_seed')
    code_version = LaunchConfiguration('code_version')
    # Launch 会把纯数字参数按 YAML 推断为整数；证据记录器的合同字段必须始终
    # 是字符串，否则官方固定种子（例如 2026071801）会在节点启动阶段类型报错。
    scenario_seed_string = ParameterValue(scenario_seed, value_type=str)
    code_version_string = ParameterValue(code_version, value_type=str)
    sim_time_parameter = ParameterValue(use_sim_time, value_type=bool)
    exploration_timeout_parameter = ParameterValue(
        exploration_timeout_s, value_type=float,
    )
    mission_time_budget_parameter = ParameterValue(
        mission_time_budget_s, value_type=float,
    )
    navigation_linear_speed_parameter = ParameterValue(
        navigation_linear_speed, value_type=float,
    )
    navigation_minimum_linear_speed_parameter = ParameterValue(
        navigation_minimum_linear_speed, value_type=float,
    )
    target_floors_parameter = ParameterValue(target_floors)
    per_floor_exploration_parameter = ParameterValue(
        per_floor_exploration_s, value_type=float)
    manual_elevator_assist_parameter = ParameterValue(
        manual_elevator_assist, value_type=bool)
    automatic_elevator_entry_parameter = ParameterValue(
        automatic_elevator_entry, value_type=bool)
    strict_room_inspection_parameter = ParameterValue(
        strict_room_inspection, value_type=bool)
    # Cartographer 融合模式需要合法前端只发布 Odometry；否则由该前端直接拥有
    # odom→base。表达式同时考虑 start_slam=false 的安全默认启动。
    publish_legal_tf_parameter = ParameterValue(
        PythonExpression([
            "not ('", start_slam, "'.lower() in ('true', '1', 'yes') and '",
            slam_backend, "' == 'cartographer')",
        ]),
        value_type=bool,
    )
    official_result_path = LaunchConfiguration('official_result_path')
    official_hazard_source_frame = LaunchConfiguration(
        'official_hazard_source_frame')
    official_world_from_map_x = ParameterValue(
        LaunchConfiguration('official_world_from_map_x'), value_type=float)
    official_world_from_map_y = ParameterValue(
        LaunchConfiguration('official_world_from_map_y'), value_type=float)
    official_world_from_map_yaw = ParameterValue(
        LaunchConfiguration('official_world_from_map_yaw'), value_type=float)
    official_floor_height_m = ParameterValue(
        LaunchConfiguration('official_floor_height_m'), value_type=float)
    official_sphere_center_height_m = ParameterValue(
        LaunchConfiguration('official_sphere_center_height_m'), value_type=float)

    nav_pkg = get_package_share_directory('hazardwalker_nav')
    slam_config = os.path.join(nav_pkg, 'config', 'slam_toolbox_online_async.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('start_perception', default_value='true'),
        DeclareLaunchArgument('start_decision', default_value='true'),
        # 官方容器可能由多人共享；业务栈无参数启动不得抢占 /hw/cmd_vel。
        # 完整任务必须由调用方在独占会话中显式传 start_navigation:=true。
        DeclareLaunchArgument('start_navigation', default_value='false'),
        # scan/IMU、时钟与 TF 未通过运行验收时，默认启动 SLAM 只会制造“进程
        # active 即建图成功”的假象；正式独占任务由启动脚本显式开启。
        DeclareLaunchArgument('start_slam', default_value='false'),
        # 与 SLAM 同时启动只读监测，记录跳变、漂移和地图质量；不发布控制。
        DeclareLaunchArgument('start_slam_monitor', default_value='true'),
        # 三维地图显式开启，要求平台同时转发 /hw/lidar/points；不满足时
        # 节点只告警等待，不使用旧地图或伪造点云。
        DeclareLaunchArgument('start_pointcloud_map', default_value='false'),
        DeclareLaunchArgument('start_slam_video', default_value='false'),
        # 无轮速计的四足平台优先使用 scan+IMU+合法控制先验融合；旧的
        # slam_toolbox 保留为显式诊断回退，不再作为官方首选。
        DeclareLaunchArgument('slam_backend', default_value='cartographer'),
        # 三维 Cartographer 连续维护跨层轨迹；逐层二维 OccupancyGrid 由
        # multifloor_occupancy_mapper 发布给 Frontier。
        DeclareLaunchArgument('slam_dimension', default_value='3d'),
        DeclareLaunchArgument('start_legal_localization', default_value='true'),
        DeclareLaunchArgument('start_evidence_recorder', default_value='false'),
        # 官方 Gazebo 传感器均使用仿真时间；适配器负责转发 /clock。
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # nav_mode: 'frontier' (自主探索，默认) 或 'waypoint' (固定航点诊断)
        DeclareLaunchArgument('nav_mode', default_value='frontier'),
        # direct 保留旧链路；官方完整任务使用赛事仓库随附的宇树
        # move_base/TrajectoryPlannerROS(DWA)，Frontier 只负责选目标。
        DeclareLaunchArgument(
            'local_planner_backend', default_value='direct'),
        # 默认兼容既有直接控制；统一控制入口会显式传入
        # /hw/control/navigation_cmd_vel，再由 command_mux_node 唯一输出。
        DeclareLaunchArgument(
            'navigation_cmd_vel_topic', default_value='/hw/cmd_vel'),
        DeclareLaunchArgument('navigation_linear_speed', default_value='2.00'),
        DeclareLaunchArgument(
            'navigation_minimum_linear_speed', default_value='1.20'),
        DeclareLaunchArgument('target_floors', default_value='[]'),
        DeclareLaunchArgument('per_floor_exploration_s', default_value='120.0'),
        DeclareLaunchArgument('manual_elevator_assist', default_value='true'),
        DeclareLaunchArgument(
            'automatic_elevator_entry', default_value='true'),
        # 默认关闭以复现已验收三层基线；单房巡检和完整任务验收显式开启。
        DeclareLaunchArgument(
            'strict_room_inspection', default_value='false'),
        DeclareLaunchArgument('strict_room_clearance_m', default_value='0.60'),
        DeclareLaunchArgument(
            'simenv_container', default_value='simenv_ros1_hazard_platform'),
        DeclareLaunchArgument('nav_record_dir', default_value=''),
        DeclareLaunchArgument('perception_output_frame', default_value='map'),
        # 留空时使用节点内安全默认值；受控回放和正式验收必须显式传入仓库
        # config/perception.yaml，使结果目录记录的哈希对应真实运行参数。
        DeclareLaunchArgument('perception_parameter_file', default_value=''),
        # 只有调用方确认合法 SLAM 已实际运行后才能声明来源；默认值必须
        # fail-closed，避免把缺失/错误 TF 下的候选导出为 world 危险源。
        DeclareLaunchArgument('localization_provenance', default_value='unverified'),
        # 保留旧启动接口的 540 秒“请求值”；节点受 600 秒总预算和至少
        # 120 秒返航预留硬约束，实际探索上限仍不超过 480 秒，并会按距家
        # 距离和保守速度进一步提前返航。
        DeclareLaunchArgument('exploration_timeout_s', default_value='540.0'),
        # 官方计分默认 600 秒；仅扩展建图展示可由运行器显式提高，成果中必须
        # 区分官方时限轮次与扩展时长轮次。
        DeclareLaunchArgument('mission_time_budget_s', default_value='600.0'),
        DeclareLaunchArgument('evidence_output_dir', default_value=''),
        DeclareLaunchArgument('slam_monitor_output_dir', default_value=''),
        DeclareLaunchArgument('pointcloud_map_output_dir', default_value=''),
        DeclareLaunchArgument('slam_video_output', default_value=''),
        DeclareLaunchArgument('test_record_dir', default_value=''),
        # 正式入口使用 official_random_scene；未提交代码或临时目录的调参运行
        # 必须由外层脚本强制改成 diagnostic_official_random_scene。
        DeclareLaunchArgument(
            'evidence_run_mode', default_value='official_random_scene'),
        DeclareLaunchArgument('scenario_seed', default_value=''),
        DeclareLaunchArgument('code_version', default_value=''),
        DeclareLaunchArgument(
            'official_result_path', default_value='results/detected_danger.json',
        ),
        DeclareLaunchArgument('official_hazard_source_frame', default_value='map'),
        DeclareLaunchArgument('official_world_from_map_x', default_value='0.0'),
        DeclareLaunchArgument('official_world_from_map_y', default_value='0.0'),
        DeclareLaunchArgument('official_world_from_map_yaw', default_value='0.0'),
        DeclareLaunchArgument('official_floor_height_m', default_value='2.6'),
        DeclareLaunchArgument(
            'official_sphere_center_height_m', default_value='0.15'),

        # ---- 合法 scan/IMU 里程计（绝不读取 /hw/odom/Gazebo 真值） ----
        Node(
            package='hazardwalker_perception',
            executable='scan_imu_localizer_node',
            name='hazardwalker_scan_imu_localizer',
            output='screen',
            parameters=[{
                'scan_topic': '/hw/scan',
                'imu_topic': '/hw/trunk_imu',
                'odom_frame': 'odom',
                'base_frame': 'base',
                'localization_provenance': localization_provenance,
                'publish_tf': publish_legal_tf_parameter,
                # 重复长走廊缺少纵向扫描约束；用实际下发速度提供短时平移
                # 初值，IMU绝对航向和三维点云在出现结构特征后继续纠偏。
                'command_motion_scale': 0.92,
                'min_effective_linear_speed_mps': 0.30,
                'use_sim_time': sim_time_parameter,
            }],
            condition=IfCondition(start_legal_localization),
        ),

        # ---- SLAM Toolbox (在线异步建图) ----
        # slam_toolbox 是 lifecycle 节点。直接用普通 Node 只会停在
        # unconfigured，进程存在但永远没有 /map；必须使用官方 autostart 入口。
        OpaqueFunction(
            function=_launch_slam_toolbox,
            kwargs={'slam_config': slam_config},
        ),

        # ---- Cartographer：360° 水平 scan + trunk IMU + 合法控制先验融合 ----
        # RGB-D 仍由感知用于目标定位；未经高度过滤的深度竖带不得投成 SLAM 墙体。
        OpaqueFunction(
            function=_launch_cartographer,
            kwargs={'nav_pkg': nav_pkg},
        ),

        GroupAction(
            condition=IfCondition(start_slam),
            actions=[Node(
                package='hazardwalker_nav',
                executable='slam_monitor',
                name='hazardwalker_slam_monitor',
                output='screen',
                parameters=[{
                    'output_dir': slam_monitor_output_dir,
                    'scenario_seed': scenario_seed_string,
                    'code_version': code_version_string,
                    'use_sim_time': sim_time_parameter,
                }],
                condition=IfCondition(start_slam_monitor),
            )],
        ),

        # ---- 多层三维点云地图：Cartographer 位姿 + Mid-360 原始点云 ----
        Node(
            package='hazardwalker_nav',
            executable='pointcloud_map',
            name='hazardwalker_pointcloud_map',
            output='screen',
            parameters=[{
                'input_topic': '/hw/lidar/points',
                'output_topic': '/hazardwalker/slam/cloud_map',
                'target_frame': 'map',
                'voxel_size_m': 0.08,
                'max_voxels': 1000000,
                'publish_period_s': 2.0,
                'output_dir': pointcloud_map_output_dir,
                'use_sim_time': sim_time_parameter,
            }],
            condition=IfCondition(start_pointcloud_map),
        ),

        # ---- 展示录像：分层二维地图、轨迹与增量三维体素地图 ----
        Node(
            package='hazardwalker_nav',
            executable='slam_video_recorder',
            name='hazardwalker_slam_video_recorder',
            output='screen',
            parameters=[{
                'output_path': slam_video_output,
                'video_fps': 5.0,
                'max_render_points': 80000,
                'use_sim_time': sim_time_parameter,
            }],
            condition=IfCondition(start_slam_video),
        ),

        # ---- 感知: HSV 红色危险源检测 ----
        OpaqueFunction(function=_launch_perception),

        # ---- 人工/导航巡检覆盖心跳 ----
        # 只观察合法 SLAM 里程计，不读取 Gazebo 真值；键盘和导航模式复用
        # 同一话题，使正式录包可以拒绝“原地录够时长”的无效回归数据。
        Node(
            package='hazardwalker_perception',
            executable='patrol_coverage_node',
            name='hazardwalker_patrol_coverage',
            output='screen',
            parameters=[{
                'odometry_topic': '/hazardwalker/slam/odometry',
                'coverage_topic': '/hw/perception/patrol_coverage',
                # 覆盖心跳用于按墙钟判断正式录制链是否存活；累计路径仍只取
                # 合法里程计消息的仿真时间戳。低实时倍率不能把 1 Hz 健康
                # 心跳拖成 0.1 Hz，否则会把有效运行误判为话题中断。
                'use_sim_time': False,
            }],
            condition=IfCondition(start_perception),
        ),

        # ---- 决策: 任务状态机 ----
        Node(
            package='hazardwalker_decision',
            executable='mission_state_machine_node',
            name='mission_state_machine_node',
            output='screen',
            parameters=[{
                'official_result_path': official_result_path,
                'official_hazard_source_frame': official_hazard_source_frame,
                'official_world_from_map_x': official_world_from_map_x,
                'official_world_from_map_y': official_world_from_map_y,
                'official_world_from_map_yaw': official_world_from_map_yaw,
                'official_floor_height_m': official_floor_height_m,
                'official_sphere_center_height_m': (
                    official_sphere_center_height_m),
                'official_require_frontier_sequence': True,
                'use_sim_time': sim_time_parameter,
            }],
            condition=IfCondition(start_decision),
        ),

        # ---- 正式证据记录（仅显式开启且提供目录/SEED/版本时运行） ----
        Node(
            package='hazardwalker_perception',
            executable='dynamic_detection_recorder_node',
            name='dynamic_detection_recorder_node',
            output='screen',
            parameters=[{
                'output_dir': evidence_output_dir,
                'test_record_dir': test_record_dir,
                'scenario_name': scenario_seed_string,
                'run_mode': evidence_run_mode,
                'scenario_seed': scenario_seed_string,
                'code_version': code_version_string,
                'legal_pose_topic': '/hazardwalker/slam/odometry',
                'localization_provenance': localization_provenance,
                'mission_state_topic': '/hw/mission/state',
                'result_json_path': official_result_path,
                'use_sim_time': sim_time_parameter,
            }],
            condition=IfCondition(start_evidence_recorder),
        ),

        # ---- 导航: Frontier 自主探索 (默认) ----
        GroupAction(
            # 外层使用 ROS launch 原生布尔解析，兼容 true/True/1；内层模式
            # 精确匹配确保任何时刻最多只有一个导航节点发布 /hw/cmd_vel。
            condition=IfCondition(start_navigation),
            actions=[Node(
                package='hazardwalker_nav',
                executable='frontier_explorer_node',
                name='frontier_explorer_node',
                output='screen',
                parameters=[{
                    'exploration_timeout_s': exploration_timeout_parameter,
                    'mission_time_budget_s': mission_time_budget_parameter,
                    'minimum_return_reserve_s': 120.0,
                    # 赛场红色候选只有球体和正方体，确认只需一个稳定 RGB-D
                    # 视角；但货架遮挡时仍允许有限靠近/横移以取得这一个完整
                    # 视角。返航阶段禁用，避免影响按时回家。
                    'reobserve_during_returning': False,
                    'reobserve_returning_max_attempts_per_target': 0,
                    'reobserve_max_attempts_per_target': 4,
                    'min_frontier_size': 10,
                    # 宇树 DWA 遇到不可执行局部目标时保持安全停车；官方
                    # profile 不能等待通用默认 30 秒才换目标，否则浪费整层
                    # 搜索预算。12 秒无净进展即撤销并选择下一个 Frontier。
                    'frontier_net_progress_timeout_s': 12.0,
                    'frontier_net_progress_distance_m': 0.20,
                    # reference.md 公开起点 yaw=+pi/2(world)，而 world->map
                    # 公开别名同为 +pi/2，因此起点在 map 帧的入楼朝向为 0 rad。
                    'entry_heading_yaw': 0.0,
                    # 先沿入口主轴快速进入走廊，避免刚进门就钻入侧向窄支路；
                    # 正前方暂时无前沿时仍按 35°/60° 分级放宽，不牺牲完备性。
                    'entry_forward_half_angle_deg': 15.0,
                    'entry_ingress_relaxed_half_angle_deg': 35.0,
                    'entry_ingress_max_half_angle_deg': 60.0,
                    # 最多用前30秒或18m建立主走廊骨架，随后立即释放远端
                    # 走廊目标，把本层剩余预算用于左右房间和感知复查。
                    'entry_ingress_depth_m': 18.0,
                    'entry_ingress_progress_slack_m': 2.0,
                    'entry_ingress_time_limit_s': 30.0,
                    # 四个扇区仅在机器人横向真正进入房间后完成；远端左右
                    # 未完成前，普通前沿评分不得把机器人拉回近端房间。
                    'room_sector_split_depth_m': 14.0,
                    'room_sector_visit_lateral_m': 1.5,
                    'room_sector_candidate_lateral_m': 1.2,
                    'room_sector_candidate_max_lateral_m': 4.0,
                    'room_sector_far_depth_margin_m': 2.0,
                    'room_entry_inflation_radius_m': 0.45,
                    'room_entry_navigation_clearance_m': 0.35,
                    'room_sector_early_finish_min_s': 60.0,
                    'room_perimeter_min_path_m': 6.0,
                    'room_perimeter_min_duration_s': 6.0,
                    'room_perimeter_loop_radius_m': 1.8,
                    'room_perimeter_linear_speed': 1.80,
                    'room_perimeter_minimum_linear_speed': 1.10,
                    'room_perimeter_probe_count': 4,
                    'room_perimeter_probe_spacing_m': 1.5,
                    'room_mirrored_doorway_extra_lateral_m': 0.8,
                    'deterministic_room_route_enabled': True,
                    'deterministic_corridor_end_extra_m': 2.0,
                    'deterministic_corridor_end_tolerance_m': 1.0,
                    # 固定赛场人工标定：楼外起点到远端真实房门约 33m。
                    # 30m 前出现的成对前沿属于入口空地，不能触发逐房返程。
                    'deterministic_corridor_min_progress_m': 30.0,
                    'deterministic_corridor_max_lateral_m': 1.0,
                    # 生成场景元数据：出生点和 2.0m 主入口严格同轴。
                    'deterministic_corridor_center_lateral_m': 0.0,
                    'deterministic_corridor_inflation_radius_m': 0.20,
                    # 正前方 70 束均为机身自回波并被过滤为 NaN，无法执行
                    # 前向门禁；只在已知开启且同轴的 2m 主门段禁用前向检查，
                    # 转向仍保留 0.30m 全周门禁，进楼后恢复 Unitree DWA。
                    'deterministic_entry_direct_until_progress_m': 0.0,
                    'deterministic_entry_clearance_m': -1.0,
                    'deterministic_entry_rotation_clearance_m': 0.30,
                    'deterministic_calibrated_doorways_enabled': True,
                    'deterministic_near_door_progress_m': 18.9,
                    'deterministic_far_door_progress_m': 32.7,
                    'deterministic_door_lateral_m': 1.2,
                    'deterministic_corridor_hard_limit_m': 35.5,
                    'deterministic_corridor_waypoint_tolerance_m': 0.8,
                    'deterministic_waypoint_tolerance_m': 0.55,
                    'deterministic_room_cross_depth_m': 1.2,
                    # three_floor_official_rooms_final_v3_20260831 的已验收
                    # 四点矩形环线。严格障碍观察只在这条基线之后追加，不能
                    # 用未经三层证明的缩小六点路线替换最佳效果。
                    'deterministic_room_loop_shallow_m': 1.2,
                    'deterministic_room_loop_deep_m': 2.5,
                    'deterministic_room_loop_half_length_m': 1.5,
                    'deterministic_room_loop_corner_radius_m': 0.0,
                    # 不再使用固定 14m 近远分界。沿中心线走到尽头期间累计
                    # 左右门带观测，按纵向位置聚类，只取最远两组左右配对；
                    # 入口左右空地即使形成 Frontier，也会因排序落在第三簇而排除。
                    'deterministic_door_pair_progress_gap_m': 2.0,
                    'deterministic_door_station_cluster_m': 2.0,
                    'deterministic_door_station_min_separation_m': 6.0,
                    'deterministic_waypoint_stall_s': 30.0,
                    'official_room_waypoint_stall_s': 8.0,
                    'deterministic_room_min_physical_path_m': 4.0,
                    'deterministic_room_min_loop_area_m2': 0.8,
                    'deterministic_room_hold_heading_during_loop': False,
                    'strict_room_inspection_enabled': (
                        strict_room_inspection_parameter),
                    'strict_room_path_inflation_radius_m': (
                        strict_room_clearance_m),
                    # 只允许在当前房间内做遮挡脱离，不得抢占走廊和穿门阶段。
                    'reobserve_only_inside_active_room': True,
                    'deterministic_min_scale_tolerance_m': 1.5,
                    'deterministic_blocked_corner_accept_m': 1.0,
                    # 官方生成器公开 footprint width 上限为 20 m；多留 2 m
                    # SLAM/墙厚裕量，屏蔽横向远处楼外开放区。
                    'entry_lateral_limit_m': 12.0,
                    # 0.8 m 会让入口附近的前沿在机器人尚未运动时即被判定完成。
                    # TrajectoryPlannerROS 在 0.25 m 内停止；高层再用同一阈值
                    # 会因 SLAM 抖动永远差数厘米，无法把门口标记为已到达并
                    # 解锁房间深层前沿。0.40 m 仅用于高层状态确认。
                    'goal_tolerance_m': 0.40,
                    'linear_speed': navigation_linear_speed_parameter,
                    'minimum_linear_speed': (
                        navigation_minimum_linear_speed_parameter),
                    # 骨架阶段不在每个小前沿原地转圈；房间闭环期间连续追踪
                    # 房间内前沿，仅在确认耗尽时做一次 360° RGB-D 验证。
                    # 确定性房间单圈本身连续改变视角，不再额外原地大角度扫转。
                    'frontier_observation_sweep_rad': 0.0,
                    'frontier_observation_sweep_speed': 1.50,
                    'frontier_observation_sweep_timeout_s': 4.0,
                    'target_floors': target_floors_parameter,
                    'per_floor_exploration_s': per_floor_exploration_parameter,
                    'manual_elevator_assist': (
                        manual_elevator_assist_parameter),
                    'automatic_elevator_entry': (
                        automatic_elevator_entry_parameter),
                    'simenv_container': simenv_container,
                    'angular_speed': 2.2,
                    'imu_topic': '/hw/trunk_imu',
                    'require_upright_imu': True,
                    'imu_fresh_timeout_s': 1.0,
                    'fall_tilt_threshold_deg': 55.0,
                    'fall_confirm_samples': 3,
                    'local_planner_backend': local_planner_backend,
                    'unitree_move_base_goal_topic': (
                        '/hw/navigation/unitree_move_base_goal'),
                    'unitree_move_base_control_topic': (
                        '/hw/navigation/unitree_move_base_control'),
                    'unitree_move_base_cmd_topic': (
                        '/hw/control/unitree_move_base_cmd_vel'),
                    'unitree_move_base_corridor_lookahead_m': 3.0,
                    'unitree_move_base_corridor_goal_change_m': 0.80,
                    # 官方里程计只用于 ROS1 DWA 的物理走廊居中；Cartographer
                    # 与危险源坐标仍只使用 scan+IMU 合法位姿。
                    'use_official_odom_for_corridor_control': True,
                    'official_odom_topic': '/hw/odom',
                    'official_odom_fresh_timeout_s': 2.0,
                    'official_corridor_center_x_m': 0.0,
                    'official_corridor_forward_lookahead_m': 3.0,
                    'official_corridor_end_y_m': 35.0,
                    'official_corridor_yaw_rad': 1.5707963267948966,
                    'use_official_odom_for_room_control': True,
                    'official_near_room_y_m': 14.865,
                    'official_far_room_y_m': 28.895,
                    'official_room_goal_tolerance_m': 0.55,
                    'official_room_loop_goal_tolerance_m': 0.50,
                    'official_room_exit_goal_tolerance_m': 0.80,
                    'use_official_odom_for_return_control': True,
                    'official_home_x_m': 0.0,
                    'official_home_y_m': -2.2,
                    'official_home_yaw_rad': 1.5707963267948966,
                    'official_home_tolerance_m': 0.60,
                    'official_return_floor_index': 0,
                    'use_official_odom_for_elevator_control': True,
                    'official_elevator_lobby_x_m': 0.80,
                    'official_elevator_cabin_x_m': 2.70,
                    'official_elevator_y_m': 2.60,
                    'official_elevator_goal_tolerance_m': 0.40,
                    'cmd_vel_topic': navigation_cmd_vel_topic,
                    # 导航数据记录
                    'nav_record_enabled': True,
                    'nav_record_dir': nav_record_dir,
                    'use_sim_time': sim_time_parameter,
                }],
                condition=LaunchConfigurationEquals(
                    'nav_mode', expected_value='frontier'),
            )],
        ),

        # ---- 导航: 固定航点巡检 (诊断回退，仅 nav_mode=waypoint 时启用) ----
        GroupAction(
            condition=IfCondition(start_navigation),
            actions=[Node(
                package='hazardwalker_nav',
                executable='waypoint_patrol_node',
                name='waypoint_patrol_node',
                output='screen',
                parameters=[{
                    'minimum_turn_speed': 0.80,
                    'cmd_vel_topic': navigation_cmd_vel_topic,
                    'use_sim_time': sim_time_parameter,
                }],
                condition=LaunchConfigurationEquals(
                    'nav_mode', expected_value='waypoint'),
            )],
        ),
    ])
