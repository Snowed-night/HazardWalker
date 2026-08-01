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
        'hazardwalker_cartographer_configuration_files',
    )
    shutil.copytree(builtin_dir, runtime_dir, dirs_exist_ok=True)
    shutil.copy2(
        os.path.join(nav_pkg, 'config', 'cartographer_official_2d.lua'),
        os.path.join(runtime_dir, 'cartographer_official_2d.lua'),
    )
    use_sim_time = _as_bool(
        LaunchConfiguration('use_sim_time').perform(context),
    )
    return [
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='hazardwalker_cartographer',
            output='screen',
            arguments=[
                '-configuration_directory', runtime_dir,
                '-configuration_basename', 'cartographer_official_2d.lua',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                # Cartographer 单雷达模式的标准输入名是 scan；只有配置两个
                # LaserScan 时才改为 scan_1、scan_2。
                ('scan', '/hw/scan'),
                ('imu', '/hw/trunk_imu'),
                ('odom', '/hazardwalker/slam/odometry'),
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='hazardwalker_cartographer_occupancy_grid',
            output='screen',
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ]


def generate_launch_description():
    """按显式开关组合业务节点，官方 profile 不引入任何模拟平台节点。"""

    start_perception = LaunchConfiguration('start_perception')
    start_decision = LaunchConfiguration('start_decision')
    start_navigation = LaunchConfiguration('start_navigation')
    start_slam = LaunchConfiguration('start_slam')
    slam_backend = LaunchConfiguration('slam_backend')
    start_legal_localization = LaunchConfiguration('start_legal_localization')
    start_evidence_recorder = LaunchConfiguration('start_evidence_recorder')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav_mode = LaunchConfiguration('nav_mode')
    perception_output_frame = LaunchConfiguration('perception_output_frame')
    localization_provenance = LaunchConfiguration('localization_provenance')
    exploration_timeout_s = LaunchConfiguration('exploration_timeout_s')
    evidence_output_dir = LaunchConfiguration('evidence_output_dir')
    test_record_dir = LaunchConfiguration('test_record_dir')
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
        # 无轮速计的四足平台优先使用 scan+IMU+合法控制先验融合；旧的
        # slam_toolbox 保留为显式诊断回退，不再作为官方首选。
        DeclareLaunchArgument('slam_backend', default_value='cartographer'),
        DeclareLaunchArgument('start_legal_localization', default_value='true'),
        DeclareLaunchArgument('start_evidence_recorder', default_value='false'),
        # 官方 Gazebo 传感器均使用仿真时间；适配器负责转发 /clock。
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # nav_mode: 'frontier' (自主探索，默认) 或 'waypoint' (固定航点诊断)
        DeclareLaunchArgument('nav_mode', default_value='frontier'),
        DeclareLaunchArgument('perception_output_frame', default_value='map'),
        # 只有调用方确认合法 SLAM 已实际运行后才能声明来源；默认值必须
        # fail-closed，避免把缺失/错误 TF 下的候选导出为 world 危险源。
        DeclareLaunchArgument('localization_provenance', default_value='unverified'),
        # 保留旧启动接口的 540 秒“请求值”；节点受 600 秒总预算和至少
        # 120 秒返航预留硬约束，实际探索上限仍不超过 480 秒，并会按距家
        # 距离和保守速度进一步提前返航。
        DeclareLaunchArgument('exploration_timeout_s', default_value='540.0'),
        DeclareLaunchArgument('evidence_output_dir', default_value=''),
        DeclareLaunchArgument('test_record_dir', default_value=''),
        DeclareLaunchArgument('scenario_seed', default_value=''),
        DeclareLaunchArgument('code_version', default_value=''),
        DeclareLaunchArgument(
            'official_result_path', default_value='results/detected_danger.json',
        ),

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
                'publish_tf': publish_legal_tf_parameter,
                'command_motion_scale': 1.0,
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

        # ---- 感知: HSV 红色危险源检测 ----
        Node(
            package='hazardwalker_perception',
            executable='hsv_detector_node',
            name='hsv_detector_node',
            output='screen',
            parameters=[{
                'camera_axis_convention': 'gazebo_link_x_forward',
                'output_frame': perception_output_frame,
                'localization_provenance': localization_provenance,
                'use_sim_time': sim_time_parameter,
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
                'run_mode': 'official_random_scene',
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
                    'mission_time_budget_s': 600.0,
                    'minimum_return_reserve_s': 120.0,
                    'min_frontier_size': 10,
                    # reference.md 公开起点 yaw=+pi/2(world)，而 world->map
                    # 公开别名同为 +pi/2，因此起点在 map 帧的入楼朝向为 0 rad。
                    'entry_heading_yaw': 0.0,
                    'entry_forward_half_angle_deg': 35.0,
                    # 在公开入口轴上至少深入 6 m 后再允许全向前沿竞争，
                    # 避免刚选中入口就被楼外南北开放边界吸走。
                    'entry_ingress_depth_m': 6.0,
                    # 官方生成器公开 footprint width 上限为 20 m；多留 2 m
                    # SLAM/墙厚裕量，屏蔽横向远处楼外开放区。
                    'entry_lateral_limit_m': 12.0,
                    # 0.8 m 会让入口附近的前沿在机器人尚未运动时即被判定完成。
                    'goal_tolerance_m': 0.25,
                    'linear_speed': 0.35,
                    'angular_speed': 1.5,
                    # 导航数据记录
                    'nav_record_enabled': True,
                    'nav_record_dir': '',
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
                    'minimum_turn_speed': 0.45,
                    'use_sim_time': sim_time_parameter,
                }],
                condition=LaunchConfigurationEquals(
                    'nav_mode', expected_value='waypoint'),
            )],
        ),
    ])
