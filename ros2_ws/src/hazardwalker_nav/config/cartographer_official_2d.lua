-- 官方 SimEnv：只使用公开 LaserScan + trunk IMU 的 Cartographer 2D 配置。
-- 不订阅 /Odometry_gazebo、场景布局或危险源真值；Cartographer 自行提供 map/odom/base。

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  -- 合法里程计只积分本系统已下发 cmd_vel，并由扫描做毫米级校正；它用于约束
  -- 长直墙/单墙场景的平移退化，不读取 /Odometry_gazebo 或任何场景真值。
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  -- 正式地图只融合 360° 水平雷达。旧的 RGB-D 中部竖带投影会把地面、
  -- 楼梯和天花板压成二维墙体，固定种子实测形成放射状弱占据拖影与伪前沿。
  -- 深度图继续供红球三维定位使用，但不再伪装成水平 LaserScan 输入 SLAM。
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.01,
  trajectory_publish_period_sec = 0.03,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.40
-- 官方 /scan 会用接近 30 m 的有限值表示量程上限。若 max_range 同为 30 m，
-- Cartographer 会把这些“未击中”端点写成整圈弱占据障碍和伪前沿。室内正式
-- 建图只把 8 m 内回波作为击中，超出部分按下方有限自由射线处理。
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
-- 子图大小 60→90：实测跳变每 5~8 秒一次，正对应 60 帧子图切换频率（每次新子图
-- 创建时位姿重新匹配、可能错配）。更大子图切换更少、上下文更多，抗错配震荡。
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.25
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.04
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)
-- 官方走廊/单墙视角的 scan translation 存在侧向多解，提高控制先验权重抗沿墙滑移。
-- 2026-08-17 试过降到 50（a3521d7）：返航 36.4→4.9min 但 occupied 1.22%→0.11%，
-- scan matching 沿墙滑移把墙擦成 free，地图一片白，不可接受。回调 100 恢复抗滑移，
-- 震荡改走「增大子图 num_range_data」方向。
-- 2026-08-18 治根尝试：对称无特征走廊里 ceres 从外推位姿出发会收敛到对称错误解（跳变）。
-- 开启实时相关性粗匹配，先全局搜出大致位置再交 ceres 精化，减少陷入局部最优。
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 100.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 100.0
POSE_GRAPH.optimize_every_n_nodes = 60
-- 官方随机楼宇包含大量外观几乎相同的长直墙。默认 15 m 搜索半径会把相隔
-- 3~7 m 的重复走廊误连成闭环，实测导致地图折叠、Frontier 在不足 1 m 处
-- 提前耗尽。只允许与控制/扫描先验相近的局部闭环，并提高接受分数；真正回到
-- 同一区域时先验已足够接近，仍可形成约束。
-- 2026-08-19 治漂移尝试：1.5 m 过保守，回环只在先验 1.5 m 内搜索，漂移一旦
-- 超过该半径就永远无法闭合，实测地图出现墙壁多重错位（4.8% 行驶路径落在墙上）。
-- 放宽到 3.0 m 让重访时能形成回环约束纠正累计漂移；配合相关性粗匹配与
-- min_score/global_localization_min_score 高门槛抗对称走廊误连。
-- 风险：相隔 3~4 m 的重复走廊可能偶发误连，需实测验证是否地图折叠复发。
POSE_GRAPH.constraint_builder.max_constraint_distance = 3.0
POSE_GRAPH.constraint_builder.min_score = 0.72
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.90

return options

