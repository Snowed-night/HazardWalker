-- 官方 SimEnv 三维 Cartographer：Mid-360 + trunk IMU 原生激光惯性定位。
-- 不读取 Gazebo 里程计、场景布局或危险源真值；同一三维轨迹连续覆盖多楼层。

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
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  -- 三维点云和 IMU 直接构成激光惯性前端。失败证据证明自写 scan/IMU
  -- 里程计在重复长走廊中会产生数十米纵向漂移，因此不能再作为外部 odom
  -- 注入 Cartographer；真实平移必须由连续三维点云配准约束。
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.01,
  trajectory_publish_period_sec = 0.03,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 0.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_3D.min_range = 0.40
TRAJECTORY_BUILDER_3D.max_range = 20.0
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 3
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 0.50
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 20.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 1.00
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 20.0
-- 使用速度外推提供初值，Ceres点到体素匹配负责平移与旋转约束。
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.30
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(3.0)
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 40.0
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.75
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.12
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(1.5)
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.15
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20.0
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 120
TRAJECTORY_BUILDER_3D.use_intensities = false

POSE_GRAPH.optimize_every_n_nodes = 180
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.constraint_builder.max_constraint_distance = 3.0
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.80
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10

return options
