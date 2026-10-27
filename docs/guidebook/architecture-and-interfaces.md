# 架构与接口

官方ROS1仿真产生RGB-D、激光、IMU与运动状态；平台适配器归一化并转发至ROS2。分层Cartographer维护本层地图，导航组织逐房巡查，感知输出候选和复查需求，导航结合障碍条件执行，确认后写入危险源位置。

| 功能 | 源码入口 |
| --- | --- |
| 正式实验和记录 | `scripts/run_stable_three_floor_experiment.sh`、`run_official_slam_exploration.py` |
| ROS1/ROS2桥接 | `scripts/official_simenv_rosbridge_ros2_adapter_node.py` |
| 命令仲裁 | 平台包`command_mux_node.py` |
| 逐房、换层、返航 | 导航包`frontier_explorer_node.py`、`official_return.py` |
| 分层SLAM | 导航包`config/cartographer_official_2d.lua`、`slam_monitor_node.py` |
| 检测与候选 | 感知包`hsv_detector_node.py`、`red_ball_detector.py` |
| 深度几何与位置 | 感知包`localize_hazard.py` |
| 跟踪和观察建议 | 感知包`track_hazards.py`、`active_view_policy.py` |
| 最终坐标输出 | 决策包`result_builder.py` |

包路径统一位于`ros2_ws/src/hazardwalker_<模块>/hazardwalker_<模块>/`。

ROS1输入主要包括`/real_sense/rgb/image_raw`、`/real_sense/depth/image_raw`、相机内参、激光和IMU。此历史版本的位姿源包含`/Odometry_gazebo`。适配器状态和运行清单记录实际来源。

业务模块使用`/hw/*`归一化接口。准确话题名和参数以两个集成launch（`official_simenv_control_interface.launch.py`、`official_simenv_business.launch.py`）及`config/`为准。变更接口时同时检查桥接、launch与记录器。

目标通过反投影和球心补偿得到相机坐标，再转到稳定odom；输出时按楼层锚点处理水平位置，以楼层高度与已知球半径约束高度。每层独立建图，当前配置关闭全局自动回环约束，保留局部扫描匹配、IMU和里程计。

迁移实机时需提供时间同步、相机标定、有效深度和TF，再替换平台位姿与速度接口。感知只提出观察请求，实际运动由导航与命令仲裁负责。本仓库没有Go2-W真机的已验收驱动和端侧链路。
