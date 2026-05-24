# ROS 2 Packages

本目录存放各个 ROS 2 功能包源码。

## 当前包

- `hazardwalker_platform`：平台适配，统一仿真和官方平台接口。
- `hazardwalker_nav`：固定航点、Frontier/SLAM/Nav2、自主探索、返航和卡死恢复。
- `hazardwalker_perception`：红球检测、相机雷达融合和三维定位。
- `hazardwalker_decision`：任务状态机、目标选择、重观察和返航约束。
- `hazardwalker_bringup`：一键启动和系统集成。
- `hazardwalker_msgs`：自定义消息定义。

## 命名约定

- Python 包、ROS 包和消息名继续使用英文小写下划线风格。
- 文档、README 和配置说明尽量使用中文，文件名可以直接表达用途。
