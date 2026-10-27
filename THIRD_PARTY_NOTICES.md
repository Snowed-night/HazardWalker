# 第三方来源与运行资源

HazardWalker 自有代码采用 Apache-2.0 许可。ROS、Gazebo、Cartographer、Unitree 及其他第三方目录保留各自作者、版权和许可证，根目录许可不替代它们。

`ros2_ws/src/hazardwalker_platform/src` 包含官方仿真、楼宇生成器、宇树 A1 描述、控制源码及其依赖。机器人网格、激光扫描模式文件保留原路径；复用时请保留原 LICENSE 和版权头。

宇树运动策略与 SDK 预编译库作为外部运行资源管理，不随 Git 分发。路径与 SHA-256 见 `reproduction/runtime-assets.json`，请从具备使用权限的官方 SimEnv 环境获取并通过资源导入脚本准备。
