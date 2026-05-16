# ROS 2 Workspace

ROS 2 工作空间目录。

各个目录分别负责以下功能：

hazardwalker_platform     平台适配，统一仿真/官方平台接口
hazardwalker_nav          SLAM、导航、自主探索和返航
hazardwalker_perception   红球检测、相机雷达融合和三维定位
hazardwalker_decision     任务状态机、NBV、重观察和返航约束
hazardwalker_bringup      一键启动和系统集成
hazardwalker_msgs         自定义消息、服务和动作定义

推荐源码放在：

```text
ros2_ws/src/
```

构建产物 `build/`、`install/`、`log/` 已在 `.gitignore` 中忽略，不要提交。
