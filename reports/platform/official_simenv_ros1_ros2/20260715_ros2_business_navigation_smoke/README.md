# 官方 SimEnv ROS2 业务导航短窗口验收

负责人：姜晨。

本目录记录当前平台分支在独立远程副本中的真实业务层烟雾测试：启动官方 ROS1 容器和 ROS2
适配器后，运行 `official_simenv_business.launch.py start_navigation:=true`，同步订阅 `/hw/cmd_vel`
与 `/hw/odom` 55 秒。

- `summary.json`：导航命令数、非零命令数、首末里程计和二维位移。
- `business_stack.log`：导航、感知和决策节点的原始启动日志。

结果：导航发布 1100 条非零 `/hw/cmd_vel`，官方真值里程计位移约 0.0805 m；感知节点处理真实
640×480 RGB 帧且 `tf=True`。这是接口到真实运动的短窗口证据，不代表已完成红球搜索、多视角确认、
三维定位或整场任务闭环。
