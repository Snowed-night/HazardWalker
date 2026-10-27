# 官方 SimEnv Gazebo Classic：red_objects

> 证据类别：内部回归；人工生成受控物体，不属于官方随机场景全流程成绩。

本目录的每张原图和标注图都由当前官方 ROS1/Gazebo Classic 世界的 `/hw/*` 话题采集。模型只在隔离容器中临时生成；案例结束后删除。

- 案例数：14
- 通过/失败：5/9
- ROS_DOMAIN_ID：77
- 临时夹具中心：[-0.24863461812054494, 1.0643436988287387, 0.15]（map->real_sense camera_forward 3.000m (test fixture only)）
- 失败案例保留在 `cases.csv` 和 `images/`，不得删除或改写为成功。
