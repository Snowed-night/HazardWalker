# 官方 SimEnv Gazebo Classic：partial_visibility

> 证据类别：内部回归；人工生成受控物体，不属于官方随机场景全流程成绩。
> 目录标签为用户指定的 20260715 阶段，真实运行日期为 2026-07-19；
> `run_id`、SEED、Git版本及原路径见 `provenance.json`，不得改写为7月15日实跑。

本目录的每张原图和标注图都由当前官方 ROS1/Gazebo Classic 世界的 `/hw/*` 话题采集。模型只在隔离容器中临时生成；案例结束后删除。

- 案例数：21
- 通过/失败：9/12
- ROS_DOMAIN_ID：77
- 临时夹具中心：[0.2562818271654639, 1.078604529680238, 0.15]（map->real_sense camera_forward 3.000m (test fixture only)）
- 失败案例保留在 `cases.csv` 和 `images/`，不得删除或改写为成功。
