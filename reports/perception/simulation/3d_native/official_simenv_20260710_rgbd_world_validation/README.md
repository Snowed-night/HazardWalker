# 官方 SimEnv ROS2 RGB-D 世界坐标验证（2026-07-10）

本目录记录一次 headless 官方 ROS2 Harmonic 平台的真实运行验证。验证对象是临时生成的半径 `0.15 m` 红球，用于检查 RGB、深度、内参、TF 和世界坐标输出；它不是比赛危险源，未读取或使用 `danger_truth.json` 等裁判真值文件。

## 已验证结果

- `/hw/camera/image_raw`、`/hw/camera/depth_image`、`/hw/camera/camera_info` 均有单一稳定发布者。
- 当前 HSV + 深度 + TF 分支检出受控球，置信度为 `0.8735`，并输出 `world` 坐标。
- 修正后输出为 `(-0.0000, -1.3745, 0.2662)`，相对受控生成位置 `(0, -1.4, 0.3)` 的完整三维误差为 `0.0423 m`。
- 修正后截图见 [controlled_red_ball_rgb_after_z_fix.png](controlled_red_ball_rgb_after_z_fix.png)。

## 动态坐标稳定性

平台 A1 控制器当前会让 `Odometry_gazebo` 发生朝向变化，但 Gazebo 公开模型世界位姿并未同步改变。为避免这种伪运动污染定位，平台新增 `world_pose_tf_node`，直接基于公开 `/world/generated_world/pose/info` 发布 `world -> hazardwalker_camera`。

- 远距离受控球转向前后的输出坐标漂移仅 `0.0025 m`。
- 该场景没有真实相机视角变化，故轨迹保持 `tentative`、`distinct_view_count=1`，未被错误确认。
- 截图见 [controlled_red_ball_far_dynamic_world.png](controlled_red_ball_far_dynamic_world.png)。
- 图像时间比 PoseInfo TF 新数毫秒时，`allow_latest_tf_fallback` 仍使最终帧保持 `localized`。

## 球心几何补偿

远距离完整球形 bbox 的投影半径可直接反推球心深度。启用该方法后，受控球 world 输出为 `(0.0004, -0.0125, 0.3043)`，完整三维误差为 `0.0132 m`，相比固定半径补偿的约 `0.0947 m` 明显下降。

## 当前平台限制

ROS2 当前为轮式占位模型，并非真实 Unitree A1 控制链。实测 `/cmd_vel` 已到达 DiffDrive、使轮速约为 `2.125 rad/s`，但模型世界位姿存在迟滞/滑移，且与 `Odometry_gazebo` 不稳定地不一致。因此本轮只能验证“错误里程计不会导致错误确认”，不能将它称为真实双视角动态识别通过。完成 `unitree_guide / junior_ctrl` 或官方等效真实运动控制迁移后，应使用本目录的测试格式复跑多视角确认和主动重观察用例。

## 已定位问题与后续动作

历史基线中世界坐标 Z 误差约 `0.18 m`，根因是平台 relay 将内部 `base_link` 当作里程计原点。已将相机 `z` 外参由 `0.17 m` 修正为模型坐标中的 `0.32 m`，重建并重启后复测通过。平台补丁的操作说明见 `docs/platform/simenv_ros2_rgbd_world_alignment_fix.md`。

详细数值见 `summary.json` 与测试组格式的 `testing_record_perception.csv/json`。
