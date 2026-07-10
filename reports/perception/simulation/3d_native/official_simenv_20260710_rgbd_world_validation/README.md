# 官方 SimEnv ROS2 RGB-D 世界坐标验证（2026-07-10）

本目录记录一次 headless 官方 ROS2 Harmonic 平台的真实运行验证。验证对象是临时生成的半径 `0.15 m` 红球，用于检查 RGB、深度、内参、TF 和世界坐标输出；它不是比赛危险源，未读取或使用 `danger_truth.json` 等裁判真值文件。

## 已验证结果

- `/hw/camera/image_raw`、`/hw/camera/depth_image`、`/hw/camera/camera_info` 均有单一稳定发布者。
- 当前 HSV + 深度 + TF 分支检出受控球，置信度为 `0.8695`，并输出 `world` 坐标。
- 输出 XY 为 `(0.0001, -1.4116)`，相对受控生成位置 `(0, -1.4)` 的平面误差为 `0.0116 m`。
- 截图见 [controlled_red_ball_rgb.png](controlled_red_ball_rgb.png)。

## 已定位问题与后续动作

当前世界坐标 Z 误差为约 `0.18 m`，根因已定位为平台 relay：其里程计原点对应模型原点，故相机 `z` 外参应使用模型坐标中的 `0.32 m`，而不是内部 `base_link` 相对差值 `0.17 m`。平台端补丁已准备且原文件已备份；完成重启后必须复跑本用例，才可将该项标记为通过。

详细数值见 `summary.json` 与测试组格式的 `testing_record_perception.csv/json`。
