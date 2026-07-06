# official_simenv_20260703_basic

官方 SimEnv 基础链路验证结果。

- `selected_images/`: 展示用标注图。
- `case_results.csv`: 每张图的检测数量和说明。
- `summary.json`: 结构化结果。
- `raw_frames.zip`: 原始 PPM 归档，不直接展开在 reports 中。

结论：官方世界可以取帧并运行检测；RealSense 默认视角未包含红球，临时 Gazebo 相机视角检测到红球候选。
