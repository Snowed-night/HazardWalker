# 平台组文档

负责范围：

- `fake_platform_node` 的话题、TF 和传感器接口说明。
- Gazebo Harmonic 最小场景、机器人模型和红球模型。
- Gazebo / 官方平台到 `/hw/*` 内部接口的 adapter。
- 楼栋场景最小需求清单。

近期重点：

1. 在主力机验证 `gazebo_minimal.launch.py`。
2. 梳理 `ros_gz_bridge.yaml` 中 Gazebo 话题与 `/hw/*` 的映射。
3. 补齐 `CameraInfo` 来源。
4. 列出从单房间扩展到楼栋场景所需的模型、传感器和 TF 要求。
