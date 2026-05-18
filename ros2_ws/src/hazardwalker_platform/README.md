# hazardwalker_platform

平台适配包，负责屏蔽自建仿真平台和官方平台差异。

职责：

- 统一传感器话题
- 统一 TF 坐标系
- 统一机器人控制接口
- 提供仿真场景和平台配置

## 当前最小节点

- `fake_platform_node`：在没有 Gazebo/官方平台时发布最小 `/hw/*` 话题，用于打通集成链路。

## 必须保持的内部接口

平台适配层应输出：

```text
/hw/camera/image_raw
/hw/camera/camera_info
/hw/lidar/points
/hw/odom
/tf
/tf_static
```

平台适配层应接收：

```text
/hw/cmd_vel
```

官方平台发布后，优先修改本包和 `config/topics.yaml`、`config/frames.yaml`，不要让算法模块直接依赖官方专有话题名。
