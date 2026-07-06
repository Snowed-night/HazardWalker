# hazardwalker_platform

平台适配包，负责把自建仿真平台和官方平台的数据统一成 HazardWalker 内部接口。

## 当前职责

- 统一传感器话题到 `/hw/*`
- 统一 TF 坐标系
- 统一机器人控制接口
- 提供仿真场景和平台适配节点

## 当前最小节点

- `fake_platform_node`：在没有 Gazebo/官方平台时发布最小 `/hw/*` 话题，用于打通集成链路。

## 需要保持的接口

- 输出：

```text
/hw/camera/image_raw
/hw/camera/camera_info
/hw/camera/depth_image
/hw/lidar/points
/hw/odom
/tf
/tf_static
```

- 输入：

```text
/hw/cmd_vel
```

## 后续扩展

- `fake_platform_node.py`：当前占位平台，后续可替换为 Gazebo 或官方平台适配节点。
- `config/topics.yaml`：统一话题名映射。
- `config/frames.yaml`：统一坐标系命名。

## 最小深度定位链路

`fake_platform_node` 当前会发布与 RGB 图像对齐的 `32FC1` 深度图，用于在没有真实仿真环境时验证：

```text
/hw/camera/image_raw + /hw/camera/camera_info + /hw/camera/depth_image + /tf
  -> 感知节点输出相对 start 坐标系的危险源位置
```

真实 Gazebo 或官方平台接入时，平台适配层必须继续提供同名 `/hw/*` 话题，并保证深度图与 RGB 图像对齐，TF 链至少能从 `camera_link` 查到 `start` 或配置的输出坐标系。
