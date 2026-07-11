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

---

## `/hw/*` 话题接口表

### 当前 fake_platform_node 已发布的话题

| 话题 | 消息类型 | 谁发布 | 谁订阅 | 作用 |
|---|---|---|---|---|
| `/hw/camera/image_raw` | `sensor_msgs/Image` | 平台组 | 感知组 | 相机原始图像，供感知组检测红色球体 |
| `/hw/camera/camera_info` | `sensor_msgs/CameraInfo` | 平台组 | 感知组 | 相机内参，供三维定位使用 |
| `/hw/lidar/points` | `sensor_msgs/PointCloud2` | 平台组 | 感知组/导航组 | 激光雷达点云 |
| `/hw/odom` | `nav_msgs/Odometry` | 平台组 | 导航组/决策组 | 机器人实时位姿 |
| `/hw/cmd_vel` | `geometry_msgs/Twist` | 导航组 | 平台组 | 速度控制指令 |

---

## TF 坐标变换链

### 当前 fake_platform_node 发布的 TF

```
odom ──→ base_link ──→ camera_link  
                   ──→ lidar_link   
```




