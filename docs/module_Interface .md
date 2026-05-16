# Interface Specification

本文档记录 HazardWalker 各模块之间的接口。任何话题名、坐标系、消息格式、参数文件发生变化，都必须更新本文档。

## Platform Topics

| Type | Topic | Description |
|---|---|---|
| Image | `/camera/image_raw` | RGB camera image |
| PointCloud | `/lidar/points` | LiDAR point cloud |
| Odometry | `/odom` | Robot odometry |
| TF | `/tf` | Transform tree |
| Control | `/cmd_vel` | Velocity command |

## Frames

| Frame | Description |
|---|---|
| `map` | Global map frame |
| `odom` | Odometry frame |
| `base_link` | Robot base frame |
| `camera_link` | Camera frame |
| `lidar_link` | LiDAR frame |

## Hazard Detection Output

```json
{
  "id": 1,
  "position": [0.0, 0.0, 0.0],
  "confidence": 0.0,
  "status": "tentative",
  "first_seen_time": 0.0,
  "last_seen_time": 0.0
}
```

`status` 可取值：

- `tentative`：疑似危险源，需要继续确认
- `confirmed`：已确认危险源
- `rejected`：已剔除目标

## Navigation State

```text
IDLE
EXPLORING
NAVIGATING
REOBSERVING
REPLANNING
RETURNING
FINISHED
FAILED
```
