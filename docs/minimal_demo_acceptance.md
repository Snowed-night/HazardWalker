# Minimal Demo Acceptance Criteria

本文档定义 HazardWalker 第一阶段最小闭环的验收标准。最小闭环的目的不是证明最终性能，而是证明平台、导航、感知、定位、决策和结果输出能够连成一条可运行链路。

## 1. Scope

第一阶段最小闭环包括：

```text
仿真启动
传感器输入
机器人移动
红球检测
坐标输出
返航
结果保存
```

第一阶段不强制包括：

- 完整未知环境自主探索。
- 多层楼复杂地图。
- NBV。
- YOLO。
- 实机运行。
- 多危险源去重。
- 完整卡死恢复。

## 2. Required Inputs

系统至少需要以下输入：

| Input | Required Topic |
|---|---|
| RGB image | `/hw/camera/image_raw` |
| Camera info | `/hw/camera/camera_info` |
| Odometry | `/hw/odom` |
| TF | `/tf`, `/tf_static` |
| Velocity control | `/hw/cmd_vel` |

如果要完成点云三维定位，还需要：

| Input | Required Topic |
|---|---|
| Point cloud | `/hw/lidar/points` |

## 3. Required Outputs

系统至少需要输出：

| Output | Target |
|---|---|
| Mission state | `/hw/mission/state` |
| Red ball detection | `/hw/perception/hazard_detections` or result JSON |
| Result file | `reports/run_results/<timestamp>_result.json` |

## 4. Minimal Scenario

建议最小场景：

- 一个简单房间或走廊。
- 一个机器人。
- 一个红色球体。
- 2-3 个固定航点。
- 起点可记录。
- 无复杂动态障碍物。

## 5. Acceptance Checklist

### 5.1 Launch

- [ ] 一条命令可以启动最小流程。
- [ ] 所有必要节点无崩溃。
- [ ] `use_sim_time` 设置正确。
- [ ] 日志中无持续刷屏错误。

建议命令：

```bash
ros2 launch hazardwalker_bringup minimal_demo.launch.py
```

### 5.2 Sensor Interface

- [ ] `/hw/camera/image_raw` 有图像。
- [ ] `/hw/camera/camera_info` 有内参。
- [ ] `/hw/odom` 有里程计。
- [ ] `/tf` 中存在 `odom -> base_link`。
- [ ] 相机 frame 和机器人 base frame 可转换。

如使用雷达：

- [ ] `/hw/lidar/points` 有点云。
- [ ] `lidar_link` 和 `base_link` 可转换。

### 5.3 Motion

- [ ] 机器人能离开起点。
- [ ] 机器人能到达至少 2 个固定航点。
- [ ] 机器人能接收 `/hw/cmd_vel` 或内部导航目标。
- [ ] 导航失败时不会导致系统直接崩溃。

### 5.4 Perception

- [ ] 相机看到红球时，检测节点能产生候选目标。
- [ ] 调试图像能显示检测框或 mask。
- [ ] 检测结果包含置信度。
- [ ] 无红球时不应持续输出高置信度目标。

### 5.5 Localization

- [ ] 系统能输出红球相对 `map` 或 `start` 的坐标。
- [ ] 输出坐标包含 frame id。
- [ ] 输出坐标包含 timestamp 或任务时间。

第一阶段如果点云定位尚未完成，可临时输出估计坐标并标记为 `tentative`，但最终版本必须接入点云或深度信息。

### 5.6 Return Home

- [ ] 系统能记录起点。
- [ ] 航点巡检结束后能切换到 `RETURNING`。
- [ ] 机器人能回到起点附近。
- [ ] 返回成功后进入 `FINISHED`。

建议第一阶段返回成功阈值：

```text
水平距离 <= 0.5 m
```

该阈值后续根据官方要求调整。

### 5.7 Result File

- [ ] 运行结束后生成 JSON 结果文件。
- [ ] 结果文件包含 mission status。
- [ ] 结果文件包含危险源列表。
- [ ] 结果文件包含运行时间。
- [ ] 结果文件包含是否返航成功。

示例：

```json
{
  "mission_id": "minimal_demo_001",
  "status": "FINISHED",
  "hazards": [
    {
      "id": 1,
      "position": [1.2, -0.4, 0.8],
      "frame_id": "start",
      "confidence": 0.91
    }
  ],
  "metrics": {
    "duration_sec": 120.5,
    "return_success": true
  }
}
```

## 6. Pass / Fail Rule

最小闭环通过条件：

- 能一键启动。
- 机器人能按固定航点移动。
- 能识别红球。
- 能输出坐标。
- 能返回起点。
- 能生成结果文件。

允许存在的问题：

- 定位精度暂时不高。
- 检测阈值还需要调。
- 场景较简单。
- 只支持一个红球。
- 暂时没有完整自主探索。

不允许存在的问题：

- 无法启动。
- 传感器话题不稳定。
- 节点频繁崩溃。
- 没有任何结果输出。
- 无法控制机器人移动。

## 7. Next Stage After Passing

最小闭环通过后，下一阶段按顺序增强：

1. 固定航点替换为 Frontier 探索。
2. 接入 SLAM Toolbox。
3. 接入点云三维定位。
4. 增加多帧确认和去重。
5. 增加卡死恢复。
6. 增加指标统计。
7. 增加 NBV 和危险源信念地图。
