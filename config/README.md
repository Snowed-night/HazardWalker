# Config

本目录存放 HazardWalker 的参数配置文件。

## 当前文件

- `topics.yaml`：内部话题名配置，统一 `/hw/*` 话题映射，包括 RGB 图像、相机内参、对齐深度图、感知结果和巡检覆盖摘要。
- `frames.yaml`：坐标系配置，统一 `map`、`odom`、`base_link`、`camera_link` 等 frame 名。
- `perception.yaml`：感知参数，主要是红球 HSV 阈值、横纵深度曲率球面门控、定位和跟踪参数。官方业务启动时通过 `perception_parameter_file` 显式加载；加载器会严格展开为 `hsv_detector_node` 的实际 ROS 参数，未知键直接报错。
- `perception_baseline.yaml`：多 SEED 回放基线参数快照，与当前正式参数一致。
- `perception_candidate.yaml`：控制变量候选参数，仅提高球面深度确认视角数。
- `perception_replay_campaign.example.json`：三 SEED、两参数方案的预注册回放模板。
- `nav.yaml`：导航参数，主要是航点、目标容差、卡死检测和 frontier 相关参数。
- `decision.yaml`：决策参数，主要是任务状态、返航约束和结果输出参数。

## 使用约定

- 参数名尽量和代码中的参数名保持一致。
- 修改话题名、frame 名或阈值时，优先先改这里，再改对应代码。
- 后续如果增加任务场景参数或地图参数，也优先放在本目录下。
