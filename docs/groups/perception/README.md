# 感知组文档

感知组负责红色球体危险源的候选检测、深度定位、跨帧去重、侧向复查请求和确认前的虚警抑制。

## 当前边界

- 比赛输出类别只有**红色球体**。圆柱、圆锥、方块和其他红色物体只用于干扰物测试，绝不作为危险源输出。
- 单帧红色区域只能形成候选；候选必须经深度几何、跨帧关联、独立侧向复查和合法三维定位后才可能确认。
- 感知节点只发布候选、轨迹和复查请求，不直接占用 `/hw/cmd_vel`。
- 离线回归通过不等同于官方随机场景闭环完成；正式结论只能来自带 SEED、Git 版本、原始图、测试记录和运行说明的证据目录。

## 代码与配置

| 模块 | 位置 | 职责 |
|---|---|---|
| 2D 检测 | `ros2_ws/src/hazardwalker_perception/hazardwalker_perception/red_ball_detector.py` | HSV/OpenCV 候选检测与粘连拆分 |
| ROS 感知节点 | `ros2_ws/src/hazardwalker_perception/hazardwalker_perception/hsv_detector_node.py` | RGB-D 输入、候选发布与深度接入 |
| 三维定位 | `ros2_ws/src/hazardwalker_perception/hazardwalker_perception/localize_hazard.py` | 内参反投影与坐标变换 |
| 轨迹确认 | `ros2_ws/src/hazardwalker_perception/hazardwalker_perception/track_hazards.py` | 多帧关联、去重、拒绝与确认门控 |
| 回放评估 | `ros2_ws/src/hazardwalker_perception/hazardwalker_perception/replay_evaluation.py` | 人工标注帧的一对一匹配、确认输出和定位指标 |
| 参数 | `config/perception.yaml` | 可调阈值与运行配置 |

## 证据与阶段材料

- [感知成果总入口](../../../reports/perception/README.md)：所有可展示实验、指标、CSV/JSON 测试记录与失败边界。
- [官方人工巡检感知录制与回放](官方人工巡检感知录制与回放.md)：固定 SEED 录包、人工复查、隔离回放和标注评估流程。
- [开发文档与算法总结](开发文档与算法总结.md)：2026-07-15 阶段总结，非当前验收结论。
- [全面测试案例设计](全面测试案例及验证鲁棒性.md)：专项测试设计与开发路径；运行命令需要先按当前脚本和平台手册复核。
- [五大模块材料汇总](五大模块材料.md)：阶段汇报材料。

新增阶段文档必须说明日期、Git 版本、输入输出、验证命令、结果、失败边界，并将原始实验成果放入 `reports/perception/`，不在本目录复制图片和测试记录。
