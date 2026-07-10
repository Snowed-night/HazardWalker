# official_simenv_20260710_rgbd_full_suite

官方 SimEnv ROS2 Harmonic headless 平台的 RGB-D 感知完整回归测试，日期为 `2026-07-10`。图像由运行中的 `/hw/camera/image_raw` 和当前感知节点实际输出生成；所有临时模型通过公开 Gazebo EntityFactory 服务生成，并按公开 PoseInfo 实体 ID 清理。

> 本目录现定位为 RGB-D 链路烟测，不再单独作为“局部球、圆柱、多球已充分解决”的结论依据。用户指出的遮挡比例、圆柱多视角和重复计数问题已由后续充分矩阵复测，见 `official_simenv_20260710_rgbd_partial_visibility`、`official_simenv_20260710_rgbd_shape_multiview`、`official_simenv_20260710_rgbd_multi_count` 和 `perception_progress_report_2026-07-10_evidence_matrix.md`。

本套件不读取 `danger_truth.json`、场景布局或裁判真值。受控红球/干扰物的位置仅用于定义本次回归用例的预期行为，不是比赛危险源。

## 内容

- `images/`：8 张原生 RGB 检测标注图，以及 `rgbd_full_suite_collage.png` 总览。
- `snapshots/`：每个场景的精简实际节点输出，含 2D 候选、深度形状证据、确认资格和轨迹状态。
- `cases.csv/json`：逐例测试结果。
- `summary.json`：汇总指标。
- `reports/perception/test_records/20260710_official_simenv_rgbd_full_suite/`：测试组镜像 CSV/JSON。

原始 RGB 和节点日志保留在受控平台的临时归档，不直接提交 Git；仓库保留可核验的标注图和结构化快照。

## 最终结果

| 场景 | 实际输出 | 验收 |
|---|---|---|
| 近距离完整球 | 1 个可确认候选 | 通过 |
| 远距离完整球 | 1 个可确认候选 | 通过 |
| FOV 边缘约 40% 可见球 | 1 个待复查候选，`confirmation_eligible=0` | 通过 |
| 红色立方体 | 0 个候选 | 通过 |
| 红色圆柱端面 | 1 个 2D 候选，但深度形状为 `flat`、不可确认 | 通过 |
| 红色平板 | 1 个待复查候选，深度形状为 `flat`、不可确认 | 通过 |
| 3 个分离受控球 | 至少 3 个受控球候选；画面另有 1 个原生环境可见红色候选 | 通过（计数下界） |
| 粘连双球 | 2 个可确认候选 | 通过 |

汇总：`8/8` 用例通过；8 例 RGB、深度、内参与 TF 均已就绪。

## 本轮解决的问题

- 低可见球：严格筛选失败时不再静默丢失，输出 `is_partial=true`、`requires_reobservation=true` 的候选，供主动视角策略转向或靠近复查；该候选不会进入三维 confirmed 轨迹。
- 圆柱误判：对近圆形红色候选计算 bbox 中心与外环的深度曲率。圆柱端面实测为 `flat`，因此仍可作为 2D 可疑物保留证据，但 `confirmation_eligible=0`，不会被上报为已确认危险源。
- 红色立方体/平板：严格形状筛选或深度平面抑制阻止其进入 confirmed 轨迹。
- 粘连球：修复 Hough 分裂路径，并在真实 Gazebo 渲染中得到 2 个独立候选。
- 测试污染：采集器按 PoseInfo 中的实体 ID 清理临时模型，避免前一案例的复合 SDF 残留到下一例。

## 仍需平台配合的边界

本轮每例均为单一静态相机视角，感知节点设置 `confirm_distinct_views=2`，所以所有轨迹保持 `tentative`，没有把单视角结果伪装成 confirmed。当前 ROS2 仍是轮式占位控制，`/cmd_vel` 的真实世界位姿存在迟滞/滑移，不能作为可信的 A1 多视角确认验证。平台完成真实四足控制迁移后，应复用本目录脚本，执行“候选 -> 建议转向/横移 -> 第二视角 -> confirmed”的动态回归。
