# official_simenv_20260710_rgbd_partial_visibility

官方 SimEnv ROS2 Harmonic 原生 3D 前景遮挡比例扫描。相机图像、深度、内参、TF 和检测输出均来自运行中的 `/hw/*` 话题；临时球和遮挡板仅用于受控评测，不读取比赛裁判真值。

## 覆盖与结果

- 1 个无遮挡基准，左右遮挡各 10 档（5%、10%、15%、25%、35%、45%、55%、65%、75%、85%），共 21 例。
- 原生帧中实际红像素比例与设计可见比例一致，例如左侧 5%/10%/15% 的实测为 `5.42%`/`10.13%`/`15.37%`，右侧为 `4.71%`/`9.78%`/`15.04%`。
- 实测可见比例低于 45% 的 10 例均输出 `requires_reobservation=true`、`confirmation_eligible=false`，不再静默漏检或错误确认。
- 45% 及以上的 10 例均进入严格球体检测；无遮挡基准同样严格检出。
- 21/21 通过。总览见 `images/partial_visibility_collage.png`。

## 内容

- `images/`：21 张真实 Gazebo 相机标注图与总览。
- `snapshots/`：每例实际节点 JSON，含候选、形状、定位准备状态与视角标签。
- `cases.csv/json`、`summary.json`：逐例比例、候选类型和结果。
- 测试组镜像：`reports/perception/test_records/official_simenv_20260710_rgbd_partial_visibility/`。

该实验说明“小部分球体”会被保留为机器狗下一步转向/靠近的任务；它不是单帧危险源确认。
