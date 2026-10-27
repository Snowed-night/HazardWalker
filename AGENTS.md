# HazardWalker 复用基线开发约定

先阅读README、docs/README.md和reproduction/baseline.json，确认当前分支是2026年9月10日旧版。

- 修改前检查git status，保护用户未提交内容。
- 算法及参数由reproduction/frozen-files.json追溯，有意修改后记录新版本，不更新旧成绩。
- 平台为ROS1 Noetic、Gazebo Classic和Unitree A1，业务为ROS2 Jazzy。实际位姿来源和结构先验见README。
- 感知提出观察请求，导航执行，命令仲裁输出速度。
- 新增或明显修改代码时使用简洁中文注释。
- 改运行方式时同步README和文档，运行适用检查和构建。
- 完整实验先确认独占，不停止他人的容器，不自动补跑。
- 不提交凭据、build/install/log、策略权重、rosbag和大型录像。
- 提交、推送来自用户明确授权；不强推，不擅自合并公共分支。
