# 三层终版 v1（2026-09-08）

本版本由负责人姜晨固定，用于保留首次同时完成三层十二房间、自动返航和四个
红球正确定位的完整正式证据。该版本不再随后续 SLAM 修复改写。

- Git 标签：`three-floor-final-v1-20260908`
- Git 提交：`174a189d7441978320de2e2b37a739cddccae3e2`
- 固定随机种子：`20260728`
- 远程原始结果：`reports/nav/three_floor_stable_174a189_20260908`
- 远程固定入口：`reports/nav/three_floor_final_v1`
- 完整性校验：结果目录内 `FINAL_V1_SHA256SUMS`
- 复现入口：`scripts/run_stable_three_floor_experiment.sh <新输出目录>`

结果：三层 `12/12` 房间、返航 `FINISHED`、四个危险源全部正确定位、零漏报、
零虚警，官方客观分 `33/37`。已知限制是二层 Cartographer 假回环，导致二层
SLAM 物理对齐 P95 为 `9.625 m`；因此本标签是导航与感知终版 v1 证据，不应
误写成 SLAM 全项通过版本。

复现时必须检出上述标签，并使用全新输出目录；固定入口脚本会独占容器、加载
受版本控制配置，并在结束后统一回收控制仲裁器、ROS 适配器和仿真容器。
