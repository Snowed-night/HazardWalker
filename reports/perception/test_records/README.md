# 感知测试记录说明

`official_simenv_20260710_*` 五套记录是历史内部回归，用于复盘算法优化路径，不是官方随机场景成绩。目录名已与 `simulation/3d_native/` 的统一实验类别对齐。
对应 JSON 已显式标记 `official_score_eligible=false`；CSV 保留当时原始字段和 `pass` 判定，仅表示旧内部协议结果。

后续复跑应在同名仿真实验目录的 `reruns/YYYYMMDD_<seed>/` 中保存新素材，并在测试记录目录新建同名日期子目录，
避免覆盖历史基线或把不同证据契约的数据混为一轮结果。
