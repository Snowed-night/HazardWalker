# 官方随机场景感知证据索引

本目录只保存官方 `auto.sh` 在未修改楼宇布局、目标位置和光照条件下生成的随机
场景结果。人工摆球、受控遮挡和专门设计的干扰物实验必须放在
`reports/perception/simulation/3d_native/`，不得混入本目录。

## 历史阶段

| SEED | 主要用途 | 当前可证明结论 |
|---|---|---|
| `2026071801` | 平台、SLAM、控制和早期探索烟测 | 部分链路及失败诊断；不能证明正式感知识别成绩 |
| `2026071802` | 合法 SLAM、Frontier、RGB-D 定位、主动复查和返航迭代 | 候选定位、复查动作和返航分别出现过；没有一轮完成非空正式识别闭环 |

历史运行必须保留失败原因，不能因为新算法通过而覆盖。节点存在、地图落盘、
候选框或空结果返航均不能替代红球严格确认。

## B5 正式鲁棒性门禁

正式活动必须在运行前预注册至少 3 个固定 SEED，并保证所有场景使用：

- 同一个干净 Git 提交；
- 同一个感知参数快照及 SHA-256；
- 官方未修改随机场景；
- 合法 RGB-D、内参、激光、IMU、SLAM 位姿和控制接口；
- 600 秒以内完整任务与本轮新生成的结果文件。

每一轮都必须通过
`validate_official_random_perception_evidence.py --require-active-reobservation`，
并在所有运行进程停止、证据封存后调用官方评测脚本生成
`evaluation_result.json`。真值只允许用于这一赛后独立评测，不能回灌检测、
跟踪、复查或导航。

活动汇总使用：

```bash
python3 scripts/summarize_official_random_perception_campaign.py \
  --campaign-manifest reports/perception/official_random/campaign_manifest.json \
  --output-dir reports/perception/official_random/campaign_summary
```

汇总器会拒绝漏跑预注册 SEED、未提交代码、参数变化、结构校验失败和只挑成功
结果。当前两个历史 SEED 不满足同一版本的多场景正式门禁，因此不得宣称 B5
已经通过。

## 当前组间依赖

感知侧已经能够输出候选、RGB-D 三维位置、重观察方向、多视角确认门禁和去重。
正式随机场景的发现率仍依赖导航组提供稳定的合法 SLAM、房间/楼层覆盖、感知
复查动作执行和返航 `FINISHED`。在这些依赖未形成同一轮实测证据前，不继续
制造无法计分的空结果随机运行。
