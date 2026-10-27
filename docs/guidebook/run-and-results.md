# 运行与结果核验

命令为`bash run.sh [seed] [new_result_name]`，默认种子202609102，输出位于`reports/nav/<new_result_name>/`。结果目录不进入Git。

| 输出 | 用途 |
| --- | --- |
| `run_manifest.json` | 实际版本、配置、任务状态和原始验收 |
| `summary.json` | 本轮摘要；启动早期失败时可能尚未产生 |
| `evaluation_result.json` | 官方目标匹配与客观分 |
| `detected_danger.json` | 最终危险源坐标 |
| `reproduction_acceptance.json` | 覆盖、返航、SLAM和识别联合核验 |
| `navigation/room_coverage.jsonl` | 房间阶段与完成记录 |
| `navigation/trajectory.jsonl` | 位姿、楼层和轨迹 |
| `navigation/failures.jsonl` | 倒地、停滞等失败事件 |
| `slam/physical_alignment.json` | 按会话刚体对齐后的轨迹误差 |
| `perception/`、`test_records/` | 候选、图像、动作和测试组CSV/JSON |
| `video/first_person.mp4` | 第一人称录像 |
| `video/slam_exploration.mp4` | 二维SLAM录像 |

完整通过同时要求：运行正常结束、FINISHED、三层各四房完成、原导航验收通过、末轨迹位于一层且距起点不超过1米、三层SLAM通过、危险源全部匹配且零漏报零虚警。

FINISHED文字本身不足以证明返航。缺失评估不计为零漏报。官方目标匹配阈值为1米；SLAM会话对齐残差与目标位置误差分别评价。

只核验已有结果：

```bash
python3 scripts/reproduction_check_result.py reports/nav/<result_name>
```

常见问题：

- 已有Gazebo：协调空闲时段，不停止他人的任务。
- 资源缺失或SHA不同：导入对应官方资源，不混用策略版本。
- GPU透传失败：默认CPU策略和软件渲染可使用`USE_GPU=0`。
- 缺少控制器：先执行`bash build.sh`。
- 目录不可写：由目录所有者处理历史root属主目录，避免全目录777。
- 工作树脏：原运行器要求正式源码已提交，在自己的分支提交改动后运行。
- 没有正式感知评估：先查failure_reason、房间和SLAM验收；原版在任务验收失败时可能跳过感知评分。

仿真秒不同于墙钟秒，908.119仿真秒的历史任务实际运行约77分钟。
