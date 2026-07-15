# 决策组测试数据汇总

## 1. 汇总概述

| 项目 | 状态 | 说明 |
|---|---|---|
| 离线测试 | ✅ 已完成 | 8 个测试用例，全部通过 |
| 结果构建测试 | ✅ 已完成 | result JSON 结构校验通过 |
| 状态机设计 | ⚠️ 进行中 | 当前 3 状态，下一版 9 状态设计中 |
| 决策-导航-感知接口 | ⚠️ 进行中 | 与黄鸣波共同设计 |
| 动态识别触发重观察 | ⬜ 待设计 | 至少一个测试设计待完成 |

## 2. 离线测试记录

### 2.1 测试基本信息

| 字段 | 内容 |
|---|---|
| 日期 | 2026-07-03 |
| 成员 | 集成测试组 |
| 分组 | 决策组 |
| 分支 | dev |
| 命令 | python scripts/run_offline_tests.py |
| 测试环境 | 本机离线 |
| 是否通过 | 通过 |
| 失败信息 | - |
| 耗时 | 约 5 秒 |
| 备注 | 决策组测试数：8 |

### 2.2 测试用例

| 测试文件 | 验证内容 |
|---|---|
| test_result_builder.py | 任务结果 JSON 构建 |
| test_evaluate_result.py | 结果 JSON 结构校验 |
| test_track_hazards.py | 多帧确认、空间去重、丢失拒绝 |

## 3. 当前状态机能力

### 3.1 当前实现的状态

| 状态 | 说明 |
|---|---|
| NAVIGATING | 导航中，执行航点任务 |
| RETURNING | 返航中，返回起点 |
| FINISHED | 任务完成 |

### 3.2 当前状态转移逻辑

```text
NAVIGATING -> RETURNING (到达最后一个航点)
NAVIGATING -> FINISHED (completed=True 或 waypoints 为空)
RETURNING -> FINISHED (到达起点)
```

## 4. 下一版状态机设计

### 4.1 设计的状态

| 状态 | 说明 |
|---|---|
| EXPLORING | 探索中，寻找新区域 |
| DETECTING | 检测中，识别红球 |
| CONFIRMING | 确认目标中，验证检测结果 |
| REOBSERVING | 重观察中，调整视角复查 |
| REPLANNING | 重新规划中，处理异常情况 |
| RETURNING | 返航中，返回起点 |
| FINISHED | 任务完成 |
| FAILED | 任务失败 |
| TIMEOUT | 任务超时 |

### 4.2 设计的状态转移

```text
EXPLORING -> DETECTING (发现疑似目标)
EXPLORING -> RETURNING (时间不足)
EXPLORING -> FINISHED (探索完成)
EXPLORING -> REPLANNING (遇到障碍物)

DETECTING -> CONFIRMING (发现候选目标)
DETECTING -> EXPLORING (无目标，继续探索)

CONFIRMING -> REOBSERVING (需要换视角复查)
CONFIRMING -> EXPLORING (确认非红球，继续探索)
CONFIRMING -> RETURNING (确认红球，写入结果后返航)

REOBSERVING -> CONFIRMING (完成视角调整)
REOBSERVING -> EXPLORING (重观察后仍无法确认)

REPLANNING -> EXPLORING (重新规划完成)
REPLANNING -> FAILED (无法重新规划)

RETURNING -> FINISHED (成功到达起点)
RETURNING -> FAILED (返航失败)

任意状态 -> TIMEOUT (时间耗尽)
任意状态 -> FAILED (系统异常)
```

## 5. 决策-导航-感知接口设计

### 5.1 接口流程

```text
感知输出候选红球 -> 决策判断是否复查
决策给导航发送观察目标/视角调整目标
导航执行视角调整 -> 感知再次确认
确认后写入 result JSON
```

### 5.2 接口定义

| 接口 | 方向 | 说明 |
|---|---|---|
| /hw/perception/hazard_detections | 感知 -> 决策 | 危险源检测结果 |
| /hw/decision/reobservation_request | 决策 -> 导航 | 重观察请求 |
| /hw/decision/observation_goal | 决策 -> 导航 | 观察目标点 |
| /hw/mission/state | 决策 -> 全组 | 任务状态 |
| /hw/mission/result | 决策 -> 全组 | 任务结果 |

## 6. 决策组指标统计

| 指标名称 | 当前值 | 状态 |
|---|---|---|
| 状态转移次数 | 3 | 当前实现 3 个状态 |
| 任务完成率 | 100% | 离线测试全部通过 |
| 决策延迟 | 待测 | 待实际运行测量 |
| 正确决策率 | 100% | 离线测试全部通过 |
| 异常恢复率 | 0% | 尚未实现异常恢复 |
| 结果汇总准确率 | 100% | 离线测试全部通过 |
| 任务执行时间 | 待测 | 待实际运行测量 |
| 重观察触发次数 | 0 | 尚未实现 |
| NBV 决策次数 | 0 | 尚未实现 |

## 7. 待完成任务

| 任务 | 负责人 | 截止时间 |
|---|---|---|
| 完成状态机草案 | 黄鸣波 + 王杰铭 | 2026-07-18 |
| 完成感知-导航-决策接口说明 | 黄鸣波 + 王杰铭 | 2026-07-18 |
| 设计至少一个动态识别触发重观察的测试 | 黄鸣波 + 王杰铭 | 2026-07-18 |
| 实现下一版状态机（9 状态） | 决策组 | 2026-07-18 |

## 8. 问题与改进方向

| 问题 | 改进方向 |
|---|---|
| 当前状态机只有 3 个状态，功能有限 | 设计并实现 9 状态版本 |
| 决策-导航-感知接口尚未完全定义 | 与黄鸣波共同完成接口设计 |
| 异常恢复机制尚未实现 | 设计异常恢复策略 |
| NBV（Next Best View）尚未实现 | 设计最佳视角选择逻辑 |

## 9. 参考文件

- 决策组代码：`ros2_ws/src/hazardwalker_decision/`
- 任务计划状态机设计：`docs/groups/platform/test_record_template.md`（第 2.5 节）