# 测试组材料目录

本目录存放测试组负责的比赛材料，包括测试指标定义、实验结果汇总、失败原因分析等。

## 目录结构

```
testing/
├── README.md                              # 本说明文件
├── 20260715_testing_metrics_definition.md # 测试指标定义文档
├── 20260715_failure_summary.md            # 失败原因汇总
├── 20260715_technical_report_testing_section.md  # 技术报告测试章节大纲
└── test_summary/                          # 实验结果汇总子目录
    ├── 20260715_nav_test_summary.md       # 导航组实验数据汇总
    ├── 20260715_perception_test_summary.md # 感知组实验数据汇总
    ├── 20260715_platform_test_summary.md  # 平台组实验数据汇总
    └── 20260715_decision_test_summary.md  # 决策组实验数据汇总
```

## 文件说明

| 文件 | 说明 |
|---|---|
| `testing_metrics_definition.md` | 五个组的完整指标体系，含定义、计算公式、验收标准 |
| `failure_summary.md` | 各模块测试失败案例、原因分析和解决方案汇总 |
| `technical_report_testing_section.md` | 比赛技术报告测试章节完整大纲 |
| `test_summary/nav_test_summary.md` | 导航组实验数据汇总，含手动航点测试记录 |
| `test_summary/perception_test_summary.md` | 感知组实验数据汇总，含五类复杂环境实验 |
| `test_summary/platform_test_summary.md` | 平台组实验数据汇总，含接口测试记录 |
| `test_summary/decision_test_summary.md` | 决策组实验数据汇总，含状态机设计 |

## 更新记录

| 日期 | 更新内容 |
|---|---|
| 2026-07-15 | 创建目录结构，完成指标定义和实验结果汇总 |