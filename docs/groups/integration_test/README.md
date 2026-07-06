# 集成测试组文档

## 负责范围

- 主力机环境验收与账号可用性检查。
- `dev` 分支构建和运行质量把关。
- 离线测试、系统测试和结果评估脚本维护。
- 统一测试记录表、失败案例归档和指标统计。
- 推动各组"每次运行必记录"的测试习惯。

## 成员

- 王杰铭
- 宋艺多
- 沈一

---

## 当前能跑什么

### 1. 离线测试（零依赖，任意 Python 环境可跑）

覆盖感知、导航、决策、平台四大方向的纯函数验证：

| 测试文件 | 验证内容 |
|---|---|
| `test_red_ball_detector.py` | 红球 HSV 检测 + 轮廓筛选 + watershed 分离 |
| `test_detection_metrics.py` | IoU、precision、recall、AP50 等指标计算 |
| `test_localize_hazard.py` | 2D bbox + 深度 + TF 的三维定位 |
| `test_track_hazards.py` | 多帧确认、空间去重、丢失拒绝 |
| `test_waypoint_controller.py` | 固定航点控制逻辑 |
| `test_result_builder.py` | 任务结果 JSON 构建 |
| `test_evaluate_result.py` | 结果 JSON 结构校验 |
| `test_platform_phase1.py` | 平台接口第一版检查 |

### 2. 结果评估脚本

- `evaluate_result.py`：检查最小 demo 输出的 result JSON 结构是否合法，统计 confirmed hazards 数量。

### 3. 测试记录表模板

- 位置：`docs/groups/platform/test_record_template.md`
- 包含：基本测试记录 + 五个组的专项指标（感知/导航/决策/平台/集成测试）

---

## 怎么运行

### 离线测试

```bash
# 在仓库根目录执行
python scripts/run_offline_tests.py
```

输出示例：

```text
PASS test_detection_metrics.py::test_iou_basic
PASS test_red_ball_detector.py::test_detect_single_ball
...
Offline tests: 12 passed, 0 failed
```

> 如果装了 pytest，也可以用 `python -m pytest tests/offline`。

### 结果评估

```bash
python scripts/evaluate_result.py reports/run_results/<timestamp>_result.json
```

输出字段：`mission_id`、`status`、`duration_sec`、`return_success`、`hazard_count`、`confirmed_hazard_count`。

### Git 推送前检查（全员必做）

```bash
git status
git pull --rebase origin dev
python scripts/run_offline_tests.py
```

离线测试全部通过后再推送。禁止直接向 `main` 推送。

---

## 输出在哪里

### 测试记录目录结构

建议各组测试记录统一按以下目录存放：

```text
reports/
├─ perception/
│   └─ test_records/
│       └─ <timestamp>/
│           ├─ testing_record_perception.json
│           └─ testing_record_perception.csv
├─ nav/
│   └─ test_records/
├─ decision/
│   └─ test_records/
└─ platform/
    └─ test_records/
```

### 记录表内容

每条测试记录至少包含：

- **基本信息**：日期、成员、分组、分支、命令、测试环境、是否通过、失败信息、耗时、备注
- **专项指标**：按各组特点填写（感知9项、导航10项、决策9项、平台8项、集成测试12项）

### 已有记录

- 感知组：`reports/perception/test_records/20260620_230333/`、`20260621_150028/`
- 集成测试组：`reports/integration_test/test_records/20260703_194325/`
---

## 填表规则（各组请遵守）

### 什么时候填

**每次运行测试后都要填**，包括：
- 离线测试跑通
- Gazebo 仿真运行
- 截图脚本执行
- 最小 demo 运行
- 失败/报错复现

### 怎么填

1. 基本信息表必填，专项指标表按组填写
2. 失败时必须填写**失败信息**（错误日志或关键堆栈）
3. 备注里写清楚环境细节（哪个账号、哪台机器、是否 headless 等）
4. 文件命名：`testing_record_<组名>.json` + `.csv`
5. 时间戳目录统一用 `YYYYMMDD_HHMMSS` 格式

### 模板位置

测试记录表模板：[test_record_template.md](../platform/test_record_template.md)

---

## 可验证产物清单

> 本阶段（6月14日-7月4日）集成测试组可展示成果

| 产物 | 位置 | 状态 |
|---|---|---|
| 集成测试组文档 | `docs/groups/integration_test/README.md` | ✅ 已完成 |
| 测试记录表模板（含5组专项指标） | `docs/groups/platform/test_record_template.md` | ✅ 已完成 |
| 填表规则文档 | `docs/groups/integration_test/test_record_guide.md` | ✅ 已完成 |
| 阶段汇报材料 | `docs/groups/integration_test/phase1_report_2026-07-04.md` | ✅ 已完成 |
| 离线测试脚本 | `scripts/run_offline_tests.py` | ✅ 64项全通过 |
| 结果评估脚本 | `scripts/evaluate_result.py` | ✅ 可用 |
| 集成测试组测试记录（本机） | `reports/integration_test/test_records/20260703_194325/` | ✅ 已生成 |

---

## 当前问题

1. **测试记录覆盖率低**：只有感知组有记录，导航/决策/平台组还没有真实测试记录。
2. **记录习惯未建立**：各组跑了测试不一定填表，需要推动养成习惯。
3. **集成测试缺失**：目前只有离线纯函数测试，缺少 ROS 级别的集成测试。
4. **主力机账号可用性**：各组账号是否都能正常 SSH、build、跑 Gazebo，还没有完整的统一检查表。
5. **远程桌面 Gazebo GUI 问题**：RDP 内 GUI 不稳定，已推出 headless + 本机 GUI 分离方案，需持续跟踪各组适配情况。

---

## 下阶段任务

### 短期

- [ ] 收集各组至少 1 条真实测试记录
- [ ] 完成各组账号可用性检查表
- [ ] 整理 7 月 4 日会议集成测试组汇报材料

### 中期（下阶段）

- [ ] 建立 ROS 级集成测试（最小 demo 链路自动验证）
- [ ] 推动各组每次运行必记录，纳入 PR 检查清单
- [ ] 搭建统一的测试结果看板（指标趋势图）
- [ ] 补充失败案例归档机制

### 长期

- [ ] CI/CD 流水线（自动跑离线测试 + 集成测试）
- [ ] 性能基准测试和回归测试
- [ ] 实机测试流程和记录表
