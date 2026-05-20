# 最小闭环脚手架更新说明

本文档用于说明当前最小闭环代码已经具备的开发基础、各模块可以从哪里开始改，以及提交代码时应遵守的模板要求。

## 1. 当前状态

当前 `minimal_demo` 已经提供一条最小链路：

```text
fake_platform_node
    -> 发布 /hw/camera/image_raw、/hw/odom、/tf
    -> 接收 /hw/cmd_vel

hsv_detector_node
    -> 订阅 /hw/camera/image_raw
    -> 发布 /hw/perception/hazard_detections

waypoint_patrol_node
    -> 订阅 /hw/odom
    -> 发布 /hw/cmd_vel 和 /hw/nav/state

mission_state_machine_node
    -> 订阅 /hw/nav/state 和 /hw/perception/hazard_detections
    -> 发布 /hw/mission/state
    -> 写出 reports/run_results/<timestamp>_result.json
```

这些节点不是最终算法，只是第一阶段开发模板。它们的作用是固定接口、说明模块边界，并让后续代码可以逐步替换占位逻辑。

## 2. 可以开始开发的模块

### 平台组

可以从以下文件开始：

```text
ros2_ws/src/hazardwalker_platform/hazardwalker_platform/fake_platform_node.py
```

任务：

- 看懂 fake 节点如何发布 `/hw/*` 内部接口。
- 后续新增 `gazebo_adapter_node.py` 或 `official_adapter_node.py`。
- 保持输出 topic 不变。

不要直接改算法模块去适配平台话题。平台差异应由 `hazardwalker_platform` 吸收。

### 感知组

可以从以下文件开始：

```text
ros2_ws/src/hazardwalker_perception/hazardwalker_perception/hsv_detector_node.py
```

任务：

- 将 HSV 检测逻辑拆成独立函数。
- 增加 OpenCV 版本的图像处理。
- 输出 bbox、confidence 和 debug image。
- 后续接入点云和 TF，替换临时 `position` 占位值。

### 导航组

可以从以下文件开始：

```text
ros2_ws/src/hazardwalker_nav/hazardwalker_nav/waypoint_patrol_node.py
```

任务：

- 先完善固定航点巡检和返航。
- 后续新增 Nav2 goal client。
- 再开发 Frontier 探索。

当前节点只用于最小闭环，不是最终导航方案。

### 决策组

可以从以下文件开始：

```text
ros2_ws/src/hazardwalker_decision/hazardwalker_decision/mission_state_machine_node.py
```

任务：

- 将状态转移逻辑拆成独立 `state_machine.py`。
- 增加 timeout、FAILED、RETURNING 等状态判断。
- 后续加入重观察和返航约束。

### 测试组

可以从以下位置开始：

```text
reports/run_results/
scripts/
tests/
```

任务：

- 编写 `scripts/evaluate_result.py`。
- 检查 result JSON 是否包含 `status`、`hazards`、`metrics`。
- 建立最小闭环测试记录表。

## 3. 模块开发模板

新增算法文件时，建议按以下结构写：

```python
"""模块说明。

说明这个文件负责什么、输入是什么、输出是什么。
"""


def algorithm_function(input_data, params):
    """算法函数说明。

    Args:
        input_data: 输入数据说明。
        params: 参数说明。

    Returns:
        输出数据说明。
    """
    # 这里写核心算法，尽量不直接依赖 ROS。
    pass
```

ROS 节点只负责：

```text
订阅 topic
读取参数
调用算法函数
发布结果
```

核心算法尽量写成普通 Python 函数，这样可以不用主力机、不用 Gazebo，也能离线测试。

## 4. 提交前检查

每个模块提交前至少检查：

```bash
python -m py_compile <你修改的 .py 文件>
git status
git diff
```

如果有 Ubuntu + ROS 2 环境，再运行：

```bash
./scripts/build.sh
```

如果本地没有 ROS 2，可以先提交离线算法函数和测试脚本，但要在 PR 说明中写明：

```text
未运行 colcon build，原因：本机无 ROS 2 Jazzy 环境。
```

## 5. 接口约束

所有模块必须遵守：

```text
平台组负责外部平台 -> /hw/*
算法组只读写 /hw/*
接口变更必须同步更新 docs/interface_spec.md 和 config/*.yaml
```

不要在算法模块中写死官方平台、Gazebo 或某个硬件专用 topic。

## 6. 当前已确认可以开始的开发方向

当前代码已经足够作为以下开发的起点：

- 固定航点巡检逻辑。
- HSV 红球检测逻辑。
- 任务状态机逻辑。
- result JSON 格式检查。
- `/hw/*` 平台适配接口理解。

## 7. 离线算法测试

为了让成员在没有主力机、没有 ROS 2、没有 Gazebo 的情况下也能开发，核心算法应优先写成普通 Python 函数。

当前已提供离线测试入口：

```bash
python -m pytest tests/offline
```

如果没有安装 pytest，也可以直接运行：

```bash
python scripts/run_offline_tests.py
```

对应测试内容：

```text
tests/offline/test_red_ball_detector.py
tests/offline/test_waypoint_controller.py
tests/offline/test_result_builder.py
tests/offline/test_evaluate_result.py
```

成员开发算法时，应优先让这些离线测试通过，再把函数接入 ROS 节点。

测试组可以用以下命令检查一次任务输出：

```bash
python scripts/evaluate_result.py reports/run_results/<timestamp>_result.json
```

暂时不建议立刻做：

- YOLO。
- 完整 NBV。
- 实机接入。
- 官方平台专用适配。
- 复杂多层楼场景。

先把最小闭环跑通，再逐步替换占位逻辑。
