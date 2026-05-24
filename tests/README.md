# Tests

本目录存放单元测试、离线测试和后续集成测试。

## 当前内容

### `tests/offline/`

当前离线测试不依赖 ROS、Gazebo 或官方平台，普通 Python 环境即可运行。

包含文件：

- `test_red_ball_detector.py`：验证红球 HSV 检测函数。
- `test_waypoint_controller.py`：验证固定航点控制函数。
- `test_result_builder.py`：验证任务结果构建函数。
- `test_evaluate_result.py`：验证结果 JSON 检查逻辑。

## 运行方式

优先使用仓库自带的轻量测试运行器：

```bash
python scripts/run_offline_tests.py
```

如果本机已安装 `pytest`，也可以直接运行：

```bash
python -m pytest tests/offline
```

## 维护约定

- 新增测试文件要按功能命名，优先使用 `test_*.py`。
- 如果测试依赖 ROS 或仿真，需要在 README 里说明运行前提。
- 离线测试优先验证纯函数，集成测试再放到后续目录中。
