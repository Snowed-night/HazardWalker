# Tests

测试目录。

建议包含：

- 单元测试
- launch 测试
- 仿真回归测试
- 指标统计脚本

## 当前离线测试

`tests/offline/` 中的测试不依赖 ROS、Gazebo 或官方平台，普通 Python 环境即可运行。

推荐命令：

```bash
python -m pytest tests/offline
```

如果本机没有安装 `pytest`，使用仓库自带的轻量测试运行器：

```bash
python scripts/run_offline_tests.py
```

这些测试用于先验证算法函数：

- 红球 HSV 检测
- 固定航点控制
- 任务结果 JSON 构建
- result JSON 格式评估
