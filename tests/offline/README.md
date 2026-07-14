# 离线回归测试

负责人：姜晨。此目录的测试由 `python scripts/run_offline_tests.py` 直接发现并执行，不依赖
pytest、ROS2 或 Gazebo。

- `test_official_simenv_mapping.py`：官方 ROS1/ROS2 `/hw/*` 映射、启动安全门和业务入口契约。
- `test_rosbridge_protocol.py`：rosbridge 大型 RGB-D 消息的分片重组、乱序、损坏和缓存上界。
- 其余 `test_*.py`：导航、感知、定位、决策等可纯 Python 验证的回归用例。

这些测试只能证明代码和协议约束；官方 ROS1、ROS2 `/hw/*` 和复杂场景闭环仍必须以运行时里程计、
传感器消息和截图/视频证据验收。
