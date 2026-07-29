"""负责人维护的官方 SimEnv 控制备用中继静态契约测试。"""

import ast
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
RELAY_SOURCE = REPO_ROOT / 'scripts' / 'official_simenv_cmd_vel_relay_node.py'


def test_backup_relay_is_parseable_and_uses_only_hw_to_ros1_cmd_vel():
    source = RELAY_SOURCE.read_text(encoding='utf-8')
    ast.parse(source)
    assert "'ros2_cmd_vel_topic', '/hw/cmd_vel'" in source
    assert "'ros1_cmd_vel_topic', '/cmd_vel'" in source
    assert "'geometry_msgs/Twist'" in source
    assert "'op': 'publish'" in source


def test_backup_relay_has_connection_status_and_fail_closed_stop_paths():
    source = RELAY_SOURCE.read_text(encoding='utf-8')
    assert '/hw/platform/cmd_vel_relay_status' in source
    assert 'def _watchdog(self)' in source
    assert 'def stop(self)' in source
    assert "'x': 0.0" in source
    assert "'缺少 websocket-client；请使用平台组提供的 ROS2 Python 环境。'" in source
    assert "if __name__ == '__main__':" in source
