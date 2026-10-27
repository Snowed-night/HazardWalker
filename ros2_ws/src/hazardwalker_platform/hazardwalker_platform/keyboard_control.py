"""负责人维护的键盘速度映射。

文件作用：
- 统一 W/S/A/D/K 的比赛控制语义。
- 将按键转换为可测试的 ``(linear_x, angular_z)``，供 ROS2 节点复用。
- 不依赖 ROS，便于离线测试映射和急停行为。
"""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class KeyboardCommand:
    """单次键盘命令的二维速度与显示名称。"""

    linear_x: float
    angular_z: float
    label: str
    is_stop: bool = False


def command_for_key(
    key: str,
    *,
    linear_speed: float,
    angular_speed: float,
) -> Optional[KeyboardCommand]:
    """把单字符按键转换为速度；未知按键不产生控制输出。"""

    normalized = key.lower()
    commands = {
        'w': KeyboardCommand(linear_speed, 0.0, '前进'),
        's': KeyboardCommand(-linear_speed, 0.0, '后退'),
        'a': KeyboardCommand(0.0, angular_speed, '左转'),
        'd': KeyboardCommand(0.0, -angular_speed, '右转'),
        'k': KeyboardCommand(0.0, 0.0, '立即停止', is_stop=True),
    }
    return commands.get(normalized)
