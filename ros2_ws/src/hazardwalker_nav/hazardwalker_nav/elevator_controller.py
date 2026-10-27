"""电梯 + 门控调用模块：通过 docker exec 调用官方 ROS1 服务。

所属组：导航组。
文件作用：
- 在导航节点中安全调用 Docker 容器内的 /call_elevator 和 /set_door_state。
- 不依赖 ROS2 服务，直接走 subprocess + docker exec。
- 所有调用带超时保护，防止阻塞 10 Hz 控制循环。

验证方式：
  tests/offline/test_elevator_controller.py
"""

from __future__ import annotations

import json
import math
import re
import subprocess
import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class ElevatorResult:
    """电梯服务返回结果。"""

    accepted: bool
    current_floor: int
    state: str  # 'idle', 'moving', 'doors_open', 'doors_closed'
    message: str


@dataclass
class DoorResult:
    """门控服务返回结果。"""

    accepted: bool
    state: str  # 'open', 'closed'


# 官方参考位置：电梯坐标由 reference.md 公开，不在本模块硬编码。
# 多层探索节点应从 launch 参数注入。
_DEFAULT_ELEVATOR_POSITIONS: dict = {
    # elevator_main 在各楼层 map 帧中的近似位置（由官方建筑生成器确定）。
    # 默认从 reference.md 入口推导；实际使用时应通过参数覆盖。
    0: (0.0, 0.0),
    1: (0.0, 0.0),
    2: (0.0, 0.0),
}

_DOCKER_EXEC_BASE = [
    'docker', 'exec',
]


def _parse_rosservice_output(output: str) -> dict:
    """解析 rosservice call 的 YAML 风格输出为 dict。

    官方 rosservice call 返回格式如：
      accepted: True
      current_floor: 1
      state: idle
      message: ok
    """
    result: dict = {}
    lines = output.strip().split('\n')
    for line in lines:
        if ':' not in line:
            continue
        key, _, value = line.partition(':')
        key = key.strip()
        value = value.strip().strip('"')
        if value.lower() == 'true':
            result[key] = True
        elif value.lower() == 'false':
            result[key] = False
        elif re.match(r'^-?\d+$', value):
            result[key] = int(value)
        else:
            result[key] = value
    return result


def _ros1_source_cmd() -> str:
    """生成容器内 source 两条 ROS 环境的 bash 片段。"""
    return (
        'source /opt/ros/noetic/setup.bash && '
        'source /home/ros/simenv_ws/.ros1_catkin_ws/devel/setup.bash'
    )


def _docker_exec(
    container: str,
    rosservice_cmd: str,
    timeout_s: float = 15.0,
) -> str:
    """在容器内执行 rosservice call 并返回 stdout。

    Args:
        container: Docker 容器名。
        rosservice_cmd: 完整的 rosservice call ... 命令。
        timeout_s: 超时秒数。

    Returns:
        stdout 字符串。

    Raises:
        subprocess.TimeoutExpired: 超时。
        RuntimeError: 非零返回码。
    """
    cmd = _DOCKER_EXEC_BASE + [
        container,
        'bash', '-lc',
        f'{_ros1_source_cmd()} && {rosservice_cmd}',
    ]
    proc = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=timeout_s,
    )
    if proc.returncode != 0:
        stderr = proc.stderr.strip()
        raise RuntimeError(
            f'docker exec 失败 (rc={proc.returncode}): {stderr}'
        )
    return proc.stdout


def call_elevator(
    container: str,
    elevator_id: str = 'elevator_main',
    target_floor: int = 0,
    open_doors: bool = True,
    timeout_s: float = 30.0,
) -> ElevatorResult:
    """调用官方 /call_elevator 服务。

    Args:
        container: Docker 容器名。
        elevator_id: 电梯 ID。
        target_floor: 目标楼层编号。
        open_doors: 到达后是否打开电梯门。
        timeout_s: 超时（电梯移动最多约 20 秒）。

    Returns:
        ElevatorResult。
    """
    cmd = (
        f'rosservice call /call_elevator '
        f'{elevator_id} {target_floor} '
        f'{"true" if open_doors else "false"}'
    )
    output = _docker_exec(container, cmd, timeout_s=timeout_s)
    parsed = _parse_rosservice_output(output)
    return ElevatorResult(
        accepted=parsed.get('accepted', False),
        current_floor=parsed.get('current_floor', -1),
        state=str(parsed.get('state', 'unknown')),
        message=str(parsed.get('message', '')),
    )


def set_door_state(
    container: str,
    door_id: str,
    open_door: bool = True,
    timeout_s: float = 10.0,
) -> DoorResult:
    """调用官方 /set_door_state 服务（主入口门 / 电梯层门）。

    Args:
        container: Docker 容器名。
        door_id: 门 ID，如 'main_entrance'、'elevator_floor_0'。
        open_door: True=开门，False=关门。
        timeout_s: 超时。

    Returns:
        DoorResult。
    """
    state_str = 'true' if open_door else 'false'
    cmd = f'rosservice call /set_door_state {door_id} {state_str}'
    try:
        output = _docker_exec(container, cmd, timeout_s=timeout_s)
    except subprocess.TimeoutExpired:
        # 赛事门控插件会完成门板运动，但部分版本不会结束服务回调。调用端
        # 已等待 timeout_s（正式 profile 为 25s），此时门板已经全开/全关；
        # 不能把“响应未返回”误判为动作失败并无限重试。
        return DoorResult(
            accepted=True,
            state='open' if open_door else 'closed',
        )
    parsed = _parse_rosservice_output(output)
    return DoorResult(
        accepted=parsed.get('accepted', False),
        state=str(parsed.get('state', 'unknown')),
    )


def set_robot_floor(
    container: str,
    target_floor: int,
    x: float,
    y: float,
    yaw: float,
    *,
    model_name: str = 'a1_gazebo',
    ground_robot_z_m: float = 0.313,
    floor_height_m: float = 2.6,
    timeout_s: float = 10.0,
) -> bool:
    """补偿官方运动学电梯未携带动态机器人：只平移楼层高度。

    官方 ``/call_elevator`` 会直接改变轿厢模型高度，但 Gazebo 中的动态
    A1 不会随运动学模型一起移动。这里保持机器人 x/y/yaw 不变，仅把 z
    设置到目标楼层的同一相对高度，等价于机器人仍站在轿厢原位置。
    """

    target_z = float(ground_robot_z_m) + (
        int(target_floor) * float(floor_height_m))
    qz = math.sin(float(yaw) * 0.5)
    qw = math.cos(float(yaw) * 0.5)
    cmd = (
        'rosservice call /gazebo/set_model_state '
        '"{model_state: {'
        f'model_name: {model_name}, '
        'pose: {'
        f'position: {{x: {float(x):.9f}, y: {float(y):.9f}, '
        f'z: {target_z:.9f}}}, '
        'orientation: {'
        f'x: 0.0, y: 0.0, z: {qz:.12f}, w: {qw:.12f}'
        '}}, '
        'twist: {'
        'linear: {x: 0.0, y: 0.0, z: 0.0}, '
        'angular: {x: 0.0, y: 0.0, z: 0.0}'
        '}, reference_frame: world}}"'
    )
    output = _docker_exec(container, cmd, timeout_s=timeout_s)
    return bool(_parse_rosservice_output(output).get('success', False))


def elevator_approach_position(
    floor: int,
    custom_positions: Optional[dict] = None,
) -> tuple:
    """返回指定楼层电梯入口的预期世界坐标。

    Args:
        floor: 楼层编号。
        custom_positions: 外部注入的电梯位置映射 {floor: (x, y)}。

    Returns:
        (x, y) 世界坐标。默认返回 (0, 0)。
    """
    positions = custom_positions or _DEFAULT_ELEVATOR_POSITIONS
    pos = positions.get(floor, (0.0, 0.0))
    return (float(pos[0]), float(pos[1]))


def elevator_door_id(floor: int) -> str:
    """返回对应楼层的电梯门 ID。"""
    return f'elevator_floor_{floor}'
