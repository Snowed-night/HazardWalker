"""官方 SimEnv 运行契约中的纯函数。

本文件不依赖 ROS，目的是让 ROS1 Noetic 的启动和控制时序可以在本地离线单测。
它只编码官方公开的 junior_ctrl `/joy` 按键协议，不读取场景真值或 Gazebo 内部状态。
"""


def activation_command(elapsed, initial_delay_s, stand_hold_s, settle_s, rl_hold_s):
    """返回自动切换 `/cmd_vel` 模式时应发布的阶段和唯一 Joy 按键。

    按官方 `keyboard_teleop.py`：button[1] 是固定站立，button[3] 是 RL `/cmd_vel`。
    先站立再等待机械姿态稳定，避免从 passive 状态直接进入策略控制造成摔倒。
    """
    if elapsed < initial_delay_s:
        return 'waiting_for_controller', None
    stand_end = initial_delay_s + stand_hold_s
    if elapsed < stand_end:
        return 'standing', 1
    settle_end = stand_end + settle_s
    if elapsed < settle_end:
        return 'settling', None
    rl_end = settle_end + rl_hold_s
    if elapsed < rl_end:
        return 'switching_to_cmd_vel', 3
    return 'ready', None
