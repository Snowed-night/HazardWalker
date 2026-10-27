"""按公开起点/电梯落点把每层 SLAM map 锚定到比赛 world 的纯函数。"""

import math


def world_from_map_at_robot_anchor(
        map_x, map_y, map_yaw, world_x, world_y, world_yaw):
    """由同一机器人锚点在 map/world 中的位姿求 world←map 平面变换。"""

    values = (map_x, map_y, map_yaw, world_x, world_y, world_yaw)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('floor anchor inputs must be finite')
    yaw = math.atan2(
        math.sin(float(world_yaw) - float(map_yaw)),
        math.cos(float(world_yaw) - float(map_yaw)),
    )
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    translation_x = (
        float(world_x)
        - cosine * float(map_x)
        + sine * float(map_y)
    )
    translation_y = (
        float(world_y)
        - sine * float(map_x)
        - cosine * float(map_y)
    )
    return translation_x, translation_y, yaw
