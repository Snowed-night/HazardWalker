"""深度 ICP 里程计的坐标变换与平面位姿积分纯函数。"""

from __future__ import annotations

import math

import numpy as np


_BASE_FROM_OPTICAL = np.array([
    [0.0, 0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0, 0.0],
    [0.0, -1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
], dtype=np.float64)


def optical_registration_to_base_delta(source_to_destination):
    """把点坐标变换 ``dst_p=Rt*src_p`` 转成机器人本体运动增量。"""

    registration = np.asarray(source_to_destination, dtype=np.float64)
    if registration.shape != (4, 4) or not np.isfinite(registration).all():
        raise ValueError('ICP 变换必须是有限的 4x4 矩阵')
    camera_motion = np.linalg.inv(registration)
    optical_from_base = np.linalg.inv(_BASE_FROM_OPTICAL)
    base_motion = _BASE_FROM_OPTICAL @ camera_motion @ optical_from_base
    yaw = math.atan2(base_motion[1, 0], base_motion[0, 0])
    return (
        float(base_motion[0, 3]),
        float(base_motion[1, 3]),
        float(yaw),
    )


def base_delta_to_optical_registration(dx, dy, dyaw):
    """由控制先验构造 OpenCV ICP 所需的源点到目标点初值。"""

    cosine = math.cos(float(dyaw))
    sine = math.sin(float(dyaw))
    base_motion = np.array([
        [cosine, -sine, 0.0, float(dx)],
        [sine, cosine, 0.0, float(dy)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=np.float64)
    camera_motion = (
        np.linalg.inv(_BASE_FROM_OPTICAL)
        @ base_motion
        @ _BASE_FROM_OPTICAL
    )
    return np.linalg.inv(camera_motion)


def integrate_planar_pose(pose, body_delta):
    """把本体系增量积分到固定里程计坐标系。"""

    x, y, yaw = map(float, pose)
    dx, dy, dyaw = map(float, body_delta)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        x + cosine * dx - sine * dy,
        y + sine * dx + cosine * dy,
        math.atan2(math.sin(yaw + dyaw), math.cos(yaw + dyaw)),
    )
