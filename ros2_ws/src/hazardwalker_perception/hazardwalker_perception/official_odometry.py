"""赛事公开里程计的起点归一化纯函数。

官方 SimEnv 在场景清单中把 ``/Odometry_gazebo`` 声明为参赛接口。运行时经
平台适配器转成 ``/hw/odom``；本模块只做刚体坐标变换，不读取场景布局、危险源
真值或裁判文件。ROS 节点与离线测试共享这里的实现。
"""

from dataclasses import dataclass
import math

from hazardwalker_perception.scan_imu_localization import normalize_angle


@dataclass(frozen=True)
class PlanarOdomPose:
    """二维里程计位姿。"""

    x: float
    y: float
    yaw: float


def relative_planar_pose(anchor, current):
    """把官方全局里程位姿转换为机器人启动时的前左坐标系。

    两个输入均可为 ``PlanarOdomPose`` 或 ``(x, y, yaw)``。返回坐标原点是
    ``anchor``，x 轴指向机器人启动朝向，保证与既有探索、楼层锚点和红球
    定位使用的 ``odom`` 语义一致。
    """

    anchor = _coerce_pose(anchor)
    current = _coerce_pose(current)
    delta_x = current.x - anchor.x
    delta_y = current.y - anchor.y
    cosine = math.cos(anchor.yaw)
    sine = math.sin(anchor.yaw)
    return PlanarOdomPose(
        x=cosine * delta_x + sine * delta_y,
        y=-sine * delta_x + cosine * delta_y,
        yaw=normalize_angle(current.yaw - anchor.yaw),
    )


def _coerce_pose(value):
    if isinstance(value, PlanarOdomPose):
        result = value
    else:
        try:
            result = PlanarOdomPose(*(float(item) for item in value))
        except (TypeError, ValueError):
            raise ValueError('里程计位姿必须是三个有限数值')
    if not all(math.isfinite(item) for item in (
            result.x, result.y, result.yaw)):
        raise ValueError('里程计位姿必须是三个有限数值')
    return result
