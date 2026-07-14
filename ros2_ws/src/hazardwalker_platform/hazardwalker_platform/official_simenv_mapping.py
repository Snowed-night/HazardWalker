"""官方 SimEnv ROS1 到 HazardWalker ROS2 接口的固定映射。

所属组：平台与仿真组。负责人：姜晨。
文件作用：集中保存官方 ROS1 Noetic + Gazebo Classic profile 的话题约定，供启动脚本、
离线测试和文档共同引用，避免把 Gazebo Harmonic 的 ``ros_gz_bridge`` 配置误作官方适配。
当前边界：跨 ROS 版本由官方容器内 ``ros1_bridge dynamic_bridge`` 完成；本项目的 ROS1
中继只负责官方原始话题和稳定 ``/hw/*`` 名称之间的转换。
验证方式：``python scripts/run_offline_tests.py`` 及
``scripts/verify_official_simenv_ros1_adapter.sh``。
"""

from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass(frozen=True)
class TopicMapping:
    """一条可审计的话题映射，direction 表示数据方向。"""

    source: str
    destination: str
    ros_type: str
    direction: str
    required: bool = True


# 传感器和里程计必须先在 ROS1 内改名为稳定 /hw 接口，再由 ros1_bridge 原样送入 ROS2 DDS。
OFFICIAL_ROS1_TO_HW: Tuple[TopicMapping, ...] = (
    TopicMapping('/Odometry_gazebo', '/hw/odom', 'nav_msgs/Odometry', 'ros1_to_ros2'),
    TopicMapping('/real_sense/rgb/image_raw', '/hw/camera/image_raw', 'sensor_msgs/Image', 'ros1_to_ros2'),
    TopicMapping('/real_sense/depth/image_raw', '/hw/camera/depth_image', 'sensor_msgs/Image', 'ros1_to_ros2'),
    TopicMapping('/real_sense/rgb/camera_info', '/hw/camera/camera_info', 'sensor_msgs/CameraInfo', 'ros1_to_ros2'),
    TopicMapping('/real_sense/depth/camera_info', '/hw/camera/depth_camera_info', 'sensor_msgs/CameraInfo', 'ros1_to_ros2'),
    TopicMapping('/real_sense/depth/points', '/hw/camera/depth_points', 'sensor_msgs/PointCloud2', 'ros1_to_ros2', False),
    TopicMapping('/livox/Pointcloud2', '/hw/lidar/points', 'sensor_msgs/PointCloud2', 'ros1_to_ros2', False),
)

# 不同官方镜像对前视 RGB 的命名存在差异。适配器默认使用第一项，运行前必须以
# ``rostopic list`` 为准，并用 ``~rgb_topic`` 覆盖；同一时刻只能选择一个物理相机作为业务输入。
OFFICIAL_RGB_TOPIC_CANDIDATES = (
    '/real_sense/rgb/image_raw',
    '/camera/image_raw',
)

# TF 在 ROS1/ROS2 两侧保持 ROS 标准名称，由 dynamic_bridge 直接桥接，不经重新发布以免回环。
TF_PASSTHROUGH: Tuple[TopicMapping, ...] = (
    TopicMapping('/tf', '/tf', 'tf2_msgs/TFMessage', 'ros1_to_ros2'),
    TopicMapping('/tf_static', '/tf_static', 'tf2_msgs/TFMessage', 'ros1_to_ros2'),
)

HW_CMD_VEL = '/hw/cmd_vel'
OFFICIAL_CMD_VEL = '/cmd_vel'
ADAPTER_STATUS_TOPIC = '/hw/platform/official_simenv_adapter_status'


def source_to_destination() -> Dict[str, TopicMapping]:
    """返回以官方 ROS1 源话题为键的映射，便于测试和启动器生成检查项。"""

    return {item.source: item for item in OFFICIAL_ROS1_TO_HW}


def validate_mapping() -> Tuple[str, ...]:
    """检查映射是否存在重复目标或不符合稳定 /hw 命名的条目。"""

    errors = []
    destinations = set()
    for item in OFFICIAL_ROS1_TO_HW:
        if not item.destination.startswith('/hw/'):
            errors.append('官方传感器目标必须位于 /hw/*：%s' % item.destination)
        if item.destination in destinations:
            errors.append('存在重复目标话题：%s' % item.destination)
        destinations.add(item.destination)
    if HW_CMD_VEL == OFFICIAL_CMD_VEL:
        errors.append('业务控制话题不得与官方原始 /cmd_vel 同名')
    return tuple(errors)
