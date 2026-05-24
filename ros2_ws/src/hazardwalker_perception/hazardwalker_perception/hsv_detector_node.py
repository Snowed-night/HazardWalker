"""HSV 红球检测 ROS 节点。

所属组：感知组。
文件作用：
- 把 `/hw/camera/image_raw` 转为危险源候选 JSON。
- 作为离线 `red_ball_detector.py` 到 ROS 话题的桥接层。

当前职责：
- 订阅图像话题并读取参数。
- 调用离线检测函数得到 2D 红球框。
- 输出临时占位的 2D/3D 结果，供决策和结果写入先联通。

后续扩展方式：
- 增加 `CameraInfo`、`PointCloud2` 和 TF 订阅后，补真实 `localize_hazard`。
- 将当前 JSON 字符串输出替换为 `hazardwalker_msgs/HazardArray`。
- 增加调试图像发布，方便人工确认阈值和误检情况。

验证方式：
- 先用 fake platform 的人造红球图像验证能稳定出框。
- 再在仿真图像上验证阈值、坐标和状态是否正确。
"""
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from hazardwalker_perception.red_ball_detector import detect_red_ball_rgb_bytes


class HsvDetectorNode(Node):
    def __init__(self):
        super().__init__('hsv_detector_node')
        self.declare_parameter('min_area_px', 80)
        self.declare_parameter('min_confidence', 0.5)

        # 只依赖平台层输出的统一图像 topic，不直接依赖 Gazebo/官方平台 topic。
        self.sub = self.create_subscription(Image, '/hw/camera/image_raw', self.on_image, 10)
        # 第一阶段用 String(JSON) 快速打通链路；稳定后迁移到 hazardwalker_msgs/HazardArray。
        self.pub = self.create_publisher(String, '/hw/perception/hazard_detections', 10)
        self.get_logger().info('HSV detector subscribed to /hw/camera/image_raw.')

    def on_image(self, msg: Image):
        # 当前只支持最常见的 rgb8/bgr8。正式版本应通过 cv_bridge 支持更多编码。
        if msg.encoding.lower() not in ('rgb8', 'bgr8'):
            self.get_logger().warn(f'Unsupported image encoding: {msg.encoding}', throttle_duration_sec=5.0)
            return

        detection_2d = detect_red_ball_rgb_bytes(
            data=msg.data,
            width=msg.width,
            height=msg.height,
            step=msg.step,
            encoding=msg.encoding,
            min_area_px=int(self.get_parameter('min_area_px').value),
            min_confidence=float(self.get_parameter('min_confidence').value),
        )
        if detection_2d is None:
            return

        # position 是临时占位，当前只用于让决策和结果写入链路先跑通。
        # 后续应改成：图像检测框 -> 相机内参 -> 点云/深度 -> TF 变换 -> start/map 坐标。
        detection = {
            'id': 1,
            'frame_id': msg.header.frame_id,
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'bbox': {
                'x_min': detection_2d.x_min,
                'y_min': detection_2d.y_min,
                'x_max': detection_2d.x_max,
                'y_max': detection_2d.y_max,
            },
            'position': [2.0, 0.0, 0.6],
            'position_frame_id': 'start',
            'confidence': detection_2d.confidence,
            'status': 'tentative',
            'observation_time': time.time(),
            'source': 'hsv_minimal',
        }
        out = String()
        out.data = json.dumps({'hazards': [detection]}, ensure_ascii=False)
        self.pub.publish(out)


def main():
    """启动 ROS 节点。

    该节点适合在仿真或 fake platform 下验证图像检测链路。
    """
    rclpy.init()
    node = HsvDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
