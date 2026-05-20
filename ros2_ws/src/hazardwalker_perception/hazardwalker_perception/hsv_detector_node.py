import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from hazardwalker_perception.red_ball_detector import detect_red_ball_rgb_bytes


class HsvDetectorNode(Node):
    """第一阶段 HSV 红球检测节点。

    当前节点只完成最小链路：
    - 订阅 `/hw/camera/image_raw`
    - 找到图像中的红色区域
    - 发布一个 JSON 格式的危险源候选

    注意：这里的三维 position 是占位值，不是真实定位结果。后续感知组要把
    点云/深度和 TF 接进来，替换为真实三维坐标。
    """

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

        # position 是临时占位，用于让决策节点和结果写入先能工作。
        # 真正比赛版本必须由 localize_hazard() 根据 CameraInfo、PointCloud2 和 TF 计算。
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
    rclpy.init()
    node = HsvDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
