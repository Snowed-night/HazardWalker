import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


def rgb_to_hsv_pixel(r, g, b):
    r_f = r / 255.0
    g_f = g / 255.0
    b_f = b / 255.0
    max_c = max(r_f, g_f, b_f)
    min_c = min(r_f, g_f, b_f)
    delta = max_c - min_c

    if delta == 0.0:
        hue = 0.0
    elif max_c == r_f:
        hue = (60.0 * ((g_f - b_f) / delta) + 360.0) % 360.0
    elif max_c == g_f:
        hue = 60.0 * ((b_f - r_f) / delta + 2.0)
    else:
        hue = 60.0 * ((r_f - g_f) / delta + 4.0)

    saturation = 0.0 if max_c == 0.0 else delta / max_c
    value = max_c
    return hue / 2.0, saturation * 255.0, value * 255.0


class HsvDetectorNode(Node):
    """Minimal HSV red-ball detector.

    It publishes JSON strings for early integration. The message type should be
    replaced by hazardwalker_msgs after custom messages are defined.
    """

    def __init__(self):
        super().__init__('hsv_detector_node')
        self.declare_parameter('min_area_px', 80)
        self.declare_parameter('min_confidence', 0.5)

        self.sub = self.create_subscription(Image, '/hw/camera/image_raw', self.on_image, 10)
        self.pub = self.create_publisher(String, '/hw/perception/hazard_detections', 10)
        self.get_logger().info('HSV detector subscribed to /hw/camera/image_raw.')

    def on_image(self, msg: Image):
        if msg.encoding.lower() not in ('rgb8', 'bgr8'):
            self.get_logger().warn(f'Unsupported image encoding: {msg.encoding}', throttle_duration_sec=5.0)
            return

        red_pixels = []
        channels = 3
        is_bgr = msg.encoding.lower() == 'bgr8'
        for y in range(msg.height):
            row = y * msg.step
            for x in range(msg.width):
                index = row + x * channels
                if index + 2 >= len(msg.data):
                    continue
                c0 = msg.data[index]
                c1 = msg.data[index + 1]
                c2 = msg.data[index + 2]
                r, g, b = (c2, c1, c0) if is_bgr else (c0, c1, c2)
                h, s, v = rgb_to_hsv_pixel(r, g, b)
                is_red = (h <= 10.0 or h >= 170.0) and s >= 80.0 and v >= 80.0
                if is_red:
                    red_pixels.append((x, y))

        min_area = int(self.get_parameter('min_area_px').value)
        if len(red_pixels) < min_area:
            return

        xs = [p[0] for p in red_pixels]
        ys = [p[1] for p in red_pixels]
        x_min = min(xs)
        x_max = max(xs)
        y_min = min(ys)
        y_max = max(ys)
        area_ratio = len(red_pixels) / float(msg.width * msg.height)
        confidence = min(1.0, max(float(self.get_parameter('min_confidence').value), area_ratio * 50.0))

        detection = {
            'id': 1,
            'frame_id': msg.header.frame_id,
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'bbox': {
                'x_min': x_min,
                'y_min': y_min,
                'x_max': x_max,
                'y_max': y_max,
            },
            'position': [2.0, 0.0, 0.6],
            'position_frame_id': 'start',
            'confidence': confidence,
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

