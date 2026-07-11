#!/usr/bin/env python3
"""保存官方 SimEnv `/hw/camera/image_raw` 的一帧原生 RGB 图像。

用于证据矩阵中的无受控目标基准帧；订阅使用传感器 QoS，以兼容平台的
Best Effort 相机发布。该脚本不读取场景真值，也不运行检测器。
"""

import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    if len(sys.argv) != 2:
        raise SystemExit('Usage: capture_official_simenv_rgb_frame.py OUTPUT.png')
    output_path = sys.argv[1]
    rclpy.init()
    node = Node('official_simenv_rgb_baseline_capture')
    received = {'message': None}

    def on_image(message):
        """缓存第一张可解释的 RGB/BGR 帧。"""

        if message.encoding.lower() in ('rgb8', 'bgr8'):
            received['message'] = message

    node.create_subscription(Image, '/hw/camera/image_raw', on_image, qos_profile_sensor_data)
    deadline = time.monotonic() + 15.0
    while received['message'] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    message = received['message']
    if message is None:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('Timed out waiting for /hw/camera/image_raw.')
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    packed = raw.reshape((message.height, message.step))[:, :message.width * 3]
    image = packed.reshape((message.height, message.width, 3))
    if message.encoding.lower() == 'rgb8':
        image = image[:, :, ::-1]
    if not cv2.imwrite(output_path, image):
        raise RuntimeError(f'Unable to save {output_path}.')
    print(f'Saved {output_path}: {message.width}x{message.height} {message.encoding}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
