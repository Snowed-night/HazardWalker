#!/usr/bin/env python3
"""依次以不同速度下发 /hw/cmd_vel，配合 calibrate_motion_scale.py 标定死区。

用法
----
    source ~/HazardWalker/ros2_ws/install/setup.zsh
    export ROS_DOMAIN_ID=42
    python3 scripts/cmd_vel_sweep.py --speeds 0.10 0.20 0.30 0.35 0.40 --each-sec 5

每个速度直行 ``--each-sec`` 秒，之间停 ``--gap-sec`` 秒，最后自动发 0 停车。
运行时逐行打印当前速度，Ctrl-C 立即停车并退出。

安全提示：机器人会直线前进数米，务必在空旷直道运行，前方无障碍。
"""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def main() -> None:
    parser = argparse.ArgumentParser(description='依次以不同速度直行，标定死区')
    parser.add_argument('--speeds', nargs='+', type=float,
                        default=[0.10, 0.20, 0.30, 0.35, 0.40],
                        help='要测试的线速度序列 (m/s)')
    parser.add_argument('--each-sec', type=float, default=5.0,
                        help='每个速度持续秒数')
    parser.add_argument('--gap-sec', type=float, default=2.0,
                        help='两个速度之间停车的秒数')
    parser.add_argument('--topic', default='/hw/cmd_vel',
                        help='cmd_vel 话题')
    parser.add_argument('--rate-hz', type=float, default=20.0,
                        help='发布频率')
    args = parser.parse_args()

    rclpy.init()
    node = Node('cmd_vel_sweep')
    pub = node.create_publisher(Twist, args.topic, 10)
    # 用 Python 原生 sleep 控节奏，避免 rclpy 的 Rate 在无 spin 时永久阻塞。
    period = 1.0 / args.rate_hz

    def send(vx: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        pub.publish(msg)

    node.get_logger().info(
        '速度扫描：%s m/s，每个 %.1fs，间隔 %.1fs'
        % (args.speeds, args.each_sec, args.gap_sec))

    try:
        for vx in args.speeds:
            node.get_logger().info('>>> 前进 %.2f m/s（%.1fs）' % (vx, args.each_sec))
            deadline = time.monotonic() + args.each_sec
            while time.monotonic() < deadline:
                send(vx)
                time.sleep(period)

            node.get_logger().info('>>> 停车（%.1fs）' % args.gap_sec)
            deadline = time.monotonic() + args.gap_sec
            while time.monotonic() < deadline:
                send(0.0)
                time.sleep(period)

        send(0.0)
        node.get_logger().info('扫描完成，已停车。')
    except KeyboardInterrupt:
        send(0.0)
        node.get_logger().info('中断，已停车。')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
