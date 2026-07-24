#!/usr/bin/env python3
"""以精确持续时间发布低速控制探针，并在任何退出路径连续发送停车指令。

所属组：平台与仿真组。
本脚本只供官方 SimEnv 启动门禁使用，不参与导航或感知运行时决策。
"""

import argparse
import time

import rospy
from geometry_msgs.msg import Twist


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/cmd_vel')
    parser.add_argument('--speed-mps', type=float, required=True)
    parser.add_argument('--duration-sec', type=float, required=True)
    parser.add_argument('--connection-timeout-sec', type=float, default=5.0)
    parser.add_argument('--rate-hz', type=float, default=20.0)
    args = parser.parse_args()

    rospy.init_node('hazardwalker_controller_motion_probe', anonymous=True)
    publisher = rospy.Publisher(args.topic, Twist, queue_size=1)
    deadline = time.monotonic() + max(0.1, args.connection_timeout_sec)
    while publisher.get_num_connections() < 1 and time.monotonic() < deadline:
        time.sleep(0.05)
    if publisher.get_num_connections() < 1:
        rospy.logerr('控制探针在超时前没有发现 /cmd_vel 订阅者。')
        return 3

    command = Twist()
    command.linear.x = float(args.speed_mps)
    period = 1.0 / max(1.0, float(args.rate_hz))
    stop = Twist()
    try:
        end_time = time.monotonic() + max(0.0, float(args.duration_sec))
        while time.monotonic() < end_time and not rospy.is_shutdown():
            publisher.publish(command)
            time.sleep(period)
    finally:
        # 多发几帧零速，覆盖桥接/订阅端的单帧丢包。
        for _ in range(5):
            publisher.publish(stop)
            time.sleep(period)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
