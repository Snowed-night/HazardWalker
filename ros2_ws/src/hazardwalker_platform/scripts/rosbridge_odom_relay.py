#!/usr/bin/env python3
"""官方 SimEnv 里程计最新值中继。

所属组：平台与仿真组。此脚本只在官方 ROS1 容器内运行，把原生
``/Odometry_gazebo`` 压缩为稳定的 ``/hazardwalker/odom``，供 ROS2 rosbridge
适配器发布诊断 ``/hw/odom``。它不是 SLAM 位姿，导航、感知定位和正式结果
不得将该话题当作比赛合法定位来源。
"""

import argparse
import threading

import rospy
from nav_msgs.msg import Odometry


class LatestOdomRelay:
    """仅保留最新一帧的低频转发器，避免启动期高频里程计挤占 rosbridge。"""

    def __init__(self, source, output, rate_hz):
        self._latest_message = None
        self._lock = threading.Lock()
        self._publisher = rospy.Publisher(output, Odometry, queue_size=1)
        self._subscriber = rospy.Subscriber(
            source, Odometry, self._on_odom, queue_size=1,
        )
        # 回调只覆盖缓存；定时器才发布。不能依赖高频 rospy 回调里的墙钟比较，
        # 因为其可能并发执行，导致“至多 20 Hz”的判断竞争失效。
        self._timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / rate_hz), self._publish_latest,
        )
        rospy.loginfo(
            'hazardwalker odom relay: %s -> %s at most %.1f Hz',
            source, output, rate_hz,
        )

    def _on_odom(self, message):
        # Gazebo 暂停期间只覆盖一份最新值，不累计待发布数据。
        with self._lock:
            self._latest_message = message

    def _publish_latest(self, _event):
        """固定频率发布最近一帧，保证 rosbridge 负载上界。"""

        with self._lock:
            message = self._latest_message
        if message is not None:
            self._publisher.publish(message)


def parse_args():
    """读取显式话题和频率参数，禁止把来源静默替换成其他里程计。"""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', default='/Odometry_gazebo')
    parser.add_argument('--output', default='/hazardwalker/odom')
    parser.add_argument('--rate-hz', type=float, default=20.0)
    args = parser.parse_args()
    if args.rate_hz <= 0.0:
        parser.error('--rate-hz must be positive')
    return args


def main():
    args = parse_args()
    rospy.init_node('hazardwalker_odom_relay', anonymous=False)
    LatestOdomRelay(args.source, args.output, args.rate_hz)
    rospy.spin()


if __name__ == '__main__':
    main()
