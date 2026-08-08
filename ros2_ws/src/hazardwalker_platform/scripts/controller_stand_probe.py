#!/usr/bin/env python3
"""负责人新增：根据官方里程计核验 A1 是否真实站立，供启动与倒地恢复共用。"""

import argparse
import math
import time
from collections import deque

import rospy
from nav_msgs.msg import Odometry


class StandObserver:
    """保存最近姿态；只使用公开 /Odometry_gazebo，不读取场景或机器人真值文件。"""

    def __init__(self) -> None:
        self.samples = deque(maxlen=120)
        self.subscriber = rospy.Subscriber(
            '/Odometry_gazebo', Odometry, self._callback, queue_size=20)

    def _callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        # 旋转矩阵第三列的 z 分量：1 代表机身正立，-1 代表机身翻倒。
        up_cos = 1.0 - 2.0 * (pose.orientation.x ** 2 + pose.orientation.y ** 2)
        self.samples.append((time.monotonic(), pose.position.z, up_cos))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='核验 A1 是否稳定站立')
    parser.add_argument('--duration-sec', type=float, default=8.0)
    parser.add_argument('--timeout-sec', type=float, default=25.0)
    parser.add_argument('--min-base-height-m', type=float, default=0.30)
    parser.add_argument('--min-upright-cos', type=float, default=0.85)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration_sec <= 0 or args.timeout_sec < args.duration_sec:
        raise SystemExit('duration-sec 必须为正，且 timeout-sec 不得小于 duration-sec。')

    rospy.init_node('hazardwalker_controller_stand_probe', anonymous=True)
    observer = StandObserver()
    started_at = time.monotonic()
    deadline = started_at + args.timeout_sec
    rate = rospy.Rate(20)

    # 用一整段墙钟时间观察，避免刚收到一帧“看似站立”的里程计就误报通过。
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if time.monotonic() - started_at >= args.duration_sec and observer.samples:
            window_start = time.monotonic() - min(2.0, args.duration_sec)
            window = [sample for sample in observer.samples if sample[0] >= window_start]
            if window:
                min_height = min(sample[1] for sample in window)
                min_up_cos = min(sample[2] for sample in window)
                if min_height >= args.min_base_height_m and min_up_cos >= args.min_upright_cos:
                    print(
                        '[A1_STAND] PASS '
                        f'base_z>={min_height:.3f}m, upright_cos>={min_up_cos:.3f}')
                    return 0
                print(
                    '[A1_STAND] FAIL '
                    f'base_z={min_height:.3f}m (min {args.min_base_height_m:.3f}), '
                    f'upright_cos={min_up_cos:.3f} (min {args.min_upright_cos:.3f})')
                return 1
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break

    print('[A1_STAND] FAIL no stable /Odometry_gazebo samples before timeout')
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
