#!/usr/bin/env python3
"""标定 scan_imu_localizer 的 command_motion_scale 与低速死区阈值。

背景
----
scan_imu_localizer 的 odometry 平移几乎 100% 来自 cmd_vel 积分（scan 校正被
``_bound_translation_correction`` 限制在每帧 1mm，故意保守以防沿墙滑移）。当
``command_motion_scale = 1.0``（假设实际速度 == 命令速度）而 A1 底盘存在低速
死区时，积分系统性偏大/偏小，odometry 漂移，Cartographer 校正漂移时产生地图
重影。

本脚本在【开发/标定阶段】临时订阅平台诊断里程计 ``/hw/odom`` 作为真实位移
参考，对比 scan_imu_localizer 的 odometry 位移，算出应设的
``command_motion_scale``。它不参与正式运行，不读场景布局或危险源真值。

用法
----
    source ~/HazardWalker/ros2_ws/install/setup.zsh
    export ROS_DOMAIN_ID=42
    python3 scripts/calibrate_motion_scale.py

然后在另一终端让机器人走一段（键盘控制或手动下发 /hw/cmd_vel）：
- 标 total scale：随意直线走一段即可；
- 标死区：依次以 0.10 / 0.20 / 0.30 / 0.35 / 0.40 m/s 各直行几秒。
走完回到本终端 Ctrl-C，脚本打印建议参数。

仅依赖 rclpy，位移用纯 math，不依赖 numpy/PIL。
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class CalibrationNode(Node):
    """同时订阅参考里程计与待标定里程计，按时间序列累计位移并分速度段统计。"""

    def __init__(self, ref_topic: str, target_topic: str, cmd_vel_topic: str):
        super().__init__('motion_scale_calibration')

        # 参考（真值）与待标定位姿的最新快照。
        self._ref_pos = None
        self._ref_stamp = 0.0
        self._target_pos = None
        self._target_stamp = 0.0
        # 首末位姿，用于总 scale。
        self._ref_first = None
        self._ref_last = None
        self._target_first = None
        self._target_last = None
        # 命令线速度最新值（body 帧 x）。
        self._cmd_vx = 0.0
        self._cmd_stamp = 0.0
        # 上一次 target 回调时刻的参考位姿快照，用于算两次 target 之间的真实位移。
        self._ref_at_last_target = None
        # 分速度段累计：{bucket: [target_dist, ref_dist, count]}
        self._buckets = {}

        self.create_subscription(Odometry, ref_topic, self._on_ref, 10)
        self.create_subscription(Odometry, target_topic, self._on_target, 10)
        if cmd_vel_topic:
            self.create_subscription(Twist, cmd_vel_topic, self._on_cmd, 10)

        self.get_logger().info(
            '标定已启动：参考 %s，待标定 %s。让机器人走一段后 Ctrl-C。'
            % (ref_topic, target_topic)
        )

    # ---- 订阅回调 ----

    def _on_ref(self, msg: Odometry) -> None:
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._ref_pos = pos
        self._ref_stamp = _stamp_sec(msg.header.stamp)
        if self._ref_first is None:
            self._ref_first = pos
        self._ref_last = pos

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd_vx = float(msg.linear.x)
        self._cmd_stamp = time.monotonic()

    def _on_target(self, msg: Odometry) -> None:
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        stamp = _stamp_sec(msg.header.stamp)

        if (self._target_pos is not None and self._ref_pos is not None
                and self._ref_at_last_target is not None):
            t_delta = math.hypot(
                pos[0] - self._target_pos[0], pos[1] - self._target_pos[1])
            r_delta = math.hypot(
                self._ref_pos[0] - self._ref_at_last_target[0],
                self._ref_pos[1] - self._ref_at_last_target[1],
            )
            self._accumulate(self._cmd_vx, t_delta, r_delta)

        self._target_pos = pos
        self._target_stamp = stamp
        self._ref_at_last_target = self._ref_pos
        if self._target_first is None:
            self._target_first = pos
        self._target_last = pos

    def _accumulate(self, cmd_vx: float, t_delta: float, r_delta: float) -> None:
        speed = abs(cmd_vx)
        bucket = self._bucket(speed)
        entry = self._buckets.setdefault(bucket, [0.0, 0.0, 0])
        entry[0] += t_delta
        entry[1] += r_delta
        entry[2] += 1

    @staticmethod
    def _bucket(speed: float) -> str:
        if speed < 0.12:
            return '<0.12'
        if speed < 0.22:
            return '0.12~0.22'
        if speed < 0.30:
            return '0.22~0.30'
        if speed < 0.40:
            return '0.30~0.40'
        return '>=0.40'

    # ---- 输出 ----

    def summarize(self) -> dict:
        t_delta = math.hypot(
            self._target_last[0] - self._target_first[0],
            self._target_last[1] - self._target_first[1],
        ) if self._target_last and self._target_first else 0.0
        r_delta = math.hypot(
            self._ref_last[0] - self._ref_first[0],
            self._ref_last[1] - self._ref_first[1],
        ) if self._ref_last and self._ref_first else 0.0
        total_scale = (r_delta / t_delta) if t_delta > 1e-6 else float('nan')
        return {
            'target_dist_m': round(t_delta, 4),
            'ref_dist_m': round(r_delta, 4),
            'total_scale': round(total_scale, 4),
            'buckets': self._buckets,
        }

    def print_report(self) -> None:
        s = self.summarize()
        print('\n================ 标定结果 ================')
        print('待标定 odometry 积分位移 : %.4f m' % s['target_dist_m'])
        print('参考里程计真实位移       : %.4f m' % s['ref_dist_m'])
        if s['ref_dist_m'] < 1e-3 and s['target_dist_m'] > 1e-2:
            print('  ⚠ 参考位移≈0：/hw/odom 可能没数据。先跑 '
                  '`ros2 topic echo --once /hw/odom` 确认话题名。')
        scale = s['total_scale']
        if math.isfinite(scale):
            print('总位移比 (ref/target)    : %.4f' % scale)
            print('  → 建议 command_motion_scale = %.4f' % scale)
            print('  → scale > 1 表示积分偏小（机器人实际走得比积分快），需放大；')
            print('    scale < 1 表示积分偏大，需缩小。')
        else:
            print('总位移比：无效（待标定几乎没动，先让机器人走一段）。')

        print('\n---- 分速度段（识别死区）----')
        print('%-10s %6s %10s %10s %8s' % ('速度档(m/s)', '样本', '积分位移m', '真实位移m', 'scale'))
        order = ['<0.12', '0.12~0.22', '0.22~0.30', '0.30~0.40', '>=0.40']
        for bucket in order:
            if bucket not in s['buckets']:
                continue
            t, r, n = s['buckets'][bucket]
            bscale = (r / t) if t > 1e-6 else float('nan')
            scale_str = '%.4f' % bscale if math.isfinite(bscale) else '  n/a'
            flag = ''
            if r < 1e-3 and t > 1e-2:
                flag = '  <-- 死区（命令有积分但实际没动）'
            print('%-10s %6d %10.4f %10.4f %8s%s' % (bucket, n, t, r, scale_str, flag))

        print('\n解读：')
        print('  - 死区档：把 min_effective_linear_speed_mps 设到该档上界；')
        print('  - 有效速度档：用该档 scale 作为 command_motion_scale（若各档一致，')
        print('    用总位移比即可）。')
        print('==========================================')


def main() -> None:
    parser = argparse.ArgumentParser(description='标定 command_motion_scale 与死区阈值')
    parser.add_argument('--ref-topic', default='/hw/odom',
                        help='真实位移参考话题（默认 /hw/odom）')
    parser.add_argument('--target-topic', default='/hazardwalker/slam/odometry',
                        help='待标定 odometry 话题（默认 scan_imu_localizer 输出）')
    parser.add_argument('--cmd-vel-topic', default='/hw/cmd_vel',
                        help='命令速度话题（空字符串则不做分速度段统计）')
    args = parser.parse_args()

    rclpy.init()
    node = CalibrationNode(
        args.ref_topic, args.target_topic,
        args.cmd_vel_topic or None,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
