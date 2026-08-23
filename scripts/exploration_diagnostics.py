#!/usr/bin/env python3
from __future__ import annotations
import math, time
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
try:
    from tf2_ros import Buffer, TransformListener
    HAS_TF = True
except ImportError:
    HAS_TF = False

class ExplorationDiagnostics(Node):
    def __init__(self):
        super().__init__('exploration_diagnostics')
        self.declare_parameter('report_interval_s', 3.0)
        self.nav_state = 'UNKNOWN'
        self.robot_x = None
        self.robot_y = None
        self.home_x = None
        self.home_y = None
        self.cmd_vel_lin = 0.0
        self.scan_min = float('inf')
        self.free_cells = 0
        self._dist_history = []
        if HAS_TF:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self._tf_timer = self.create_timer(0.2, self._update_pose)
        self.create_subscription(String, '/hw/nav/state', self._on_state, 10)
        self.create_subscription(TwistStamped, '/hw/cmd_vel', self._on_cmd, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 1)
        self.create_subscription(LaserScan, '/hw/scan', self._on_scan, 1)
        self.create_timer(float(self.get_parameter('report_interval_s').value), self._report)
        self._start = time.monotonic()

    def _update_pose(self):
        if not HAS_TF: return
        try:
            t = self.tf_buffer.lookup_transform('map', 'base', rclpy.time.Time())
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
            if self.home_x is not None:
                d = math.hypot(self.robot_x - self.home_x, self.robot_y - self.home_y)
                self._dist_history.append((time.monotonic(), d))
                if len(self._dist_history) > 300: self._dist_history.pop(0)
        except Exception: pass

    def _on_state(self, msg): self.nav_state = msg.data
    def _on_cmd(self, msg): self.cmd_vel_lin = msg.twist.linear.x
    def _on_map(self, msg):
        import numpy as np
        grid = np.array(msg.data, dtype=np.int8)
        self.free_cells = int(((grid >= 0) & (grid <= 49)).sum())
        if self.home_x is None and self.robot_x is not None:
            self.home_x = self.robot_x
            self.home_y = self.robot_y
    def _on_scan(self, msg):
        valid = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max and not math.isinf(r)]
        if valid: self.scan_min = min(valid)

    def _report(self):
        now = time.monotonic()
        print('\n' + '='*60)
        print(f'  诊断报告 @ t={now:.1f}s')
        print('='*60)
        print(f'导航状态: {self.nav_state}')
        if self.robot_x is not None:
            print(f'位置: ({self.robot_x:.2f}, {self.robot_y:.2f})')
        else:
            print(f'位置: (无TF)')
        if self.home_x is not None and self.robot_x is not None:
            d = math.hypot(self.robot_x - self.home_x, self.robot_y - self.home_y)
            print(f'离家距离: {d:.2f}m')
        print(f'速度指令: linear={self.cmd_vel_lin:.3f}')
        print(f'激光最近障碍: {self.scan_min:.2f}m')
        print(f'自由栅格: {self.free_cells}')
        if self.scan_min < 0.5 and self.nav_state in ('RETURNING', 'EXPLORING'):
            print('*** 警告: 激光最近障碍<0.5m，可能被门或障碍物挡住 ***')
        if len(self._dist_history) >= 10:
            recent = self._dist_history[-20:]
            first = recent[0][1]
            last = recent[-1][1]
            if first > 0 and last - first > 1.0:
                print(f'*** 离家距离从 {first:.2f}m 增加到 {last:.2f}m，越走越远！ ***')

def main():
    rclpy.init()
    node = ExplorationDiagnostics()
    try: rclpy.spin(node)
    except ExternalShutdownException: pass
    finally:
        if rclpy.ok(): node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
