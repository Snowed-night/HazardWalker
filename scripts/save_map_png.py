#!/usr/bin/env python3
"""订阅 /map 一帧，保存为 PNG + YAML 地图图片。

用途：跳变测试建图后，把 Cartographer 的 /map 抓下来看地图，直观检查
墙壁是否有多重错位/重影。

依赖：仅 rclpy + numpy，PNG 用纯 stdlib(zlib/struct) 手写，
不依赖 nav2_map_server / PIL / matplotlib。

用法：
    source ~/HazardWalker/ros2_ws/install/setup.zsh     # 或 setup.bash
    export ROS_DOMAIN_ID=42
    python3 scripts/save_map_png.py                       # 抓一帧即退出
    python3 scripts/save_map_png.py --loop 5              # 每 5 秒抓一帧（看建图过程）
    python3 scripts/save_map_png.py --out /tmp/maps       # 指定输出目录
"""

from __future__ import annotations

import argparse
import os
import struct
import time
import zlib

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


# ---- PNG 手写（纯 stdlib，灰度图 color type 0） ----


def _png_chunk(tag: bytes, data: bytes) -> bytes:
    return (
        struct.pack('>I', len(data))
        + tag
        + data
        + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF)
    )


def write_grayscale_png(path: str, gray: np.ndarray) -> None:
    """把 uint8 二维数组写成灰度 PNG。"""
    height, width = gray.shape
    raw = b''.join(b'\x00' + gray[i].tobytes() for i in range(height))
    png = b'\x89PNG\r\n\x1a\n'
    png += _png_chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0))
    png += _png_chunk(b'IDAT', zlib.compress(raw, 9))
    png += _png_chunk(b'IEND', b'')
    with open(path, 'wb') as fp:
        fp.write(png)


def grid_to_gray(grid: np.ndarray) -> np.ndarray:
    """OccupancyGrid 语义转灰度：占用=黑(0)、自由=白(255)、未知=灰(127)、模糊区=浅灰(190)。"""
    gray = np.full(grid.shape, 127, dtype=np.uint8)
    gray[(grid >= 0) & (grid <= 49)] = 255
    gray[(grid >= 50) & (grid <= 64)] = 190
    gray[grid >= 65] = 0
    return gray


def write_yaml(path: str, msg: OccupancyGrid) -> None:
    """写 map_server 兼容的 yaml，便于后续 rviz2 / map_server 直接加载。"""
    image_name = os.path.basename(path).replace('.yaml', '.png')
    yaml_content = (
        f'image: {image_name}\n'
        f'mode: trinary\n'
        f'resolution: {msg.info.resolution:.4f}\n'
        f'origin: [{msg.info.origin.position.x:.4f}, '
        f'{msg.info.origin.position.y:.4f}, 0.0000]\n'
        f'negate: 0\n'
        f'occupied_thresh: 0.65\n'
        f'free_thresh: 0.196\n'
    )
    with open(path, 'w', encoding='utf-8') as fp:
        fp.write(yaml_content)


class MapSaver(Node):
    """订阅 /map，按需保存 PNG + YAML。"""

    def __init__(self, out_dir: str, loop_sec):
        super().__init__('map_saver')
        self._out = out_dir
        self._loop = loop_sec
        self._latest = None
        os.makedirs(self._out, exist_ok=True)

        self.sub = self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        if loop_sec is None:
            self.get_logger().info(
                f'等待 /map 一帧后保存并退出（输出 {self._out}）...')
        else:
            self.create_timer(loop_sec, self._save)
            self.get_logger().info(
                f'每 {loop_sec:.1f}s 抓一帧 /map 到 {self._out} ...')

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest = msg
        if self._loop is None:
            self._save()

    def _save(self) -> None:
        msg = self._latest
        if msg is None:
            self.get_logger().warning(
                '还没收到 /map，跳过。', throttle_duration_sec=2.0)
            return
        try:
            grid = np.array(msg.data, dtype=np.int8).reshape(
                msg.info.height, msg.info.width)
        except Exception as exc:
            self.get_logger().error(f'/map 数据 reshape 失败：{exc}')
            return
        gray = grid_to_gray(grid)
        stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime())
        base = os.path.join(self._out, f'map_{stamp}')
        write_grayscale_png(base + '.png', gray)
        write_yaml(base + '.yaml', msg)
        self.get_logger().info(
            f'已保存 {base}.png ({msg.info.width}x{msg.info.height}, '
            f'{msg.info.resolution:.3f} m/格) —— 黑=墙, 白=自由, 灰=未知')
        if self._loop is None:
            rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description='保存 /map 为 PNG + YAML')
    parser.add_argument(
        '--out', default='',
        help='输出目录；默认 ~/HazardWalker/reports/slam_monitor/map_<时间戳>/')
    parser.add_argument(
        '--loop', type=float, default=None, metavar='SEC',
        help='每 SEC 秒抓一帧（不传则抓一帧即退出）')
    args = parser.parse_args()

    if not args.out:
        ts = time.strftime('%Y%m%d_%H%M%S', time.localtime())
        root = os.environ.get(
            'HAZARDWALKER_ROOT',
            os.path.join(os.path.expanduser('~'), 'HazardWalker'),
        )
        args.out = os.path.join(root, 'reports', 'slam_monitor', f'map_{ts}')

    rclpy.init()
    node = MapSaver(args.out, args.loop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
