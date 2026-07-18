"""rosbridge v2 消息分片与 ROS1 JSON 字节字段转换。

所属组：平台与仿真组。负责人：姜晨。
本文件不依赖 rclpy，便于离线验证 rosbridge 对大 RGB-D 消息的 fragment 重组边界。
"""

import json
import math
import time


class FragmentAssembler:
    """有界重组 rosbridge ``fragment`` 包，超时或异常序号立即丢弃。"""

    def __init__(self, max_messages=8, timeout_sec=2.0):
        self._max_messages = max_messages
        self._timeout_sec = timeout_sec
        self._parts = {}

    def accept(self, packet):
        """接收一个 fragment，完整时返回原始 JSON 文本，否则返回 None。"""
        self._expire()
        identity = str(packet.get('id', ''))
        number = int(packet.get('num', -1))
        total = int(packet.get('total', 0))
        data = packet.get('data')
        if not identity or not isinstance(data, str) or total <= 0 or number < 0 or number >= total:
            return None
        entry = self._parts.setdefault(identity, {'total': total, 'chunks': {}, 'created': time.monotonic()})
        if entry['total'] != total:
            self._parts.pop(identity, None)
            return None
        entry['chunks'][number] = data
        if len(entry['chunks']) != total:
            self._bound()
            return None
        try:
            merged = ''.join(entry['chunks'][index] for index in range(total))
        except KeyError:
            return None
        self._parts.pop(identity, None)
        return merged

    def _expire(self):
        now = time.monotonic()
        for identity in list(self._parts):
            if now - self._parts[identity]['created'] > self._timeout_sec:
                self._parts.pop(identity, None)

    def _bound(self):
        while len(self._parts) > self._max_messages:
            oldest = min(self._parts, key=lambda key: self._parts[key]['created'])
            self._parts.pop(oldest, None)


def decode_packet(text, assembler):
    """解析普通或 fragment 形式的 rosbridge JSON；未完成分片返回 None。"""
    packet = json.loads(text)
    if packet.get('op') != 'fragment':
        return packet
    merged = assembler.accept(packet)
    return json.loads(merged) if merged is not None else None


def decode_laser_ranges(values):
    """把 rosbridge 的 LaserScan 数组恢复为 ROS 浮点值。

    ROS1 LaserScan 使用 ``+inf`` 表示该方向没有回波；严格 JSON 没有 Infinity，
    rosbridge 会把它编码成 ``null``。直接调用 ``float(None)`` 会断开整条传感器
    WebSocket，因此这里把 null 恢复为正无穷，并保留其他有限/字符串数值。
    """

    decoded = []
    for value in values or []:
        decoded.append(math.inf if value is None else float(value))
    return decoded


def filter_scan_self_returns(values, minimum_range_m):
    """过滤已标定为机身遮挡的近场回波，同时保留原始数组长度与无回波语义。

    官方 A1 的水平 ray 原点位于机身内部，固定产生约 0.10–0.34 m 的自身回波。
    这些光束不可能看到外部障碍；应以 ``NaN`` 丢弃，而不能改成表示“该方向
    无回波”的 ``+inf``，否则 Cartographer 会错误清空一条自由射线。
    """

    threshold = max(0.0, float(minimum_range_m))
    filtered = []
    for value in values or []:
        distance = float(value)
        if math.isfinite(distance) and 0.0 < distance < threshold:
            filtered.append(math.nan)
        else:
            filtered.append(distance)
    return filtered


def decode_ros_time(value):
    """兼容 ROS1 与 ROS2 JSON 字段名并返回 ``(sec, nanosec)``。

    ROS1 rosbridge 使用 ``secs/nsecs``，部分桥接版本会输出 ROS2 风格
    ``sec/nanosec``。统一解码可避免 `/clock` 和传感器时间处于不同数量级。
    """

    value = value or {}
    seconds = int(value.get('secs', value.get('sec', 0)))
    nanoseconds = int(value.get('nsecs', value.get('nanosec', 0)))
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError('invalid ROS time')
    return seconds, nanoseconds
