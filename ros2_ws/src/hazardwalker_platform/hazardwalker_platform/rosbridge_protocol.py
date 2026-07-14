"""rosbridge v2 消息分片与 ROS1 JSON 字节字段转换。

所属组：平台与仿真组。负责人：姜晨。
本文件不依赖 rclpy，便于离线验证 rosbridge 对大 RGB-D 消息的 fragment 重组边界。
"""

import json
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
        entry = self._parts.get(identity)
        if entry is not None and entry['total'] != total:
            self._parts.pop(identity, None)
            return None
        # rosbridge 对同一订阅通常复用 id。高频 RGB-D 的下一帧可能在上一帧未收全时再次从 num=0 开始；
        # 若继续复用旧 chunks，会把两帧 base64 片段拼到一起，最终出现 padding/长度错误。
        # WebSocket 保序，因此观察到新的 num=0 即可安全丢弃上一个不完整帧并从头重组。
        if entry is not None and number == 0 and 0 in entry['chunks']:
            self._parts.pop(identity, None)
            entry = None
        if entry is None:
            entry = {'total': total, 'chunks': {}, 'created': time.monotonic()}
            self._parts[identity] = entry
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
