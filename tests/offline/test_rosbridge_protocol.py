"""官方 SimEnv rosbridge 大消息协议的离线回归测试。

负责人：姜晨。此文件覆盖 RGB-D 等大消息的分片重组边界，不需要 ROS2 或 Gazebo。
"""

import json
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PLATFORM_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform'
if str(PLATFORM_SRC) not in sys.path:
    sys.path.insert(0, str(PLATFORM_SRC))

from hazardwalker_platform.rosbridge_protocol import (  # noqa: E402
    FragmentAssembler,
    decode_laser_ranges,
    decode_packet,
    decode_ros_time,
    filter_scan_self_returns,
)


def _fragment(identity, number, total, data):
    return {'op': 'fragment', 'id': identity, 'num': number, 'total': total, 'data': data}


def test_fragment_assembler_reassembles_out_of_order_rgbd_packet():
    """同一帧的分片乱序到达时，必须按 num 恢复完整 JSON。"""
    original = json.dumps({'op': 'publish', 'topic': '/real_sense/rgb/image_raw',
                           'msg': {'data': 'abcdef', 'height': 1, 'width': 2}})
    chunks = [original[:17], original[17:41], original[41:]]
    assembler = FragmentAssembler()

    assert assembler.accept(_fragment('rgb-frame', 2, 3, chunks[2])) is None
    assert assembler.accept(_fragment('rgb-frame', 0, 3, chunks[0])) is None
    assert assembler.accept(_fragment('rgb-frame', 1, 3, chunks[1])) == original


def test_decode_packet_returns_none_until_fragment_is_complete():
    original = json.dumps({'op': 'publish', 'topic': '/Odometry_gazebo', 'msg': {'child_frame_id': 'base'}})
    split = len(original) // 2
    assembler = FragmentAssembler()

    first = json.dumps(_fragment('odom-frame', 0, 2, original[:split]))
    second = json.dumps(_fragment('odom-frame', 1, 2, original[split:]))
    assert decode_packet(first, assembler) is None
    assert decode_packet(second, assembler)['topic'] == '/Odometry_gazebo'


def test_invalid_fragment_does_not_create_unbounded_state():
    assembler = FragmentAssembler(max_messages=1)
    assert assembler.accept(_fragment('', 0, 1, '{}')) is None
    assert assembler.accept(_fragment('bad-index', 2, 2, '{}')) is None
    assert assembler._parts == {}

    assembler.accept(_fragment('old', 0, 2, '{'))
    assembler.accept(_fragment('new', 0, 2, '{'))
    assert len(assembler._parts) == 1
    assert 'new' in assembler._parts


def test_fragment_total_change_discards_corrupted_frame():
    assembler = FragmentAssembler()
    assert assembler.accept(_fragment('same', 0, 2, '{')) is None
    assert assembler.accept(_fragment('same', 0, 3, '{')) is None
    assert 'same' not in assembler._parts


def test_new_frame_zero_fragment_discards_incomplete_previous_frame():
    """同一订阅 id 的高频下一帧不能混入上一帧尚未收齐的 base64 数据。"""
    assembler = FragmentAssembler()
    assert assembler.accept(_fragment('rgb-subscription', 0, 3, 'old-0')) is None
    assert assembler.accept(_fragment('rgb-subscription', 1, 3, 'old-1')) is None

    assert assembler.accept(_fragment('rgb-subscription', 0, 3, 'new-0')) is None
    assert assembler.accept(_fragment('rgb-subscription', 1, 3, 'new-1')) is None
    assert assembler.accept(_fragment('rgb-subscription', 2, 3, 'new-2')) == 'new-0new-1new-2'


def test_laserscan_null_ranges_are_restored_as_positive_infinity():
    """rosbridge 的严格 JSON null 是 LaserScan +inf，不得使适配器重连。"""

    decoded = decode_laser_ranges([None, 1.25, '2.5'])

    assert decoded[0] == float('inf')
    assert decoded[1:] == [1.25, 2.5]


def test_scan_self_filter_removes_only_calibrated_body_returns():
    filtered = filter_scan_self_returns(
        [float('inf'), 0.10, 0.34, 0.40, 0.75, float('nan')],
        0.40,
    )
    assert filtered[0] == float('inf')  # 原始无回波语义保持不变。
    assert filtered[1] != filtered[1]
    assert filtered[2] != filtered[2]
    assert filtered[3:5] == [0.40, 0.75]
    assert filtered[5] != filtered[5]  # NaN 仍保持无效，由下游按标准规则忽略。


def test_clock_decoder_accepts_ros1_and_ros2_field_names():
    assert decode_ros_time({'secs': 94, 'nsecs': 12}) == (94, 12)
    assert decode_ros_time({'sec': 95, 'nanosec': 13}) == (95, 13)
