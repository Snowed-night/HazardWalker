"""RGB/Depth 独立 WebSocket 到达顺序的离线回归。"""

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PERCEPTION_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
if str(PERCEPTION_SRC) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SRC))

from hazardwalker_perception.rgbd_pairing import DeferredRgbDepthPairer


def test_rgb_waits_for_same_stamp_depth_arriving_afterward():
    pairer = DeferredRgbDepthPairer(max_delta_sec=0.02)

    assert pairer.push_rgb(10.0, 'rgb_10') == []
    dispatched = pairer.push_depth(10.0)

    assert [item.payload for item in dispatched] == ['rgb_10']
    assert dispatched[0].depth_synchronized is True
    assert dispatched[0].depth_stamp_delta_sec == 0.0


def test_depth_arriving_before_rgb_dispatches_immediately():
    pairer = DeferredRgbDepthPairer(max_delta_sec=0.02)

    assert pairer.push_depth(12.0) == []
    dispatched = pairer.push_rgb(12.0, 'rgb_12')

    assert [item.payload for item in dispatched] == ['rgb_12']
    assert dispatched[0].depth_synchronized is True


def test_previous_50ms_depth_is_not_paired_but_next_exact_depth_is():
    pairer = DeferredRgbDepthPairer(max_delta_sec=0.02)
    pairer.push_depth(20.00)

    assert pairer.push_rgb(20.05, 'rgb_20_05') == []
    dispatched = pairer.push_depth(20.05)

    assert [item.payload for item in dispatched] == ['rgb_20_05']
    assert dispatched[0].depth_synchronized is True
    assert dispatched[0].depth_stamp_delta_sec == 0.0


def test_missing_depth_flushes_old_rgb_when_next_rgb_arrives():
    pairer = DeferredRgbDepthPairer(max_delta_sec=0.02)
    assert pairer.push_rgb(30.0, 'rgb_old') == []

    dispatched = pairer.push_rgb(30.5, 'rgb_new')

    assert [item.payload for item in dispatched] == ['rgb_old']
    assert dispatched[0].depth_synchronized is False
    assert dispatched[0].depth_stamp_delta_sec is None


def test_invalid_threshold_or_stamp_fails_closed():
    try:
        DeferredRgbDepthPairer(max_delta_sec=-0.1)
    except ValueError:
        pass
    else:
        raise AssertionError('negative sync threshold must be rejected')

    pairer = DeferredRgbDepthPairer(max_delta_sec=0.02)
    try:
        pairer.push_rgb(float('nan'), 'bad')
    except ValueError:
        pass
    else:
        raise AssertionError('non-finite stamp must be rejected')
