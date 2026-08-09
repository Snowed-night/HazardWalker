"""感知建议、人工辅助对准与统一控制仲裁的纯逻辑集成测试。"""

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PLATFORM_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform'
if str(PLATFORM_SRC) not in sys.path:
    sys.path.insert(0, str(PLATFORM_SRC))

from hazardwalker_platform.assist_alignment import (  # noqa: E402
    compute_alignment_decision,
    evaluate_control_takeover,
)
from hazardwalker_platform.control_arbitration import (  # noqa: E402
    ControlArbitrator,
)


def _candidate_payload(x_min, x_max):
    return {
        'image_width': 1000,
        'view_recommendation': {
            'action': 'turn_left' if x_max < 500 else 'hold_observation',
            'target_id': 'candidate-1',
        },
        'detections_2d': [{
            'candidate_id': 'candidate-1',
            'confidence': 0.9,
            'requires_reobservation': True,
            'bbox': {
                'x_min': x_min,
                'x_max': x_max,
                'y_min': 100,
                'y_max': 220,
            },
        }],
    }


def test_user_confirmed_alignment_and_navigation_replacement_share_one_mux():
    """证明感知不直接控制，且键盘/辅助/导航可安全切换。"""

    mux = ControlArbitrator(default_mode='keyboard')
    mux.update_source(
        'keyboard', linear_x=0.45, linear_y=0.0, angular_z=0.0,
        received_monotonic_sec=1.0,
    )
    assert mux.resolve(1.1).linear_x == 0.45

    # 感知决策只返回对准角速度；用户确认后执行层才切换模式。
    alignment = compute_alignment_decision(_candidate_payload(100, 220))
    assert alignment.valid and alignment.angular_z > 0.0
    waiting = evaluate_control_takeover(
        'keyboard', elapsed_sec=0.1, since_request_sec=0.1,
        timeout_sec=1.5, retry_sec=0.2)
    assert not waiting.ready
    # 控制模式尚未确认时，辅助计算结果不能进入最终输出。
    assert mux.resolve(1.11).mode == 'keyboard'
    mux.select_mode('assist')
    takeover = evaluate_control_takeover(
        'assist', elapsed_sec=0.2, since_request_sec=0.0,
        timeout_sec=1.5, retry_sec=0.2)
    assert takeover.ready
    assert mux.resolve(1.11).reason == 'source_never_received'
    mux.update_source(
        'assist', linear_x=0.0, linear_y=0.0,
        angular_z=alignment.angular_z,
        received_monotonic_sec=1.12,
    )
    assist_output = mux.resolve(1.13)
    assert assist_output.linear_x == 0.0
    assert assist_output.angular_z == alignment.angular_z

    centered = compute_alignment_decision(_candidate_payload(470, 530))
    assert centered.centered and centered.angular_z == 0.0
    mux.update_source(
        'assist', linear_x=0.0, linear_y=0.0, angular_z=0.0,
        received_monotonic_sec=1.14,
    )
    mux.select_mode('keyboard')
    assert mux.resolve(1.15).reason == 'source_never_received'

    # 导航替换键盘后不需要修改感知载荷；但必须在接管后
    # 重新发送速度，切换前的旧命令不会被复用。
    mux.update_source(
        'navigation', linear_x=0.30, linear_y=0.0, angular_z=-0.2,
        received_monotonic_sec=2.0,
    )
    mux.select_mode('navigation')
    assert mux.resolve(2.01).reason == 'source_never_received'
    mux.update_source(
        'navigation', linear_x=0.30, linear_y=0.0, angular_z=-0.2,
        received_monotonic_sec=2.02,
    )
    navigation_output = mux.resolve(2.03)
    assert navigation_output.mode == 'navigation'
    assert navigation_output.linear_x == 0.30
    assert navigation_output.angular_z == -0.2
