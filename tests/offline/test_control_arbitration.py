"""统一控制仲裁的离线稳定性测试。"""

import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = (
    REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform'
    / 'hazardwalker_platform' / 'control_arbitration.py'
)
SPEC = importlib.util.spec_from_file_location('control_arbitration', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_stability_limiter_keeps_full_straight_speed():
    result = MODULE.limit_planar_yaw_coupling(
        0.8, 0.0, 0.0, max_planar_yaw_product=0.6)
    assert result == (0.8, 0.0, 0.0)


def test_stability_limiter_reduces_translation_during_fast_turn():
    x, y, yaw = MODULE.limit_planar_yaw_coupling(
        0.8, 0.0, 1.8, max_planar_yaw_product=0.6)
    assert abs(x - (0.6 / 1.8)) < 1e-9
    assert y == 0.0
    assert yaw == 1.8
    assert abs(x * yaw) <= 0.6 + 1e-9


def test_stability_limiter_scales_holonomic_translation_together():
    x, y, yaw = MODULE.limit_planar_yaw_coupling(
        0.6, 0.8, -1.2, max_planar_yaw_product=0.6)
    assert abs(x - 0.3) < 1e-9
    assert abs(y - 0.4) < 1e-9
    assert yaw == -1.2


def test_stability_limiter_rejects_invalid_limit():
    try:
        MODULE.limit_planar_yaw_coupling(
            0.8, 0.0, 1.0, max_planar_yaw_product=0.0)
    except ValueError:
        return
    raise AssertionError('零稳定性上限必须被拒绝')
