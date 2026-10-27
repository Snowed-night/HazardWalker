"""A阶段静态检测入口的合规回归。"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_stage_a_detector_launcher_is_localization_only_and_fail_closed():
    source = (
        ROOT / 'scripts' / 'run_official_simenv_stage_a_detector.sh'
    ).read_text(encoding='utf-8')

    assert 'real_sense' in source and 'base' in source
    assert 'map/world' in source
    assert 'max_rgb_depth_sync_delta_sec:="$RGBD_MAX_DELTA_SEC"' in source
    assert 'rmw_fastrtps_cpp' in source
    assert 'FASTDDS_BUILTIN_TRANSPORTS' in source
    assert '/cmd_vel' not in source
    assert 'geometry_msgs/msg/Twist' not in source
