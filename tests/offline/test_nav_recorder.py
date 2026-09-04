"""导航记录器按仿真时间降采样的回归测试。"""

import json
import sys
from pathlib import Path
import tempfile


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / 'ros2_ws' / 'src' / 'hazardwalker_nav'
sys.path.insert(0, str(PACKAGE))

from hazardwalker_nav.nav_recorder import NavRecorder  # noqa: E402


def _read_jsonl(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]


def test_recorder_downsamples_by_sim_time_not_wall_time():
    with tempfile.TemporaryDirectory() as temporary:
        output = Path(temporary)
        recorder = NavRecorder(
            output_dir=str(output), record_trajectory_hz=10.0,
            record_cmd_vel_hz=10.0)
        for stamp in (1.0, 1.01, 1.05, 1.099, 1.101, 1.15, 1.205):
            recorder.record_pose(
                stamp, stamp, 0.0, 0.0, 'EXPLORING',
                odom_pose=(stamp, -stamp), home_distance_m=stamp)
            recorder.record_cmd_vel(stamp, 0.6, 0.0)
        recorder.close(1.3, final_state='FINISHED', total_frontiers_visited=1)

        poses = _read_jsonl(output / 'trajectory.jsonl')
        commands = _read_jsonl(output / 'cmd_vel.jsonl')
        assert [row['ros_sec'] for row in poses] == [1.0, 1.101, 1.205]
        assert poses[-1]['odom_x'] == 1.205
        assert poses[-1]['odom_y'] == -1.205
        assert poses[-1]['home_distance_m'] == 1.205
        assert [row['ros_sec'] for row in commands] == [1.0, 1.101, 1.205]


def test_recorder_accepts_first_sample_after_sim_time_rollback():
    with tempfile.TemporaryDirectory() as temporary:
        output = Path(temporary)
        recorder = NavRecorder(output_dir=str(output))
        recorder.record_pose(5.0, 0.0, 0.0, 0.0, 'EXPLORING')
        recorder.record_pose(1.0, 0.0, 0.0, 0.0, 'EXPLORING')
        recorder.close(1.1, final_state='INTERRUPTED')

        poses = _read_jsonl(output / 'trajectory.jsonl')
        assert [row['ros_sec'] for row in poses] == [5.0, 1.0]


def test_recorder_writes_structured_room_coverage_evidence():
    with tempfile.TemporaryDirectory() as temporary:
        output = Path(temporary)
        recorder = NavRecorder(output_dir=str(output))
        recorder.record_room_coverage(
            10.0, 1, 'far_left', 'entered', 18.0, 2.1)
        recorder.record_room_coverage(
            35.0, 1, 'far_left', 'completed', 18.1, 2.0,
            path_m=12.3456, duration_s=25.4321, probe_count=4,
            reason='trajectory_loop_closed', obstacle_count=5,
            inspection_goal_count=15, inspection_completed_count=15)
        recorder.close(36.0, final_state='FINISHED')

        rows = _read_jsonl(output / 'room_coverage.jsonl')
        assert [row['phase'] for row in rows] == ['entered', 'completed']
        assert rows[1]['floor'] == 1
        assert rows[1]['sector'] == 'far_left'
        assert rows[1]['path_m'] == 12.346
        assert rows[1]['duration_s'] == 25.432
        assert rows[1]['probe_count'] == 4
        assert rows[1]['reason'] == 'trajectory_loop_closed'
        assert rows[1]['obstacle_count'] == 5
        assert rows[1]['inspection_goal_count'] == 15
        assert rows[1]['inspection_completed_count'] == 15
