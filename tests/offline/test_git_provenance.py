"""正式实验 Git 来源过滤离线测试。"""

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / 'scripts'
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import git_provenance  # noqa: E402
from git_provenance import code_dirty_entries  # noqa: E402


def test_report_and_ros_build_outputs_do_not_mark_algorithm_code_dirty():
    porcelain = '\n'.join([
        '?? reports/perception/simulation/3d_native/run/summary.json',
        ' M ros2_ws/build/hazardwalker_perception/cache.txt',
        '?? ros2_ws/install/setup.bash',
        '?? ros2_ws/log/latest/build.log',
        '?? results/diagnostic/frame.png',
        ' M ros2_ws/src/hazardwalker_platform/generated_building/competition_scene.world',
        ' M ros2_ws/src/hazardwalker_platform/generated_building/danger_truth.json',
        '?? ros2_ws/src/hazardwalker_platform/results/detected_danger.json',
        '?? ros2_ws/src/hazardwalker_platform/logs/gzserver.log',
        '?? ros2_ws/src/hazardwalker_platform/.ros1_catkin_ws/devel/setup.bash',
    ])
    assert code_dirty_entries(porcelain) == []


def test_source_config_docs_and_renamed_code_are_code_changes():
    porcelain = '\n'.join([
        ' M config/perception.yaml',
        '?? scripts/new_detector.py',
        'R  scripts/old.py -> scripts/new.py',
        ' M docs/groups/perception/guide.md',
        ' M ros2_ws/src/hazardwalker_platform/src/building_obstacles/scripts/generate_competition_scene.py',
    ])
    assert code_dirty_entries(porcelain) == [
        'config/perception.yaml',
        'docs/groups/perception/guide.md',
        'ros2_ws/src/hazardwalker_platform/src/building_obstacles/scripts/generate_competition_scene.py',
        'scripts/new.py',
        'scripts/new_detector.py',
    ]


def test_git_output_preserves_first_porcelain_status_column():
    """通用 Git 读取不能吞掉 porcelain 第一行的前导状态空格。"""

    original = git_provenance.subprocess.check_output
    git_provenance.subprocess.check_output = lambda *args, **kwargs: (
        ' M ros2_ws/src/hazardwalker_platform/generated_building/world.sdf\n')
    try:
        output = git_provenance._git_value(REPO_ROOT, 'status', '--porcelain')
    finally:
        git_provenance.subprocess.check_output = original
    assert output.startswith(' M ')
    assert code_dirty_entries(output) == []
