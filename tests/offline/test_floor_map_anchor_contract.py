"""每层 SLAM 锚定节点的启动与数据来源合同测试。"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_floor_anchor_node_never_reads_forbidden_truth_sources():
    source = (
        ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
        / 'hazardwalker_perception' / 'floor_map_anchor_node.py'
    ).read_text(encoding='utf-8')
    executable = (
        ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception' / 'setup.py'
    ).read_text(encoding='utf-8')

    assert "'/hw/trunk_imu'" in source
    assert "'/hazardwalker/navigation/floor_index'" in source
    assert 'Initial floor keeps the pre-motion runner map anchor.' in source
    assert 'self.seen_floors' in source
    assert "'/hw/odom'" not in source
    assert "'/Odometry_gazebo'" not in source
    assert 'danger_truth.json' not in source
    assert 'floor_map_anchor_node = ' in executable


def test_business_launch_and_result_writer_use_floor_anchors():
    launch = (
        ROOT / 'ros2_ws' / 'src' / 'hazardwalker_bringup' / 'launch'
        / 'official_simenv_business.launch.py'
    ).read_text(encoding='utf-8')
    decision = (
        ROOT / 'ros2_ws' / 'src' / 'hazardwalker_decision'
        / 'hazardwalker_decision' / 'mission_state_machine_node.py'
    ).read_text(encoding='utf-8')

    assert "executable='floor_map_anchor_node'" in launch
    assert "'/hazardwalker/slam/floor_anchors'" in launch
    assert 'self.floor_world_from_map' in decision
    assert 'world_from_source_by_floor=self.floor_world_from_map' in decision
    assert "floor_map_anchors.json" in decision
    assert "'hazardwalker_floor_map_anchor_set_v1'" in decision
