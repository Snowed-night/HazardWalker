"""任务结果构建离线测试。

所属组：决策组 / 测试组。
文件作用：
- 验证 `result_builder.py` 生成的 result JSON 结构。
- 不依赖 ROS，只检查 Python 字典内容。

当前验证内容：
- 未显式写状态的危险源默认按 `confirmed` 处理。
- `num_confirmed_hazards` 能正确统计确认目标数量。
- `return_success` 和基础字段能正确写入。

后续扩展：
- 如果 result 增加定位误差、运行距离、虚警估计等字段，这里同步补测试。
"""
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_decision'))

from hazardwalker_decision.result_builder import (
    build_mission_result,
    build_official_detected_danger_result,
)


def test_build_mission_result_counts_confirmed_hazards():
    """验证结果构建函数能正确统计 confirmed 危险源数量。"""
    result = build_mission_result(
        mission_id='test_run',
        status='FINISHED',
        hazards=[
            {'id': 1, 'position': [1.0, 2.0, 0.5], 'confidence': 0.9},
            {'id': 2, 'position': [3.0, 2.0, 0.5], 'confidence': 0.7, 'status': 'tentative'},
        ],
        duration_sec=12.5,
        return_success=True,
    )

    assert result['mission_id'] == 'test_run'
    assert result['status'] == 'FINISHED'
    assert len(result['hazards']) == 2
    assert result['hazards'][0]['status'] == 'confirmed'
    assert result['hazards'][1]['status'] == 'tentative'
    assert result['metrics']['num_confirmed_hazards'] == 1
    assert result['metrics']['return_success'] is True


def test_official_result_only_exports_confirmed_world_frame_unique_sources():
    """候选、非 world 坐标和重复轨迹均不得进入官方危险源输出。"""
    result = build_official_detected_danger_result(
        hazards=[
            {'id': 1, 'status': 'confirmed', 'position_frame_id': 'world',
             'position': [1.0, 2.0, 0.3], 'confidence': 0.90},
            {'id': 2, 'status': 'confirmed', 'position_frame_id': 'world',
             'position': [1.12, 2.02, 0.30], 'confidence': 0.60},
            {'id': 3, 'status': 'tentative', 'position_frame_id': 'world',
             'position': [3.0, 2.0, 0.3], 'confidence': 0.99},
            {'id': 4, 'status': 'rejected_non_spherical', 'position_frame_id': 'world',
             'position': [4.0, 2.0, 0.3], 'confidence': 0.99},
            {'id': 5, 'status': 'confirmed', 'position_frame_id': 'start',
             'position': [5.0, 2.0, 0.3], 'confidence': 0.99},
            {'id': 6, 'status': 'confirmed', 'position_frame_id': 'world',
             'position': [6.0, 2.0, 0.3], 'confidence': 0.80},
        ],
        exploration_time_sec=98.7654,
    )

    assert result == {
        'exploration_time': 98.765,
        'detected_danger_sources': [
            {'position': [1.0, 2.0, 0.3]},
            {'position': [6.0, 2.0, 0.3]},
        ],
    }


def test_official_result_can_require_legal_slam_provenance():
    """比赛模式不得把未验证/Gazebo 真值定位的 confirmed 轨迹写入提交文件。"""
    result = build_official_detected_danger_result(
        hazards=[
            {'id': 1, 'status': 'confirmed', 'position_frame_id': 'world',
             'position': [1.0, 2.0, 0.3], 'confidence': 0.99,
             'localization_provenance': 'unverified'},
            {'id': 2, 'status': 'confirmed', 'position_frame_id': 'world',
             'position': [2.0, 2.0, 0.3], 'confidence': 0.98,
             'localization_provenance': 'lidar_imu_slam'},
        ],
        exploration_time_sec=12.0,
        require_legal_localization=True,
    )

    assert result['detected_danger_sources'] == [{'position': [2.0, 2.0, 0.3]}]


def test_official_result_can_require_multiview_sphere_evidence():
    """最终评分文件不能仅凭 status=confirmed 接纳未记录球面复查证据的目标。"""
    result = build_official_detected_danger_result(
        hazards=[
            {'id': 1, 'status': 'confirmed', 'position_frame_id': 'world',
             'position': [1.0, 2.0, 0.3], 'confidence': 0.99,
             'evidence_status': 'single_view_flat_or_non_spherical'},
            {'id': 2, 'status': 'confirmed', 'position_frame_id': 'world',
             'position': [2.0, 2.0, 0.3], 'confidence': 0.98,
             'evidence_status': 'multi_view_sphere_consistent'},
        ],
        exploration_time_sec=12.0,
        require_multiview_sphere_evidence=True,
    )

    assert result['detected_danger_sources'] == [{'position': [2.0, 2.0, 0.3]}]
