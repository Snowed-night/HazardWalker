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

from hazardwalker_decision.result_builder import build_mission_result


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
