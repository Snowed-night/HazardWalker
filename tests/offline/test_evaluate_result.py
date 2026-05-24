"""结果 JSON 检查脚本离线测试。

所属组：测试组。
文件作用：
- 验证 `scripts/evaluate_result.py` 的结构检查逻辑。
- 不读取真实文件，直接构造 result 字典。

当前验证内容：
- 合法 result 能通过检查并输出摘要。
- `num_confirmed_hazards` 与实际确认数量不一致时会报错。

后续扩展：
- 如果 result schema 增加新字段，这里同步增加合法和非法样例。
"""
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'scripts'))

from evaluate_result import evaluate_result


def test_evaluate_result_accepts_valid_result():
    """验证合法 result 字典可以通过检查。"""
    result = {
        'mission_id': 'test_run',
        'status': 'FINISHED',
        'hazards': [
            {'id': 1, 'status': 'confirmed', 'position': [1.0, 0.0, 0.5]},
        ],
        'metrics': {
            'duration_sec': 10.0,
            'return_success': True,
            'num_confirmed_hazards': 1,
        },
    }

    ok, errors, summary = evaluate_result(result)

    assert ok
    assert errors == []
    assert summary['confirmed_hazard_count'] == 1


def test_evaluate_result_rejects_mismatched_confirmed_count():
    """验证确认数量统计不一致时检查失败。"""
    result = {
        'mission_id': 'test_run',
        'status': 'FINISHED',
        'hazards': [
            {'id': 1, 'status': 'confirmed', 'position': [1.0, 0.0, 0.5]},
        ],
        'metrics': {
            'duration_sec': 10.0,
            'return_success': True,
            'num_confirmed_hazards': 0,
        },
    }

    ok, errors, _summary = evaluate_result(result)

    assert not ok
    assert errors
