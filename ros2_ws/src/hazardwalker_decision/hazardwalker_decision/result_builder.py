"""任务结果构建函数。

所属组：决策组 / 测试组。
文件作用：
- 把任务状态、危险源列表和运行统计整理成最终结果字典。
- 给结果写文件和结果检查脚本提供统一的 JSON 结构。

当前函数职责：
- `build_mission_result`：生成可直接写入 JSON 的结果对象，并统一补齐 `confirmed` 计数。

后续扩展方式：
- 如果将来结果结构要增加 `mission_time`, `return_pose`, `false_positive_estimate` 等字段，应优先在这里集中改。
- 只要这个函数输出结构稳定，`mission_state_machine_node.py` 和 `scripts/evaluate_result.py` 都能同步复用。

验证方式：
- 用 `tests/offline/test_result_builder.py` 构造确认/未确认危险源，检查 `num_confirmed_hazards` 是否正确。
"""


def build_mission_result(mission_id, status, hazards, duration_sec, return_success=True):
    """构建任务结果字典。

    Args:
        mission_id: 本次任务 ID。
        status: 任务结束状态，例如 FINISHED 或 FAILED。
        hazards: 危险源列表，每个元素是 dict。
        duration_sec: 任务持续时间。
        return_success: 是否成功返航。

    Returns:
        可直接写入 JSON 的 dict。

    说明：
    - `hazards` 里的单项数据应尽量和 `hazardwalker_msgs/Hazard` 保持一致。
    - 当前默认把未显式标记状态的危险源视为 `confirmed`，便于最小 demo 输出结果。
    """

    normalized_hazards = []
    for hazard in hazards:
        item = dict(hazard)
        item.setdefault('status', 'confirmed')
        normalized_hazards.append(item)

    confirmed_count = sum(1 for hazard in normalized_hazards if hazard.get('status') == 'confirmed')
    return {
        'mission_id': mission_id,
        'status': status,
        'hazards': normalized_hazards,
        'metrics': {
            'duration_sec': float(duration_sec),
            'return_success': bool(return_success),
            'num_confirmed_hazards': confirmed_count,
        },
    }
