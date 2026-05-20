"""任务结果构建函数。

本文件不依赖 ROS。测试组可以直接构造 hazards 和状态，检查最终 result.json
是否满足接口文档要求。
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
