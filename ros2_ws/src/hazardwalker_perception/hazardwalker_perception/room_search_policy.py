"""复杂房间视角覆盖与候选复查调度纯函数。

本模块把“看见候选后换视角”和“没有候选时继续覆盖房间”放进同一个状态机。
它只输出语义动作，不直接发布速度；导航/仿真执行层必须用真实位姿反馈完成动作，
并在相机稳定后才把对应扇区记为已覆盖。
"""

from dataclasses import dataclass
from typing import Optional

from .active_view_policy import choose_active_view_action


@dataclass(frozen=True)
class RoomSearchPolicyConfig:
    """房间水平视场离散和复查预算。"""

    sector_count: int = 5
    max_rechecks_per_target: int = 2
    active_priority_threshold: int = 50


@dataclass(frozen=True)
class RoomSearchAction:
    """搜索状态机给导航层的一次动作建议。"""

    action: str
    reason: str
    priority: int
    target_id: str = ''
    # 不使用 ``int | None``，确保后续接入官方 ROS1 Noetic 时仍可由 Python 3.8 解析。
    target_sector: Optional[int] = None
    mode: str = 'coverage'

    def to_dict(self):
        return {
            'action': self.action,
            'reason': self.reason,
            'priority': self.priority,
            'target_id': self.target_id,
            'target_sector': self.target_sector,
            'mode': self.mode,
        }


def choose_room_search_action(
    detections,
    image_width,
    image_height,
    visited_sectors,
    current_sector,
    target_recheck_counts=None,
    config=None,
):
    """优先处理有价值候选，否则选择最近的未覆盖水平扇区。

    visited_sectors 只能由稳定图像和真实机器人位姿更新。候选复查达到预算后，
    状态机恢复房间覆盖，避免被同一个红色干扰物永久拖住。
    """

    policy = config or RoomSearchPolicyConfig()
    _validate_policy(policy)
    current = _clamp_sector(current_sector, policy.sector_count)
    visited = {
        _clamp_sector(value, policy.sector_count)
        for value in visited_sectors
    }
    rechecks = target_recheck_counts or {}

    if detections:
        active = choose_active_view_action(detections, image_width, image_height)
        target_id = str(active.target_id)
        used = int(rechecks.get(target_id, 0))
        if (
            active.priority >= policy.active_priority_threshold
            and used < policy.max_rechecks_per_target
        ):
            return RoomSearchAction(
                active.action,
                f'候选驱动复查（{used + 1}/{policy.max_rechecks_per_target}）：{active.reason}',
                active.priority + 100,
                target_id=target_id,
                target_sector=current,
                mode='candidate_recheck',
            )

    target_sector = _nearest_unvisited_sector(visited, current, policy.sector_count)
    if target_sector is None:
        return RoomSearchAction(
            'search_complete',
            '所有水平扇区均已在稳定真实视角下覆盖。',
            0,
            target_sector=current,
            mode='complete',
        )
    if target_sector == current:
        return RoomSearchAction(
            'hold_observation',
            '当前扇区尚未形成稳定证据，先停稳采集再继续。',
            40,
            target_sector=target_sector,
        )
    action = 'turn_left' if target_sector < current else 'turn_right'
    return RoomSearchAction(
        action,
        f'选择距离当前视角最近的未覆盖扇区 {target_sector}，减少无效转向。',
        30,
        target_sector=target_sector,
    )


def choose_fixed_sweep_action(visited_sectors, current_sector, order=(0, 1, 2, 3, 4)):
    """固定巡检基线：完全按预设顺序访问，不响应候选。"""

    visited = {int(value) for value in visited_sectors}
    remaining = [int(value) for value in order if int(value) not in visited]
    current = int(current_sector)
    if not remaining:
        return RoomSearchAction(
            'search_complete',
            '固定巡检序列已完成。',
            0,
            target_sector=current,
            mode='complete',
        )
    target = remaining[0]
    if target == current:
        action = 'hold_observation'
    else:
        action = 'turn_left' if target < current else 'turn_right'
    return RoomSearchAction(
        action,
        f'固定巡检访问预设扇区 {target}。',
        10,
        target_sector=target,
        mode='fixed_sweep',
    )


def coverage_ratio(visited_sectors, sector_count=5):
    """返回合法扇区覆盖率，忽略越界和重复输入。"""

    if int(sector_count) <= 0:
        raise ValueError('sector_count must be positive')
    valid = {
        int(value)
        for value in visited_sectors
        if 0 <= int(value) < int(sector_count)
    }
    return len(valid) / float(sector_count)


def _nearest_unvisited_sector(visited, current, sector_count):
    remaining = [sector for sector in range(sector_count) if sector not in visited]
    if not remaining:
        return None
    # 距离相同时优先朝画面中心一侧，减少扫到房间边界后的往返。
    center = (sector_count - 1) / 2.0
    return min(remaining, key=lambda sector: (
        abs(sector - current),
        abs(sector - center),
        sector,
    ))


def _clamp_sector(value, sector_count):
    return min(max(int(value), 0), int(sector_count) - 1)


def _validate_policy(policy):
    if int(policy.sector_count) <= 0:
        raise ValueError('sector_count must be positive')
    if int(policy.max_rechecks_per_target) < 0:
        raise ValueError('max_rechecks_per_target cannot be negative')
