"""根据感知候选计算辅助对准转向命令。

文件作用：
- 从感知 JSON 中选择当前复查目标；
- 根据目标框中心与画面中心的偏差生成限幅角速度；
- 只提供纯计算结果，不发布速度、不改变控制模式。

真正的运动由独立执行节点和控制仲裁器完成。
"""

from dataclasses import dataclass
import math
from typing import Any, Dict, Iterable, Optional


@dataclass(frozen=True)
class AlignmentConfig:
    """辅助对准参数，均可由 ROS 参数覆盖。"""

    center_tolerance_ratio: float = 0.08
    angular_kp: float = 1.2
    min_angular_speed: float = 0.25
    max_angular_speed: float = 0.80


@dataclass(frozen=True)
class AlignmentDecision:
    """单帧对准决策。"""

    valid: bool
    centered: bool
    target_id: str
    center_error_ratio: Optional[float]
    angular_z: float
    reason: str


@dataclass(frozen=True)
class ControlTakeoverDecision:
    """辅助执行器等待统一仲裁器确认接管时的纯状态。"""

    ready: bool
    failed: bool
    should_retry: bool
    reason: str


def evaluate_control_takeover(
    selected_mode: str,
    *,
    elapsed_sec: float,
    since_request_sec: float,
    timeout_sec: float,
    retry_sec: float,
) -> ControlTakeoverDecision:
    """确认 ``assist`` 模式已经生效，超时前按固定周期重发请求。

    该函数不会把“模式请求已发布”等同于“控制权已取得”。执行节点只有收到
    仲裁器状态中的 ``mode=assist`` 后才允许发布非零辅助速度。
    """

    values = (elapsed_sec, since_request_sec, timeout_sec, retry_sec)
    if not all(_finite_float(value) is not None for value in values):
        raise ValueError('控制接管时间参数必须是有限数值')
    if elapsed_sec < 0.0 or since_request_sec < 0.0:
        raise ValueError('控制接管经过时间不能为负数')
    if timeout_sec <= 0.0 or retry_sec <= 0.0:
        raise ValueError('控制接管超时和重试周期必须为正数')
    if selected_mode == 'assist':
        return ControlTakeoverDecision(
            ready=True,
            failed=False,
            should_retry=False,
            reason='control_takeover_confirmed',
        )
    if elapsed_sec >= timeout_sec:
        return ControlTakeoverDecision(
            ready=False,
            failed=True,
            should_retry=False,
            reason='control_takeover_timeout',
        )
    return ControlTakeoverDecision(
        ready=False,
        failed=False,
        should_retry=since_request_sec >= retry_sec,
        reason='waiting_for_control_takeover',
    )


def compute_alignment_decision(
    payload: Dict[str, Any],
    config: AlignmentConfig = AlignmentConfig(),
    target_id_override: str = '',
) -> AlignmentDecision:
    """对当前复查目标计算原地转向命令。

    画面左侧误差为负，对应 ROS 正角速度（左转），因此角速度取
    ``-kp * error``。函数只允许原地转向，靠近和横移继续交给导航层避障执行。
    """

    validate_alignment_config(config)
    width = _positive_float(payload.get('image_width'))
    if width is None:
        return _invalid('missing_image_width')
    recommendation = payload.get('view_recommendation')
    if not isinstance(recommendation, dict):
        recommendation = {}
    # 辅助会话开始后由执行节点传入首次确认的目标 ID。后续即使感知策略把
    # 推荐目标切到另一颗球，也必须继续追踪原目标或失败停止，不能暗中换球。
    target_id = _canonical_id(
        target_id_override or recommendation.get('target_id'))
    detections = payload.get('detections_2d')
    if not isinstance(detections, list):
        return _invalid('missing_detections')
    target = _select_target(detections, target_id)
    if target is None:
        return _invalid('target_not_visible', target_id=target_id)
    bbox = target.get('bbox')
    if not isinstance(bbox, dict):
        return _invalid('missing_bbox', target_id=_detection_id(target))
    x_min = _finite_float(bbox.get('x_min'))
    x_max = _finite_float(bbox.get('x_max'))
    if (
        x_min is None or x_max is None or x_min >= x_max
        or x_max <= 0.0 or x_min >= width
    ):
        return _invalid('invalid_bbox', target_id=_detection_id(target))
    center_x = (x_min + x_max) * 0.5
    error = (center_x - width * 0.5) / (width * 0.5)
    if abs(error) <= config.center_tolerance_ratio:
        return AlignmentDecision(
            valid=True,
            centered=True,
            target_id=_detection_id(target),
            center_error_ratio=error,
            angular_z=0.0,
            reason='target_centered',
        )
    magnitude = min(
        config.max_angular_speed,
        max(config.min_angular_speed, abs(config.angular_kp * error)),
    )
    return AlignmentDecision(
        valid=True,
        centered=False,
        target_id=_detection_id(target),
        center_error_ratio=error,
        angular_z=magnitude if error < 0.0 else -magnitude,
        reason='turn_left' if error < 0.0 else 'turn_right',
    )


def _select_target(
    detections: Iterable[Dict[str, Any]], target_id: str,
) -> Optional[Dict[str, Any]]:
    valid = [
        item for item in detections
        if isinstance(item, dict)
        and item.get('track_status') not in (
            'rejected', 'rejected_non_spherical')
    ]
    if target_id:
        for item in valid:
            if target_id in _detection_ids(item):
                return item
        # 推荐目标已离开当前帧时必须停止，不能擅自切到另一个候选，避免
        # 多球场景中辅助转向追错目标。
        return None
    candidates = [
        item for item in valid
        if item.get('requires_reobservation')
        or item.get('track_status') not in (
            'confirmed', 'rejected', 'rejected_non_spherical',
        )
    ]
    if not candidates:
        return None
    return max(
        candidates,
        key=lambda item: float(item.get('confidence', 0.0) or 0.0),
    )


def _detection_ids(item: Dict[str, Any]) -> set:
    values = {
        _canonical_id(item.get('id')),
        _canonical_id(item.get('track_id')),
        _canonical_id(item.get('candidate_id')),
    }
    return {value for value in values if value}


def _detection_id(item: Dict[str, Any]) -> str:
    identities = _detection_ids(item)
    return sorted(identities)[0] if identities else ''


def _canonical_id(value) -> str:
    text = str(value or '').strip()
    return text.split(':', 1)[1] if text.startswith('untracked:') else text


def _positive_float(value) -> Optional[float]:
    number = _finite_float(value)
    if number is None:
        return None
    return number if number > 0.0 else None


def _finite_float(value) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _invalid(reason: str, target_id: str = '') -> AlignmentDecision:
    return AlignmentDecision(
        valid=False,
        centered=False,
        target_id=target_id,
        center_error_ratio=None,
        angular_z=0.0,
        reason=reason,
    )


def validate_alignment_config(config: AlignmentConfig) -> None:
    """在 ROS 节点接入控制话题前也可复用的参数校验。"""

    values = (
        config.center_tolerance_ratio,
        config.angular_kp,
        config.min_angular_speed,
        config.max_angular_speed,
    )
    if not all(_finite_float(value) is not None for value in values):
        raise ValueError('辅助对准参数必须是有限数值')
    if not 0.0 < config.center_tolerance_ratio < 1.0:
        raise ValueError('center_tolerance_ratio 必须在 0 到 1 之间')
    if config.angular_kp <= 0.0:
        raise ValueError('angular_kp 必须为正数')
    if not 0.0 < config.min_angular_speed <= config.max_angular_speed:
        raise ValueError('辅助对准角速度范围无效')
