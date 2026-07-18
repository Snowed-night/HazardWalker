"""危险源主动重观察策略纯函数。

所属组：感知定位组。
文件作用：
根据二维红球候选的贴边情况、面积、红色像素数、圆度和可选深度，
为决策/导航模块给出一次可解释的重观察建议。
当前实现边界：
本模块不发布速度指令，也不负责多帧确认；它只输出建议动作和理由。
验证方式：
运行 ``python scripts/run_offline_tests.py``，查看 ``test_active_view_policy.py``。
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ActiveViewPolicyConfig:
    """主动观察的保守阈值，可在仿真实验后再写入参数配置。"""

    edge_margin_ratio: float = 0.05
    min_bbox_area_px: int = 900
    min_red_pixel_count: int = 300
    min_circularity: float = 0.72
    min_confidence: float = 0.70
    far_distance_m: float = 5.0
    dense_iou_threshold: float = 0.15
    min_normalized_depth_curvature: float = 0.10
    max_normalized_depth_curvature: float = 0.30


@dataclass(frozen=True)
class ViewRecommendation:
    """给决策状态机使用的一次观察建议。"""

    action: str
    reason: str
    priority: int
    target_id: str = ''

    def to_dict(self):
        """转成可直接写入动态记录 JSON 的稳定字段。"""

        return {
            'action': self.action,
            'reason': self.reason,
            'priority': self.priority,
            'target_id': self.target_id,
        }


def choose_active_view_action(detections, image_width, image_height, config=None):
    """为当前帧选择优先级最高的一次重观察动作。

    ``detections`` 兼容 HSV 节点发布的 dict；缺失形状或深度字段时按保守值处理。
    动作语义：``turn_left/right`` 均表示朝候选所在方向转向，使贴边目标回到画面中央。
    """

    policy = config or ActiveViewPolicyConfig()
    normalized = [_normalize_detection(item, index) for index, item in enumerate(detections, start=1)]
    if not normalized:
        return ViewRecommendation('continue_exploring', '当前帧没有红球候选。', 0)

    target = max(normalized, key=lambda item: _target_score(item, policy))
    edge_action = _edge_action(target, image_width, image_height, policy)
    if edge_action:
        return edge_action

    if target['depth_shape_status'] in ('flat', 'anisotropic', 'non_spherical'):
        return _lateral_action(
            target, image_width, 94,
            '深度轮廓为平面或单轴曲面，疑似红色非球体；从侧面复查轮廓变化后再决定是否丢弃。',
        )

    if target['normalized_depth_curvature'] is not None and not (
        policy.min_normalized_depth_curvature
        <= target['normalized_depth_curvature']
        <= policy.max_normalized_depth_curvature
    ):
        return _lateral_action(
            target, image_width, 93,
            '深度曲率不在球体稳定区间，疑似圆锥端面、扁平物或深度异常；必须侧向复查。',
        )

    if target['requires_reobservation']:
        return _lateral_action(
            target, image_width, 92, '候选局部可见或可能合并，沿目标所在侧横移后复查。',
        )

    dense_action = _dense_target_action(normalized, image_width, policy)
    if dense_action:
        return dense_action

    if target['red_pixel_count'] < policy.min_red_pixel_count or target['bbox_area_px'] < policy.min_bbox_area_px:
        return ViewRecommendation('move_forward', '候选红色面积过小，建议靠近后重新观察。', 80, target['id'])

    if target['depth_m'] is not None and target['depth_m'] > policy.far_distance_m:
        return ViewRecommendation('move_forward', '候选距离较远，建议靠近后提高像素覆盖率。', 75, target['id'])

    if target['circularity'] < policy.min_circularity:
        return _lateral_action(
            target, image_width, 70, '候选圆度不稳定，横移后从侧面复查轮廓是否仍为球形。',
        )

    if target['confidence'] < policy.min_confidence:
        return ViewRecommendation('hold_observation', '候选置信度偏低，建议保持视角采集更多帧。', 60, target['id'])

    return _lateral_action(
        target, image_width, 55,
        '单视角圆形仍可能是圆柱或圆锥端面；完成当前稳定帧后获取独立侧视再确认。',
    )


def bbox_iou(a, b):
    """计算两个 ``x_min/y_min/x_max/y_max`` 检测框的 IoU。"""

    x_min = max(a['x_min'], b['x_min'])
    y_min = max(a['y_min'], b['y_min'])
    x_max = min(a['x_max'], b['x_max'])
    y_max = min(a['y_max'], b['y_max'])
    if x_max < x_min or y_max < y_min:
        return 0.0
    intersection = float((x_max - x_min + 1) * (y_max - y_min + 1))
    area_a = float((a['x_max'] - a['x_min'] + 1) * (a['y_max'] - a['y_min'] + 1))
    area_b = float((b['x_max'] - b['x_min'] + 1) * (b['y_max'] - b['y_min'] + 1))
    return intersection / max(area_a + area_b - intersection, 1.0)


def annotate_detections_with_tracks(detections, tracks, merge_distance_m):
    """把当前二维候选关联到稳定三维轨迹，供主动复查使用稳定 target_id。

    已拒绝非球体也参与关联，使同一圆柱不会在下一帧重新变成“新候选”。这里
    只使用感知进程由合法 RGB-D/TF 建立的轨迹，不读取真值。
    """

    result = []
    threshold = max(0.0, float(merge_distance_m))
    for detection in detections:
        item = dict(detection)
        position = item.get('localized_position')
        nearest = None
        nearest_distance = None
        if isinstance(position, (list, tuple)) and len(position) == 3:
            for track in tracks:
                distance = math.sqrt(sum(
                    (float(position[index]) - float(track.position[index])) ** 2
                    for index in range(3)
                ))
                if distance > threshold:
                    continue
                if nearest_distance is None or distance < nearest_distance:
                    nearest = track
                    nearest_distance = distance
        if nearest is not None:
            item['track_id'] = str(nearest.track_id)
            item['track_status'] = str(nearest.status)
            # 策略的 target_id 必须跨帧稳定，导航才能限制单目标复查预算。
            item['id'] = str(nearest.track_id)
        else:
            item['track_id'] = ''
            item['track_status'] = 'untracked'
            item['id'] = 'untracked:%s' % item.get('id', '')
        result.append(item)
    return result


def _normalize_detection(item, index):
    bbox = item.get('bbox', item)
    x_min = int(bbox.get('x_min', 0))
    y_min = int(bbox.get('y_min', 0))
    x_max = max(x_min, int(bbox.get('x_max', x_min)))
    y_max = max(y_min, int(bbox.get('y_max', y_min)))
    shape = item.get('shape', {})
    depth = item.get('depth_m')
    depth_shape = item.get('depth_shape', {})
    curvature = depth_shape.get('curvature_m')
    diameter = item.get('apparent_diameter_m')
    normalized_curvature = None
    if curvature is not None and diameter is not None and float(diameter) > 0.0:
        normalized_curvature = float(curvature) / float(diameter)
    return {
        'id': str(item.get('id', index)),
        'x_min': x_min,
        'y_min': y_min,
        'x_max': x_max,
        'y_max': y_max,
        'bbox_area_px': (x_max - x_min + 1) * (y_max - y_min + 1),
        'red_pixel_count': int(item.get('red_pixel_count', 0)),
        'circularity': float(shape.get('circularity', item.get('circularity', 0.0))),
        'confidence': float(item.get('confidence', 0.0)),
        'depth_m': float(depth) if depth is not None else None,
        'depth_shape_status': str(depth_shape.get('status', 'unknown')),
        'normalized_depth_curvature': normalized_curvature,
        'requires_reobservation': bool(item.get('requires_reobservation', False)),
    }


def _target_score(item, policy):
    score = item['confidence'] + item['circularity'] * 0.25
    score += min(item['red_pixel_count'] / float(max(policy.min_red_pixel_count, 1)), 1.0) * 0.15
    if item['depth_m'] is not None:
        score += max(0.0, 1.0 - item['depth_m'] / max(policy.far_distance_m, 0.1)) * 0.10
    return score


def _dense_target_action(detections, image_width, policy):
    for left_index, first in enumerate(detections):
        for second in detections[left_index + 1:]:
            if bbox_iou(first, second) >= policy.dense_iou_threshold:
                target = max((first, second), key=lambda item: _target_score(item, policy))
                return _lateral_action(
                    target, image_width, 90, '多个候选框重叠，沿目标所在侧横移以产生运动视差。',
                )
    return None


def _lateral_action(target, image_width, priority, reason):
    """把模糊的“横移”落成可执行方向，并尽量让目标留在视场内。"""

    center_x = (target['x_min'] + target['x_max']) / 2.0
    action = 'move_left' if center_x <= max(1, image_width) / 2.0 else 'move_right'
    direction = '左' if action == 'move_left' else '右'
    return ViewRecommendation(action, f'{reason} 当前选择向{direction}横移。', priority, target['id'])


def _edge_action(target, image_width, image_height, policy):
    if image_width <= 0 or image_height <= 0:
        return None
    margin_x = max(1, int(image_width * policy.edge_margin_ratio))
    margin_y = max(1, int(image_height * policy.edge_margin_ratio))
    if target['x_min'] <= margin_x:
        return ViewRecommendation('turn_left', '候选框贴近左边界，建议向目标方向左转复查。', 100, target['id'])
    if target['x_max'] >= image_width - 1 - margin_x:
        return ViewRecommendation('turn_right', '候选框贴近右边界，建议向目标方向右转复查。', 100, target['id'])
    if target['y_min'] <= margin_y or target['y_max'] >= image_height - 1 - margin_y:
        return _lateral_action(
            target, image_width, 95,
            '候选框贴近上下边界且当前相机俯仰固定，改用侧向视差复查。',
        )
    return None
