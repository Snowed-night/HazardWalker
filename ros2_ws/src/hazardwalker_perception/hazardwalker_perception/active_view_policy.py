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

from hazardwalker_perception.localize_hazard import project_output_point_to_image


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


class TransientCandidateMemory:
    """给尚未建立三维轨迹的局部候选分配短时稳定 ID。

    该记忆只做图像空间一对一关联，不创建危险源轨迹、也不参与确认。候选一旦
    关联到正式三维轨迹，仍保留 ``candidate_id`` 作为复查会话别名，正式结果
    始终使用 ``track_id``。
    """

    def __init__(
            self, ttl_s=8.0, min_iou=0.05,
            max_center_shift_ratio=1.5):
        self.ttl_s = max(0.1, float(ttl_s))
        self.min_iou = max(0.0, min(1.0, float(min_iou)))
        self.max_center_shift_ratio = max(
            0.1, float(max_center_shift_ratio),
        )
        self._entries = {}
        self._next_id = 1

    def annotate(self, detections, stamp_sec):
        """返回带 ``candidate_id`` 的拷贝，同帧每个别名最多分配一次。"""

        now = float(stamp_sec)
        self._entries = {
            key: value for key, value in self._entries.items()
            if 0.0 <= now - value['last_seen_sec'] <= self.ttl_s
        }
        result = [dict(item) for item in detections]
        pairs = []
        for detection_index, detection in enumerate(result):
            bbox = _normalized_bbox(detection)
            if bbox is None:
                continue
            for candidate_id, entry in self._entries.items():
                cost = _transient_candidate_cost(
                    bbox,
                    entry['bbox'],
                    self.min_iou,
                    self.max_center_shift_ratio,
                )
                if cost is not None:
                    pairs.append((cost, detection_index, candidate_id))

        assignments = {}
        used_ids = set()
        for _cost, detection_index, candidate_id in sorted(pairs):
            if detection_index in assignments or candidate_id in used_ids:
                continue
            assignments[detection_index] = candidate_id
            used_ids.add(candidate_id)

        for detection_index, detection in enumerate(result):
            bbox = _normalized_bbox(detection)
            if bbox is None:
                continue
            candidate_id = assignments.get(detection_index)
            if candidate_id is None:
                candidate_id = 'candidate-%d' % self._next_id
                self._next_id += 1
            previous_track_id = str(
                self._entries.get(candidate_id, {}).get('track_id') or ''
            ).strip()
            track_id = str(detection.get('track_id') or '').strip()
            self._entries[candidate_id] = {
                'bbox': bbox,
                'last_seen_sec': now,
                'track_id': track_id or previous_track_id,
            }
            detection['candidate_id'] = candidate_id
            detection['candidate_aliases'] = [candidate_id]
            if not track_id and previous_track_id:
                # 只作为下一步三维关联提示，不提前把候选伪装成正式轨迹。
                detection['_candidate_track_id_hint'] = previous_track_id
            if str(detection.get('track_status', '')) == 'untracked':
                detection['id'] = 'untracked:' + candidate_id
        return result

    def remember_track_ids(self, detections):
        """把本帧正式三维关联写回候选记忆，供下一帧跨定位漂移恢复。"""

        for detection in detections:
            candidate_id = str(detection.get('candidate_id') or '').strip()
            track_id = str(detection.get('track_id') or '').strip()
            if not candidate_id or not track_id or candidate_id not in self._entries:
                continue
            self._entries[candidate_id]['track_id'] = track_id


class ActiveViewDirectionMemory:
    """在同一候选复查会话内保持侧移方向，避免越过画面中心后左右振荡。"""

    def __init__(self, ttl_s=12.0, reversal_edge_ratio=0.12):
        self.ttl_s = max(0.1, float(ttl_s))
        self.reversal_edge_ratio = max(
            0.01, min(0.45, float(reversal_edge_ratio)),
        )
        self._entries = {}

    def stabilize(
            self, recommendation, detections, image_width, stamp_sec):
        """返回带方向滞回的建议；目标接近反侧边缘时才允许反向。"""

        now = float(stamp_sec)
        self._entries = {
            key: value for key, value in self._entries.items()
            if 0.0 <= now - value['last_seen_sec'] <= self.ttl_s
        }
        action = str(recommendation.action)
        target_id = str(recommendation.target_id or '')
        lateral_actions = {'move_left', 'move_right'}
        if action not in lateral_actions or not target_id:
            return recommendation

        center_ratio = _candidate_center_ratio(
            detections, target_id, image_width,
        )
        previous = self._entries.get(target_id)
        # 部分可见轮廓的质心偏斜直接描述遮挡边缘在哪一侧，比普通 bbox
        # 中心更可靠；该强证据必须允许反向，否则方向记忆会把目标持续推离
        # 视野。没有质心证据时仍保留中心区域防振荡。
        strong_occlusion_direction = (
            '可见红色轮廓质心' in str(recommendation.reason)
        )
        if (
            previous
            and previous['action'] in lateral_actions
            and previous['action'] != action
            and not strong_occlusion_direction
            and center_ratio is not None
            and self.reversal_edge_ratio
            < center_ratio
            < 1.0 - self.reversal_edge_ratio
        ):
            action = previous['action']
            recommendation = ViewRecommendation(
                action,
                recommendation.reason
                + ' 为保持视差方向并避免中心线附近左右振荡，本轮沿用上一侧移方向。',
                recommendation.priority,
                target_id,
            )
        self._entries[target_id] = {
            'action': action,
            'last_seen_sec': now,
        }
        return recommendation


def attach_candidate_aliases_to_hazards(hazards, detections):
    """把当前二维候选别名附到对应三维轨迹快照，供复查闭环解析。"""

    aliases_by_track = {}
    for detection in detections:
        track_id = str(detection.get('track_id') or '').strip()
        candidate_id = str(detection.get('candidate_id') or '').strip()
        if track_id and candidate_id:
            aliases_by_track.setdefault(track_id, set()).add(candidate_id)
    result = []
    for hazard in hazards:
        item = dict(hazard)
        track_id = str(item.get('track_id', item.get('id')) or '').strip()
        aliases = sorted(aliases_by_track.get(track_id, set()))
        if aliases:
            item['candidate_ids'] = aliases
        result.append(item)
    return result


def choose_active_view_action(detections, image_width, image_height, config=None):
    """为当前帧选择优先级最高的一次重观察动作。

    ``detections`` 兼容 HSV 节点发布的 dict；缺失形状或深度字段时按保守值处理。
    动作语义：``turn_left/right`` 均表示朝候选所在方向转向，使贴边目标回到画面中央。
    """

    policy = config or ActiveViewPolicyConfig()
    normalized = [_normalize_detection(item, index) for index, item in enumerate(detections, start=1)]
    if not normalized:
        return ViewRecommendation('continue_exploring', '当前帧没有红球候选。', 0)

    # 必须逐个检查紧急候选。若先选最高置信度目标，完整大球会长期压住同帧
    # partial/粘连小球，使后者虽被检出却永远得不到导航侧复查机会。
    urgent_actions = []
    for candidate in normalized:
        action = _urgent_target_action(
            candidate, image_width, image_height, policy,
        )
        if action is not None:
            urgent_actions.append((action, _target_score(candidate, policy)))
    if urgent_actions:
        return max(
            urgent_actions,
            key=lambda item: (item[0].priority, item[1]),
        )[0]

    dense_action = _dense_target_action(normalized, image_width, policy)
    if dense_action:
        return dense_action

    target = max(normalized, key=lambda item: _target_score(item, policy))
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


def _urgent_target_action(target, image_width, image_height, policy):
    """返回单候选的高优先级复查动作；普通稳定候选返回 ``None``。"""

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
        # 极小/远距离局部弧段直接横移容易立刻丢出视场，而且可用视差仍不足。
        # 先把已居中的候选放大到可稳定定位的尺度，再获取侧视；贴边候选已由
        # 上面的 edge_action 先行居中。这里仍只输出语义建议，不控制机器人。
        if (
            target['red_pixel_count'] < policy.min_red_pixel_count
            or target['bbox_area_px'] < policy.min_bbox_area_px
        ):
            return ViewRecommendation(
                'move_forward',
                '局部候选面积过小，先靠近扩大红色弧段，再横移获取独立侧视。',
                96,
                target['id'],
            )
        if target['depth_m'] is not None and target['depth_m'] > policy.far_distance_m:
            return ViewRecommendation(
                'move_forward',
                '局部候选距离较远，先靠近提高像素与深度稳定性，再横移复查。',
                96,
                target['id'],
            )
        return _lateral_action(
            target, image_width, 92, '候选局部可见或可能合并，沿目标所在侧横移后复查。',
        )
    return None


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


def project_tracks_for_image_association(
        tracks, camera_to_output, intrinsics, current_stamp_sec=0.0,
        max_track_age_s=2.0, sphere_radius_m=0.15,
        min_depth_m=0.25, max_depth_m=20.0,
        camera_axis_convention='optical_z_forward'):
    """把近期世界轨迹投影成图像门控，供 partial 保持稳定 target_id。"""

    projected = []
    if camera_to_output is None or intrinsics is None:
        return projected
    for track in tracks:
        age_s = (
            float(current_stamp_sec) - float(track.last_seen_sec)
            if current_stamp_sec and track.last_seen_sec else 0.0
        )
        if age_s < 0.0 or age_s > max(0.0, float(max_track_age_s)):
            continue
        projection = project_output_point_to_image(
            track.position,
            camera_to_output,
            intrinsics,
            camera_axis_convention=camera_axis_convention,
        )
        if projection is None:
            continue
        center_u, center_v, depth_m = projection
        if depth_m < float(min_depth_m) or depth_m > float(max_depth_m):
            continue
        radius_px = (
            (float(intrinsics.fx) + float(intrinsics.fy)) * 0.5
            * float(sphere_radius_m) / depth_m
        )
        if not math.isfinite(radius_px) or radius_px <= 0.0:
            continue
        projected.append({
            'track_id': str(track.track_id),
            'center_u': float(center_u),
            'center_v': float(center_v),
            'radius_px': float(radius_px),
            'depth_m': float(depth_m),
        })
    return projected


def annotate_detections_with_tracks(
        detections, tracks, merge_distance_m, projected_tracks=None):
    """把当前二维候选关联到稳定三维轨迹，供主动复查使用稳定 target_id。

    已拒绝非球体也参与关联，使同一圆柱不会在下一帧重新变成“新候选”。这里
    只使用感知进程由合法 RGB-D/TF 建立的轨迹，不读取真值。
    """

    result = [dict(detection) for detection in detections]
    threshold = max(0.0, float(merge_distance_m))
    track_by_id = {str(track.track_id): track for track in tracks}
    projection_by_id = {
        str(item.get('track_id')): item for item in (projected_tracks or [])
    }
    candidates_by_detection = {}
    for detection_index, item in enumerate(result):
        position = item.get('localized_position')
        if position is None:
            position = item.get('position')
        pairs = []
        for track_id, track in track_by_id.items():
            best_cost = None
            association = ''
            if (
                threshold > 0.0
                and isinstance(position, (list, tuple))
                and len(position) == 3
            ):
                distance = math.sqrt(sum(
                    (float(position[index]) - float(track.position[index])) ** 2
                    for index in range(3)
                ))
                if distance <= threshold:
                    best_cost = distance / threshold
                    association = 'world_distance'
            projection = projection_by_id.get(track_id)
            projection_cost = _projected_bbox_association_cost(item, projection)
            if projection_cost is not None and (
                best_cost is None or projection_cost < best_cost
            ):
                best_cost = projection_cost
                association = 'image_projection'
            if best_cost is not None:
                pairs.append((best_cost, track_id, association))
        pairs.sort(key=lambda pair: pair[0])
        # 两条世界轨迹在图像中几乎重合时保持未关联，不能猜错 ID 后让导航
        # 对另一个球消耗复查预算。
        if len(pairs) >= 2 and pairs[1][0] - pairs[0][0] < 0.20:
            pairs = []
        candidates_by_detection[detection_index] = pairs

    assignments = {}
    used_track_ids = set()
    all_pairs = sorted(
        (
            (cost, detection_index, track_id, association)
            for detection_index, pairs in candidates_by_detection.items()
            for cost, track_id, association in pairs
        ),
        key=lambda pair: pair[0],
    )
    for _cost, detection_index, track_id, association in all_pairs:
        if detection_index in assignments or track_id in used_track_ids:
            continue
        assignments[detection_index] = (track_id, association)
        used_track_ids.add(track_id)

    for detection_index, item in enumerate(result):
        assignment = assignments.get(detection_index)
        if assignment is None:
            item['track_id'] = ''
            item['track_status'] = 'untracked'
            item['id'] = 'untracked:%s' % item.get('id', '')
            item['track_association'] = 'none'
            continue
        track_id, association = assignment
        track = track_by_id[track_id]
        item['track_id'] = track_id
        item['track_status'] = str(track.status)
        item['track_association'] = association
        # 策略的 target_id 必须跨帧稳定，导航才能限制单目标复查预算。
        item['id'] = track_id
    return result


def _projected_bbox_association_cost(detection, projection):
    """计算 partial bbox 与预测完整球框的归一化代价；不满足门槛返回 None。"""

    if not projection:
        return None
    bbox = detection.get('bbox', detection)
    try:
        x_min = float(bbox['x_min'])
        y_min = float(bbox['y_min'])
        x_max = float(bbox['x_max'])
        y_max = float(bbox['y_max'])
        center_u = float(projection['center_u'])
        center_v = float(projection['center_v'])
        radius_px = float(projection['radius_px'])
    except (KeyError, TypeError, ValueError):
        return None
    if radius_px <= 0.0 or x_max < x_min or y_max < y_min:
        return None
    candidate_center_u = (x_min + x_max) * 0.5
    candidate_center_v = (y_min + y_max) * 0.5
    center_distance = math.hypot(
        candidate_center_u - center_u, candidate_center_v - center_v,
    )
    center_gate_px = max(20.0, 1.5 * radius_px)
    if center_distance > center_gate_px:
        return None

    half_size = radius_px * 1.25
    predicted = {
        'x_min': center_u - half_size,
        'y_min': center_v - half_size,
        'x_max': center_u + half_size,
        'y_max': center_v + half_size,
    }
    intersection_width = max(
        0.0, min(x_max, predicted['x_max']) - max(x_min, predicted['x_min']),
    )
    intersection_height = max(
        0.0, min(y_max, predicted['y_max']) - max(y_min, predicted['y_min']),
    )
    candidate_area = max(1.0, (x_max - x_min) * (y_max - y_min))
    overlap_ratio = intersection_width * intersection_height / candidate_area
    if overlap_ratio < 0.50:
        return None
    return 0.65 * center_distance / center_gate_px + 0.35 * (1.0 - overlap_ratio)


def _normalize_detection(item, index):
    bbox = item.get('bbox', item)
    x_min = int(bbox.get('x_min', 0))
    y_min = int(bbox.get('y_min', 0))
    x_max = max(x_min, int(bbox.get('x_max', x_min)))
    y_max = max(y_min, int(bbox.get('y_max', y_min)))
    shape = item.get('shape', {})
    # ROS2 正式检测节点发布 ``raw_surface_depth_m``；历史离线记录使用
    # ``depth_m``。两者均表示候选可见表面深度，策略只用于远近分级。
    depth = item.get('depth_m', item.get('raw_surface_depth_m'))
    depth_shape = item.get('depth_shape', {})
    curvature = depth_shape.get('curvature_m')
    diameter = item.get('apparent_diameter_m')
    normalized_curvature = None
    if curvature is not None and diameter is not None and float(diameter) > 0.0:
        normalized_curvature = float(curvature) / float(diameter)
    return {
        # 已建立三维轨迹后仍沿用候选别名，避免一次复查在升级瞬间被当作新目标；
        # 正式危险源输出仍只使用 track_id。
        'id': str(item.get('candidate_id', item.get('id', index))),
        'x_min': x_min,
        'y_min': y_min,
        'x_max': x_max,
        'y_max': y_max,
        'bbox_area_px': (x_max - x_min + 1) * (y_max - y_min + 1),
        'red_pixel_count': int(item.get('red_pixel_count', 0)),
        'circularity': float(shape.get('circularity', item.get('circularity', 0.0))),
        'visible_centroid_x_ratio': float(
            shape.get('visible_centroid_x_ratio', 0.5)
        ),
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
    centroid_ratio = float(target.get('visible_centroid_x_ratio', 0.5))
    if target['requires_reobservation'] and centroid_ratio < 0.47:
        # 右侧圆弧被竖直遮挡后，红色面积集中在 bbox 左半部；向右侧移才能
        # 绕过前景边缘。反向情况同理。这里只决定观察方向，不改变确认门槛。
        action = 'move_right'
        reason += ' 可见红色轮廓质心偏左，推断右侧绕障更快。'
    elif target['requires_reobservation'] and centroid_ratio > 0.53:
        action = 'move_left'
        reason += ' 可见红色轮廓质心偏右，推断左侧绕障更快。'
    else:
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


def _normalized_bbox(detection):
    bbox = detection.get('bbox', detection)
    try:
        x_min = float(bbox['x_min'])
        y_min = float(bbox['y_min'])
        x_max = float(bbox['x_max'])
        y_max = float(bbox['y_max'])
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (
            x_min, y_min, x_max, y_max)):
        return None
    if x_max < x_min or y_max < y_min:
        return None
    return {
        'x_min': x_min,
        'y_min': y_min,
        'x_max': x_max,
        'y_max': y_max,
    }


def _candidate_center_ratio(detections, target_id, image_width):
    """查找候选当前水平中心；只用于方向滞回，不改变目标选择。"""

    width = max(1.0, float(image_width))
    for detection in detections:
        identities = {
            str(detection.get('candidate_id') or ''),
            str(detection.get('track_id') or ''),
            str(detection.get('id') or ''),
        }
        if str(target_id) not in identities:
            continue
        bbox = _normalized_bbox(detection)
        if bbox is None:
            return None
        return 0.5 * (bbox['x_min'] + bbox['x_max']) / width
    return None


def _transient_candidate_cost(
        current, previous, min_iou, max_center_shift_ratio):
    """返回短时图像关联代价；位移过大且无重叠时拒绝关联。"""

    overlap = bbox_iou(current, previous)
    current_width = current['x_max'] - current['x_min'] + 1.0
    current_height = current['y_max'] - current['y_min'] + 1.0
    previous_width = previous['x_max'] - previous['x_min'] + 1.0
    previous_height = previous['y_max'] - previous['y_min'] + 1.0
    scale = max(
        12.0,
        math.hypot(current_width, current_height),
        math.hypot(previous_width, previous_height),
    )
    current_center = (
        0.5 * (current['x_min'] + current['x_max']),
        0.5 * (current['y_min'] + current['y_max']),
    )
    previous_center = (
        0.5 * (previous['x_min'] + previous['x_max']),
        0.5 * (previous['y_min'] + previous['y_max']),
    )
    shift_ratio = math.hypot(
        current_center[0] - previous_center[0],
        current_center[1] - previous_center[1],
    ) / scale
    if overlap < min_iou and shift_ratio > max_center_shift_ratio:
        return None
    overlap_cost = 1.0 - overlap
    motion_cost = shift_ratio / max_center_shift_ratio
    return min(overlap_cost, motion_cost)
