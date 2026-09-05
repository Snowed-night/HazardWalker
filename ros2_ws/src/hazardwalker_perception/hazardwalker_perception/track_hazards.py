"""危险源多帧确认和空间去重纯函数。

所属组：感知组。
文件作用：
把每帧三维定位结果合并成稳定危险源列表，降低重复上报和单帧误检。
当前实现边界：
按三维欧氏距离做最近邻合并，并同时检查深度形状反证、目标三维尺寸
稳定性和可配置的离散视角门槛。赛场仅有红色正方体干扰时可启用单视角
RGB-D 球面确认；存在圆柱或圆盘干扰时仍可恢复多视角门槛。
暂不做卡尔曼滤波或跨相机重识别，后续可在保持输出字段稳定的前提下升级。
验证方式：
使用 tests/offline/test_track_hazards.py 构造多帧观测，验证合并、确认、missed 计数和新目标创建。
"""

from dataclasses import dataclass, field
import math
from typing import Optional


@dataclass
class HazardObservation:
    """单帧危险源三维观测。"""

    position: tuple
    confidence: float
    stamp_sec: float = 0.0
    source_id: str = ''
    view_id: str = ''
    confirmation_eligible: bool = True
    depth_shape_status: str = 'unknown'
    # 使用 Optional 而非 PEP 604，确保 ROS1 Noetic 默认 Python 3.8 可解析本纯函数模块。
    apparent_diameter_m: Optional[float] = None
    aspect_ratio: Optional[float] = None
    depth_curvature_m: Optional[float] = None
    # 相机到候选三维中心的水平视线方位。它让确认器能区分“向前靠近”与
    # “真正绕到侧面复查”；未提供时保持兼容旧的纯 2D/离线链路。
    view_bearing_rad: Optional[float] = None
    # 由上一时刻轨迹投影与当前二维框的唯一匹配产生；不是目标真值。
    # 有提示时可跨越 SLAM 世界坐标漂移恢复同一轨迹，同时仍由一帧一轨约束
    # 防止两个同时可见红球合并。
    track_id_hint: Optional[int] = None


@dataclass
class HazardTrack:
    """跨帧合并后的危险源轨迹。"""

    track_id: int
    position: tuple
    confidence: float
    status: str = 'tentative'
    observation_count: int = 1
    missed_count: int = 0
    first_seen_sec: float = 0.0
    last_seen_sec: float = 0.0
    source_ids: list = field(default_factory=list)
    view_ids: list = field(default_factory=list)
    eligible_observation_count: int = 0
    eligible_view_ids: list = field(default_factory=list)
    # 历史字段名保持兼容；除 flat 外也记录 anisotropic 等明确非球面视角。
    flat_view_ids: list = field(default_factory=list)
    # 记录有深度正证据的离散视角。仅 RGB 圆形或深度未知不能替代球面几何确认。
    spherical_view_ids: list = field(default_factory=list)
    apparent_diameters_m: list = field(default_factory=list)
    aspect_ratios: list = field(default_factory=list)
    depth_curvatures_m: list = field(default_factory=list)
    diameters_by_view: dict = field(default_factory=dict)
    aspects_by_view: dict = field(default_factory=dict)
    curvatures_by_view: dict = field(default_factory=dict)
    curvature_ratios_by_view: dict = field(default_factory=dict)
    bearings_by_view: dict = field(default_factory=dict)
    # 每个离散视角分别累计三维位置，再对各视角均值取坐标中位数。这样相机在
    # 某一视角停留较久时不会凭帧数压倒其他视角，也能抑制长距离复查中的
    # SLAM 单向漂移；运行期只使用公开 TF/深度观测，不读取测试真值。
    positions_by_view: dict = field(default_factory=dict)
    evidence_status: str = 'collecting_views'


@dataclass
class HazardTrackerConfig:
    """危险源跟踪参数。"""

    confirm_observation_count: int = 3
    reject_after_missed_count: int = 10
    merge_distance_m: float = 0.5
    min_distinct_views: int = 1
    max_apparent_diameter_cv: float = 0.35
    # 官方红球半径为 0.15 m。设为正值时，除要求多视角尺寸稳定外，还要求
    # 中位表观直径接近该物理尺寸；零表示兼容未知尺寸目标的历史纯函数链路。
    expected_sphere_diameter_m: float = 0.0
    max_sphere_diameter_relative_error: float = 0.35
    min_non_spherical_views_to_reject: int = 2
    # 正式 RGB-D 模式可设为 2：至少两个独立视角的深度形状都应支持球面。
    # 默认为 0 以兼容不含深度的历史离线纯函数测试；ROS 节点会显式启用该门槛。
    min_spherical_views_for_confirm: int = 0
    min_multiview_aspect_ratio: float = 0.88
    # Gazebo 深度边缘会使同一球体的曲率在不同视角波动较大，因此只把极端
    # 不稳定曲率作为反证；同时要求曲率相对表观直径达到下限，用来排除正面
    # 看似圆形、实际深度几乎为平面的扁椭球/圆片。
    max_depth_curvature_cv: float = 0.65
    min_normalized_depth_curvature: float = 0.10
    max_median_normalized_depth_curvature: float = 0.30
    # 为零表示兼容旧链路，不强制方位视差；正式 RGB-D 运行应设置约 20--30 度。
    min_view_bearing_span_deg: float = 0.0
    # ``view_median`` 适合低漂移定位；``earliest_view_anchor`` 适合官方当前
    # scan-matching 位姿在长距离侧移中单向漂移的情况。后者仅锚定首个完整、
    # 可确认 RGB-D 视角，不会使用测试真值。
    position_fusion_mode: str = 'view_median'
    # 常规合并门仍保持 0.5 m。只有场内恰好一个兼容球面轨迹时，才允许在
    # SLAM 漂移下用更宽距离恢复；多个候选轨迹时一律不猜。
    single_track_reacquire_distance_m: float = 0.0
    single_track_reacquire_diameter_relative_error: float = 0.25
    # 同一 RGB-D 帧偶尔会为一个球产生两个几乎重合的球面轮廓。若直接按
    # “一帧一轨”处理，第二个轮廓会被误建为新轨迹。该门限只在同一次
    # update 内、两者均有球面深度正证据且直径一致时生效；零表示关闭。
    same_frame_duplicate_distance_m: float = 0.0
    same_frame_duplicate_diameter_relative_error: float = 0.25


class HazardTracker:
    """维护危险源轨迹状态，逐帧更新观测并输出去重结果。"""

    def __init__(self, config=None):
        self.config = config or HazardTrackerConfig()
        self.tracks = []
        self._next_track_id = 1

    def update(self, observations, stamp_sec=0.0):
        normalized_observations = []
        for observation in observations:
            normalized = _normalize_observation(observation, stamp_sec)
            duplicate_index = _same_frame_duplicate_index(
                normalized_observations, normalized, self.config,
            )
            if duplicate_index is None:
                normalized_observations.append(normalized)
            elif (
                float(normalized.confidence)
                > float(normalized_observations[duplicate_index].confidence)
            ):
                # 保留同帧重复轮廓中置信度更高的一条，不增加观测数或轨迹数。
                normalized_observations[duplicate_index] = normalized

        matched_track_ids = set()
        for normalized in normalized_observations:
            track = self._find_matching_track(normalized, matched_track_ids)
            if track is None:
                track = self._create_track(normalized)
            else:
                _merge_observation_into_track(
                    track, normalized, self.config.position_fusion_mode,
                )
            matched_track_ids.add(track.track_id)

        for track in self.tracks:
            if track.track_id not in matched_track_ids:
                track.missed_count += 1

        self._refresh_statuses()
        return self.active_tracks()

    def active_tracks(self):
        return [
            track for track in self.tracks
            if not track.status.startswith('rejected')
        ]

    def published_tracks(self):
        """返回完整状态快照，包含用于撤销旧确认结果的 rejected 轨迹。

        `active_tracks()` 仍供复查策略和最终结果筛选使用；对外状态话题必须发布
        已拒绝轨迹，否则下游按 ID 聚合时会永久保留该轨迹较早的 confirmed 状态。
        """

        return list(self.tracks)

    def confirmed_tracks(self):
        return [
            track for track in self.active_tracks()
            if track.status == 'confirmed'
        ]

    def to_hazard_dicts(self):
        return [track_to_hazard_dict(track) for track in self.active_tracks()]

    def _find_matching_track(self, observation, already_matched):
        hinted_track_id = observation.track_id_hint
        if hinted_track_id is not None:
            for track in self.tracks:
                if (
                    track.track_id == int(hinted_track_id)
                    and track.track_id not in already_matched
                    and track.status != 'rejected'
                ):
                    return track
        best_track = None
        best_distance = None
        # 明确非球体轨迹仍需参与空间关联。否则第二个反证视角刚把圆柱标为
        # rejected_non_spherical，下一帧同一圆柱就会被创建成“全新候选”，
        # 导致导航无限复查并可能在噪声下重新确认。普通 lost_track 才不再复用。
        for track in self.tracks:
            if track.status == 'rejected':
                continue
            if track.track_id in already_matched:
                continue
            distance = distance_m(track.position, observation.position)
            if distance > self.config.merge_distance_m:
                continue
            if best_distance is None or distance < best_distance:
                best_track = track
                best_distance = distance
        if best_track is not None:
            return best_track

        reacquire_distance = max(
            0.0, float(self.config.single_track_reacquire_distance_m),
        )
        if (
            reacquire_distance <= self.config.merge_distance_m
            or observation.depth_shape_status != 'spherical'
            or not _valid_diameter(observation.apparent_diameter_m)
        ):
            return None
        compatible = []
        for track in self.tracks:
            if (
                track.status in ('rejected', 'rejected_non_spherical')
                or track.track_id in already_matched
                or track.missed_count <= 0
                or not track.spherical_view_ids
            ):
                continue
            if distance_m(track.position, observation.position) > reacquire_distance:
                continue
            if not _reacquire_diameter_compatible(
                track,
                observation.apparent_diameter_m,
                self.config.single_track_reacquire_diameter_relative_error,
            ):
                continue
            compatible.append(track)
        return compatible[0] if len(compatible) == 1 else None

    def _create_track(self, observation):
        track = HazardTrack(
            track_id=self._next_track_id,
            position=tuple(float(v) for v in observation.position),
            confidence=float(observation.confidence),
            observation_count=1,
            missed_count=0,
            first_seen_sec=float(observation.stamp_sec),
            last_seen_sec=float(observation.stamp_sec),
            source_ids=[observation.source_id] if observation.source_id else [],
            view_ids=[observation.view_id] if observation.view_id else [],
            eligible_observation_count=1 if observation.confirmation_eligible else 0,
            eligible_view_ids=(
                [observation.view_id]
                if observation.confirmation_eligible and observation.view_id else []
            ),
            flat_view_ids=(
                [observation.view_id]
                if _is_non_spherical_depth_status(observation.depth_shape_status)
                and observation.view_id else []
            ),
            spherical_view_ids=(
                [observation.view_id]
                if (observation.confirmation_eligible
                    and observation.depth_shape_status == 'spherical'
                    and observation.view_id) else []
            ),
            apparent_diameters_m=(
                [float(observation.apparent_diameter_m)]
                if (observation.confirmation_eligible
                    and _valid_diameter(observation.apparent_diameter_m)) else []
            ),
            aspect_ratios=(
                [float(observation.aspect_ratio)]
                if (observation.confirmation_eligible
                    and _valid_aspect_ratio(observation.aspect_ratio)) else []
            ),
            depth_curvatures_m=(
                [float(observation.depth_curvature_m)]
                if (observation.confirmation_eligible
                    and _valid_positive(observation.depth_curvature_m)) else []
            ),
            diameters_by_view=_initial_view_evidence(
                observation.view_id, observation.apparent_diameter_m, _valid_diameter,
            ) if observation.confirmation_eligible else {},
            aspects_by_view=_initial_view_evidence(
                observation.view_id, observation.aspect_ratio, _valid_aspect_ratio,
            ) if observation.confirmation_eligible else {},
            curvatures_by_view=_initial_view_evidence(
                observation.view_id, observation.depth_curvature_m, _valid_positive,
            ) if observation.confirmation_eligible else {},
            curvature_ratios_by_view=_initial_curvature_ratio_evidence(observation),
            bearings_by_view=_initial_view_evidence(
                observation.view_id, observation.view_bearing_rad, _valid_bearing,
            ) if observation.confirmation_eligible else {},
            positions_by_view=(
                _initial_position_evidence(observation)
                if observation.confirmation_eligible else {}
            ),
        )
        self._next_track_id += 1
        self.tracks.append(track)
        return track

    def _refresh_statuses(self):
        for track in self.tracks:
            if track.status == 'rejected_non_spherical':
                # 两个独立稳定视角已经给出明确非球面证据后，本轮任务内永久拒绝。
                # 后续正面圆形投影可能只是同一圆柱/圆锥的端面，绝不能用更多
                # RGB 圆形帧把已拒绝物体“复活”为 confirmed。
                continue
            if track.missed_count >= self.config.reject_after_missed_count:
                if (track.status == 'confirmed'
                        and track.missed_count < self.config.reject_after_missed_count * 10):
                    # 已确认危险源短时离开视场时保留，避免一次转向就从任务结果消失。
                    continue
                track.status = 'rejected'
                track.evidence_status = 'lost_track'
                continue

            eligible_views = _eligible_distinct_view_count(track)
            flat_views = len(track.flat_view_ids)
            spherical_views = len(track.spherical_view_ids)
            diameter_cv = _coefficient_of_variation(_view_medians(track.diameters_by_view))
            representative_diameter_m = _median(
                _view_medians(track.diameters_by_view), default=0.0,
            )
            diameter_prior_ok = _matches_expected_diameter(
                representative_diameter_m,
                self.config.expected_sphere_diameter_m,
                self.config.max_sphere_diameter_relative_error,
            )
            representative_aspect_ratio = min(_view_medians(track.aspects_by_view), default=1.0)
            curvature_cv = _coefficient_of_variation(_view_medians(track.curvatures_by_view))
            normalized_curvatures = _normalized_curvature_by_view(track)
            normalized_curvature = _median(normalized_curvatures, default=0.0)
            view_bearing_span_deg = _bearing_span_deg(track.bearings_by_view)
            lateral_parallax_ok = (
                self.config.min_view_bearing_span_deg <= 0.0
                or view_bearing_span_deg >= self.config.min_view_bearing_span_deg
            )
            normalized_curvature_ok = (
                not normalized_curvatures
                or (
                    normalized_curvature >= self.config.min_normalized_depth_curvature
                    and normalized_curvature <= self.config.max_median_normalized_depth_curvature
                )
            )
            if (flat_views >= self.config.min_non_spherical_views_to_reject
                    and flat_views >= eligible_views):
                track.status = 'rejected_non_spherical'
                track.evidence_status = (
                    'single_view_flat_or_non_spherical'
                    if flat_views == 1
                    else 'multi_view_flat_or_non_spherical'
                )
            elif flat_views > 0 and eligible_views == 0:
                # 第一帧就取得平面深度时，尚不足以把圆柱/圆盘永久拒绝，
                # 但也不能继续显示为普通 tentative；必须明确要求换到侧视。
                track.status = 'needs_reobservation'
                track.evidence_status = 'single_view_flat_or_non_spherical'
            elif (track.eligible_observation_count >= self.config.confirm_observation_count
                  and eligible_views >= self.config.min_distinct_views
                  and diameter_cv <= self.config.max_apparent_diameter_cv
                  and diameter_prior_ok
                  and representative_aspect_ratio >= self.config.min_multiview_aspect_ratio
                  and curvature_cv <= self.config.max_depth_curvature_cv
                  and normalized_curvature_ok
                  and lateral_parallax_ok
                  and spherical_views >= self.config.min_spherical_views_for_confirm
                  and (flat_views == 0 or eligible_views >= flat_views + 2)):
                track.status = 'confirmed'
                track.evidence_status = (
                    'single_view_sphere_confirmed'
                    if eligible_views == 1
                    else 'multi_view_sphere_consistent'
                )
            elif (eligible_views >= self.config.min_distinct_views
                  and (diameter_cv > self.config.max_apparent_diameter_cv
                       or not diameter_prior_ok
                       or representative_aspect_ratio < self.config.min_multiview_aspect_ratio
                       or curvature_cv > self.config.max_depth_curvature_cv
                       or not normalized_curvature_ok
                       or not lateral_parallax_ok
                       or spherical_views < self.config.min_spherical_views_for_confirm
                       or flat_views > 0)):
                track.status = 'needs_reobservation'
                track.evidence_status = (
                    'inconsistent_apparent_size'
                    if diameter_cv > self.config.max_apparent_diameter_cv
                    else 'inconsistent_absolute_sphere_diameter'
                    if not diameter_prior_ok
                    else 'excessive_normalized_depth_curvature'
                    if (normalized_curvatures and normalized_curvature
                        > self.config.max_median_normalized_depth_curvature)
                    else 'insufficient_normalized_depth_curvature'
                    if not normalized_curvature_ok
                    else 'inconsistent_depth_curvature'
                    if curvature_cv > self.config.max_depth_curvature_cv
                    else 'insufficient_lateral_parallax'
                    if not lateral_parallax_ok
                    else 'insufficient_multiview_spherical_depth'
                    if spherical_views < self.config.min_spherical_views_for_confirm
                    else 'inconsistent_multiview_aspect'
                    if representative_aspect_ratio < self.config.min_multiview_aspect_ratio
                    else 'contradictory_depth_shape'
                )
            else:
                track.status = 'tentative'
                track.evidence_status = 'collecting_views'


"""计算两个三维位置之间的欧氏距离。"""
def distance_m(a, b):
    if len(a) != 3 or len(b) != 3:
        raise ValueError('Position must contain exactly 3 values.')
    return math.sqrt(
        (float(a[0]) - float(b[0])) ** 2 +
        (float(a[1]) - float(b[1])) ** 2 +
        (float(a[2]) - float(b[2])) ** 2
    )


"""将 HazardTrack 转为后续 JSON/结果文件可直接使用的 dict。"""
def track_to_hazard_dict(track):
    return {
        'id': track.track_id,
        'position': [round(float(v), 4) for v in track.position],
        'position_frame_id': 'start',
        'confidence': round(float(track.confidence), 4),
        'status': track.status,
        'observation_count': track.observation_count,
        'missed_count': track.missed_count,
        'first_seen_sec': track.first_seen_sec,
        'last_seen_sec': track.last_seen_sec,
        'source_ids': list(track.source_ids),
        'distinct_view_count': _eligible_distinct_view_count(track),
        'view_ids': list(track.view_ids),
        'eligible_observation_count': track.eligible_observation_count,
        'eligible_view_ids': list(track.eligible_view_ids),
        'flat_view_ids': list(track.flat_view_ids),
        'spherical_view_ids': list(track.spherical_view_ids),
        'apparent_diameter_cv': round(
            _coefficient_of_variation(_view_medians(track.diameters_by_view)), 4,
        ),
        'median_apparent_diameter_m': round(
            _median(_view_medians(track.diameters_by_view), default=0.0), 4,
        ),
        'min_multiview_aspect_ratio': round(
            min(_view_medians(track.aspects_by_view), default=1.0), 4,
        ),
        'depth_curvature_cv': round(
            _coefficient_of_variation(_view_medians(track.curvatures_by_view)), 4,
        ),
        'median_normalized_depth_curvature': round(
            _median(_normalized_curvature_by_view(track), default=0.0), 4,
        ),
        'view_bearing_span_deg': round(_bearing_span_deg(track.bearings_by_view), 3),
        'evidence_status': track.evidence_status,
    }


"""把 dict 或 HazardObservation 统一成观测对象。"""
def _normalize_observation(observation, default_stamp_sec):
    if isinstance(observation, HazardObservation):
        stamp = observation.stamp_sec if observation.stamp_sec else default_stamp_sec
        return HazardObservation(
            position=tuple(float(v) for v in observation.position),
            confidence=float(observation.confidence),
            stamp_sec=float(stamp),
            source_id=observation.source_id,
            view_id=observation.view_id,
            confirmation_eligible=bool(observation.confirmation_eligible),
            depth_shape_status=str(observation.depth_shape_status),
            apparent_diameter_m=observation.apparent_diameter_m,
            aspect_ratio=observation.aspect_ratio,
            depth_curvature_m=observation.depth_curvature_m,
            view_bearing_rad=observation.view_bearing_rad,
            track_id_hint=observation.track_id_hint,
        )

    position = observation.get('position')
    if position is None:
        raise ValueError('Observation must contain position.')
    stamp = observation.get('stamp_sec', default_stamp_sec)
    return HazardObservation(
        position=tuple(float(v) for v in position),
        confidence=float(observation.get('confidence', 0.0)),
        stamp_sec=float(stamp),
        source_id=str(observation.get('source_id', observation.get('id', ''))),
        view_id=str(observation.get('view_id', '')),
        confirmation_eligible=bool(observation.get('confirmation_eligible', True)),
        depth_shape_status=str(observation.get('depth_shape_status', 'unknown')),
        apparent_diameter_m=observation.get('apparent_diameter_m'),
        aspect_ratio=observation.get('aspect_ratio'),
        depth_curvature_m=observation.get('depth_curvature_m'),
        view_bearing_rad=observation.get('view_bearing_rad'),
        track_id_hint=observation.get('track_id_hint'),
    )


"""按视角等权的稳健位置融合更新轨迹和置信度。"""
def _merge_observation_into_track(
        track, observation, position_fusion_mode='view_median',
):
    old_count = max(1, int(track.observation_count))
    new_count = old_count + 1
    # 轨迹坐标只由可确认的完整 RGB-D 观测估计。partial、贴边、粘连框和
    # 非球面反证只用于复查/拒绝；即使上游误把它们传入，也不能把已确认红球
    # 的世界坐标逐帧拖向错误位置。
    if observation.confirmation_eligible:
        old_eligible_count = max(0, int(track.eligible_observation_count))
        if old_eligible_count == 0:
            track.position = tuple(float(value) for value in observation.position)
            track.positions_by_view = _initial_position_evidence(observation)
        else:
            _append_position_evidence(
                track.positions_by_view, observation.view_id, observation.position,
            )
            track.position = _position_from_views(
                track.positions_by_view, position_fusion_mode,
            )
    track.confidence = max(float(track.confidence), float(observation.confidence))
    track.observation_count = new_count
    track.missed_count = 0
    track.last_seen_sec = float(observation.stamp_sec)
    if observation.source_id and observation.source_id not in track.source_ids:
        track.source_ids.append(observation.source_id)
    if observation.view_id and observation.view_id not in track.view_ids:
        track.view_ids.append(observation.view_id)
    if observation.confirmation_eligible:
        track.eligible_observation_count += 1
        if observation.view_id and observation.view_id not in track.eligible_view_ids:
            track.eligible_view_ids.append(observation.view_id)
    if (_is_non_spherical_depth_status(observation.depth_shape_status)
            and observation.view_id
            and observation.view_id not in track.flat_view_ids):
        track.flat_view_ids.append(observation.view_id)
    if (observation.confirmation_eligible
            and observation.depth_shape_status == 'spherical' and observation.view_id
            and observation.view_id not in track.spherical_view_ids):
        track.spherical_view_ids.append(observation.view_id)
    if (observation.confirmation_eligible
            and _valid_diameter(observation.apparent_diameter_m)):
        track.apparent_diameters_m.append(float(observation.apparent_diameter_m))
        _append_view_evidence(track.diameters_by_view, observation.view_id, observation.apparent_diameter_m)
    if (observation.confirmation_eligible
            and _valid_aspect_ratio(observation.aspect_ratio)):
        track.aspect_ratios.append(float(observation.aspect_ratio))
        _append_view_evidence(track.aspects_by_view, observation.view_id, observation.aspect_ratio)
    if (observation.confirmation_eligible
            and _valid_positive(observation.depth_curvature_m)):
        track.depth_curvatures_m.append(float(observation.depth_curvature_m))
        _append_view_evidence(track.curvatures_by_view, observation.view_id, observation.depth_curvature_m)
    if (observation.confirmation_eligible and _valid_positive(observation.depth_curvature_m)
            and _valid_diameter(observation.apparent_diameter_m)):
        _append_view_evidence(
            track.curvature_ratios_by_view, observation.view_id,
            float(observation.depth_curvature_m) / float(observation.apparent_diameter_m),
        )
    if observation.confirmation_eligible and _valid_bearing(observation.view_bearing_rad):
        _append_view_evidence(track.bearings_by_view, observation.view_id, observation.view_bearing_rad)


"""没有提供视角标识时按兼容旧链路的单一视角计数。"""
def _eligible_distinct_view_count(track):
    """只统计可确认观测的真实离散视角，反证视角不能把轨迹推成 confirmed。"""

    if track.eligible_view_ids:
        return len(track.eligible_view_ids)
    return 1 if track.eligible_observation_count > 0 else 0


def _valid_diameter(value):
    return value is not None and math.isfinite(float(value)) and float(value) > 0.0


def _valid_positive(value):
    return value is not None and math.isfinite(float(value)) and float(value) > 0.0


def _valid_aspect_ratio(value):
    return value is not None and math.isfinite(float(value)) and 0.0 < float(value) <= 1.0


def _same_frame_duplicate_index(existing, candidate, config):
    """返回同帧重复球面观测索引；真实分离目标仍保留为独立轨迹。"""

    distance_limit = max(
        0.0, float(config.same_frame_duplicate_distance_m),
    )
    if (
        distance_limit <= 0.0
        or not candidate.confirmation_eligible
        or candidate.depth_shape_status != 'spherical'
        or not _valid_diameter(candidate.apparent_diameter_m)
    ):
        return None
    max_relative_error = max(
        0.0, float(config.same_frame_duplicate_diameter_relative_error),
    )
    for index, current in enumerate(existing):
        if (
            not current.confirmation_eligible
            or current.depth_shape_status != 'spherical'
            or not _valid_diameter(current.apparent_diameter_m)
        ):
            continue
        if distance_m(current.position, candidate.position) > distance_limit:
            continue
        reference = max(float(current.apparent_diameter_m), 1e-6)
        relative_error = (
            abs(float(candidate.apparent_diameter_m) - reference) / reference
        )
        if relative_error <= max_relative_error:
            return index
    return None


def _reacquire_diameter_compatible(track, diameter_m, max_relative_error):
    values = [
        float(value) for value in track.apparent_diameters_m
        if _valid_diameter(value)
    ]
    if not values:
        return False
    ordered = sorted(values)
    middle = len(ordered) // 2
    reference = (
        ordered[middle]
        if len(ordered) % 2
        else (ordered[middle - 1] + ordered[middle]) / 2.0
    )
    return (
        abs(float(diameter_m) - reference) / max(reference, 1e-6)
        <= max(0.0, float(max_relative_error))
    )


def _position_view_key(view_id):
    """旧链路没有 view_id 时归入一个兼容桶，仍退化为普通逐帧均值。"""

    normalized = str(view_id or '').strip()
    return normalized if normalized else '__legacy_single_view__'


def _initial_position_evidence(observation):
    point = tuple(float(value) for value in observation.position)
    return {_position_view_key(observation.view_id): [point[0], point[1], point[2], 1]}


def _append_position_evidence(store, view_id, position):
    key = _position_view_key(view_id)
    point = tuple(float(value) for value in position)
    bucket = store.setdefault(key, [0.0, 0.0, 0.0, 0])
    for index in range(3):
        bucket[index] += point[index]
    bucket[3] += 1


def _position_from_views(store, mode='view_median'):
    """先求每个离散视角均值，再对视角均值逐坐标取中位数。"""

    per_view = [
        tuple(float(bucket[index]) / max(1, int(bucket[3])) for index in range(3))
        for bucket in store.values()
        if len(bucket) == 4 and int(bucket[3]) > 0
    ]
    if not per_view:
        raise ValueError('没有可用于定位融合的视角位置证据。')
    normalized_mode = str(mode or '').strip()
    if normalized_mode == 'earliest_view_anchor':
        return per_view[0]
    if normalized_mode != 'view_median':
        raise ValueError(f'不支持的轨迹位置融合模式：{mode}')

    def median(values):
        ordered = sorted(float(value) for value in values)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return (ordered[middle - 1] + ordered[middle]) / 2.0

    return tuple(median(point[index] for point in per_view) for index in range(3))


def _valid_bearing(value):
    return value is not None and math.isfinite(float(value))


def _is_non_spherical_depth_status(status):
    """统一识别明确的 RGB-D 反证；unknown 仍留给后续主动复查。"""

    return str(status).strip().lower() in ('flat', 'anisotropic', 'non_spherical')


def _matches_expected_diameter(measured_m, expected_m, max_relative_error):
    """检查已知标准球尺寸；禁用先验时保持历史未知尺寸兼容行为。"""

    expected = float(expected_m)
    if expected <= 0.0:
        return True
    if not _valid_diameter(measured_m):
        return False
    relative_error = abs(float(measured_m) - expected) / expected
    return relative_error <= max(0.0, float(max_relative_error))


def _diameter_cv(values):
    """返回表观三维直径的变异系数；样本不足时不提前否决。"""

    valid = [float(value) for value in values if _valid_diameter(value)]
    if len(valid) < 2:
        return 0.0
    mean = sum(valid) / len(valid)
    variance = sum((value - mean) ** 2 for value in valid) / len(valid)
    return math.sqrt(variance) / max(mean, 1e-9)


def _coefficient_of_variation(values):
    valid = [float(value) for value in values if _valid_positive(value)]
    if len(valid) < 2:
        return 0.0
    mean = sum(valid) / len(valid)
    variance = sum((value - mean) ** 2 for value in valid) / len(valid)
    return math.sqrt(variance) / max(mean, 1e-9)


def _median(values, default=0.0):
    valid = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not valid:
        return float(default)
    middle = len(valid) // 2
    if len(valid) % 2:
        return valid[middle]
    return (valid[middle - 1] + valid[middle]) / 2.0


def _initial_view_evidence(view_id, value, validator):
    if not validator(value):
        return {}
    return {str(view_id or 'unknown'): [float(value)]}


def _append_view_evidence(mapping, view_id, value):
    mapping.setdefault(str(view_id or 'unknown'), []).append(float(value))


def _view_medians(mapping):
    """先压缩同一量化视角内的连续帧，避免运动过渡帧支配跨视角门控。"""

    return [_median(values) for values in mapping.values() if values]


def _initial_curvature_ratio_evidence(observation):
    if (not observation.confirmation_eligible
            or not _valid_positive(observation.depth_curvature_m)
            or not _valid_diameter(observation.apparent_diameter_m)):
        return {}
    ratio = float(observation.depth_curvature_m) / float(observation.apparent_diameter_m)
    return {str(observation.view_id or 'unknown'): [ratio]}


def _view_percentiles(mapping, fraction):
    values = []
    for samples in mapping.values():
        ordered = sorted(float(value) for value in samples if math.isfinite(float(value)))
        if not ordered:
            continue
        index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * fraction))))
        values.append(ordered[index])
    return values


def _normalized_curvature_by_view(track):
    """用同视角曲率上四分位数/直径中位数，避免单帧小框把比值放大。"""

    ratios = []
    for view_id, curvature_samples in track.curvatures_by_view.items():
        diameter_samples = track.diameters_by_view.get(view_id, [])
        if not curvature_samples or not diameter_samples:
            continue
        curvature = _percentile(curvature_samples, 0.75)
        diameter = _median(diameter_samples)
        if diameter > 0.0:
            ratios.append(curvature / diameter)
    return ratios


def _bearing_span_deg(bearings_by_view):
    """返回任意两个稳定视角的最大水平视线夹角，处理 -pi/pi 环绕。"""
    bearings = _view_medians(bearings_by_view)
    if len(bearings) < 2:
        return 0.0
    max_span = 0.0
    for first_index, first in enumerate(bearings):
        for second in bearings[first_index + 1:]:
            delta = math.atan2(math.sin(second - first), math.cos(second - first))
            max_span = max(max_span, abs(math.degrees(delta)))
    return max_span


def _percentile(values, fraction):
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return 0.0
    index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * fraction))))
    return ordered[index]
