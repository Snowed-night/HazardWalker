"""感知配置加载与 ROS 参数展开。

所属组：感知定位组。负责人：姜晨。
文件作用：把仓库级 ``config/perception.yaml`` 的分组配置转换为
``hsv_detector_node`` 实际使用的扁平 ROS 参数，避免“报告记录了参数文件、
运行节点却仍使用默认值”的不可追溯情况。
"""


_DIRECT_SECTIONS = {
    'red_ball_detector': {
        'min_area_px', 'min_circularity', 'min_aspect_ratio', 'min_extent',
        'max_extent', 'min_confidence', 'max_detections', 'detector_backend',
        'split_touching_red_balls',
    },
    'localization': {
        'max_detection_range_m', 'min_sphere_depth_curvature_m',
        'min_sphere_depth_shape_points', 'min_sphere_axis_depth_points',
        'min_sphere_axis_curvature_ratio', 'sphere_radius_m',
        'max_rgb_depth_sync_delta_sec', 'use_sphere_projection_geometry',
        'allow_latest_tf_fallback', 'max_latest_tf_fallback_delta_sec',
        'roi_padding_px', 'min_depth_points_in_roi', 'output_frame',
        'camera_axis_convention',
    },
    'tracking': {
        'confirm_observation_count', 'confirm_distinct_views',
        'min_view_bearing_span_deg', 'reject_after_missed_count',
        'reject_after_missed_sec',
        'merge_distance_m', 'single_track_reacquire_distance_m',
        'single_track_reacquire_diameter_relative_error',
        'same_frame_duplicate_distance_m',
        'same_frame_duplicate_diameter_relative_error',
        'max_apparent_diameter_cv', 'expected_sphere_diameter_m',
        'max_sphere_diameter_relative_error', 'min_multiview_aspect_ratio',
        'min_spherical_views_for_confirm', 'max_depth_curvature_cv',
        'min_normalized_depth_curvature',
        'max_median_normalized_depth_curvature',
        'track_position_fusion_mode', 'track_projection_max_age_s',
    },
    'active_reobservation': {
        'emit_partial_candidates', 'partial_min_area_px',
        'partial_min_circularity', 'partial_min_aspect_ratio',
        'partial_min_value', 'stable_view_min_frames',
        'stable_view_max_translation_m', 'stable_view_max_yaw_deg',
        'track_projection_max_age_s',
        'candidate_memory_ttl_s', 'active_view_direction_memory_ttl_s',
        'candidate_memory_min_iou', 'candidate_memory_max_center_shift_ratio',
    },
}

_ACTIVE_VIEW_RENAMES = {
    'edge_margin_ratio': 'active_view_edge_margin_ratio',
    'min_bbox_area_px': 'active_view_min_bbox_area_px',
    'min_red_pixel_count': 'active_view_min_red_pixel_count',
    'min_circularity': 'active_view_min_circularity',
    'min_confidence': 'active_view_min_confidence',
    'far_distance_m': 'active_view_far_distance_m',
    'dense_iou_threshold': 'active_view_dense_iou_threshold',
    'min_normalized_depth_curvature': (
        'active_view_min_normalized_depth_curvature'),
    'max_normalized_depth_curvature': (
        'active_view_max_normalized_depth_curvature'),
}


def flatten_perception_config(document):
    """校验并展开仓库级感知配置；未知键直接报错，禁止静默忽略。"""

    if not isinstance(document, dict) or not isinstance(
            document.get('perception'), dict):
        raise ValueError('perception config must contain a mapping named perception')
    root = document['perception']
    unknown_sections = set(root) - set(_DIRECT_SECTIONS)
    if unknown_sections:
        raise ValueError(
            f'unsupported perception config sections: {sorted(unknown_sections)}')

    parameters = {}
    for section_name, allowed_keys in _DIRECT_SECTIONS.items():
        section = root.get(section_name, {})
        if not isinstance(section, dict):
            raise ValueError(f'{section_name} must be a mapping')
        unknown_keys = set(section) - allowed_keys
        if section_name == 'red_ball_detector':
            unknown_keys -= {
                'color_space', 'lower_red_1', 'upper_red_1',
                'lower_red_2', 'upper_red_2',
            }
        if section_name == 'localization':
            # 点云方案尚未接入本节点；显式 false/true 仅作为路线记录，不能
            # 被误认为已经切换运行实现。
            unknown_keys -= {'use_point_cloud', 'min_points_in_roi'}
        if section_name == 'active_reobservation':
            unknown_keys -= set(_ACTIVE_VIEW_RENAMES)
        if unknown_keys:
            raise ValueError(
                f'unsupported keys in {section_name}: {sorted(unknown_keys)}')

        for key in allowed_keys:
            if key in section:
                parameters[key] = section[key]
        if section_name == 'localization' and 'min_points_in_roi' in section:
            parameters['min_depth_points_in_roi'] = section['min_points_in_roi']
        if (section_name == 'localization'
                and bool(section.get('use_point_cloud', False))):
            raise ValueError(
                'use_point_cloud=true is not implemented by hsv_detector_node')
        if section_name == 'active_reobservation':
            for source, target in _ACTIVE_VIEW_RENAMES.items():
                if source in section:
                    parameters[target] = section[source]

    _flatten_hsv_ranges(root.get('red_ball_detector', {}), parameters)
    return parameters


def _flatten_hsv_ranges(section, parameters):
    """把两段 HSV 数组转换为检测节点可声明、可审计的标量参数。"""

    names = ('lower_red_1', 'upper_red_1', 'lower_red_2', 'upper_red_2')
    if not any(name in section for name in names):
        return
    if str(section.get('color_space', 'HSV')).upper() != 'HSV':
        raise ValueError('only HSV color_space is supported')
    ranges = {}
    for name in names:
        value = section.get(name)
        if not isinstance(value, list) or len(value) != 3:
            raise ValueError(f'{name} must contain exactly three HSV values')
        ranges[name] = value
    if ranges['upper_red_1'][1:] != [255, 255] or ranges['upper_red_2'][1:] != [255, 255]:
        raise ValueError('HSV upper saturation/value must remain [255, 255]')
    if ranges['lower_red_1'][1:] != ranges['lower_red_2'][1:]:
        raise ValueError('both red hue ranges must use the same S/V lower bounds')
    parameters.update({
        'red_hue_min_1': ranges['lower_red_1'][0],
        'red_hue_max_1': ranges['upper_red_1'][0],
        'red_hue_min_2': ranges['lower_red_2'][0],
        'red_hue_max_2': ranges['upper_red_2'][0],
        'red_min_saturation': ranges['lower_red_1'][1],
        'red_min_value': ranges['lower_red_1'][2],
    })
