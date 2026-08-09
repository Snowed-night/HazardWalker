"""感知运行参数加载离线测试。

所属组：感知定位组 / 测试组。负责人：姜晨。
文件作用：验证仓库配置会被严格转换为检测节点真正使用的 ROS 参数，未知
字段和未实现的点云开关不会被静默忽略。
"""
from copy import deepcopy
from pathlib import Path
import sys

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(
    REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'))

from hazardwalker_perception.perception_config import (  # noqa: E402
    flatten_perception_config,
)


def _repository_config():
    return yaml.safe_load(
        (REPO_ROOT / 'config' / 'perception.yaml').read_text(encoding='utf-8'))


def test_repository_config_maps_to_runtime_parameters():
    params = flatten_perception_config(_repository_config())
    assert params['min_area_px'] == 200
    assert params['red_hue_min_1'] == 0
    assert params['red_hue_max_2'] == 180
    assert params['red_min_saturation'] == 80
    assert params['min_depth_points_in_roi'] == 5
    assert params['min_sphere_depth_shape_bbox_px'] == 40
    assert params['active_view_min_bbox_area_px'] == 1600
    assert params['active_view_min_confidence'] == 0.70
    assert params['track_projection_max_age_s'] == 180.0


def test_unknown_parameter_is_rejected_instead_of_silently_ignored():
    config = deepcopy(_repository_config())
    config['perception']['tracking']['typo_parameter'] = 1
    with pytest.raises(ValueError, match='typo_parameter'):
        flatten_perception_config(config)


def test_unimplemented_point_cloud_switch_is_rejected():
    config = deepcopy(_repository_config())
    config['perception']['localization']['use_point_cloud'] = True
    with pytest.raises(ValueError, match='use_point_cloud=true'):
        flatten_perception_config(config)
