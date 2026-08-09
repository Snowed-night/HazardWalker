"""感知运行参数加载离线测试。

所属组：感知定位组 / 测试组。负责人：姜晨。
文件作用：验证仓库配置会被严格转换为检测节点真正使用的 ROS 参数，未知
字段和未实现的点云开关不会被静默忽略。
"""
from copy import deepcopy
import json
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


def test_replay_parameter_snapshots_change_only_spherical_view_count():
    """控制变量快照必须可运行，且候选方案只能改变声明的单一参数。"""

    baseline_document = yaml.safe_load(
        (REPO_ROOT / 'config' / 'perception_baseline.yaml').read_text(
            encoding='utf-8'))
    candidate_document = yaml.safe_load(
        (REPO_ROOT / 'config' / 'perception_candidate.yaml').read_text(
            encoding='utf-8'))
    baseline = flatten_perception_config(baseline_document)
    candidate = flatten_perception_config(candidate_document)

    differences = {
        key: (baseline.get(key), candidate.get(key))
        for key in set(baseline) | set(candidate)
        if baseline.get(key) != candidate.get(key)
    }
    assert differences == {'min_spherical_views_for_confirm': (2, 3)}
    assert baseline == flatten_perception_config(_repository_config())


def test_replay_campaign_template_references_existing_distinct_snapshots():
    plan_path = (
        REPO_ROOT / 'config' / 'perception_replay_campaign.example.json')
    plan = json.loads(plan_path.read_text(encoding='utf-8'))

    assert plan['schema'] == (
        'hazardwalker_perception_replay_campaign_plan_v1')
    assert len(plan['variants']) == 2
    parameter_files = [
        REPO_ROOT / item['parameter_file'] for item in plan['variants']]
    assert all(path.is_file() for path in parameter_files)
    assert parameter_files[0].read_bytes() != parameter_files[1].read_bytes()
    assert set(plan['annotations_by_seed']) == {
        'REPLACE_SEED_1', 'REPLACE_SEED_2', 'REPLACE_SEED_3'}
