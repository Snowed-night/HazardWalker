"""HazardWalker Phase 1 — 平台组产出版本校验。

所属组：平台组。
文件作用：
  - 在 **纯 Python** 环境下校验 Phase 1 所有产出文件的完整性。
  - 不依赖 ROS 2、Gazebo、OpenCV 或任何 Linux 专属库。
  - → Windows 上可以直接跑：`cd HazardWalker && python tests/offline/test_platform_phase1.py`

当前验证内容：
  1. SDF 文件存在且是合法 XML。
  2. 世界文件包含预期的模型引用。
  3. 机器人模型包含相机 / LiDAR / IMU 传感器和 DiffDrive 插件。
  4. 红球模型是静态球体。
  5. ros_gz_bridge YAML 配置合法且覆盖所有关键话题。
  6. 启动文件存在。

后续扩展：
  - Phase 2 增加 gazebo_adapter_node.py 后在这里补节点测试。
  - 若 SDF 结构变更，同步更新此处的校验断言。
"""

import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# ---------------------------------------------------------------------------
# 路径
# ---------------------------------------------------------------------------
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
PLATFORM_DIR = os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_platform')
MODELS_DIR = os.path.join(PLATFORM_DIR, 'models')
WORLDS_DIR = os.path.join(PLATFORM_DIR, 'worlds')
CONFIG_DIR = os.path.join(PLATFORM_DIR, 'config')
LAUNCH_DIR = os.path.join(PLATFORM_DIR, 'launch')

# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------

def _safe_xml(path):
    """解析 XML/SDF 文件，解析失败时返回 None 和错误信息。"""
    try:
        tree = ET.parse(path)
        return tree, None
    except ET.ParseError as exc:
        return None, str(exc)


def _sdf_ns():
    """SDFormat 命名空间。"""
    return ''

# ---------------------------------------------------------------------------
# 1. 文件存在性
# ---------------------------------------------------------------------------

def test_all_phase1_files_exist():
    """验证 Phase 1 全部产出文件存在于正确位置。"""
    expected = [
        os.path.join(MODELS_DIR, 'red_ball', 'model.sdf'),
        os.path.join(MODELS_DIR, 'red_ball', 'model.config'),
        os.path.join(MODELS_DIR, 'simple_robot', 'model.sdf'),
        os.path.join(MODELS_DIR, 'simple_robot', 'model.config'),
        os.path.join(WORLDS_DIR, 'hazardwalker_minimal.sdf'),
        os.path.join(WORLDS_DIR, 'hazardwalker_red_ball_gallery.sdf'),
        os.path.join(CONFIG_DIR, 'ros_gz_bridge.yaml'),
        os.path.join(LAUNCH_DIR, 'gazebo_minimal.launch.py'),
    ]
    missing = [f for f in expected if not os.path.isfile(f)]
    if missing:
        msg = '\n  '.join(missing)
        raise AssertionError(f'缺少文件:\n  {msg}')
    assert True


# ---------------------------------------------------------------------------
# 2. 红球模型 SDF 校验
# ---------------------------------------------------------------------------

_RED_BALL_SDF = os.path.join(MODELS_DIR, 'red_ball', 'model.sdf')


def test_red_ball_sdf_is_valid_xml():
    """红球 SDF 文件是合法 XML。"""
    _, err = _safe_xml(_RED_BALL_SDF)
    if err:
        raise AssertionError(f'红球 SDF XML 解析失败: {err}')
    assert True


def test_red_ball_model_config_is_valid_xml():
    """红球模型包含 Gazebo 可解析的 model.config。"""
    path = os.path.join(MODELS_DIR, 'red_ball', 'model.config')
    tree, err = _safe_xml(path)
    if err:
        raise AssertionError(f'红球 model.config XML 解析失败: {err}')
    root = tree.getroot()
    assert root.findtext('name') == 'red_ball'
    assert root.find('sdf') is not None


def test_red_ball_is_static():
    """红球模型应标记为 static。"""
    tree, _ = _safe_xml(_RED_BALL_SDF)
    model = tree.getroot().find('model')
    static = model.find('static')
    assert static is not None, '红球模型缺少 <static> 标签'
    assert static.text.strip() == 'true', f'红球应设为 static=true，实际: {static.text}'


def test_red_ball_has_sphere_geometry():
    """红球模型包含 sphere 几何体（半径 0.15m）。"""
    tree, _ = _safe_xml(_RED_BALL_SDF)
    spheres = tree.getroot().findall('.//sphere')
    assert len(spheres) > 0, '红球模型缺少 <sphere> 几何体'
    radii = [float(s.find('radius').text) for s in spheres if s.find('radius') is not None]
    assert any(abs(r - 0.15) < 0.01 for r in radii), \
        f'红球半径应为 0.15m，实际: {radii}'


def test_red_ball_model_name():
    """红球模型名为 red_ball。"""
    tree, _ = _safe_xml(_RED_BALL_SDF)
    model = tree.getroot().find('model')
    assert model is not None
    assert model.get('name') == 'red_ball', \
        f'模型名应为 red_ball，实际: {model.get("name")}'

# ---------------------------------------------------------------------------
# 3. 机器人模型 SDF 校验
# ---------------------------------------------------------------------------

_ROBOT_SDF = os.path.join(MODELS_DIR, 'simple_robot', 'model.sdf')


def test_robot_sdf_is_valid_xml():
    """机器人 SDF 文件是合法 XML。"""
    _, err = _safe_xml(_ROBOT_SDF)
    if err:
        raise AssertionError(f'机器人 SDF XML 解析失败: {err}')
    assert True


def test_robot_model_config_is_valid_xml():
    """机器人模型包含 Gazebo 可解析的 model.config。"""
    path = os.path.join(MODELS_DIR, 'simple_robot', 'model.config')
    tree, err = _safe_xml(path)
    if err:
        raise AssertionError(f'机器人 model.config XML 解析失败: {err}')
    root = tree.getroot()
    assert root.findtext('name') == 'simple_robot'
    assert root.find('sdf') is not None


def test_robot_has_chassis():
    """机器人必须有 chassis link。"""
    tree, _ = _safe_xml(_ROBOT_SDF)
    links = tree.getroot().findall('.//link')
    link_names = [lk.get('name') for lk in links]
    assert 'chassis' in link_names, f'缺少 chassis link，现有 link: {link_names}'


def test_robot_has_camera_sensor():
    """机器人模型必须包含 camera 传感器。"""
    tree, _ = _safe_xml(_ROBOT_SDF)
    sensors = tree.getroot().findall('.//sensor')
    sensor_types = [s.get('type') for s in sensors]
    assert 'camera' in sensor_types, f'缺少 camera 传感器，现有类型: {sensor_types}'


def test_robot_has_lidar_sensor():
    """机器人模型必须包含 gpu_lidar 传感器。"""
    tree, _ = _safe_xml(_ROBOT_SDF)
    sensors = tree.getroot().findall('.//sensor')
    sensor_types = [s.get('type') for s in sensors]
    assert 'gpu_lidar' in sensor_types, f'缺少 gpu_lidar 传感器，现有类型: {sensor_types}'


def test_robot_has_imu_sensor():
    """机器人模型必须包含 IMU 传感器。"""
    tree, _ = _safe_xml(_ROBOT_SDF)
    sensors = tree.getroot().findall('.//sensor')
    sensor_types = [s.get('type') for s in sensors]
    assert 'imu' in sensor_types, f'缺少 IMU 传感器，现有类型: {sensor_types}'


def test_robot_has_diff_drive_plugin():
    """机器人模型必须包含 DiffDrive 插件。"""
    tree, _ = _safe_xml(_ROBOT_SDF)
    plugins = tree.getroot().findall('.//plugin')
    plugin_names = [p.get('name', '') for p in plugins]
    diffdrive_found = any('DiffDrive' in name for name in plugin_names)
    assert diffdrive_found, f'缺少 DiffDrive 插件，现有插件: {plugin_names}'


def test_robot_has_drive_wheels():
    """机器人必须有 left_wheel_joint 和 right_wheel_joint 两个驱动关节。"""
    tree, _ = _safe_xml(_ROBOT_SDF)
    joints = tree.getroot().findall('.//joint')
    joint_names = [j.get('name') for j in joints]
    assert 'left_wheel_joint' in joint_names, f'缺少 left_wheel_joint，现有关节: {joint_names}'
    assert 'right_wheel_joint' in joint_names, f'缺少 right_wheel_joint，现有关节: {joint_names}'


def test_robot_sensor_links_match_fake_platform():
    """机器人传感器挂载点与 fake_platform_node 中硬编码的位置一致。"""
    tree, _ = _safe_xml(_ROBOT_SDF)
    links = tree.getroot().findall('.//link')
    camera_link = None
    lidar_link = None
    for lk in links:
        if lk.get('name') == 'camera_link':
            pose_el = lk.find('pose')
            if pose_el is not None:
                camera_link = [float(x) for x in pose_el.text.split()]
        if lk.get('name') == 'lidar_link':
            pose_el = lk.find('pose')
            if pose_el is not None:
                lidar_link = [float(x) for x in pose_el.text.split()]

    assert camera_link is not None, '未找到 camera_link'
    assert lidar_link is not None, '未找到 lidar_link'

    # fake_platform_node 中: camera_link 在 (0.25, 0, 0.35)
    assert abs(camera_link[0] - 0.25) < 0.02
    assert abs(camera_link[1] - 0.0) < 0.02
    assert abs(camera_link[2] - 0.35) < 0.02

    # fake_platform_node 中: lidar_link 在 (0.15, 0, 0.45)
    assert abs(lidar_link[0] - 0.15) < 0.02
    assert abs(lidar_link[1] - 0.0) < 0.02
    assert abs(lidar_link[2] - 0.45) < 0.02

# ---------------------------------------------------------------------------
# 4. 世界文件 SDF 校验
# ---------------------------------------------------------------------------

_WORLD_SDF = os.path.join(WORLDS_DIR, 'hazardwalker_minimal.sdf')
_GALLERY_WORLD_SDF = os.path.join(WORLDS_DIR, 'hazardwalker_red_ball_gallery.sdf')


def test_world_sdf_is_valid_xml():
    """世界 SDF 文件是合法 XML。"""
    _, err = _safe_xml(_WORLD_SDF)
    if err:
        raise AssertionError(f'世界 SDF XML 解析失败: {err}')
    assert True


def test_gallery_world_sdf_is_valid_xml():
    """红球截图测试世界 SDF 文件是合法 XML。"""
    _, err = _safe_xml(_GALLERY_WORLD_SDF)
    if err:
        raise AssertionError(f'红球截图测试世界 SDF XML 解析失败: {err}')
    assert True


def test_gallery_world_has_multiple_red_balls_and_cameras():
    """截图测试世界包含多红球和多个测试相机。"""
    tree, _ = _safe_xml(_GALLERY_WORLD_SDF)
    includes = tree.getroot().findall('.//include')
    red_ball_uris = []
    for inc in includes:
        uri_el = inc.find('uri')
        if uri_el is not None and uri_el.text == 'model://red_ball':
            red_ball_uris.append(uri_el.text)

    sensors = tree.getroot().findall('.//sensor')
    camera_topics = []
    for sensor in sensors:
        if sensor.get('type') == 'camera':
            topic_el = sensor.find('topic')
            if topic_el is not None:
                camera_topics.append(topic_el.text)

    assert len(red_ball_uris) >= 4, f'截图测试世界红球数量不足: {len(red_ball_uris)}'
    assert '/gallery/center_full/image' in camera_topics
    assert '/gallery/left_partial/image' in camera_topics
    assert '/gallery/top_partial/image' in camera_topics
    assert '/gallery/multi_visible/image' in camera_topics


def test_world_has_four_walls():
    """世界文件包含四面墙（wall_north, wall_south, wall_east, wall_west）。"""
    tree, _ = _safe_xml(_WORLD_SDF)
    models = tree.getroot().findall('.//model')
    model_names = [m.get('name') for m in models]
    for w in ['wall_north', 'wall_south', 'wall_east', 'wall_west']:
        assert w in model_names, f'缺少墙壁 {w}，现有模型: {model_names}'


def test_world_has_ground_plane():
    """世界文件包含地面。"""
    tree, _ = _safe_xml(_WORLD_SDF)
    models = tree.getroot().findall('.//model')
    model_names = [m.get('name') for m in models]
    assert 'ground_plane' in model_names, f'缺少 ground_plane，现有模型: {model_names}'


def test_world_includes_robot_and_red_ball():
    """世界文件通过 <include> 引用机器人模型和红球模型。"""
    tree, _ = _safe_xml(_WORLD_SDF)
    includes = tree.getroot().findall('.//include')
    uris = []
    for inc in includes:
        uri_el = inc.find('uri')
        if uri_el is not None:
            uris.append(uri_el.text)
    assert 'model://simple_robot' in uris, f'世界文件未引用 simple_robot，现有引用: {uris}'
    assert 'model://red_ball' in uris, f'世界文件未引用 red_ball，现有引用: {uris}'


def test_world_has_lighting():
    """世界文件包含光照。"""
    tree, _ = _safe_xml(_WORLD_SDF)
    lights = tree.getroot().findall('.//light')
    assert len(lights) > 0, '世界文件缺少光照'

# ---------------------------------------------------------------------------
# 5. ros_gz_bridge YAML 配置校验
# ---------------------------------------------------------------------------

_BRIDGE_YAML = os.path.join(CONFIG_DIR, 'ros_gz_bridge.yaml')


def test_bridge_yaml_is_valid():
    """桥接 YAML 文件存在且能被解析为列表。"""
    try:
        import yaml
    except ImportError:
        # Windows 上可能没有 pyyaml，跳过。主力机上一定可用。
        return

    with open(_BRIDGE_YAML, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    assert data is not None, 'YAML 文件为空'
    assert isinstance(data, list), f'YAML 顶层应为 list，实际: {type(data)}'
    assert len(data) > 0, 'YAML 配置列表为空'


def test_bridge_has_all_required_topics():
    """桥接配置覆盖所有必需的 /hw/* 话题。"""
    try:
        import yaml
    except ImportError:
        return

    with open(_BRIDGE_YAML, 'r', encoding='utf-8') as f:
        entries = yaml.safe_load(f)

    ros_topics = {entry.get('ros_topic_name', '') for entry in entries}

    required = {
        '/hw/camera/image_raw',
        '/hw/lidar/points',
        '/hw/lidar/scan',
        '/hw/odom',
        '/tf',
        '/hw/cmd_vel',
    }
    missing = required - ros_topics
    if missing:
        raise AssertionError(
            f'桥接配置缺少话题: {sorted(missing)}\n  已有话题: {sorted(ros_topics)}'
        )
    assert True

# ---------------------------------------------------------------------------
# 6. 启动文件校验
# ---------------------------------------------------------------------------

_LAUNCH_PY = os.path.join(LAUNCH_DIR, 'gazebo_minimal.launch.py')


def test_launch_file_exists_and_has_windows_guard():
    """启动文件存在，且包含 Windows 平台检测。"""
    assert os.path.isfile(_LAUNCH_PY), f'启动文件不存在: {_LAUNCH_PY}'

    with open(_LAUNCH_PY, 'r', encoding='utf-8') as f:
        content = f.read()

    assert 'platform.system()' in content, \
        '启动文件缺少 platform.system() 平台检测'
    assert 'IS_LINUX' in content or '_IS_LINUX' in content, \
        '启动文件缺少 _IS_LINUX 标志'
    assert 'sys.exit' in content, \
        '启动文件缺少非 Linux 环境的安全退出'


def test_launch_file_is_valid_python_syntax():
    """启动文件 Python 语法正确（只做 AST 解析，不执行导入）。"""
    import ast
    with open(_LAUNCH_PY, 'r', encoding='utf-8') as f:
        source = f.read()
    try:
        ast.parse(source)
    except SyntaxError as exc:
        raise AssertionError(f'启动文件 Python 语法错误: {exc}')


def test_launch_file_runs_on_windows_without_error():
    """在 Windows 上执行启动文件应打印提示并正常退出。

    此测试使用 subprocess 运行脚本，验证 Windows 检测逻辑生效。
    只在 Windows 平台上执行。
    """
    import platform as _platform
    if _platform.system() != 'Windows':
        return  # 只在 Windows 上验证此行为

    import subprocess
    result = subprocess.run(
        [sys.executable, _LAUNCH_PY],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0, \
        f'启动文件在 Windows 上应以 exit code 0 退出，实际: {result.returncode}'
    assert 'Windows' in result.stdout, \
        f'启动文件在 Windows 上应打印提示信息，实际 stdout:\n{result.stdout}'


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    # 简单测试运行器，不依赖 pytest。
    # 与项目其他离线测试风格一致。
    tests = [
        # 1. 文件存在性
        test_all_phase1_files_exist,
        # 2. 红球模型
        test_red_ball_sdf_is_valid_xml,
        test_red_ball_is_static,
        test_red_ball_has_sphere_geometry,
        test_red_ball_model_name,
        test_red_ball_model_config_is_valid_xml,
        # 3. 机器人模型
        test_robot_sdf_is_valid_xml,
        test_robot_model_config_is_valid_xml,
        test_robot_has_chassis,
        test_robot_has_camera_sensor,
        test_robot_has_lidar_sensor,
        test_robot_has_imu_sensor,
        test_robot_has_diff_drive_plugin,
        test_robot_has_drive_wheels,
        test_robot_sensor_links_match_fake_platform,
        # 4. 世界文件
        test_world_sdf_is_valid_xml,
        test_gallery_world_sdf_is_valid_xml,
        test_gallery_world_has_multiple_red_balls_and_cameras,
        test_world_has_four_walls,
        test_world_has_ground_plane,
        test_world_includes_robot_and_red_ball,
        test_world_has_lighting,
        # 5. ros_gz_bridge 配置
        test_bridge_yaml_is_valid,
        test_bridge_has_all_required_topics,
        # 6. 启动文件
        test_launch_file_exists_and_has_windows_guard,
        test_launch_file_is_valid_python_syntax,
        test_launch_file_runs_on_windows_without_error,
    ]

    passed = 0
    failed = 0
    for test_fn in tests:
        try:
            test_fn()
            print(f'  PASS  {test_fn.__name__}')
            passed += 1
        except AssertionError as exc:
            print(f'  FAIL  {test_fn.__name__}: {exc}')
            failed += 1
        except Exception as exc:
            print(f'  ERROR {test_fn.__name__}: {exc}')
            failed += 1

    print(f'\n{passed} passed, {failed} failed')
    if failed > 0:
        sys.exit(1)
