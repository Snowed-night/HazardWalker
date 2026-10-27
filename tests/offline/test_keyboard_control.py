"""负责人维护的 W/S/A/D/K 键盘控制离线契约测试。"""

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PLATFORM_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform'
if str(PLATFORM_SRC) not in sys.path:
    sys.path.insert(0, str(PLATFORM_SRC))

from hazardwalker_platform.keyboard_control import command_for_key  # noqa: E402


def test_keyboard_mapping_matches_navigation_contract():
    expected = {
        'w': (0.3, 0.0, False),
        's': (-0.3, 0.0, False),
        'a': (0.0, 0.6, False),
        'd': (0.0, -0.6, False),
        'k': (0.0, 0.0, True),
    }
    for key, values in expected.items():
        command = command_for_key(
            key,
            linear_speed=0.3,
            angular_speed=0.6,
        )
        assert command is not None
        assert (command.linear_x, command.angular_z, command.is_stop) == values


def test_keyboard_mapping_accepts_uppercase_and_ignores_other_keys():
    assert command_for_key(
        'W', linear_speed=0.2, angular_speed=0.4).linear_x == 0.2
    assert command_for_key(
        'K', linear_speed=0.2, angular_speed=0.4).is_stop
    assert command_for_key(
        'x', linear_speed=0.2, angular_speed=0.4) is None


def test_platform_package_registers_keyboard_node():
    setup_source = (PLATFORM_SRC / 'setup.py').read_text(encoding='utf-8')
    assert (
        'keyboard_control_node = '
        'hazardwalker_platform.keyboard_control_node:main'
    ) in setup_source


def test_keyboard_node_also_supports_python_module_startup():
    """共享机尚未构建工作区时，仍可安全地按模块方式启动节点。"""

    source = (PLATFORM_SRC / 'hazardwalker_platform' /
              'keyboard_control_node.py').read_text(encoding='utf-8')
    assert "if __name__ == '__main__':" in source
    assert '    main()' in source


def test_legacy_keyboard_paths_keep_the_same_wsadk_semantics():
    controller = (
        PLATFORM_SRC / 'src' / 'unitree_guide' / 'unitree_guide' /
        'unitree_guide' / 'src' / 'interface' / 'KeyBoard.cpp'
    ).read_text(encoding='utf-8')
    ros1_tool = (
        PLATFORM_SRC / 'src' / 'unitree_guide' / 'unitree_ros_to_real' /
        'unitree_legged_real' / 'src' / 'exe' /
        'control_via_keyboard.cpp'
    ).read_text(encoding='utf-8')
    assert "case 'k':case 'K':" in controller
    assert "case 'a':case 'A':" in controller
    assert 'userValue.rx = max<float>' in controller
    assert "case 'k':" in ros1_tool
    assert "case 'a':" in ros1_tool
    assert 'twist.angular.z = 1.0;' in ros1_tool
