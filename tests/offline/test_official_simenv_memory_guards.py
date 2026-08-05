"""官方 SimEnv 长时运行内存保护的离线结构测试。

文件作用：防止 gzserver 存活检查或 Unitree 控制插件幂等注册在后续合并中被误删。
真实内存曲线仍需在官方容器内验证，本测试只校验关键保护代码已进入仓库。
"""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PLATFORM_ROOT = REPO_ROOT / "ros2_ws" / "src" / "hazardwalker_platform"


def test_container_health_requires_gzserver_process():
    """仿真子进程被 OOM 杀死后，不得依靠旧 ready 文件误报 healthy。"""
    compose = (PLATFORM_ROOT / "docker" / "docker-compose.yml").read_text(
        encoding="utf-8"
    )
    assert "pgrep -x gzserver" in compose
    assert "test -f /tmp/hazardwalker-simenv-runtime-ready" in compose


def test_unitree_controller_registration_is_idempotent():
    """同名插件重复注册时必须复用工厂，避免 MetaObject 无界泄漏。"""
    source = (
        PLATFORM_ROOT
        / "src"
        / "unitree_guide"
        / "unitree_ros"
        / "unitree_legged_control"
        / "src"
        / "joint_controller.cpp"
    ).read_text(encoding="utf-8")

    assert "registerUnitreeJointControllerOnce" in source
    assert "factory_map.find(class_name)" in source
    assert "existing->second->addOwningClassLoader" in source
    assert "PLUGINLIB_EXPORT_CLASS(" not in source
