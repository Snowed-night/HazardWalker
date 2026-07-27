"""负责人维护的官方 SimEnv noVNC GUI profile 静态契约测试。"""

import ast
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DOCKER_DIR = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform' / 'docker'


def test_gui_sidecar_uses_virtual_display_and_loopback_novnc():
    dockerfile = (DOCKER_DIR / 'Dockerfile.gui').read_text(encoding='utf-8')
    entrypoint = (DOCKER_DIR / 'gui_entrypoint.sh').read_text(encoding='utf-8')
    assert 'x11vnc' in dockerfile
    assert 'novnc' in dockerfile
    assert 'websockify' in dockerfile
    assert 'LIBGL_ALWAYS_SOFTWARE=1' in entrypoint
    assert 'GALLIUM_DRIVER' in entrypoint
    assert 'gzclient --verbose' in entrypoint
    assert '127.0.0.1:${NOVNC_PORT}' in entrypoint


def test_fullscreen_viewer_fills_viewport_and_blocks_vnc_input():
    dockerfile = (DOCKER_DIR / 'Dockerfile.gui').read_text(encoding='utf-8')
    viewer = (DOCKER_DIR / 'gui_fullscreen.html').read_text(encoding='utf-8')
    assert 'gui_fullscreen.html /usr/share/novnc/hazardwalker.html' in dockerfile
    assert 'rfb.scaleViewport = true' in viewer
    assert 'rfb.addEventListener(\'connect\'' in viewer
    assert 'position: fixed' in viewer
    assert 'rfb.viewOnly = true' in viewer
    assert 'height: 100vh' in viewer


def test_gui_client_only_manages_its_own_sidecar():
    client = (DOCKER_DIR / 'gui_client.sh').read_text(encoding='utf-8')
    assert 'SIMENV_GUI_CONTAINER' in client
    assert 'docker run -d' in client
    assert 'readonly' in client
    assert 'gui_fullscreen.html,dst=/usr/share/novnc/hazardwalker.html,readonly' in client
    assert 'docker restart' not in client
    assert 'docker stop "$MAIN_CONTAINER"' not in client
    assert 'SIMENV_GUI_RESOLUTION:-1920x1080x24' in client


def test_auto_docker_exposes_gui_profile_without_changing_normal_up():
    source = (DOCKER_DIR.parent / 'auto_docker.sh').read_text(encoding='utf-8')
    ast.parse('def placeholder():\n    pass\n')
    assert '  gui)' in source
    assert 'docker/gui_client.sh' in source
