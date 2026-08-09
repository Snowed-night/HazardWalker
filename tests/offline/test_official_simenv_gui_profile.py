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
    assert 'openbox' in dockerfile
    assert 'wmctrl' in dockerfile
    assert 'LIBGL_ALWAYS_SOFTWARE=1' in entrypoint
    assert 'GALLIUM_DRIVER' in entrypoint
    assert 'GUI_GEOMETRY' in entrypoint
    assert 'gzclient -geometry "$GUI_GEOMETRY" --verbose' in entrypoint
    assert 'openbox --sm-disable' in entrypoint
    assert 'wmctrl -r Gazebo -b add,maximized_vert,maximized_horz' in entrypoint
    assert 'DISPLAY_LOCK' in entrypoint
    assert '127.0.0.1:${NOVNC_PORT}' in entrypoint


def test_fullscreen_viewer_fills_viewport_and_keeps_gazebo_mouse_input():
    dockerfile = (DOCKER_DIR / 'Dockerfile.gui').read_text(encoding='utf-8')
    viewer = (DOCKER_DIR / 'gui_fullscreen.html').read_text(encoding='utf-8')
    assert 'gui_fullscreen.html /usr/share/novnc/hazardwalker.html' in dockerfile
    assert 'rfb.scaleViewport = true' in viewer
    assert 'rfb.addEventListener(\'connect\'' in viewer
    assert 'position: fixed' in viewer
    # GUI 视角必须允许鼠标旋转、平移和缩放，机器狗速度仍只由 ROS2 键盘节点发布。
    assert 'rfb.viewOnly = false' in viewer
    assert 'rfb.clipViewport = false' in viewer
    assert 'height: 100vh' in viewer


def test_gui_client_only_manages_its_own_sidecar():
    client = (DOCKER_DIR / 'gui_client.sh').read_text(encoding='utf-8')
    assert 'SIMENV_GUI_CONTAINER' in client
    assert 'docker run -d' in client
    assert 'readonly' in client
    assert 'gui_fullscreen.html,dst=/usr/share/novnc/hazardwalker.html,readonly' in client
    assert 'gui_entrypoint.sh,dst=/usr/local/bin/hazardwalker_gui_entrypoint.sh,readonly' in client
    assert 'docker restart' not in client
    assert 'docker stop "$MAIN_CONTAINER"' not in client
    assert 'SIMENV_GUI_RESOLUTION:-1280x720x24' in client


def test_auto_docker_exposes_gui_profile_without_changing_normal_up():
    source = (DOCKER_DIR.parent / 'auto_docker.sh').read_text(encoding='utf-8')
    ast.parse('def placeholder():\n    pass\n')
    assert '  gui)' in source
    assert 'docker/gui_client.sh' in source
    assert '  first_person)' in source
    assert 'docker/first_person_client.sh' in source


def test_first_person_sidecar_waits_for_http_health_before_claiming_ready():
    client = (DOCKER_DIR / 'first_person_client.sh').read_text(encoding='utf-8')
    assert 'wait_for_health()' in client
    assert '/healthz' in client
    assert 'curl --fail --silent --show-error' in client
    assert '已有第一人称 sidecar 无健康响应' in client
    assert 'docker logs --tail 80' in client
    assert 'docker rm -f "$FPV_CONTAINER"' in client


def test_first_person_page_uses_confirmed_assist_api_without_velocity_control():
    """网页只可请求既有辅助服务，不能直接发布任意速度。"""

    source = (DOCKER_DIR / 'first_person_server.py').read_text(encoding='utf-8')
    assert 'class OverlayState' in source
    assert 'path == "/state"' in source
    assert "'/hazardwalker/gui/perception'" in source
    assert 'detection.requires_reobservation' in source
    assert "detection.track_status === 'confirmed'" in source
    assert "trackStatus.startsWith('rejected')" in source
    assert "'已排除非球体'" in source
    assert 'detection.raw_surface_depth_m' in source
    assert 'detection.localized_position' in source
    assert 'linkedHazard.position' in source
    assert 'view_recommendation' in source
    assert 'state_age_sec' in source
    assert '感知状态超时' in source
    assert 'assistReasonLabels' in source
    assert "target_not_visible: '目标已离开画面'" in source
    assert "turn_left: '左转复查'" in source
    assert 'Number(ages.perception) <= 1.0' in source
    assert 'Math.abs(frameStamp - detectionStamp) <= 0.25' in source
    assert '画面与检测未同步' in source
    assert '"ros_stamp_sec": self._ros_stamp_sec' in source
    assert "const frameBuffer = document.createElement('canvas')" in source
    assert 'context.drawImage(frameBuffer' in source
    assert 'window.confirm' in source
    assert "'/assist/start': 'start'" in source
    assert "'/assist/cancel': 'cancel'" in source
    assert "'X-HazardWalker-Confirm': '1'" in source
    assert "self.headers.get('X-HazardWalker-Confirm') != '1'" in source
    assert "'/hazardwalker/gui/assist_request'" in source
    assert '/cmd_vel' not in source
    ast.parse(source, feature_version=(3, 8))
