#!/usr/bin/env python3
"""只读验收官方人工巡检的传感器、感知、控制和定位实时链路。

所属组：感知定位组。负责人：姜晨。
文件作用：在正式录包前检查 ROS2 图和短时消息流，不发布控制命令；重点阻止
多发布者争用 ``/hw/cmd_vel``、错误控制源、无合法 SLAM、无 RGB-D 或无感知输出
的无效巡检进入成果目录。
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
from urllib.error import URLError
from urllib.request import Request, urlopen


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / 'scripts'
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from git_provenance import read_git_state  # noqa: E402


TOPIC_TYPES = {
    '/clock': 'rosgraph_msgs/msg/Clock',
    '/hw/camera/image_raw': 'sensor_msgs/msg/Image',
    '/hw/camera/depth_image': 'sensor_msgs/msg/Image',
    '/hw/camera/camera_info': 'sensor_msgs/msg/CameraInfo',
    '/hw/camera/depth_camera_info': 'sensor_msgs/msg/CameraInfo',
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
    '/hazardwalker/slam/odometry': 'nav_msgs/msg/Odometry',
    '/hazardwalker/slam/localization_provenance': 'std_msgs/msg/String',
    '/hw/scan': 'sensor_msgs/msg/LaserScan',
    '/hw/trunk_imu': 'sensor_msgs/msg/Imu',
    '/map': 'nav_msgs/msg/OccupancyGrid',
    '/hw/control/assist_cmd_vel': 'geometry_msgs/msg/Twist',
    '/hw/control/status': 'std_msgs/msg/String',
    '/hw/control/assist_status': 'std_msgs/msg/String',
    '/hw/cmd_vel': 'geometry_msgs/msg/Twist',
    '/hw/platform/official_simenv_adapter_status': 'std_msgs/msg/String',
    '/hw/perception/hazard_detections': 'std_msgs/msg/String',
    '/hw/perception/view_recommendation': 'std_msgs/msg/String',
    '/hw/perception/patrol_coverage': 'std_msgs/msg/String',
}

CONTROL_SOURCE_TOPICS = {
    'keyboard': '/hw/control/keyboard_cmd_vel',
    'navigation': '/hw/control/navigation_cmd_vel',
}

NAVIGATION_TOPIC_TYPES = {
    '/hw/scan': 'sensor_msgs/msg/LaserScan',
    '/map': 'nav_msgs/msg/OccupancyGrid',
}

# 静态 TF 可能在预检订阅建立前已发布，不以短时计数判失败；其他项目必须在
# 仿真解除暂停后持续产生，才能证明本轮将录到可回放数据。
REQUIRED_LIVE_TOPICS = (
    '/clock',
    '/hw/camera/image_raw',
    '/hw/camera/depth_image',
    '/hw/camera/camera_info',
    '/hw/camera/depth_camera_info',
    '/tf',
    '/hazardwalker/slam/odometry',
    '/hazardwalker/slam/localization_provenance',
    '/hw/scan',
    '/hw/trunk_imu',
    '/map',
    '/hw/control/status',
    '/hw/control/assist_status',
    '/hw/cmd_vel',
    '/hw/platform/official_simenv_adapter_status',
    '/hw/perception/hazard_detections',
    '/hw/perception/view_recommendation',
    '/hw/perception/patrol_coverage',
)

# “短窗口出现过消息”只能证明链路存在，不能证明辅助对准可实时闭环。
# 正式预检对关键 RGB-D 与感知输出施加保守的最低实测频率；5 Hz 默认桥接
# 在共享主机轻度抖动时仍应稳定高于 2 Hz。
MINIMUM_LIVE_RATE_HZ = {
    '/hw/camera/image_raw': 2.0,
    '/hw/camera/depth_image': 2.0,
    '/hw/perception/hazard_detections': 2.0,
    '/hw/perception/view_recommendation': 2.0,
    '/hw/cmd_vel': 5.0,
    '/hw/perception/patrol_coverage': 0.5,
}

REQUIRED_SERVICES = (
    '/hw/control/assist_align/start',
    '/hw/control/assist_align/cancel',
    '/hw/perception/patrol_coverage/reset',
)

PROVENANCE_PUBLISHERS = {
    'lidar_imu_slam': 'hazardwalker_scan_imu_localizer',
    'lidar_imu_slam+public_floor_action': 'hazardwalker_scan_imu_localizer',
    'visual_inertial_slam': 'hazardwalker_visual_inertial_localizer',
}

PLATFORM_RELAY_TOPICS = (
    '/clock',
    '/hw/camera/image_raw',
    '/hw/camera/depth_image',
    '/hw/camera/camera_info',
    '/hw/camera/depth_camera_info',
    '/hw/scan',
    '/hw/trunk_imu',
)

ALLOWED_MAP_PUBLISHERS = (
    # 当前 launch 显式把 Cartographer 占据栅格节点命名为下列名称；同时
    # 保留上游默认名，兼容直接启动官方 cartographer launch 的诊断流程。
    'hazardwalker_cartographer_occupancy_grid',
    'cartographer_occupancy_grid_node',
    'slam_toolbox',
)


def evaluate_git_state(state: dict, *, graph_only: bool) -> list[str]:
    """正式预检绑定干净提交；graph-only 诊断不阻断本地开发。"""

    if graph_only:
        return []
    failures = []
    if not str(state.get('commit', '')).strip():
        failures.append('正式预检无法解析 Git 提交')
    if state.get('dirty') is not False:
        failures.append('正式预检拒绝未提交代码或配置')
    return failures


def fetch_first_person_health(url: str, timeout_sec: float = 2.0) -> dict:
    """只读获取第一人称 sidecar 健康状态，不发送辅助或速度请求。"""

    if not str(url).strip() or timeout_sec <= 0.0:
        raise ValueError('第一人称健康检查参数无效')
    request = Request(str(url), headers={'Accept': 'application/json'})
    try:
        with urlopen(request, timeout=timeout_sec) as response:
            payload = json.loads(response.read().decode('utf-8'))
    except (OSError, URLError, UnicodeDecodeError,
            json.JSONDecodeError) as exc:
        raise ValueError(f'第一人称健康接口不可用：{exc}') from exc
    if not isinstance(payload, dict):
        raise ValueError('第一人称健康接口返回格式错误')
    return payload


def _node_path(endpoint: dict) -> str:
    namespace = str(endpoint.get('node_namespace', '/')).rstrip('/')
    return f"{namespace}/{endpoint.get('node_name', '')}".replace('//', '/')


def evaluate_snapshot(
        snapshot: dict, control_source: str,
        expected_localization_provenance: str = '') -> list[str]:
    """对可序列化的 ROS 图快照做确定性门禁，便于离线回归。"""

    if control_source not in CONTROL_SOURCE_TOPICS:
        raise ValueError(f'未知控制源：{control_source}')
    if (expected_localization_provenance
            and expected_localization_provenance not in PROVENANCE_PUBLISHERS):
        raise ValueError(
            f'未知定位来源：{expected_localization_provenance}')
    failures = []
    topics = snapshot.get('topics', {})
    expected_types = dict(TOPIC_TYPES)
    source_topic = CONTROL_SOURCE_TOPICS[control_source]
    expected_types[source_topic] = 'geometry_msgs/msg/Twist'
    if control_source == 'navigation':
        expected_types.update(NAVIGATION_TOPIC_TYPES)
    for topic, expected_type in expected_types.items():
        entry = topics.get(topic)
        if not entry:
            failures.append(f'缺少话题：{topic}')
            continue
        if expected_type not in entry.get('types', []):
            failures.append(
                f'话题类型错误：{topic}，期望 {expected_type}，'
                f"实际 {entry.get('types', [])}")

    output = topics.get('/hw/cmd_vel', {})
    output_publishers = output.get('publishers', [])
    if len(output_publishers) != 1:
        failures.append(
            f'/hw/cmd_vel 必须恰好一个发布者，实际 {len(output_publishers)}')
    elif not _node_path(output_publishers[0]).endswith(
            '/hazardwalker_command_mux'):
        failures.append(
            '/hw/cmd_vel 唯一发布者不是 hazardwalker_command_mux：'
            + _node_path(output_publishers[0]))
    if not output.get('subscribers'):
        failures.append('/hw/cmd_vel 没有平台适配器订阅者')

    selected = topics.get(source_topic, {})
    if not selected.get('publishers'):
        failures.append(f'所选控制源没有发布者：{source_topic}')
    if not any(_node_path(item).endswith('/hazardwalker_command_mux')
               for item in selected.get('subscribers', [])):
        failures.append(f'控制复用器未订阅所选控制源：{source_topic}')

    if control_source == 'navigation':
        detection_subscribers = topics.get(
            '/hw/perception/hazard_detections', {}).get('subscribers', [])
        if not any(_node_path(item).endswith('/frontier_explorer_node')
                   for item in detection_subscribers):
            failures.append(
                '导航模式缺少 frontier_explorer_node 对感知复查载荷的订阅')

    for topic, expected_node in (
        ('/hw/control/assist_cmd_vel', 'hazardwalker_assist_alignment'),
        ('/hw/control/status', 'hazardwalker_command_mux'),
        ('/hw/control/assist_status', 'hazardwalker_assist_alignment'),
        ('/hw/platform/official_simenv_adapter_status',
         'hazardwalker_official_rosbridge_adapter'),
        ('/hw/perception/hazard_detections', 'hsv_detector_node'),
        ('/hw/perception/view_recommendation', 'hsv_detector_node'),
        ('/hw/perception/patrol_coverage',
         'hazardwalker_patrol_coverage'),
    ):
        publishers = topics.get(topic, {}).get('publishers', [])
        if len(publishers) != 1:
            failures.append(f'{topic} 必须恰好一个发布者，实际 {len(publishers)}')
        elif not _node_path(publishers[0]).endswith('/' + expected_node):
            failures.append(
                f'{topic} 发布者错误：{_node_path(publishers[0])}')

    for topic in PLATFORM_RELAY_TOPICS:
        publishers = topics.get(topic, {}).get('publishers', [])
        if len(publishers) != 1:
            failures.append(
                f'{topic} 必须恰好一个平台适配器发布者，实际 '
                f'{len(publishers)}')
        elif not _node_path(publishers[0]).endswith(
                '/hazardwalker_official_rosbridge_adapter'):
            failures.append(
                f'{topic} 发布者不是官方平台适配器：'
                f'{_node_path(publishers[0])}')

    map_publishers = topics.get('/map', {}).get('publishers', [])
    if len(map_publishers) != 1:
        failures.append(
            f'/map 必须恰好一个合法建图发布者，实际 {len(map_publishers)}')
    elif not any(_node_path(map_publishers[0]).endswith('/' + node)
                 for node in ALLOWED_MAP_PUBLISHERS):
        failures.append(
            '/map 发布者不是受支持的 SLAM 节点：'
            + _node_path(map_publishers[0]))

    # 定位话题必须由声明来源对应的合法节点唯一拥有。只检查消息存在会让
    # 误接的 Gazebo 派生里程计或旧残留节点混入三维定位证据。
    provenance_payload = snapshot.get('latest_string_payloads', {}).get(
        '/hazardwalker/slam/localization_provenance', '').strip()
    declared_provenance = (
        expected_localization_provenance or provenance_payload)
    expected_pose_publisher = PROVENANCE_PUBLISHERS.get(declared_provenance)
    for topic in (
        '/hazardwalker/slam/odometry',
        '/hazardwalker/slam/localization_provenance',
    ):
        publishers = topics.get(topic, {}).get('publishers', [])
        if len(publishers) != 1:
            failures.append(
                f'{topic} 必须恰好一个合法定位发布者，实际 {len(publishers)}')
        elif expected_pose_publisher and not _node_path(
                publishers[0]).endswith('/' + expected_pose_publisher):
            failures.append(
                f'{topic} 发布者与定位来源不符：'
                f'{_node_path(publishers[0])}')

    services = set(snapshot.get('services', []))
    for service in REQUIRED_SERVICES:
        if service not in services:
            label = ('巡检覆盖清零服务' if 'patrol_coverage' in service
                     else '辅助复查服务')
            failures.append(f'缺少{label}：{service}')

    adapter_payload = snapshot.get('latest_string_payloads', {}).get(
        '/hw/platform/official_simenv_adapter_status')
    if snapshot.get('traffic_checked') and not adapter_payload:
        failures.append('缺少平台适配器实时状态载荷')
    elif snapshot.get('traffic_checked') and adapter_payload:
        try:
            adapter_status = json.loads(adapter_payload)
        except json.JSONDecodeError:
            failures.append('平台适配器状态不是有效 JSON')
        else:
            if adapter_status.get('managed_lifecycle') is not True:
                failures.append('平台适配器不是由 auto_docker.sh 统一管理的实例')
            if not adapter_status.get('lifecycle_container'):
                failures.append('平台适配器未声明其生命周期所属容器')
            if adapter_status.get('enable_cmd_vel_relay') is not True:
                failures.append('平台适配器未启用 /hw/cmd_vel→ROS1 /cmd_vel 控制转发')
            if adapter_status.get('enable_gui_overlay_relay') is not True:
                failures.append('平台适配器未启用第一人称 GUI 状态与辅助请求转发')
            if adapter_status.get('gui_assist_request_topic') != (
                    '/hazardwalker/gui/assist_request'):
                failures.append('平台适配器 GUI 辅助请求话题不符合统一合同')
            try:
                image_throttle_ms = int(
                    adapter_status.get('image_throttle_rate_ms'))
            except (TypeError, ValueError):
                failures.append('平台适配器未声明有效图像节流周期')
            else:
                if not 1 <= image_throttle_ms <= 250:
                    failures.append(
                        '平台适配器图像桥接过慢；正式实时感知要求不超过 250 ms')

    coverage_payload = snapshot.get('latest_string_payloads', {}).get(
        '/hw/perception/patrol_coverage')
    if snapshot.get('traffic_checked'):
        try:
            coverage_status = json.loads(coverage_payload or '')
        except json.JSONDecodeError:
            failures.append('巡检覆盖状态不是有效 JSON')
        else:
            if not isinstance(coverage_status, dict) or any(
                    key not in coverage_status for key in (
                        'sample_count', 'planar_path_length_m',
                        'planar_span_m')):
                failures.append('巡检覆盖状态字段不完整')

    if snapshot.get('traffic_checked'):
        if provenance_payload not in PROVENANCE_PUBLISHERS:
            failures.append('运行时定位来源声明缺失或不合法')
        elif (expected_localization_provenance
              and provenance_payload != expected_localization_provenance):
            failures.append(
                '运行时定位来源与本轮预检声明不一致：'
                f'{provenance_payload} != '
                f'{expected_localization_provenance}')
        counts = snapshot.get('message_counts', {})
        live_topics = list(REQUIRED_LIVE_TOPICS)
        if control_source == 'navigation':
            live_topics.extend(NAVIGATION_TOPIC_TYPES)
        for topic in live_topics:
            if int(counts.get(topic, 0)) <= 0:
                failures.append(
                    f'采样窗口无消息：{topic}（确认仿真已解除暂停）')
        sample_sec = float(snapshot.get('sample_sec', 0.0) or 0.0)
        if sample_sec > 0.0:
            for topic, minimum_hz in MINIMUM_LIVE_RATE_HZ.items():
                measured_hz = float(counts.get(topic, 0)) / sample_sec
                if measured_hz < minimum_hz:
                    failures.append(
                        f'实时频率不足：{topic}={measured_hz:.2f} Hz，'
                        f'最低 {minimum_hz:.2f} Hz')
        first_person = snapshot.get('first_person_health')
        if not isinstance(first_person, dict):
            failures.append('缺少第一人称 GUI 健康状态')
        else:
            frame = first_person.get('frame', {})
            if frame.get('ready') is not True:
                failures.append('第一人称 GUI 尚未收到相机帧')
            frame_age = frame.get('frame_age_sec')
            if not isinstance(frame_age, (int, float)) or frame_age > 2.0:
                failures.append('第一人称 GUI 相机帧已超时')
            ages = first_person.get('overlay', {}).get('state_age_sec', {})
            for name, maximum in (
                    ('perception', 1.0), ('control', 2.5), ('assist', 2.5)):
                age = ages.get(name)
                if not isinstance(age, (int, float)) or age > maximum:
                    failures.append(f'第一人称 GUI {name} 叠加状态缺失或超时')
            overlay = first_person.get('overlay', {})
            perception_overlay = overlay.get('perception', {})
            frame_stamp = frame.get('ros_stamp_sec')
            detection_stamp = (
                perception_overlay.get('stamp_sec')
                if isinstance(perception_overlay, dict) else None)
            if not all(
                    isinstance(value, (int, float))
                    and math.isfinite(float(value))
                    for value in (frame_stamp, detection_stamp)):
                failures.append('第一人称 GUI 缺少画面/检测时间戳')
            elif abs(float(frame_stamp) - float(detection_stamp)) > 0.25:
                failures.append('第一人称 GUI 画面与检测叠加未同步')
            try:
                dimensions_valid = (
                    int(perception_overlay.get('image_width', 0)) > 0
                    and int(perception_overlay.get('image_height', 0)) > 0
                ) if isinstance(perception_overlay, dict) else False
            except (TypeError, ValueError):
                dimensions_valid = False
            if (
                not isinstance(perception_overlay, dict)
                or not isinstance(perception_overlay.get('detections_2d'), list)
                or not isinstance(
                    perception_overlay.get('view_recommendation'), dict)
                or not dimensions_valid
            ):
                failures.append('第一人称 GUI 感知叠加载荷不完整')
    return failures


def capture_snapshot(control_source: str, graph_timeout_sec: float,
                     sample_sec: float, graph_only: bool) -> dict:
    """使用 rclpy 捕获端点及短时消息计数；本函数不创建任何发布者。"""

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = Node('hazardwalker_perception_live_chain_verifier')
    subscriptions = []
    counts = defaultdict(int)
    latest_string_payloads = {}
    expected_types = dict(TOPIC_TYPES)
    expected_types[CONTROL_SOURCE_TOPICS[control_source]] = (
        'geometry_msgs/msg/Twist')
    if control_source == 'navigation':
        expected_types.update(NAVIGATION_TOPIC_TYPES)
    deadline = time.monotonic() + graph_timeout_sec
    topic_map = {}
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            topic_map = dict(node.get_topic_names_and_types())
            if all(topic in topic_map for topic in expected_types):
                break

        for topic, type_name in expected_types.items():
            if graph_only or topic not in topic_map:
                continue
            message_type = get_message(type_name)

            def on_message(message, observed_topic=topic):
                counts[observed_topic] += 1
                if hasattr(message, 'data'):
                    latest_string_payloads[observed_topic] = str(message.data)

            qos = qos_profile_sensor_data
            if topic == '/hazardwalker/slam/localization_provenance':
                qos = QoSProfile(depth=1)
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            subscriptions.append(node.create_subscription(
                message_type, topic, on_message, qos))

        if not graph_only:
            traffic_deadline = time.monotonic() + sample_sec
            while time.monotonic() < traffic_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)

        topic_map = dict(node.get_topic_names_and_types())
        topics = {}
        for topic in expected_types:
            publishers = node.get_publishers_info_by_topic(topic)
            subscribers = node.get_subscriptions_info_by_topic(topic)
            topics[topic] = {
                'types': topic_map.get(topic, []),
                'publishers': [{
                    'node_name': item.node_name,
                    'node_namespace': item.node_namespace,
                } for item in publishers],
                'subscribers': [{
                    'node_name': item.node_name,
                    'node_namespace': item.node_namespace,
                } for item in subscribers if item.node_name != node.get_name()],
            }
        services = sorted(name for name, _types in
                          node.get_service_names_and_types())
        return {
            'schema': 'hazardwalker_perception_live_chain_preflight_v1',
            'generated_at_utc': datetime.now(timezone.utc).isoformat(),
            'control_source': control_source,
            'graph_timeout_sec': graph_timeout_sec,
            'sample_sec': 0.0 if graph_only else sample_sec,
            'traffic_checked': not graph_only,
            'topics': topics,
            'services': services,
            'message_counts': dict(counts),
            'latest_string_payloads': latest_string_payloads,
        }
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--control-source', choices=sorted(CONTROL_SOURCE_TOPICS),
        default='keyboard')
    parser.add_argument('--graph-timeout-sec', type=float, default=15.0)
    parser.add_argument('--sample-sec', type=float, default=3.0)
    parser.add_argument(
        '--graph-only', action='store_true',
        help='只检查端点；正式录包前不应使用此选项')
    parser.add_argument(
        '--first-person-health-url',
        default='http://127.0.0.1:6082/healthz',
        help='第一人称 sidecar 健康接口；正式预检必须可访问')
    parser.add_argument(
        '--localization-provenance',
        choices=sorted(PROVENANCE_PUBLISHERS), default='',
        help='本轮合法定位来源；正式预检必须显式填写并与运行时声明一致')
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    if args.graph_timeout_sec <= 0 or args.sample_sec <= 0:
        raise SystemExit('graph-timeout-sec 和 sample-sec 必须为正数')
    if not args.graph_only and not args.localization_provenance:
        raise SystemExit('正式预检必须显式指定 --localization-provenance')
    snapshot = capture_snapshot(
        args.control_source, args.graph_timeout_sec,
        args.sample_sec, args.graph_only)
    snapshot['git'] = read_git_state(REPO_ROOT)
    snapshot['expected_localization_provenance'] = (
        args.localization_provenance)
    if not args.graph_only:
        try:
            snapshot['first_person_health'] = fetch_first_person_health(
                args.first_person_health_url)
        except ValueError as exc:
            snapshot['first_person_health_error'] = str(exc)
    failures = evaluate_snapshot(
        snapshot, args.control_source, args.localization_provenance)
    failures.extend(evaluate_git_state(
        snapshot['git'], graph_only=bool(args.graph_only)))
    snapshot['passed'] = not failures
    snapshot['failures'] = failures
    payload = json.dumps(snapshot, ensure_ascii=False, indent=2) + '\n'
    if args.output:
        output = Path(args.output).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(payload, encoding='utf-8')
    print(payload, end='')
    return 0 if snapshot['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
