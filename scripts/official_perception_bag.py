#!/usr/bin/env python3
"""官方人工巡检 rosbag2 数据集录制与安全离线回放入口。

录制内容覆盖 RGB、深度、内参、TF、合法定位、控制命令、检测结果和复查状态。
回放默认只发布传感器与定位输入，控制和历史检测话题会改名到 ``/hw/replay/*``，
避免误控在线机器人或与待测算法争用同名输出。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shlex
import shutil
import signal
import socket
import sqlite3
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / 'scripts'
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
PERCEPTION_SOURCE_DIR = (
    REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception')
if str(PERCEPTION_SOURCE_DIR) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SOURCE_DIR))

from git_provenance import read_git_state  # noqa: E402
from hazardwalker_perception.patrol_coverage import (  # noqa: E402
    validate_patrol_coverage,
)

RECORD_TOPICS = (
    '/clock',
    '/hw/camera/image_raw',
    '/hw/camera/depth_image',
    '/hw/camera/camera_info',
    '/hw/camera/depth_camera_info',
    '/hw/scan',
    '/hw/trunk_imu',
    '/tf',
    '/tf_static',
    '/hazardwalker/slam/odometry',
    '/hazardwalker/slam/localization_provenance',
    '/map',
    '/hw/platform/official_simenv_adapter_status',
    '/hw/control/keyboard_cmd_vel',
    '/hw/control/navigation_cmd_vel',
    '/hw/control/assist_cmd_vel',
    '/hw/control/mode_request',
    '/hw/control/status',
    '/hw/control/assist_status',
    '/hw/cmd_vel',
    '/hw/perception/hazard_detections',
    '/hw/perception/view_recommendation',
    '/hw/perception/patrol_coverage',
    '/hw/mission/state',
    '/hw/mission/event',
    '/hw/mission/result',
)

# 缺少任一公共项都无法完成“感知输入—控制—结果”闭环。键盘和导航是可
# 替换控制源，只要求至少一个源存在；否则切换到导航后会被错误拒绝录制。
REQUIRED_RECORD_TOPICS = (
    '/clock',
    '/hw/camera/image_raw',
    '/hw/camera/depth_image',
    '/hw/camera/camera_info',
    '/hw/camera/depth_camera_info',
    '/hw/scan',
    '/hw/trunk_imu',
    '/tf',
    '/tf_static',
    '/hazardwalker/slam/odometry',
    '/hazardwalker/slam/localization_provenance',
    '/map',
    '/hw/platform/official_simenv_adapter_status',
    '/hw/control/status',
    '/hw/control/assist_status',
    '/hw/cmd_vel',
    '/hw/perception/hazard_detections',
    '/hw/perception/view_recommendation',
    '/hw/perception/patrol_coverage',
)

REQUIRED_ANY_TOPIC_GROUPS = {
    'control_source': (
        '/hw/control/keyboard_cmd_vel',
        '/hw/control/navigation_cmd_vel',
    ),
}

REPLAY_INPUT_TOPICS = (
    '/hw/camera/image_raw',
    '/hw/camera/depth_image',
    '/hw/camera/camera_info',
    '/hw/camera/depth_camera_info',
    '/hw/scan',
    '/hw/trunk_imu',
    '/tf',
    '/tf_static',
    '/hazardwalker/slam/odometry',
    '/hazardwalker/slam/localization_provenance',
    '/map',
)

_CLEAN_RECORD_EXIT_CODES = {0, 130, -int(signal.SIGINT)}

ALLOWED_LOCALIZATION_PROVENANCE = {
    'lidar_imu_slam',
    'visual_inertial_slam',
    'lidar_imu_slam+public_floor_action',
}

ADAPTER_CONTRACT_KEYS = (
    'managed_lifecycle',
    'lifecycle_container',
    'enable_cmd_vel_relay',
    'enable_gui_overlay_relay',
    'gui_assist_request_topic',
    'enable_image_relay',
    'image_throttle_rate_ms',
    'enable_clock_relay',
    'clock_throttle_rate_ms',
    'enable_pointcloud_relay',
    'enable_livox_imu_relay',
    'enable_trunk_imu_relay',
    'enable_odom_relay',
    'odom_throttle_rate_ms',
    'enable_tf_relay',
    'tf_throttle_rate_ms',
    'scan_throttle_rate_ms',
    'scan_self_filter_range_m',
    'sources',
)

# 少于一分钟的启动探测不能代表完成了一轮复杂楼宇人工巡检，也不允许进入
# 固定 SEED 回归集。该门槛只用于拒绝明显无效的短录包，不替代覆盖率验收。
MINIMUM_PATROL_DURATION_SEC = 60.0
MINIMUM_PATROL_COVERAGE_SAMPLES = 20
MINIMUM_PATROL_PATH_LENGTH_M = 8.0
MINIMUM_PATROL_PLANAR_SPAN_M = 3.0

AUDIT_REPLAY_TOPICS = (
    '/hw/platform/official_simenv_adapter_status',
    '/hw/control/keyboard_cmd_vel',
    '/hw/control/navigation_cmd_vel',
    '/hw/control/assist_cmd_vel',
    '/hw/control/mode_request',
    '/hw/control/status',
    '/hw/control/assist_status',
    '/hw/cmd_vel',
    '/hw/perception/hazard_detections',
    '/hw/perception/view_recommendation',
    '/hw/perception/patrol_coverage',
    '/hw/mission/state',
    '/hw/mission/event',
    '/hw/mission/result',
)


def build_record_command(bag_dir: Path, topics: Sequence[str]) -> list[str]:
    """构造 rosbag2 录制命令，并以官方 ``/clock`` 作为袋时间轴。"""

    return [
        'ros2', 'bag', 'record',
        '--storage', 'sqlite3',
        '--output', str(bag_dir),
        '--use-sim-time',
        '--topics',
        *topics,
    ]


def build_replay_command(
    bag_dir: Path,
    *,
    rate: float = 1.0,
    include_audit_topics: bool = False,
    recompute_localization: bool = False,
) -> list[str]:
    """构造不会发布在线控制话题的回放命令。"""

    if rate <= 0.0:
        raise ValueError('回放倍率必须为正数')
    topics = list(REPLAY_INPUT_TOPICS)
    if recompute_localization:
        # 重跑定位/SLAM 时只回放公开原始传感器；删除历史定位、地图和 TF，
        # 防止新旧算法同时发布同名坐标链。
        topics = [
            topic for topic in topics
            if topic not in (
                '/tf', '/tf_static', '/hazardwalker/slam/odometry',
                '/hazardwalker/slam/localization_provenance', '/map'
            )
        ]
    command = [
        'ros2', 'bag', 'play', str(bag_dir),
        '--clock', '--rate', str(rate),
    ]
    if include_audit_topics:
        topics.extend(AUDIT_REPLAY_TOPICS)
        # Jazzy 的 ``--remap`` 是单个 nargs='+' 参数；重复选项会覆盖前值，
        # 因此所有规则必须跟在同一个 --remap 后，再由 --topics 终止该列表。
        command.extend([
            '--remap',
            *(f'{topic}:={_audit_topic(topic)}'
              for topic in AUDIT_REPLAY_TOPICS),
        ])
    command.extend(['--topics', *topics])
    return command


def git_metadata() -> dict:
    """读取可复现版本信息；失败时显式留空，不伪造提交。"""

    return read_git_state(REPO_ROOT)


def validate_git_state_for_record(
        state: dict, *, skip_topic_preflight: bool,
        allow_dirty_worktree: bool) -> None:
    """正式巡检只接受已提交代码；脏代码仅能生成诊断录包。"""

    if state.get('dirty') is not True:
        return
    if skip_topic_preflight and allow_dirty_worktree:
        return
    entries = state.get('dirty_entries', [])
    changed = '、'.join(str(item) for item in entries[:5]) or 'Git 来源不可解析'
    raise ValueError(
        '正式巡检拒绝未提交代码或配置；临时诊断必须同时使用 '
        '--skip-topic-preflight --allow-dirty-worktree。'
        f' 当前改动：{changed}')


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        while True:
            block = handle.read(4 * 1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def fingerprint_bag_directory(bag_dir: Path) -> dict:
    """对关闭后的 rosbag 全部文件取稳定指纹，证明多轮回放输入相同。"""

    root = Path(bag_dir).resolve()
    if not root.is_dir():
        raise ValueError(f'缺少 rosbag 目录：{root}')
    files = []
    for path in sorted(
            (item for item in root.rglob('*') if item.is_file()),
            key=lambda item: item.relative_to(root).as_posix()):
        files.append({
            'relative_path': path.relative_to(root).as_posix(),
            'size_bytes': path.stat().st_size,
            'sha256': _sha256_file(path),
        })
    if not files:
        raise ValueError(f'rosbag 目录没有文件：{root}')
    encoded = json.dumps(
        files, ensure_ascii=False, sort_keys=True,
        separators=(',', ':'),
    ).encode('utf-8')
    return {
        'sha256': hashlib.sha256(encoded).hexdigest(),
        'files': files,
    }


def write_manifest(path: Path, payload: dict) -> None:
    """以 UTF-8 原子替换清单，异常结束也保留最后状态。"""

    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    temporary.replace(path)


def parse_topic_list(text: str) -> set[str]:
    """解析 ``ros2 topic list``，忽略日志和空行。"""

    return {
        line.strip()
        for line in str(text).splitlines()
        if line.strip().startswith('/')
    }


def parse_localization_provenance_echo(text: str) -> str:
    """解析 ``ros2 topic echo --field data`` 或完整 String YAML 输出。"""

    for raw_line in str(text).splitlines():
        line = raw_line.strip()
        if not line or line == '---':
            continue
        if line.startswith('data:'):
            line = line.split(':', 1)[1].strip()
        value = line.strip("'\"")
        # ros2 CLI 可能把 daemon/QoS 警告和消息一起写入 stdout；只接受
        # 已知来源，不能把第一行诊断文字记录成定位证明。
        if value in ALLOWED_LOCALIZATION_PROVENANCE:
            return value
    return ''


def read_runtime_localization_provenance(timeout_sec: float) -> str:
    """读取 transient-local 定位来源，证明本轮参数与实际节点一致。"""

    if timeout_sec <= 0.0:
        raise ValueError('定位来源等待时间必须为正数')
    try:
        output = subprocess.check_output([
            'ros2', 'topic', 'echo',
            '/hazardwalker/slam/localization_provenance',
            'std_msgs/msg/String',
            '--field', 'data', '--once',
            '--qos-durability', 'transient_local',
            # Jazzy/FastDDS 下仅指定 durability 时，CLI 可能使用与发布者
            # 不完全匹配的默认策略，导致已有缓存样本仍等待至超时。
            '--qos-reliability', 'reliable',
            '--qos-history', 'keep_last',
            '--qos-depth', '1',
        ], cwd=REPO_ROOT, text=True, stderr=subprocess.STDOUT,
           timeout=timeout_sec)
    except (OSError, subprocess.CalledProcessError,
            subprocess.TimeoutExpired) as exc:
        raise RuntimeError('无法读取运行时合法定位来源声明') from exc
    value = parse_localization_provenance_echo(output)
    if not value:
        raise RuntimeError('运行时合法定位来源声明为空')
    return value


def parse_adapter_status_echo(text: str) -> dict:
    """从 ROS2 String 回显中提取适配器 JSON，忽略 CLI 诊断行。"""

    for raw_line in str(text).splitlines():
        line = raw_line.strip()
        if not line or line == '---':
            continue
        if line.startswith('data:'):
            line = line.split(':', 1)[1].strip()
        candidate = line.strip("'\"")
        try:
            payload = json.loads(candidate)
        except (json.JSONDecodeError, TypeError):
            continue
        if isinstance(payload, dict) and payload.get('adapter') == 'rosbridge_ros2':
            return payload
    return {}


def adapter_contract_snapshot(status: dict) -> dict:
    """提取录制期间必须保持不变的数据流配置，排除动态计数。"""

    if status.get('managed_lifecycle') is not True:
        raise ValueError('平台适配器不是 auto_docker.sh 统一管理的实例')
    if not str(status.get('lifecycle_container') or '').strip():
        raise ValueError('平台适配器未声明生命周期所属容器')
    if status.get('enable_cmd_vel_relay') is not True:
        raise ValueError('平台适配器未启用控制转发')
    if status.get('enable_gui_overlay_relay') is not True:
        raise ValueError('平台适配器未启用第一人称 GUI 状态转发')
    if status.get('gui_assist_request_topic') != '/hazardwalker/gui/assist_request':
        raise ValueError('平台适配器辅助复查请求话题不符合合同')
    try:
        image_throttle_ms = int(status.get('image_throttle_rate_ms'))
    except (TypeError, ValueError) as exc:
        raise ValueError('平台适配器未声明有效图像节流周期') from exc
    if not 1 <= image_throttle_ms <= 250:
        raise ValueError('正式实时感知要求图像桥接周期不超过 250 ms')
    return {key: status.get(key) for key in ADAPTER_CONTRACT_KEYS}


def read_runtime_adapter_status(timeout_sec: float) -> dict:
    """读取实时适配器状态，并返回已验证的完整 JSON。"""

    if timeout_sec <= 0.0:
        raise ValueError('适配器状态等待时间必须为正数')
    try:
        output = subprocess.check_output([
            'ros2', 'topic', 'echo',
            '/hw/platform/official_simenv_adapter_status',
            'std_msgs/msg/String', '--field', 'data', '--once',
        ], cwd=REPO_ROOT, text=True, stderr=subprocess.STDOUT,
           timeout=timeout_sec)
    except (OSError, subprocess.CalledProcessError,
            subprocess.TimeoutExpired) as exc:
        raise RuntimeError('无法读取实时平台适配器状态') from exc
    status = parse_adapter_status_echo(output)
    if not status:
        raise RuntimeError('实时平台适配器状态为空或不是有效 JSON')
    adapter_contract_snapshot(status)
    return status


def parse_patrol_coverage_echo(text: str) -> dict:
    """从 ROS2 String 回显提取巡检覆盖 JSON。"""

    for raw_line in str(text).splitlines():
        line = raw_line.strip()
        if not line or line == '---':
            continue
        if line.startswith('data:'):
            line = line.split(':', 1)[1].strip()
        candidate = line.strip("'\"")
        try:
            payload = json.loads(candidate)
        except (json.JSONDecodeError, TypeError):
            continue
        if isinstance(payload, dict) and 'sample_count' in payload:
            return payload
    return {}


def reset_runtime_patrol_coverage(timeout_sec: float) -> None:
    """在 rosbag 启动前清零覆盖计数，不发布任何运动命令。"""

    if timeout_sec <= 0.0:
        raise ValueError('巡检覆盖服务等待时间必须为正数')
    try:
        output = subprocess.check_output([
            'ros2', 'service', 'call',
            '/hw/perception/patrol_coverage/reset',
            'std_srvs/srv/Trigger', '{}',
        ], cwd=REPO_ROOT, text=True, stderr=subprocess.STDOUT,
           timeout=timeout_sec)
    except (OSError, subprocess.CalledProcessError,
            subprocess.TimeoutExpired) as exc:
        raise RuntimeError('无法清零本轮巡检覆盖计数') from exc
    if 'success=true' not in output.lower() and 'success: true' not in output.lower():
        raise RuntimeError('巡检覆盖节点拒绝清零请求')


def read_runtime_patrol_coverage(timeout_sec: float) -> dict:
    """读取覆盖节点最终心跳，供正式录包完成门禁使用。"""

    if timeout_sec <= 0.0:
        raise ValueError('巡检覆盖状态等待时间必须为正数')
    try:
        output = subprocess.check_output([
            'ros2', 'topic', 'echo',
            '/hw/perception/patrol_coverage',
            'std_msgs/msg/String', '--field', 'data', '--once',
        ], cwd=REPO_ROOT, text=True, stderr=subprocess.STDOUT,
           timeout=timeout_sec)
    except (OSError, subprocess.CalledProcessError,
            subprocess.TimeoutExpired) as exc:
        raise RuntimeError('无法读取本轮巡检覆盖状态') from exc
    payload = parse_patrol_coverage_echo(output)
    if not payload:
        raise RuntimeError('本轮巡检覆盖状态为空或不是有效 JSON')
    return payload


def find_missing_topic_requirements(
        available_topics: Iterable[str],
        required_topics: Sequence[str] = REQUIRED_RECORD_TOPICS,
        required_any_groups: dict[str, Sequence[str]] | None = None,
) -> list[str]:
    """返回缺失公共话题及未满足的可替换控制源组。"""

    available = set(available_topics)
    groups = (
        REQUIRED_ANY_TOPIC_GROUPS
        if required_any_groups is None else required_any_groups
    )
    missing = sorted(set(required_topics) - available)
    missing.extend(
        f"{name}:one_of({','.join(candidates)})"
        for name, candidates in groups.items()
        if not set(candidates) & available
    )
    return missing


def inspect_sqlite_bag_message_counts(bag_dir: Path) -> dict[str, int]:
    """汇总 rosbag2 SQLite 分片内各话题消息数。

    只在录制进程退出、数据库完成收尾后调用。若数据库缺失或结构损坏则抛错，
    正式数据集不能仅凭目录存在就标记为完成。
    """

    databases = sorted(Path(bag_dir).glob('*.db3'))
    if not databases:
        raise ValueError(f'rosbag 未生成 SQLite 数据库：{bag_dir}')
    counts: dict[str, int] = {}
    for database in databases:
        connection = sqlite3.connect(str(database))
        try:
            rows = connection.execute(
                'SELECT topics.name, COUNT(messages.id) '
                'FROM topics LEFT JOIN messages '
                'ON messages.topic_id = topics.id GROUP BY topics.id'
            ).fetchall()
        except sqlite3.DatabaseError as exc:
            raise ValueError(f'rosbag 数据库损坏：{database}') from exc
        finally:
            connection.close()
        for topic, count in rows:
            counts[str(topic)] = counts.get(str(topic), 0) + int(count)
    return dict(sorted(counts.items()))


def inspect_sqlite_bag_statistics(bag_dir: Path) -> dict:
    """读取逐话题消息数和完整时间跨度，拒绝缺失时间戳的损坏数据。"""

    counts = inspect_sqlite_bag_message_counts(bag_dir)
    databases = sorted(Path(bag_dir).glob('*.db3'))
    first_timestamp_ns: int | None = None
    last_timestamp_ns: int | None = None
    for database in databases:
        connection = sqlite3.connect(str(database))
        try:
            first, last = connection.execute(
                'SELECT MIN(timestamp), MAX(timestamp) FROM messages'
            ).fetchone()
        except sqlite3.DatabaseError as exc:
            raise ValueError(
                f'rosbag 消息时间戳不可读：{database}') from exc
        finally:
            connection.close()
        if first is None or last is None:
            continue
        first_timestamp_ns = (
            int(first) if first_timestamp_ns is None
            else min(first_timestamp_ns, int(first))
        )
        last_timestamp_ns = (
            int(last) if last_timestamp_ns is None
            else max(last_timestamp_ns, int(last))
        )
    if first_timestamp_ns is None or last_timestamp_ns is None:
        raise ValueError(f'rosbag 没有带时间戳的消息：{bag_dir}')
    return {
        'message_counts': counts,
        'first_timestamp_ns': first_timestamp_ns,
        'last_timestamp_ns': last_timestamp_ns,
        'duration_sec': max(
            0.0, (last_timestamp_ns - first_timestamp_ns) / 1_000_000_000.0),
    }


def validate_record_contract(seed: str, localization_provenance: str) -> None:
    """校验正式录制元数据，防止不可追溯数据进入回归集。"""

    if not str(seed).strip():
        raise ValueError('正式巡检必须提供非空固定 SEED')
    if str(localization_provenance) not in ALLOWED_LOCALIZATION_PROVENANCE:
        raise ValueError('正式巡检必须声明白名单内的合法 SLAM 定位来源')


def validate_live_preflight_report(
        report_path: Path, *, expected_localization_provenance: str,
        maximum_age_sec: float, expected_git_commit: str = '',
        now_utc: datetime | None = None) -> dict:
    """验证刚完成的实时门禁，拒绝失败、过期或来源不一致的报告。"""

    path = Path(report_path).expanduser().resolve()
    if maximum_age_sec <= 0.0:
        raise ValueError('预检报告最大有效期必须为正数')
    if not path.is_file():
        raise ValueError(f'缺少实时预检报告：{path}')
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f'实时预检报告损坏：{path}') from exc
    if not isinstance(payload, dict):
        raise ValueError('实时预检报告格式错误')
    if payload.get('passed') is not True or payload.get('failures'):
        raise ValueError('实时预检未通过')
    if payload.get('traffic_checked') is not True:
        raise ValueError('graph-only 报告不能用于正式录包')
    if payload.get('control_source') not in ('keyboard', 'navigation'):
        raise ValueError('实时预检控制源无效')
    if payload.get('expected_localization_provenance') != (
            expected_localization_provenance):
        raise ValueError('实时预检定位来源与录包参数不一致')
    preflight_git = payload.get('git', {})
    preflight_commit = str(preflight_git.get('commit', '')).strip()
    if not preflight_commit:
        raise ValueError('实时预检报告缺少 Git 提交')
    if preflight_git.get('dirty') is not False:
        raise ValueError('实时预检使用了未提交代码')
    if expected_git_commit and preflight_commit != expected_git_commit:
        raise ValueError('实时预检与录包使用的 Git 提交不一致')
    generated_text = str(payload.get('generated_at_utc', '')).strip()
    try:
        generated_at = datetime.fromisoformat(generated_text)
    except ValueError as exc:
        raise ValueError('实时预检报告缺少有效生成时间') from exc
    if generated_at.tzinfo is None:
        raise ValueError('实时预检报告生成时间缺少时区')
    current = now_utc or datetime.now(timezone.utc)
    age_sec = (current - generated_at.astimezone(timezone.utc)).total_seconds()
    if age_sec < -30.0:
        raise ValueError('实时预检报告时间晚于当前系统时间')
    if age_sec > maximum_age_sec:
        raise ValueError(
            f'实时预检报告已过期：{age_sec:.1f}s > '
            f'{maximum_age_sec:.1f}s')
    return {
        'path': str(path),
        'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'generated_at_utc': generated_at.astimezone(timezone.utc).isoformat(),
        'age_sec_at_record_start': max(0.0, age_sec),
        'control_source': payload.get('control_source'),
        'expected_localization_provenance': (
            expected_localization_provenance),
        'git': {
            'branch': str(preflight_git.get('branch', '')),
            'commit': preflight_commit,
            'dirty': False,
        },
        'passed': True,
    }


def validate_completed_session_manifest(manifest: dict) -> list[str]:
    """复核完成清单，不信任历史或人工填写的单个 ``passed`` 字段。"""

    errors: list[str] = []
    if manifest.get('status') != 'complete':
        errors.append(f"status={manifest.get('status')!r}")
    validation = manifest.get('bag_validation', {})
    if validation.get('status') != 'passed':
        errors.append('bag_validation.status 不是 passed')
    if manifest.get('truth_inputs_used') is not False:
        errors.append('未明确证明运行期未使用仿真真值')
    git_state = manifest.get('git', {})
    if not str(git_state.get('commit', '')).strip():
        errors.append('缺少巡检 Git 提交')
    if git_state.get('dirty') is not False:
        errors.append('巡检使用了未提交代码')
    seed = str(manifest.get('scenario_seed', '')).strip()
    if not seed:
        errors.append('缺少 scenario_seed')
    provenance = str(manifest.get('localization_provenance', 'unverified'))
    if provenance not in ALLOWED_LOCALIZATION_PROVENANCE:
        errors.append('定位来源不在合法 SLAM 白名单')
    runtime_provenance = str(
        manifest.get('runtime_localization_provenance', ''))
    if runtime_provenance != provenance:
        errors.append('运行时定位来源与清单声明不一致')
    live_preflight = manifest.get('live_chain_preflight', {})
    if not isinstance(live_preflight, dict):
        live_preflight = {}
    if live_preflight.get('passed') is not True:
        errors.append('缺少通过的实时链路预检证明')
    if live_preflight.get('expected_localization_provenance') != provenance:
        errors.append('实时预检定位来源与清单声明不一致')
    preflight_git = live_preflight.get('git', {})
    if preflight_git.get('dirty') is not False:
        errors.append('实时预检使用了未提交代码')
    if preflight_git.get('commit') != git_state.get('commit'):
        errors.append('实时预检与巡检 Git 提交不一致')
    preflight_hash = str(live_preflight.get('sha256', ''))
    if len(preflight_hash) != 64 or any(
            character not in '0123456789abcdef'
            for character in preflight_hash.lower()):
        errors.append('实时预检报告 SHA-256 无效')
    if live_preflight.get('relative_path') != 'live_chain_preflight.json':
        errors.append('实时预检报告未按数据集合同保存')
    counts = validation.get('message_counts', {})
    if not isinstance(counts, dict):
        errors.append('bag_validation.message_counts 格式错误')
        counts = {}
    positive_topics = {
        str(topic) for topic, count in counts.items()
        if isinstance(count, (int, float)) and count > 0
    }
    missing = find_missing_topic_requirements(positive_topics)
    if missing:
        errors.append('关键话题无有效消息：' + ', '.join(missing))
    selected_source = str(live_preflight.get('control_source', '')).strip()
    selected_topic = {
        'keyboard': '/hw/control/keyboard_cmd_vel',
        'navigation': '/hw/control/navigation_cmd_vel',
    }.get(selected_source, '')
    if not selected_topic:
        errors.append('实时预检控制源无效')
    elif counts.get(selected_topic, 0) <= 0:
        errors.append(f'巡检未录到所选控制源：{selected_topic}')
    try:
        duration_sec = float(validation.get('duration_sec', -1.0))
    except (TypeError, ValueError):
        duration_sec = -1.0
    if duration_sec < MINIMUM_PATROL_DURATION_SEC:
        errors.append(
            f'巡检时长 {duration_sec:.3f}s 小于最低 '
            f'{MINIMUM_PATROL_DURATION_SEC:.1f}s')
    coverage = manifest.get('patrol_coverage', {})
    if not isinstance(coverage, dict):
        coverage = {}
    if coverage.get('status') != 'passed':
        errors.append('巡检运动覆盖门禁未通过')
    expected_coverage_contract = {
        'minimum_samples': MINIMUM_PATROL_COVERAGE_SAMPLES,
        'minimum_path_length_m': MINIMUM_PATROL_PATH_LENGTH_M,
        'minimum_planar_span_m': MINIMUM_PATROL_PLANAR_SPAN_M,
    }
    for key, expected in expected_coverage_contract.items():
        if coverage.get(key) != expected:
            errors.append(f'巡检覆盖门槛 {key} 与当前正式合同不一致')
    errors.extend(validate_patrol_coverage(
        coverage.get('metrics', {}),
        minimum_samples=MINIMUM_PATROL_COVERAGE_SAMPLES,
        minimum_path_length_m=MINIMUM_PATROL_PATH_LENGTH_M,
        minimum_planar_span_m=MINIMUM_PATROL_PLANAR_SPAN_M,
    ))
    fingerprint = str(validation.get('content_fingerprint_sha256', ''))
    if len(fingerprint) != 64 or any(
            character not in '0123456789abcdef'
            for character in fingerprint.lower()):
        errors.append('rosbag 内容指纹无效')
    if not isinstance(validation.get('files'), list) or not validation.get(
            'files'):
        errors.append('rosbag 文件指纹清单为空')
    return errors


def validate_session_bag_payload(bag_dir: Path, manifest: dict) -> list[str]:
    """直接复核 SQLite 内容与清单，防止清单被修改后伪造有效数据集。"""

    try:
        observed = inspect_sqlite_bag_statistics(bag_dir)
    except ValueError as exc:
        return [str(exc)]
    errors: list[str] = []
    live_preflight = manifest.get('live_chain_preflight', {})
    preflight_relative_path = str(
        live_preflight.get('relative_path', ''))
    preflight_path = Path(bag_dir).parent / preflight_relative_path
    if not preflight_relative_path or not preflight_path.is_file():
        errors.append('数据集缺少实时预检报告副本')
    else:
        preflight_bytes = preflight_path.read_bytes()
        observed_preflight_hash = hashlib.sha256(preflight_bytes).hexdigest()
        if observed_preflight_hash != live_preflight.get('sha256'):
            errors.append('实时预检报告副本哈希与清单不一致')
        try:
            preflight_payload = json.loads(preflight_bytes.decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError):
            preflight_payload = None
            errors.append('实时预检报告副本损坏')
        if not isinstance(preflight_payload, dict):
            errors.append('实时预检报告副本格式错误')
        else:
            if (preflight_payload.get('passed') is not True
                    or preflight_payload.get('failures')):
                errors.append('实时预检报告副本未通过')
            if preflight_payload.get('traffic_checked') is not True:
                errors.append('实时预检报告副本未检查消息流')
            for key in (
                    'control_source', 'expected_localization_provenance',
                    'generated_at_utc'):
                if preflight_payload.get(key) != live_preflight.get(key):
                    errors.append(f'实时预检报告副本 {key} 与清单不一致')
            payload_git = preflight_payload.get('git', {})
            manifest_git = live_preflight.get('git', {})
            if (payload_git.get('commit') != manifest_git.get('commit')
                    or payload_git.get('dirty') is not False):
                errors.append('实时预检报告副本 Git 来源与清单不一致')
    observed_counts = observed['message_counts']
    missing = find_missing_topic_requirements(
        topic for topic, count in observed_counts.items() if count > 0)
    if missing:
        errors.append('rosbag 关键话题无有效消息：' + ', '.join(missing))
    observed_duration = float(observed['duration_sec'])
    if observed_duration < MINIMUM_PATROL_DURATION_SEC:
        errors.append(
            f'rosbag 实际时长 {observed_duration:.3f}s 小于最低 '
            f'{MINIMUM_PATROL_DURATION_SEC:.1f}s')

    validation = manifest.get('bag_validation', {})
    try:
        observed_fingerprint = fingerprint_bag_directory(bag_dir)
    except ValueError as exc:
        errors.append(str(exc))
        observed_fingerprint = {'sha256': '', 'files': []}
    if observed_fingerprint['sha256'] != validation.get(
            'content_fingerprint_sha256'):
        errors.append('rosbag 实际内容指纹与清单不一致')
    if observed_fingerprint['files'] != validation.get('files'):
        errors.append('rosbag 文件指纹清单与实际内容不一致')
    recorded_counts = validation.get('message_counts', {})
    if recorded_counts != observed_counts:
        errors.append('清单消息计数与 rosbag SQLite 实际内容不一致')
    try:
        recorded_duration = float(validation.get('duration_sec', -1.0))
    except (TypeError, ValueError):
        recorded_duration = -1.0
    if abs(recorded_duration - observed_duration) > 1e-6:
        errors.append('清单录制时长与 rosbag SQLite 实际时间跨度不一致')
    for key in ('first_timestamp_ns', 'last_timestamp_ns'):
        if validation.get(key) != observed.get(key):
            errors.append(f'清单 {key} 与 rosbag SQLite 不一致')
    return errors


def wait_for_required_topics(
    required_topics: Sequence[str],
    *,
    timeout_sec: float,
    poll_interval_sec: float = 0.5,
    required_any_groups: dict[str, Sequence[str]] | None = None,
) -> tuple[set[str], list[str]]:
    """等待公共必需话题及每组至少一个可替换话题。"""

    if timeout_sec < 0.0 or poll_interval_sec <= 0.0:
        raise ValueError('话题等待时间无效')
    deadline = time.monotonic() + timeout_sec
    available: set[str] = set()
    groups = required_any_groups or {}
    while True:
        try:
            output = subprocess.check_output(
                ['ros2', 'topic', 'list'],
                cwd=REPO_ROOT,
                text=True,
                stderr=subprocess.STDOUT,
                timeout=max(2.0, poll_interval_sec * 2.0),
            )
        except (OSError, subprocess.CalledProcessError,
                subprocess.TimeoutExpired) as exc:
            if time.monotonic() >= deadline:
                raise RuntimeError('无法读取 ROS2 话题列表') from exc
        else:
            available = parse_topic_list(output)
            missing = find_missing_topic_requirements(
                available, required_topics, groups)
            if not missing:
                return available, []
            if time.monotonic() >= deadline:
                return available, missing
        time.sleep(min(poll_interval_sec, max(0.0, deadline - time.monotonic())))


def record(args) -> int:
    try:
        validate_record_contract(args.seed, args.localization_provenance)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    current_git = git_metadata()
    try:
        validate_git_state_for_record(
            current_git,
            skip_topic_preflight=bool(args.skip_topic_preflight),
            allow_dirty_worktree=bool(args.allow_dirty_worktree),
        )
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    session_dir = Path(args.output).expanduser().resolve()
    if session_dir.exists():
        raise SystemExit(f'输出目录已存在，拒绝覆盖：{session_dir}')
    available_topics: set[str] = set()
    missing_required_topics: list[str] = []
    runtime_localization_provenance = ''
    adapter_status_start: dict = {}
    live_preflight = {}
    if not args.skip_topic_preflight:
        if not str(args.preflight_report).strip():
            raise SystemExit('正式录包必须提供 --preflight-report')
        try:
            live_preflight = validate_live_preflight_report(
                Path(args.preflight_report),
                expected_localization_provenance=str(
                    args.localization_provenance),
                maximum_age_sec=float(args.preflight_max_age_sec),
                expected_git_commit=str(current_git.get('commit', '')),
            )
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc
        available_topics, missing_required_topics = wait_for_required_topics(
            REQUIRED_RECORD_TOPICS,
            timeout_sec=float(args.topic_wait_timeout_sec),
            required_any_groups=REQUIRED_ANY_TOPIC_GROUPS,
        )
        if missing_required_topics:
            raise SystemExit(
                '关键话题未就绪，拒绝开始正式录制：'
                + ', '.join(missing_required_topics)
            )
        runtime_localization_provenance = (
            read_runtime_localization_provenance(
                float(args.topic_wait_timeout_sec)))
        if runtime_localization_provenance != str(
                args.localization_provenance):
            raise SystemExit(
                '运行时定位来源与录包参数不一致：'
                f'{runtime_localization_provenance} != '
                f'{args.localization_provenance}')
        try:
            adapter_status_start = read_runtime_adapter_status(
                float(args.topic_wait_timeout_sec))
        except (RuntimeError, ValueError) as exc:
            raise SystemExit(str(exc)) from exc
        try:
            reset_runtime_patrol_coverage(
                float(args.topic_wait_timeout_sec))
        except (RuntimeError, ValueError) as exc:
            raise SystemExit(str(exc)) from exc
    session_dir.mkdir(parents=True)
    if live_preflight:
        source_preflight = Path(live_preflight['path'])
        copied_preflight = session_dir / 'live_chain_preflight.json'
        shutil.copy2(source_preflight, copied_preflight)
        copied_hash = hashlib.sha256(copied_preflight.read_bytes()).hexdigest()
        if copied_hash != live_preflight['sha256']:
            raise SystemExit('复制实时预检报告后哈希不一致，拒绝录包')
        live_preflight['source_path'] = live_preflight.pop('path')
        live_preflight['relative_path'] = copied_preflight.name
    bag_dir = session_dir / 'bag'
    topics = list(RECORD_TOPICS)
    topics.extend(args.extra_topic)
    topics = list(dict.fromkeys(topics))
    command = build_record_command(bag_dir, topics)
    now = datetime.now(timezone.utc)
    manifest = {
        'schema_version': 1,
        'run_id': args.run_id or session_dir.name,
        'scenario_seed': str(args.seed),
        'operator': args.operator,
        'localization_provenance': str(args.localization_provenance),
        'runtime_localization_provenance': (
            runtime_localization_provenance),
        'adapter_status': {
            'start': adapter_status_start,
            'end': {},
            'contract_consistent': None,
        },
        'started_at_utc': now.isoformat(),
        'finished_at_utc': None,
        'host': socket.gethostname(),
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', ''),
        'git': current_git,
        'bag_relative_path': 'bag',
        'topics': topics,
        'topic_preflight': {
            'skipped': bool(args.skip_topic_preflight),
            'required_topics': list(REQUIRED_RECORD_TOPICS),
            'required_any_topic_groups': {
                key: list(value)
                for key, value in REQUIRED_ANY_TOPIC_GROUPS.items()
            },
            'available_topics': sorted(available_topics),
            'missing_required_topics': missing_required_topics,
            'passed': not args.skip_topic_preflight and not missing_required_topics,
        },
        'live_chain_preflight': live_preflight,
        'record_command': shlex.join(command),
        'bag_validation': {
            'status': 'pending',
            'message_counts': {},
            'missing_required_topics': [],
            'minimum_duration_sec': MINIMUM_PATROL_DURATION_SEC,
            'duration_sec': None,
            'content_fingerprint_sha256': '',
            'files': [],
        },
        'patrol_coverage': {
            'status': 'pending',
            'minimum_samples': MINIMUM_PATROL_COVERAGE_SAMPLES,
            'minimum_path_length_m': MINIMUM_PATROL_PATH_LENGTH_M,
            'minimum_planar_span_m': MINIMUM_PATROL_PLANAR_SPAN_M,
            'metrics': {},
            'errors': [],
        },
        'truth_inputs_used': False,
        'historical_localization_reuse_eligible': False,
        'status': 'recording',
        'exit_code': None,
    }
    manifest_path = session_dir / 'run_manifest.json'
    write_manifest(manifest_path, manifest)
    process = subprocess.Popen(command, cwd=REPO_ROOT)
    recorder_exit_code = 1
    try:
        recorder_exit_code = process.wait()
    except KeyboardInterrupt:
        process.send_signal(signal.SIGINT)
        try:
            recorder_exit_code = process.wait(timeout=30.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            recorder_exit_code = process.wait(timeout=10.0)
    finally:
        validation_error = ''
        message_counts: dict[str, int] = {}
        duration_sec = 0.0
        first_timestamp_ns = None
        last_timestamp_ns = None
        bag_fingerprint = {'sha256': '', 'files': []}
        adapter_status_end: dict = {}
        adapter_contract_consistent = None
        adapter_validation_error = ''
        coverage_payload: dict = {}
        coverage_errors: list[str] = []
        missing_recorded_topics = list(REQUIRED_RECORD_TOPICS)
        try:
            statistics = inspect_sqlite_bag_statistics(bag_dir)
            bag_fingerprint = fingerprint_bag_directory(bag_dir)
            message_counts = statistics['message_counts']
            duration_sec = float(statistics['duration_sec'])
            first_timestamp_ns = statistics['first_timestamp_ns']
            last_timestamp_ns = statistics['last_timestamp_ns']
            missing_recorded_topics = find_missing_topic_requirements(
                (topic for topic, count in message_counts.items() if count > 0),
            )
        except ValueError as exc:
            validation_error = str(exc)
        if not args.skip_topic_preflight:
            try:
                adapter_status_end = read_runtime_adapter_status(
                    float(args.topic_wait_timeout_sec))
                adapter_contract_consistent = (
                    adapter_contract_snapshot(adapter_status_start)
                    == adapter_contract_snapshot(adapter_status_end)
                )
                if not adapter_contract_consistent:
                    raise ValueError('录制期间平台适配器数据流配置发生变化')
            except (RuntimeError, ValueError) as exc:
                adapter_validation_error = str(exc)
                validation_error = '; '.join(filter(None, (
                    validation_error, adapter_validation_error)))
            try:
                coverage_payload = read_runtime_patrol_coverage(
                    float(args.topic_wait_timeout_sec))
                coverage_errors = validate_patrol_coverage(
                    coverage_payload,
                    minimum_samples=MINIMUM_PATROL_COVERAGE_SAMPLES,
                    minimum_path_length_m=MINIMUM_PATROL_PATH_LENGTH_M,
                    minimum_planar_span_m=MINIMUM_PATROL_PLANAR_SPAN_M,
                )
            except (RuntimeError, ValueError) as exc:
                coverage_errors = [str(exc)]
        else:
            coverage_errors = ['诊断录包未执行正式巡检覆盖门禁']
        if coverage_errors:
            validation_error = '; '.join(filter(None, (
                validation_error, *coverage_errors)))
        clean_stop = recorder_exit_code in _CLEAN_RECORD_EXIT_CODES
        valid_bag = bool(
            clean_stop
            and not validation_error
            and not missing_recorded_topics
            and not adapter_validation_error
            and not coverage_errors
            and duration_sec >= MINIMUM_PATROL_DURATION_SEC
        )
        if (not validation_error
                and duration_sec < MINIMUM_PATROL_DURATION_SEC):
            validation_error = (
                f'巡检时长 {duration_sec:.3f}s 小于最低 '
                f'{MINIMUM_PATROL_DURATION_SEC:.1f}s')
        localization_reuse_eligible = bool(
            valid_bag
            and str(args.localization_provenance)
            in ALLOWED_LOCALIZATION_PROVENANCE
            and runtime_localization_provenance == str(
                args.localization_provenance)
            and message_counts.get('/hazardwalker/slam/odometry', 0) > 0
            and message_counts.get('/tf', 0) > 0
            and message_counts.get('/tf_static', 0) > 0
        )
        exit_code = 0 if valid_bag else (
            recorder_exit_code if recorder_exit_code not in _CLEAN_RECORD_EXIT_CODES
            else 2
        )
        manifest.update({
            'finished_at_utc': datetime.now(timezone.utc).isoformat(),
            'status': 'complete' if valid_bag else (
                'invalid' if clean_stop else 'aborted'
            ),
            'record_process_exit_code': recorder_exit_code,
            'exit_code': exit_code,
            'historical_localization_reuse_eligible': (
                localization_reuse_eligible),
            'adapter_status': {
                'start': adapter_status_start,
                'end': adapter_status_end,
                'contract_consistent': adapter_contract_consistent,
                'error': adapter_validation_error,
            },
            'bag_validation': {
                'status': 'passed' if valid_bag else 'failed',
                'message_counts': message_counts,
                'missing_required_topics': missing_recorded_topics,
                'minimum_duration_sec': MINIMUM_PATROL_DURATION_SEC,
                'duration_sec': duration_sec,
                'first_timestamp_ns': first_timestamp_ns,
                'last_timestamp_ns': last_timestamp_ns,
                'content_fingerprint_sha256': bag_fingerprint['sha256'],
                'files': bag_fingerprint['files'],
                'error': validation_error,
            },
            'patrol_coverage': {
                'status': 'passed' if not coverage_errors else 'failed',
                'minimum_samples': MINIMUM_PATROL_COVERAGE_SAMPLES,
                'minimum_path_length_m': MINIMUM_PATROL_PATH_LENGTH_M,
                'minimum_planar_span_m': MINIMUM_PATROL_PLANAR_SPAN_M,
                'metrics': coverage_payload,
                'errors': coverage_errors,
            },
        })
        write_manifest(manifest_path, manifest)
    return exit_code


def replay(args) -> int:
    session_dir = Path(args.session).expanduser().resolve()
    manifest_path = session_dir / 'run_manifest.json'
    if not manifest_path.is_file():
        raise SystemExit(f'缺少运行清单：{manifest_path}')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    bag_dir = session_dir / str(manifest.get('bag_relative_path', 'bag'))
    if not bag_dir.is_dir():
        raise SystemExit(f'缺少 rosbag 目录：{bag_dir}')
    command = build_replay_command(
        bag_dir,
        rate=args.rate,
        include_audit_topics=args.include_audit_topics,
        recompute_localization=args.recompute_localization,
    )
    print(shlex.join(command), flush=True)
    return subprocess.call(command, cwd=REPO_ROOT)


def _audit_topic(topic: str) -> str:
    return '/hw/replay' + topic.removeprefix('/hw')


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)

    record_parser = subparsers.add_parser('record', help='录制一次人工巡检')
    record_parser.add_argument('--output', required=True)
    record_parser.add_argument('--seed', required=True)
    record_parser.add_argument('--run-id', default='')
    record_parser.add_argument('--operator', default='')
    record_parser.add_argument(
        '--allow-dirty-worktree', action='store_true',
        help='仅与 --skip-topic-preflight 同用：允许脏代码生成非正式诊断录包',
    )
    record_parser.add_argument(
        '--localization-provenance', required=True,
        choices=sorted(ALLOWED_LOCALIZATION_PROVENANCE),
        help='本轮 TF/里程计的公开合法 SLAM 定位来源',
    )
    record_parser.add_argument(
        '--topic-wait-timeout-sec', type=float, default=20.0,
        help='开始录包前等待关键话题的最长时间',
    )
    record_parser.add_argument(
        '--preflight-report', default='',
        help='刚生成且通过的 verify_perception_live_chain.py JSON 报告',
    )
    record_parser.add_argument(
        '--preflight-max-age-sec', type=float, default=300.0,
        help='正式录包接受实时预检报告的最大年龄',
    )
    record_parser.add_argument(
        '--skip-topic-preflight', action='store_true',
        help='仅限故障诊断；正式巡检不得跳过关键话题检查',
    )
    record_parser.add_argument(
        '--extra-topic', action='append', default=[],
        help='额外录制话题，可重复指定',
    )
    record_parser.set_defaults(func=record)

    replay_parser = subparsers.add_parser(
        'replay', help='安全回放传感器输入')
    replay_parser.add_argument('--session', required=True)
    replay_parser.add_argument('--rate', type=float, default=1.0)
    replay_parser.add_argument(
        '--include-audit-topics', action='store_true',
        help='同时回放控制/检测历史，但统一改名到 /hw/replay/*',
    )
    replay_parser.add_argument(
        '--recompute-localization', action='store_true',
        help='仅回放公开原始传感器，由当前定位/SLAM 重新生成 TF、里程计和地图',
    )
    replay_parser.set_defaults(func=replay)
    return parser


def main() -> int:
    args = _parser().parse_args()
    return int(args.func(args))


if __name__ == '__main__':
    raise SystemExit(main())
