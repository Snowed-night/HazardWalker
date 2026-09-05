#!/usr/bin/env python3
"""官方 SimEnv SLAM + Frontier 整场运行与证据封存入口。

所属组：导航探索组。负责人：姜晨。
文件作用：在不管理共享容器的前提下，校验固定 SEED、唯一控制链和 ROS2
公开输入，启动统一业务栈，等待导航进入 FINISHED，再安全停止业务进程并输出
README、summary、CSV/JSON 测试记录。运行期不读取 Gazebo 真值或场景文件。
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time


REPO_ROOT = Path(__file__).resolve().parents[1]
FORMAL_ROOT = (REPO_ROOT / 'reports' / 'nav').resolve()
REQUIRED_TOPICS = {
    '/clock',
    '/hw/scan',
    '/hw/trunk_imu',
    '/hw/cmd_vel',
    '/hw/control/status',
    '/hw/platform/official_simenv_adapter_status',
}
BUSINESS_NODE_NAMES = {
    '/frontier_explorer_node',
    '/hazardwalker_cartographer',
    '/hazardwalker_cartographer_occupancy_grid',
    '/hazardwalker_multifloor_occupancy_mapper',
    '/hazardwalker_scan_imu_localizer',
    '/hazardwalker_slam_monitor',
    '/hazardwalker_pointcloud_map',
}
RUNTIME_GIT_EXCLUDES = (
    'install',
    'build',
    'log',
    'ros2_ws/src/hazardwalker_platform/.ros1_catkin_ws',
    'ros2_ws/src/hazardwalker_platform/generated_building',
    'ros2_ws/src/hazardwalker_platform/results',
)


def ensure_workspace_overlay() -> None:
    """确保无论从交互终端还是后台任务启动，都使用当前工作树的 ROS2 产物。"""

    install_root = (REPO_ROOT / 'install').resolve()
    workspace_setup = install_root / 'setup.bash'
    prefixes = {
        Path(value).resolve()
        for value in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
        if value.strip()
    }
    if any(
            prefix == install_root or install_root in prefix.parents
            for prefix in prefixes):
        return
    if not workspace_setup.is_file():
        raise RuntimeError(
            f'当前工作树尚未构建：缺少 {workspace_setup}；请先执行 colcon build')
    if os.environ.get('HAZARDWALKER_OVERLAY_BOOTSTRAPPED') == '1':
        raise RuntimeError(
            '已尝试加载当前工作树 ROS2 环境，但 AMENT_PREFIX_PATH 仍未生效')

    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy').strip() or 'jazzy'
    ros_setup = Path('/opt/ros') / ros_distro / 'setup.bash'
    if not ros_setup.is_file():
        raise RuntimeError(f'缺少 ROS2 基础环境：{ros_setup}')

    # bash -lc 的位置参数避免把仓库路径和用户参数拼进 shell 字符串；
    # 这样后台/nohup、SSH 与普通终端均走同一套可复现环境。
    environment = os.environ.copy()
    environment['HAZARDWALKER_OVERLAY_BOOTSTRAPPED'] = '1'
    command = (
        'unset COLCON_CURRENT_PREFIX; '
        'source "$1"; source "$2"; '
        'exec "$3" "$4" "${@:5}"'
    )
    os.execvpe(
        'bash',
        [
            'bash', '-lc', command, 'hazardwalker-runner',
            str(ros_setup), str(workspace_setup), sys.executable,
            str(Path(__file__).resolve()), *sys.argv[1:],
        ],
        environment,
    )


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_git_state(repo_root: Path) -> dict:
    """读取实际代码版本；正式运行拒绝脏工作树。"""

    commit = subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=repo_root,
        text=True, stderr=subprocess.DEVNULL,
    ).strip()
    status_command = [
        'git', 'status', '--porcelain', '--untracked-files=all', '--', '.',
    ] + [f':(exclude){path}' for path in RUNTIME_GIT_EXCLUDES]
    dirty_lines = subprocess.check_output(
        status_command,
        cwd=repo_root, text=True,
    ).splitlines()
    return {
        'commit': commit,
        'dirty': bool(dirty_lines),
        'dirty_entries': dirty_lines,
    }


def validate_output_dir(
        output_dir: Path,
        *,
        git_dirty: bool,
        allow_dirty_diagnostic: bool,
) -> str:
    """正式成果只允许仓库 reports/nav；脏代码只能写外部诊断目录。"""

    resolved = output_dir.expanduser().resolve()
    inside_formal = False
    try:
        relative = resolved.relative_to(FORMAL_ROOT)
        inside_formal = bool(relative.parts)
    except ValueError:
        pass
    if git_dirty:
        if not allow_dirty_diagnostic:
            raise ValueError('工作树未提交；正式仿真拒绝启动')
        if inside_formal:
            raise ValueError('脏代码诊断结果不得写入 reports/nav 正式目录')
        return 'diagnostic'
    if not inside_formal:
        raise ValueError('干净代码正式成果必须写入 reports/nav 的子目录')
    return 'formal'


def parse_adapter_status(text: str) -> dict:
    """从 ros2 topic echo 输出中提取适配器 JSON。"""

    value = str(text).strip()
    try:
        decoded = json.loads(value)
        if isinstance(decoded, dict):
            return decoded
        if isinstance(decoded, str):
            value = decoded
    except json.JSONDecodeError:
        pass
    start = value.find('{')
    end = value.rfind('}')
    if start < 0 or end < start:
        raise ValueError('适配器状态不含 JSON 对象')
    candidate = value[start:end + 1].replace('\\"', '"')
    payload = json.loads(candidate)
    if not isinstance(payload, dict):
        raise ValueError('适配器状态必须是 JSON 对象')
    return payload


def build_launch_command(
        output_dir: Path,
        *,
        scenario_seed: str,
        code_version: str,
        exploration_timeout_s: float = 480.0,
        mission_time_budget_s: float = 600.0,
        target_floors: tuple[int, ...] = (),
        per_floor_exploration_s: float = 120.0,
        simenv_container: str = 'simenv_ros1_hazard_platform',
        strict_room_inspection: bool = False,
        enable_3d_map: bool = False,
        world_from_map: tuple[float, float, float] = (0.0, 0.0, 0.0),
        floor_height_m: float = 2.6,
        sphere_center_height_m: float = 0.15,
        room_clearance_m: float = 0.60,
) -> list[str]:
    """构造唯一业务 launch；平台 adapter/mux 继续由平台生命周期管理。"""

    localization_provenance = (
        'lidar_imu_slam+public_floor_action'
        if target_floors else 'lidar_imu_slam')
    command = [
        'ros2', 'launch', 'hazardwalker_bringup',
        'official_simenv_control_interface.launch.py',
        'start_command_mux:=false',
        'start_assist_alignment:=false',
        'control_mode:=navigation',
        'start_slam:=true',
        'start_pointcloud_map:='
        + ('true' if enable_3d_map else 'false'),
        'start_slam_video:=true',
        'slam_backend:=cartographer',
        'slam_dimension:=' + ('3d' if enable_3d_map else '2d'),
        'start_navigation:=true',
        'nav_mode:=frontier',
        'local_planner_backend:=unitree_move_base',
        'start_perception:='
        + ('true' if strict_room_inspection else 'false'),
        'start_decision:=true',
        'start_evidence_recorder:='
        + ('true' if strict_room_inspection else 'false'),
        # SLAM 在公开入门动作之后启动，感知必须保留 map 坐标；结果层再用
        # 本轮实测入门距离恢复 world，不能套用出生点的静态 world→map。
        'perception_output_frame:=map',
        f'localization_provenance:={localization_provenance}',
        'use_sim_time:=true',
        'navigation_linear_speed:=2.00',
        'navigation_minimum_linear_speed:=1.20',
        f'exploration_timeout_s:={float(exploration_timeout_s):.3f}',
        f'mission_time_budget_s:={float(mission_time_budget_s):.3f}',
        f'simenv_container:={simenv_container}',
        'strict_room_inspection:='
        + ('true' if strict_room_inspection else 'false'),
        f'strict_room_clearance_m:={float(room_clearance_m):.6f}',
        f'nav_record_dir:={output_dir / "navigation"}',
        f'evidence_output_dir:={output_dir / "perception"}',
        f'test_record_dir:={output_dir / "test_records"}',
        f'official_result_path:={output_dir / "detected_danger.json"}',
        f'slam_monitor_output_dir:={output_dir / "slam"}',
        f'pointcloud_map_output_dir:={output_dir / "slam_3d"}',
        f'slam_video_output:={output_dir / "video" / "slam_exploration.mp4"}',
        f'scenario_seed:={scenario_seed}',
        f'code_version:={code_version}',
    ]
    if target_floors:
        command.extend([
            'target_floors:=' + json.dumps(
                list(target_floors), separators=(',', ':')),
            f'per_floor_exploration_s:={float(per_floor_exploration_s):.3f}',
            'manual_elevator_assist:=true',
            'automatic_elevator_entry:=true',
        ])
    if strict_room_inspection:
        # 正式感知验收必须加载仓库内受版本控制的配置。禁止依赖节点默认值，
        # 否则配置文件、运行参数和结果清单会互相矛盾。
        command.extend([
            f'perception_parameter_file:={REPO_ROOT / "config" / "perception.yaml"}',
            'official_hazard_source_frame:=map',
            f'official_world_from_map_x:={float(world_from_map[0]):.6f}',
            f'official_world_from_map_y:={float(world_from_map[1]):.6f}',
            f'official_world_from_map_yaw:={float(world_from_map[2]):.6f}',
            f'official_floor_height_m:={float(floor_height_m):.6f}',
            'official_sphere_center_height_m:='
            f'{float(sphere_center_height_m):.6f}',
        ])
    return command


def map_origin_after_straight_ingress(
        start_world_x: float,
        start_world_y: float,
        start_world_yaw: float,
        distance_m: float,
) -> tuple[float, float, float]:
    """由公开出生位姿和合法入门里程计算 SLAM 启动时的 map 原点。"""

    values = (
        float(start_world_x), float(start_world_y),
        float(start_world_yaw), float(distance_m),
    )
    if not all(math.isfinite(value) for value in values) or values[3] < 0.0:
        raise ValueError('出生位姿和入门距离必须为有限值，距离不得为负')
    return (
        values[0] + math.cos(values[2]) * values[3],
        values[1] + math.sin(values[2]) * values[3],
        values[2],
    )


def validate_perception_mission_config(path: Path) -> dict:
    """在机器人移动前加载真实 YAML，并核对本次单视角比赛合同。"""

    import yaml
    module_path = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
        / 'hazardwalker_perception' / 'perception_config.py'
    )
    spec = importlib.util.spec_from_file_location(
        'hazardwalker_perception_config_contract', module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError('无法加载感知配置校验器')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    config_path = Path(path).resolve()
    document = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    parameters = module.flatten_perception_config(document)
    localization = document['perception'].get('localization', {})
    expected = {
        'confirm_distinct_views': 1,
        'min_spherical_views_for_confirm': 1,
        'reject_non_spherical_tracks': False,
        'emit_partial_candidates': True,
    }
    mismatches = {
        key: {'expected': value, 'actual': parameters.get(key)}
        for key, value in expected.items()
        if parameters.get(key) != value
    }
    if bool(localization.get('use_point_cloud', False)):
        mismatches['use_point_cloud'] = {
            'expected': False, 'actual': True}
    if mismatches:
        raise RuntimeError(
            '感知配置不符合正式单视角合同：'
            + json.dumps(mismatches, ensure_ascii=False, sort_keys=True))
    return {
        'path': str(config_path),
        'sha256': hashlib.sha256(config_path.read_bytes()).hexdigest(),
        'parameters': expected,
        'use_point_cloud': False,
    }


def validate_navigation_clearance_contract(
        container: str, configured_clearance_m: float) -> dict:
    """在移动前核对 A* 规划净空不小于 ROS1 DWA 的真实 A1 footprint。"""

    command = [
        'docker', 'exec', str(container), 'bash', '-lc',
        'source /opt/ros/noetic/setup.bash; '
        'rosparam get /move_base/local_costmap/footprint',
    ]
    completed = subprocess.run(
        command, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, timeout=15.0,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            '无法读取 Unitree move_base footprint：'
            + completed.stderr.strip())
    try:
        import yaml
        points = yaml.safe_load(completed.stdout)
        if isinstance(points, str):
            points = yaml.safe_load(points)
        radius = max(math.hypot(float(x), float(y)) for x, y in points)
    except (ValueError, TypeError, SyntaxError) as exc:
        raise RuntimeError('Unitree footprint 格式无效') from exc
    clearance = float(configured_clearance_m)
    if not math.isfinite(clearance) or clearance + 1e-6 < radius:
        raise RuntimeError(
            f'房间规划净空 {clearance:.3f}m 小于 A1 footprint '
            f'外接半径 {radius:.3f}m')
    return {
        'configured_clearance_m': clearance,
        'footprint_radius_m': radius,
        'footprint': points,
    }


def parse_target_floors(value: str) -> tuple[int, ...]:
    """解析 0,1,2 格式，保持用户顺序并拒绝重复/异常楼层。"""

    text = str(value).strip()
    if not text:
        return ()
    try:
        floors = tuple(int(part.strip()) for part in text.split(','))
    except ValueError as exc:
        raise ValueError('target-floors 必须是逗号分隔整数') from exc
    if len(set(floors)) != len(floors):
        raise ValueError('target-floors 不得重复')
    if not floors:
        return ()
    return floors


def build_official_evaluation_command(
        truth_file: Path, detected_file: Path, output_file: Path,
) -> list[str]:
    """构造赛后官方评分命令；该命令不得在任务运行期调用。"""

    evaluator = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform' / 'src'
        / 'building_obstacles' / 'scripts' / 'evaluate_danger.py'
    )
    return [
        'python3', str(evaluator),
        '--truth-file', str(Path(truth_file)),
        '--detected-file', str(Path(detected_file)),
        '--output-file', str(Path(output_file)),
    ]


def evaluate_completed_run(
        truth_file: Path, detected_file: Path, output_file: Path,
) -> dict:
    """业务栈停止后计算官方客观分，返回评估 JSON。"""

    truth = Path(truth_file)
    detected = Path(detected_file)
    if not truth.is_file():
        raise RuntimeError(f'评分真值文件不存在：{truth}')
    if not detected.is_file():
        raise RuntimeError(f'任务未生成官方检测结果：{detected}')
    completed = subprocess.run(
        build_official_evaluation_command(truth, detected, output_file),
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=60.0,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            '官方评分失败：'
            + (completed.stderr.strip() or completed.stdout.strip()))
    try:
        payload = json.loads(Path(output_file).read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError('官方评分未生成合法 JSON') from exc
    if not isinstance(payload, dict) or 'scores' not in payload:
        raise RuntimeError('官方评分 JSON 缺少 scores')
    return payload


def _read_jsonl(path: Path) -> list[dict]:
    rows = []
    for line_number, line in enumerate(
            Path(path).read_text(encoding='utf-8').splitlines(), start=1):
        if not line.strip():
            continue
        try:
            payload = json.loads(line)
        except json.JSONDecodeError as exc:
            raise RuntimeError(
                f'{path} 第 {line_number} 行不是合法 JSON') from exc
        if not isinstance(payload, dict):
            raise RuntimeError(f'{path} 第 {line_number} 行必须是 JSON 对象')
        rows.append(payload)
    return rows


def validate_navigation_acceptance(
        output_dir: Path,
        target_floors: tuple[int, ...],
        expected_rooms_per_floor: int = 4,
        strict_room_inspection: bool = False,
) -> dict:
    """依据结构化运行证据验证楼层、房间、巡检和失败记录。"""

    navigation_dir = Path(output_dir) / 'navigation'
    coverage_path = navigation_dir / 'room_coverage.jsonl'
    failure_path = navigation_dir / 'failures.jsonl'
    if not coverage_path.is_file():
        raise RuntimeError('导航验收缺少 room_coverage.jsonl')
    coverage_rows = _read_jsonl(coverage_path)
    failure_rows = _read_jsonl(failure_path) if failure_path.is_file() else []
    if failure_rows:
        raise RuntimeError(
            f'导航验收发现 {len(failure_rows)} 条失败记录')

    completed = [
        row for row in coverage_rows if row.get('phase') == 'completed'
    ]
    floors = tuple(target_floors) if target_floors else (0,)
    expected_count = max(1, int(expected_rooms_per_floor))
    expected_official_sectors = {
        'far_left', 'far_right', 'near_left', 'near_right'}
    summary = {}
    for floor in floors:
        rows = [row for row in completed if int(row.get('floor', -999)) == floor]
        sectors = {str(row.get('sector', '')) for row in rows}
        if len(sectors) != expected_count:
            raise RuntimeError(
                f'楼层 {floor} 仅完成 {len(sectors)}/{expected_count} 个房间：'
                f'{sorted(sectors)}')
        if expected_count == 4 and sectors != expected_official_sectors:
            raise RuntimeError(
                f'楼层 {floor} 房间集合不完整：{sorted(sectors)}')
        if strict_room_inspection:
            for row in rows:
                if row.get('reason') != (
                        'deterministic_loop_and_strict_inspection'):
                    raise RuntimeError(
                        f'楼层 {floor} 房间 {row.get("sector")} '
                        '没有严格巡检完成原因')
                required = int(row.get('inspection_goal_count', 0))
                captured = int(row.get('inspection_completed_count', 0))
                visibility = float(row.get('visibility_coverage_ratio', 0.0))
                visibility_required = float(row.get(
                    'required_visibility_coverage_ratio', 1.0))
                visibility_targets = int(row.get(
                    'visibility_target_cell_count', 0))
                visibility_covered = int(row.get(
                    'visibility_covered_cell_count', 0))
                required_visible_cells = int(math.ceil(
                    visibility_required * visibility_targets - 1e-12))
                if (required <= 0 or captured != required
                        or visibility_targets <= 0
                        or visibility_covered + 1 < required_visible_cells):
                    raise RuntimeError(
                        f'楼层 {floor} 房间 {row.get("sector")} 巡检证据不足：'
                        f'captures={captured}/{required}, '
                        f'visibility={visibility:.1%}/'
                        f'{visibility_required:.1%}')
        summary[str(floor)] = {
            'completed_room_count': len(sectors),
            'sectors': sorted(sectors),
            'strict_inspection': bool(strict_room_inspection),
        }
    return {
        'passed': True,
        'expected_rooms_per_floor': expected_count,
        'floors': summary,
        'failure_record_count': 0,
    }


def _scan_sector_median(
        ranges, angle_min: float, angle_increment: float,
        low_deg: float, high_deg: float) -> float:
    """返回指定激光扇区的有效量程中值，无有效束时返回无穷大。"""

    values = []
    angle = float(angle_min)
    for raw_value in ranges:
        value = float(raw_value)
        degree = math.degrees(angle)
        if (low_deg <= degree <= high_deg
                and math.isfinite(value) and value > 0.05):
            values.append(value)
        angle += float(angle_increment)
    if not values:
        return math.inf
    values.sort()
    middle = len(values) // 2
    if len(values) % 2:
        return values[middle]
    return 0.5 * (values[middle - 1] + values[middle])


def entrance_lobby_structure_detected(
        ranges, angle_min: float, angle_increment: float) -> bool:
    """仅凭公开激光判断机器人已穿过大门并进入一楼大厅。"""

    valid_total = sum(
        1 for raw_value in ranges
        if math.isfinite(float(raw_value)) and float(raw_value) > 0.05)
    front = _scan_sector_median(
        ranges, angle_min, angle_increment, -15.0, 15.0)
    front_right = _scan_sector_median(
        ranges, angle_min, angle_increment, -70.0, -20.0)
    return (
        valid_total >= 340
        and 4.0 <= front <= 10.0
        and front_right >= 5.0
    )


def update_entrance_structure_state(
        door_frame_seen: bool, stable_streak: int,
        front_median: float, absolute_lobby: bool) -> tuple[bool, int]:
    """更新“见到门框后进入大厅”的连续帧状态。"""

    seen = bool(door_frame_seen) or 1.50 <= front_median <= 3.50
    crossed = seen and front_median >= 4.0
    streak = int(stable_streak) + 1 if absolute_lobby or crossed else 0
    return seen, streak


def run_ros2_cli(arguments: list[str], timeout_sec: float = 10.0) -> str:
    env = os.environ.copy()
    env['ROS2CLI_DISABLE_DAEMON'] = '1'
    result = subprocess.run(
        ['ros2', *arguments],
        cwd=REPO_ROOT,
        env=env,
        text=True,
        capture_output=True,
        timeout=timeout_sec,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"ros2 {' '.join(arguments)} 失败：{result.stderr.strip()}")
    return result.stdout


def save_pointcloud_map() -> dict:
    """在停止 launch 前显式等待三维地图服务完成，避免 SIGINT 打断封存。"""

    output = run_ros2_cli([
        'service', 'call', '/hazardwalker/slam/save_cloud_map',
        'std_srvs/srv/Trigger', '{}',
    ], timeout_sec=120.0)
    success = ('success=True' in output or 'success: true' in output.lower())
    if not success:
        raise RuntimeError(f'三维地图保存服务未成功：{output.strip()}')
    return {'service': '/hazardwalker/slam/save_cloud_map', 'success': True}


def _safe_run_slug(run_id: str) -> str:
    """生成同时可用于主机路径和容器进程匹配的安全短标识。"""

    value = re.sub(r'[^A-Za-z0-9_.-]+', '_', str(run_id)).strip('._-')
    return (value or 'slam_run')[:96]


def start_first_person_recording(container: str, run_id: str) -> dict:
    """在ROS1容器内直接录RGB，避免图像经rosbridge占用三维SLAM带宽。"""

    slug = _safe_run_slug(run_id)
    host_dir = (
        REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform' /
        'results' / 'video_capture')
    host_dir.mkdir(parents=True, exist_ok=True)
    host_avi = host_dir / f'{slug}.avi'
    if host_avi.exists():
        host_avi.unlink()
    container_dir = '/home/ros/simenv_ws/results/video_capture'
    container_avi = f'{container_dir}/{slug}.avi'
    command = (
        'source /opt/ros/noetic/setup.bash && '
        'source /home/ros/simenv_ws/.ros1_catkin_ws/devel/setup.bash && '
        f'mkdir -p {container_dir} && '
        'exec rosrun image_view video_recorder '
        'image:=/real_sense/rgb/image_raw '
        f'_filename:={container_avi} _fps:=5 _codec:=MJPG')
    result = subprocess.run(
        ['docker', 'exec', '-d', container, 'bash', '-lc', command],
        text=True, capture_output=True, timeout=15.0)
    if result.returncode != 0:
        raise RuntimeError(
            f'第一人称录像启动失败：{result.stderr.strip()}')
    time.sleep(2.0)
    probe = subprocess.run([
        'docker', 'exec', container, 'bash', '-lc',
        f"pgrep -af 'video_recorder.*{slug}'",
    ], text=True, capture_output=True, timeout=10.0)
    if probe.returncode != 0:
        raise RuntimeError(
            f'第一人称录像进程未存活：{probe.stderr.strip()}')
    return {
        'slug': slug,
        'container': container,
        'container_avi': container_avi,
        'host_avi': str(host_avi),
        'started': True,
    }


def stop_first_person_recording(
        capture: dict | None, output_dir: Path) -> dict | None:
    """停止容器录像、转H.264 MP4并清理本轮临时AVI。"""

    if not capture:
        return None
    container = str(capture['container'])
    slug = str(capture['slug'])
    subprocess.run([
        'docker', 'exec', container, 'pkill', '-INT', '-f',
        f'video_recorder.*{slug}',
    ], text=True, capture_output=True, timeout=10.0)
    time.sleep(2.0)
    source = Path(str(capture['host_avi']))
    video_dir = output_dir / 'video'
    video_dir.mkdir(parents=True, exist_ok=True)
    target = video_dir / 'first_person.mp4'
    if not source.is_file() or source.stat().st_size <= 0:
        raise RuntimeError('第一人称录像文件不存在或为空')
    encode = subprocess.run([
        'ffmpeg', '-y', '-loglevel', 'error', '-i', str(source),
        '-an', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '23',
        '-movflags', '+faststart', str(target),
    ], text=True, capture_output=True, timeout=180.0)
    if encode.returncode != 0 or not target.is_file():
        raise RuntimeError(
            f'第一人称录像转码失败：{encode.stderr.strip()}')
    source.unlink(missing_ok=True)
    return {
        'path': str(target),
        'size_bytes': target.stat().st_size,
        'fps': 5,
    }


def preflight(expected_seed: str, require_pointcloud: bool = False) -> dict:
    """验证公开输入、固定 SEED、唯一控制节点且没有旧业务栈。"""

    perception_executables = set(run_ros2_cli([
        'pkg', 'executables', 'hazardwalker_perception',
    ]).splitlines())
    required_localizer = 'hazardwalker_perception scan_imu_localizer_node'
    if required_localizer not in perception_executables:
        raise RuntimeError(
            '当前 ROS2 工作区找不到入口局部定位器；请重新构建并加载本工作树 '
            'install/setup.bash')
    topics = set(run_ros2_cli(['topic', 'list']).splitlines())
    required_topics = set(REQUIRED_TOPICS)
    if require_pointcloud:
        required_topics.add('/hw/lidar/points')
    missing = sorted(required_topics - topics)
    if missing:
        raise RuntimeError(f'正式导航缺少话题：{missing}')
    nodes = run_ros2_cli(['node', 'list']).splitlines()
    mux_count = nodes.count('/hazardwalker_command_mux')
    if mux_count != 1:
        raise RuntimeError(f'控制仲裁器数量必须为 1，当前 {mux_count}')
    conflicts = sorted(BUSINESS_NODE_NAMES.intersection(nodes))
    if conflicts:
        raise RuntimeError(f'仍有旧 SLAM/导航节点：{conflicts}')
    raw_status = run_ros2_cli([
        'topic', 'echo', '--once',
        '/hw/platform/official_simenv_adapter_status',
        'std_msgs/msg/String', '--field', 'data',
    ])
    adapter = parse_adapter_status(raw_status)
    runtime_seed = str(adapter.get('scenario_seed') or '')
    if runtime_seed != str(expected_seed):
        raise RuntimeError(
            f'容器固定 SEED 不一致：{runtime_seed!r} != {expected_seed!r}')
    if adapter.get('enable_cmd_vel_relay') is not True:
        raise RuntimeError('平台适配器未启用控制转发')
    if adapter.get('enable_odom_relay') is not True:
        raise RuntimeError(
            '赛事 DWA 控制要求平台转发只读 /hw/odom；该话题不得接入 '
            'Cartographer 或危险源定位')
    if adapter.get('enable_odom_tf_relay') is not False:
        raise RuntimeError(
            '禁止平台把 Gazebo odom 转发为 odom→base TF；SLAM 与感知必须 '
            '继续使用合法 scan+IMU 位姿树')
    if (require_pointcloud
            and adapter.get('enable_pointcloud_relay') is not True):
        raise RuntimeError('三维 SLAM 成果要求平台启用 Mid-360 点云转发')
    if adapter.get('enable_unitree_move_base_bridge') is not True:
        raise RuntimeError(
            '正式探索要求平台启用赛事仓库宇树 move_base 桥接；请以 '
            'START_UNITREE_MOVE_BASE=1 重启容器（平台会自动启用 2D 激光与 '
            'Mid-360）')
    container = str(adapter.get('lifecycle_container') or '').strip()
    if not container or any(character.isspace() for character in container):
        raise RuntimeError('适配器未报告合法的受管容器名')
    move_base_probe = subprocess.run([
        'docker', 'exec', container, 'bash', '-lc',
        'source /opt/ros/noetic/setup.bash && '
        'source /home/ros/simenv_ws/.ros1_catkin_ws/devel/setup.bash && '
        'rosnode ping -c 1 /move_base >/dev/null',
    ], text=True, capture_output=True, timeout=10.0)
    if move_base_probe.returncode != 0:
        raise RuntimeError(
            '赛事仓库宇树 /move_base 未运行；平台启动链路不完整')
    return {
        'topics': sorted(topics),
        'nodes': sorted(nodes),
        'adapter_status': adapter,
    }


def open_main_entrance(container: str) -> dict:
    """通过官方公开服务打开主入口；不读取门坐标或场景配置。"""

    if not container or any(character.isspace() for character in container):
        raise ValueError('适配器未提供合法容器名')
    command = [
        'docker', 'exec', container, 'bash', '-lc',
        'source /opt/ros/noetic/setup.bash && '
        'source /home/ros/simenv_ws/.ros1_catkin_ws/devel/setup.bash && '
        'rosservice call /set_door_state main_entrance true',
    ]
    result = subprocess.run(
        command, text=True, capture_output=True, timeout=20.0)
    output = (result.stdout + '\n' + result.stderr).strip()
    accepted = result.returncode == 0 and 'accepted: True' in output
    opened = 'state: "open"' in output or 'state: open' in output
    if not accepted or not opened:
        raise RuntimeError(
            f'主入口公开开门服务失败 (rc={result.returncode})：{output}')
    return {
        'service': '/set_door_state',
        'door_id': 'main_entrance',
        'accepted': True,
        'state': 'open',
    }


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    temporary.replace(path)


def stop_process_group(process: subprocess.Popen) -> int:
    """分级停止本轮 launch 进程组，不触碰平台容器或生命周期管理器。"""

    leader_return_code = process.poll()

    def group_exists():
        try:
            os.killpg(process.pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    def wait_group(timeout_sec):
        deadline = time.monotonic() + float(timeout_sec)
        while group_exists() and time.monotonic() < deadline:
            time.sleep(0.1)
        return not group_exists()

    # launch leader 可能先因缺失可执行文件退出，而已启动的节点随后被
    # reparent 到 PID 1。此时仍需按原 PGID 清理，不能仅检查 poll()。
    if not group_exists():
        return int(leader_return_code or 0)
    try:
        os.killpg(process.pid, signal.SIGINT)
        if wait_group(30.0):
            return int(process.poll() if process.poll() is not None else 0)
    except ProcessLookupError:
        return int(leader_return_code or 0)
    if group_exists():
        os.killpg(process.pid, signal.SIGTERM)
        if not wait_group(10.0) and group_exists():
            os.killpg(process.pid, signal.SIGKILL)
            wait_group(5.0)
    process.poll()
    return int(
        process.returncode
        if process.returncode is not None
        else leader_return_code
        if leader_return_code is not None
        else 0)


def perform_entrance_ingress(
        *, distance_m: float = 3.6, speed_mps: float = 0.45,
        wall_timeout_sec: float = 360.0) -> dict:
    """仅用公开激光、IMU和控制接口穿过大门，再把室内位置作为SLAM原点。"""

    if distance_m <= 0.0 or speed_mps <= 0.0 or wall_timeout_sec <= 0.0:
        raise ValueError('入口行驶距离、速度和超时必须为正数')

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    localizer_command = [
        'ros2', 'run', 'hazardwalker_perception',
        'scan_imu_localizer_node', '--ros-args',
        '-r', '__node:=hazardwalker_ingress_localizer',
        '-p', 'use_sim_time:=true',
        '-p', 'publish_tf:=false',
        '-p', 'localization_provenance:=lidar_imu_slam',
    ]
    localizer_log = tempfile.TemporaryFile(mode='w+', encoding='utf-8')
    localizer = subprocess.Popen(
        localizer_command, cwd=REPO_ROOT,
        stdout=localizer_log, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    rclpy.init()

    class IngressNode(Node):
        def __init__(self):
            super().__init__('hazardwalker_entrance_ingress')
            self.pose = None
            self.pose_wall_time = None
            self.front_clearance = math.inf
            self.lobby_structure_streak = 0
            self.lobby_structure_confirmed = False
            self.entrance_door_frame_seen = False
            self.cmd_pub = self.create_publisher(
                Twist, '/hw/control/navigation_cmd_vel', 10)
            self.mode_pub = self.create_publisher(
                String, '/hw/control/mode_request', 10)
            self.create_subscription(
                Odometry, '/hazardwalker/slam/odometry',
                self._on_odom, 10)
            self.create_subscription(
                LaserScan, '/hw/scan', self._on_scan, 10)

        def _on_odom(self, message):
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            siny = 2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y)
            cosy = 1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z)
            self.pose = (
                float(position.x), float(position.y),
                math.atan2(siny, cosy),
            )
            self.pose_wall_time = time.monotonic()

        def _on_scan(self, message):
            values = []
            angle = float(message.angle_min)
            for value in message.ranges:
                if abs(angle) <= math.radians(18.0):
                    numeric = float(value)
                    if math.isfinite(numeric) and numeric > 0.05:
                        values.append(numeric)
                angle += float(message.angle_increment)
            self.front_clearance = min(values) if values else math.inf
            front_median = _scan_sector_median(
                message.ranges, message.angle_min,
                message.angle_increment, -15.0, 15.0)
            absolute_lobby = entrance_lobby_structure_detected(
                    message.ranges, message.angle_min,
                    message.angle_increment)
            (self.entrance_door_frame_seen,
             self.lobby_structure_streak) = update_entrance_structure_state(
                 self.entrance_door_frame_seen,
                 self.lobby_structure_streak,
                 front_median,
                 absolute_lobby,
             )
            self.lobby_structure_confirmed = (
                self.lobby_structure_streak >= 3)

        def publish(self, linear: float, angular: float = 0.0):
            mode = String()
            mode.data = 'navigation'
            self.mode_pub.publish(mode)
            command = Twist()
            command.linear.x = float(linear)
            command.angular.z = float(angular)
            self.cmd_pub.publish(command)

    node = IngressNode()
    started = time.monotonic()
    start_pose = None
    travelled = 0.0
    try:
        while time.monotonic() - started < min(60.0, wall_timeout_sec):
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pose is not None:
                start_pose = node.pose
                break
            if localizer.poll() is not None:
                localizer_log.flush()
                localizer_log.seek(0)
                details = localizer_log.read()[-4000:].strip()
                raise RuntimeError(
                    f'入口局部定位器提前退出 (rc={localizer.returncode})：'
                    f'{details or "未输出错误详情"}')
        if start_pose is None:
            raise RuntimeError('入口阶段未收到激光/IMU里程计')

        start_x, start_y, start_yaw = start_pose
        while time.monotonic() - started < wall_timeout_sec:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.pose is None or node.pose_wall_time is None:
                continue
            if time.monotonic() - node.pose_wall_time > 2.0:
                raise RuntimeError('入口阶段里程计话题中断')
            x, y, yaw = node.pose
            travelled = math.hypot(x - start_x, y - start_y)
            # 大厅结构证明已经过门，合法相对里程保证完整 A1 footprint 也
            # 离开门框代价区；二者必须同时满足，不能刚看到大厅就停在门槛。
            if (node.lobby_structure_confirmed
                    and travelled >= float(distance_m)):
                break
            # 主门在机器人开始行驶前已由公开服务完全打开时，水平激光可能
            # 从第一帧起就直接看进大厅，因而不会经历“先见门框、后见深处”
            # 的特征序列。此时以合法 scan/IMU 相对里程清出完整 A1 footprint，
            # 并要求前方仍有足够净空；不能因门框特征缺失继续冲到保护上限。
            if (not node.lobby_structure_confirmed
                    and travelled >= float(distance_m)
                    and node.front_clearance >= 0.80):
                break
            if travelled >= max(8.0, 2.5 * distance_m):
                raise RuntimeError(
                    '入口累计里程已超保护上限，但未识别到大厅结构')
            if node.front_clearance < 0.48:
                raise RuntimeError(
                    f'入口前方净空不足：{node.front_clearance:.2f} m')
            heading_error = math.atan2(
                math.sin(start_yaw - yaw), math.cos(start_yaw - yaw))
            node.publish(
                speed_mps,
                max(-0.40, min(0.40, 1.2 * heading_error)),
            )
        else:
            raise RuntimeError(
                f'入口行驶墙钟超时：仅完成 {travelled:.2f}/{distance_m:.2f} m')
        for _ in range(10):
            node.publish(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        try:
            for _ in range(3):
                node.publish(0.0, 0.0)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            stop_process_group(localizer)
            localizer_log.close()

    return {
        'method': 'public_scan_imu_relative_ingress',
        'distance_target_m': round(float(distance_m), 3),
        'distance_reached_m': round(float(travelled), 3),
        'lobby_structure_confirmed': bool(
            node.lobby_structure_confirmed),
        'distance_clearance_fallback': bool(
            not node.lobby_structure_confirmed
            and travelled >= float(distance_m)
            and node.front_clearance >= 0.80),
        'entrance_door_frame_seen': bool(
            node.entrance_door_frame_seen),
        'speed_command_mps': round(float(speed_mps), 3),
        'wall_duration_sec': round(time.monotonic() - started, 3),
    }


def write_handoff(output_dir: Path, manifest: dict) -> None:
    """写导航测试组可读的摘要、CSV/JSON和专业简洁 README。"""

    summary = {
        'schema': 'hazardwalker_slam_exploration_summary_v1',
        'run_id': manifest['run_id'],
        'scenario_seed': manifest['scenario_seed'],
        'status': manifest['status'],
        'final_nav_state': manifest['final_nav_state'],
        'wall_duration_sec': manifest['wall_duration_sec'],
        'git': manifest['git'],
        'run_mode': manifest['run_mode'],
        'navigation_dir': 'navigation',
        'slam_monitor_dir': 'slam',
        'pointcloud_map_dir': 'slam_3d',
        'video_dir': 'video',
        'first_person_video': manifest.get('first_person_video'),
        'failure_reason': manifest['failure_reason'],
    }
    write_json(output_dir / 'summary.json', summary)
    record = {
        'record_id': manifest['run_id'],
        'scenario_seed': manifest['scenario_seed'],
        'stage': 'official_slam_frontier',
        'expected': '稳定建图、有效探索、返航 FINISHED',
        'result': manifest['status'],
        'final_nav_state': manifest['final_nav_state'],
        'wall_duration_sec': manifest['wall_duration_sec'],
        'git_commit': manifest['git']['commit'],
        'git_dirty': manifest['git']['dirty'],
        'failure_reason': manifest['failure_reason'],
    }
    write_json(output_dir / 'testing_record_nav.json', {'records': [record]})
    with (output_dir / 'testing_record_nav.csv').open(
            'w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(record))
        writer.writeheader()
        writer.writerow(record)
    readme = (
        '# 官方 SimEnv SLAM 与自主探索运行\n\n'
        f'- SEED：`{manifest["scenario_seed"]}`\n'
        f'- 代码：`{manifest["git"]["commit"]}`\n'
        f'- 模式：`{manifest["run_mode"]}`\n'
        f'- 状态：`{manifest["status"]}`\n'
        f'- 最终导航状态：`{manifest["final_nav_state"] or "未收到"}`\n'
        f'- 墙钟耗时：`{manifest["wall_duration_sec"]}` 秒\n\n'
        '本目录只记录公开传感器、合法 SLAM、导航控制和结构化运行结果。'
        '是否达到正式验收仍需结合 `navigation/`、`slam/` 中的覆盖、返航、'
        '位姿跳变、漂移、二维地图及 `slam_3d/` 三维体素地图指标判断。\n'
    )
    (output_dir / 'README.md').write_text(readme, encoding='utf-8')


def main() -> int:
    ensure_workspace_overlay()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--wall-timeout-sec', type=float, default=7200.0)
    parser.add_argument('--exploration-timeout-sec', type=float, default=480.0)
    parser.add_argument(
        '--mission-time-budget-sec', type=float, default=600.0,
        help='任务总仿真时长；官方计分保持 600，扩展建图展示必须显式提高。')
    parser.add_argument('--target-floors', default='')
    parser.add_argument('--per-floor-exploration-sec', type=float, default=150.0)
    # 2.8 m 实测只让机体中心到达门槛附近，宇树 costmap 会把完整 A1
    # footprint 判为已经与门框重叠。3.6 m 使整机进入大厅；实时激光门禁
    # 仍可在前方结构异常时提前停车并令预检失败。
    parser.add_argument('--entrance-distance-m', type=float, default=3.6)
    # 入口段必须落在 A1 控制器稳定速度范围，避免高速命令造成里程计与
    # 物理位置不一致；进入主走廊后 Frontier 仍使用 0.60 m/s。
    parser.add_argument('--entrance-speed-mps', type=float, default=0.45)
    parser.add_argument('--entrance-wall-timeout-sec', type=float, default=360.0)
    parser.add_argument('--public-start-world-x', type=float, default=0.0)
    parser.add_argument('--public-start-world-y', type=float, default=-2.2)
    parser.add_argument('--public-start-world-yaw', type=float, default=1.5708)
    parser.add_argument('--floor-height-m', type=float, default=2.6)
    parser.add_argument('--sphere-center-height-m', type=float, default=0.15)
    parser.add_argument('--room-clearance-m', type=float, default=0.60)
    parser.add_argument('--allow-dirty-diagnostic', action='store_true')
    parser.add_argument(
        '--strict-room-inspection', action='store_true',
        help='在已验收基础环线后执行严格逐障碍观察；无采帧确认不得完成房间。')
    parser.add_argument(
        '--truth-file',
        default=str(
            REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform'
            / 'results' / 'danger_truth.json'),
        help='仅在任务结束并停止业务栈后用于官方评分的真值文件。')
    parser.add_argument(
        '--expected-rooms-per-floor', type=int, default=4,
        help='赛后验收使用，不参与导航决策；官方三层楼每层为 4。')
    parser.add_argument(
        '--enable-3d-map', action='store_true',
        help='额外启动 Mid-360 三维地图；正式评分默认使用低负载二维 SLAM。')
    args = parser.parse_args()
    if (args.wall_timeout_sec <= 0.0 or args.exploration_timeout_sec <= 0.0
            or args.mission_time_budget_sec <= 0.0
            or args.per_floor_exploration_sec <= 0.0
            or args.expected_rooms_per_floor <= 0
            or args.floor_height_m <= 0.0
            or args.sphere_center_height_m < 0.0
            or args.room_clearance_m <= 0.0):
        raise SystemExit('运行及逐层探索超时必须为正数')
    try:
        target_floors = parse_target_floors(args.target_floors)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    git_state = read_git_state(REPO_ROOT)
    output_dir = Path(args.output_dir).expanduser().resolve()
    try:
        run_mode = validate_output_dir(
            output_dir,
            git_dirty=bool(git_state['dirty']),
            allow_dirty_diagnostic=bool(args.allow_dirty_diagnostic),
        )
        preflight_payload = preflight(
            str(args.seed), require_pointcloud=bool(args.enable_3d_map))
        container = str(
            preflight_payload['adapter_status'].get('lifecycle_container')
            or '').strip()
        if args.strict_room_inspection:
            preflight_payload['perception_contract'] = (
                validate_perception_mission_config(
                    REPO_ROOT / 'config' / 'perception.yaml'))
            preflight_payload['navigation_clearance_contract'] = (
                validate_navigation_clearance_contract(
                    container, args.room_clearance_m))
        preflight_payload['main_entrance'] = open_main_entrance(container)
        preflight_payload['entrance_ingress'] = perform_entrance_ingress(
            distance_m=float(args.entrance_distance_m),
            speed_mps=float(args.entrance_speed_mps),
            wall_timeout_sec=float(args.entrance_wall_timeout_sec),
        )
        world_from_map = map_origin_after_straight_ingress(
            args.public_start_world_x,
            args.public_start_world_y,
            args.public_start_world_yaw,
            float(preflight_payload['entrance_ingress']['distance_reached_m']),
        )
        preflight_payload['world_from_map'] = {
            'x': world_from_map[0],
            'y': world_from_map[1],
            'yaw': world_from_map[2],
            'source': 'public_start_pose+public_scan_imu_ingress',
        }
    except (ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        raise SystemExit(str(exc)) from exc

    output_dir.mkdir(parents=True, exist_ok=False)
    run_id = output_dir.name
    first_person_capture = start_first_person_recording(container, run_id)
    command = build_launch_command(
        output_dir,
        scenario_seed=str(args.seed),
        code_version=git_state['commit'],
        exploration_timeout_s=args.exploration_timeout_sec,
        mission_time_budget_s=args.mission_time_budget_sec,
        target_floors=target_floors,
        per_floor_exploration_s=args.per_floor_exploration_sec,
        simenv_container=container,
        strict_room_inspection=bool(args.strict_room_inspection),
        enable_3d_map=bool(args.enable_3d_map),
        world_from_map=world_from_map,
        floor_height_m=float(args.floor_height_m),
        sphere_center_height_m=float(args.sphere_center_height_m),
        room_clearance_m=float(args.room_clearance_m),
    )
    manifest = {
        'schema': 'hazardwalker_slam_exploration_run_v1',
        'run_id': run_id,
        'scenario_seed': str(args.seed),
        'run_mode': run_mode,
        'status': 'running',
        'started_at_utc': utc_now(),
        'finished_at_utc': None,
        'wall_duration_sec': None,
        'git': git_state,
        'preflight': preflight_payload,
        'launch_command': command,
        'launch_exit_code': None,
        'final_nav_state': '',
        'failure_reason': '',
        'pointcloud_save': None,
        'first_person_video': None,
        'target_floors': list(target_floors),
        'strict_room_inspection': bool(args.strict_room_inspection),
        'enable_3d_map': bool(args.enable_3d_map),
        'evaluation': None,
        'navigation_acceptance': None,
    }
    manifest_path = output_dir / 'run_manifest.json'
    write_json(manifest_path, manifest)

    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String

    class StateObserver(Node):
        def __init__(self):
            super().__init__('hazardwalker_slam_exploration_runner')
            self.latest_state = ''
            self.start_clock_sec = None
            self.latest_clock_sec = None
            self.create_subscription(
                String, '/hw/nav/state', self._on_state, 10)
            self.create_subscription(
                Clock, '/clock', self._on_clock, 10)

        def _on_state(self, message):
            self.latest_state = str(message.data)

        def _on_clock(self, message):
            value = (
                float(message.clock.sec)
                + float(message.clock.nanosec) * 1e-9
            )
            if self.start_clock_sec is None or value < self.start_clock_sec:
                self.start_clock_sec = value
            self.latest_clock_sec = value

        @property
        def simulation_elapsed_sec(self):
            if self.start_clock_sec is None or self.latest_clock_sec is None:
                return None
            return max(0.0, self.latest_clock_sec - self.start_clock_sec)

    log_handle = (output_dir / 'launch.log').open(
        'w', encoding='utf-8', buffering=1)
    process = subprocess.Popen(
        command,
        cwd=REPO_ROOT,
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    rclpy.init()
    observer = StateObserver()
    started = time.monotonic()
    try:
        while True:
            rclpy.spin_once(observer, timeout_sec=0.2)
            manifest['final_nav_state'] = observer.latest_state
            simulation_elapsed = observer.simulation_elapsed_sec
            manifest['simulation_elapsed_sec'] = (
                round(simulation_elapsed, 3)
                if simulation_elapsed is not None else None)
            if observer.latest_state in ('FINISHED', 'FAILED'):
                if observer.latest_state == 'FAILED':
                    manifest['status'] = 'failed'
                    manifest['failure_reason'] = (
                        '导航进入 FAILED；检查 navigation/failures.jsonl '
                        '与 room_coverage.jsonl')
                    break
                if args.enable_3d_map:
                    try:
                        manifest['pointcloud_save'] = save_pointcloud_map()
                        manifest['status'] = 'complete'
                    except (RuntimeError, subprocess.SubprocessError) as exc:
                        manifest['status'] = 'failed'
                        manifest['failure_reason'] = str(exc)
                else:
                    manifest['pointcloud_save'] = {
                        'enabled': False,
                        'reason': '2d_slam_profile',
                    }
                    manifest['status'] = 'complete'
                break
            if process.poll() is not None:
                manifest['status'] = 'failed'
                manifest['failure_reason'] = '业务 launch 提前退出'
                break
            if time.monotonic() - started >= args.wall_timeout_sec:
                manifest['status'] = 'failed'
                manifest['failure_reason'] = (
                    '墙钟超时且未进入 FINISHED/FAILED')
                break
            # 节点自身状态机失效时，独立运行器仍必须按仿真任务预算终止。
            # 宽限只用于保存地图、结束返航状态，不允许无限旋转数小时。
            if (simulation_elapsed is not None
                    and simulation_elapsed
                    >= args.mission_time_budget_sec + 30.0):
                manifest['status'] = 'failed'
                manifest['failure_reason'] = (
                    '超过仿真任务预算 30 秒仍未进入 FINISHED/FAILED')
                break
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        manifest['status'] = 'interrupted'
        manifest['failure_reason'] = '人工中断'
    finally:
        observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        manifest['launch_exit_code'] = stop_process_group(process)
        log_handle.close()
        try:
            manifest['first_person_video'] = stop_first_person_recording(
                first_person_capture, output_dir)
        except (RuntimeError, subprocess.SubprocessError) as exc:
            if manifest['status'] == 'complete':
                manifest['status'] = 'failed'
                manifest['failure_reason'] = str(exc)
            else:
                manifest['failure_reason'] = (
                    f'{manifest["failure_reason"]}; {exc}').strip('; ')
        manifest['finished_at_utc'] = utc_now()
        manifest['wall_duration_sec'] = round(
            time.monotonic() - started, 3)
        if manifest['status'] == 'complete':
            try:
                manifest['navigation_acceptance'] = (
                    validate_navigation_acceptance(
                        output_dir,
                        target_floors,
                        expected_rooms_per_floor=(
                            args.expected_rooms_per_floor),
                        strict_room_inspection=bool(
                            args.strict_room_inspection),
                    )
                )
            except RuntimeError as exc:
                manifest['status'] = 'failed'
                manifest['failure_reason'] = str(exc)
        if manifest['status'] == 'complete' and args.strict_room_inspection:
            try:
                manifest['evaluation'] = evaluate_completed_run(
                    Path(args.truth_file).expanduser().resolve(),
                    output_dir / 'detected_danger.json',
                    output_dir / 'evaluation_result.json',
                )
            except (RuntimeError, subprocess.SubprocessError) as exc:
                manifest['status'] = 'failed'
                manifest['failure_reason'] = str(exc)
        write_json(manifest_path, manifest)
        write_handoff(output_dir, manifest)

    return 0 if manifest['status'] == 'complete' else 1


if __name__ == '__main__':
    raise SystemExit(main())
