#!/usr/bin/env python3
"""在独立 ROS 域中执行一次可追溯的感知 rosbag 回放实验。

所属组：感知定位组。负责人：姜晨。
文件作用：校验固定 SEED 数据集，启动指定感知参数、证据记录器和可选合法
定位重算，播放一次 rosbag，安全收尾，并在提供人工标注时自动计算指标。
该脚本拒绝 ROS_DOMAIN_ID=42，避免离线回放干扰在线机器狗。
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / 'scripts'
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from official_perception_bag import (  # noqa: E402
    ALLOWED_LOCALIZATION_PROVENANCE,
    build_replay_command,
    validate_completed_session_manifest,
    validate_session_bag_payload,
)
from git_provenance import read_git_state  # noqa: E402


REQUIRED_REPLAY_NODES = {
    '/dynamic_detection_recorder_node',
    '/hazardwalker_command_mux',
    '/hsv_detector_node',
}

REQUIRED_NORMALIZED_OUTPUTS = (
    'frames.jsonl',
    'summary.json',
    'testing_record_perception.csv',
    'testing_record_perception.json',
    'README.md',
    'run_manifest.json',
    'failure_reasons.json',
    'perception_config.yaml',
    'replay_experiment_manifest.json',
)
REQUIRED_LABELED_OUTPUTS = (
    'evaluation.json',
    'algorithm_run_manifest.json',
    'testing_record_perception_labeled.csv',
    'testing_record_perception_labeled.json',
)
_CLEAN_LAUNCH_EXIT_CODES = {0, 130, -int(signal.SIGINT)}
FORMAL_EVIDENCE_ROOT = (REPO_ROOT / 'reports' / 'perception').resolve()


def load_valid_session(session_dir: Path) -> dict:
    """只接受已完成且逐话题数据库校验通过的数据集。"""

    manifest_path = session_dir / 'run_manifest.json'
    if not manifest_path.is_file():
        raise ValueError(f'缺少数据集清单：{manifest_path}')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    contract_errors = validate_completed_session_manifest(manifest)
    if contract_errors:
        raise ValueError('数据集合同无效：' + '；'.join(contract_errors))
    bag_dir = session_dir / str(manifest.get('bag_relative_path', 'bag'))
    if not bag_dir.is_dir():
        raise ValueError(f'缺少 rosbag 目录：{bag_dir}')
    payload_errors = validate_session_bag_payload(bag_dir, manifest)
    if payload_errors:
        raise ValueError('rosbag 实体校验失败：' + '；'.join(payload_errors))
    return manifest


def load_focus_diagnostic_session(session_dir: Path) -> tuple[dict, list[str]]:
    """允许只因巡检覆盖不足而失效的录包用于非正式专项复盘。

    该入口仍逐项校验话题、时长、预检副本和 rosbag 指纹，只放行完整巡检
    的路程/跨度门禁。调用方必须把输出写到正式成果目录之外，并在实验清单
    中保留全部合同错误，避免专项片段被误称为官方有效巡检。
    """

    manifest_path = session_dir / 'run_manifest.json'
    if not manifest_path.is_file():
        raise ValueError(f'缺少数据集清单：{manifest_path}')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    contract_errors = validate_completed_session_manifest(manifest)
    allowed_prefixes = (
        "status='invalid'",
        'bag_validation.status 不是 passed',
        '巡检运动覆盖门禁未通过',
        '平面路程 ',
        '平面覆盖跨度 ',
    )
    unexpected_errors = [
        error for error in contract_errors
        if not any(error.startswith(prefix) for prefix in allowed_prefixes)
    ]
    if unexpected_errors:
        raise ValueError(
            '专项诊断源仍有非覆盖类合同错误：' + '；'.join(unexpected_errors))
    if not contract_errors:
        return manifest, []
    bag_dir = session_dir / str(manifest.get('bag_relative_path', 'bag'))
    if not bag_dir.is_dir():
        raise ValueError(f'缺少 rosbag 目录：{bag_dir}')
    payload_errors = validate_session_bag_payload(bag_dir, manifest)
    if payload_errors:
        raise ValueError('rosbag 实体校验失败：' + '；'.join(payload_errors))
    return manifest, contract_errors


def resolve_localization_provenance(
        source_manifest: dict, *, recompute_localization: bool,
        requested_provenance: str) -> str:
    """解析本轮合法定位来源；历史来源不明时禁止直接复用 TF。"""

    if recompute_localization:
        if requested_provenance not in ALLOWED_LOCALIZATION_PROVENANCE:
            raise ValueError('当前重算定位来源不在官方合法白名单')
        return requested_provenance
    provenance = str(
        source_manifest.get('localization_provenance', 'unverified'))
    if (provenance not in ALLOWED_LOCALIZATION_PROVENANCE
            or not source_manifest.get(
                'historical_localization_reuse_eligible', False)):
        raise ValueError(
            '数据集未证明历史 TF/里程计来自合法 SLAM；请使用 '
            '--recompute-localization，或重新录制并声明定位来源')
    return provenance


def build_launch_command(
        *, output_dir: Path, parameter_file: Path, seed: str,
        code_version: str, recompute_localization: bool,
        localization_provenance: str = 'lidar_imu_slam') -> list[str]:
    """构造与报告参数哈希一致的业务启动命令。"""

    return [
        'ros2', 'launch', 'hazardwalker_bringup',
        'official_simenv_control_interface.launch.py',
        'control_mode:=stopped',
        'start_assist_alignment:=false',
        'start_navigation:=false',
        f'start_slam:={str(recompute_localization).lower()}',
        'slam_backend:=cartographer',
        f'start_legal_localization:={str(recompute_localization).lower()}',
        'start_decision:=false',
        'start_evidence_recorder:=true',
        f'perception_parameter_file:={parameter_file}',
        f'evidence_output_dir:={output_dir}',
        f'test_record_dir:={output_dir}',
        f'scenario_seed:={seed}',
        f'code_version:={code_version}',
        f'localization_provenance:={localization_provenance}',
    ]


def wait_for_nodes(
        required_nodes: set[str], *, env: dict, timeout_sec: float) -> set[str]:
    """等待回放消费者出现；超时返回最后一次节点集合。"""

    deadline = time.monotonic() + timeout_sec
    available: set[str] = set()
    while time.monotonic() <= deadline:
        try:
            output = subprocess.check_output(
                ['ros2', 'node', 'list'], cwd=REPO_ROOT, env=env,
                text=True, stderr=subprocess.STDOUT, timeout=3.0,
            )
            available = {
                line.strip() for line in output.splitlines()
                if line.strip().startswith('/')
            }
            if required_nodes <= available:
                return available
        except (OSError, subprocess.CalledProcessError,
                subprocess.TimeoutExpired):
            pass
        time.sleep(0.5)
    return available


def stop_process_group(process: subprocess.Popen, timeout_sec: float = 30.0) -> int:
    """先用 SIGINT 让记录器落盘，超时后再逐级终止整个 launch 进程组。"""

    if process.poll() is not None:
        return int(process.returncode)
    try:
        os.killpg(process.pid, signal.SIGINT)
        return int(process.wait(timeout=timeout_sec))
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            return int(process.wait(timeout=10.0))
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            return int(process.wait(timeout=5.0))


def build_segment_preroll_command(bag_dir: Path) -> list[str]:
    """回放数据集开头的瞬态静态合同，再跳转到困难片段。

    ``ros2 bag play --start-offset`` 不会补发偏移点之前的 ``/tf_static``
    和单次定位来源声明。先在同一离线 ROS 域短暂回放这两个允许话题，依靠
    transient-local QoS 锁存，避免为片段回放人工拼接坐标或伪造来源。
    """

    return [
        'ros2', 'bag', 'play', str(bag_dir),
        '--clock', '--rate', '10.0',
        '--playback-duration', '10.0',
        '--disable-keyboard-controls',
        '--topics',
        '/tf_static',
        '/hazardwalker/slam/localization_provenance',
    ]


def validate_normalized_outputs(output_dir: Path) -> list[str]:
    """验证回放目录足以复核 RGB-D、合法定位和规范测试材料。"""

    output_dir = Path(output_dir)
    failures = [
        f'缺少输出：{name}'
        for name in REQUIRED_NORMALIZED_OUTPUTS
        if not (output_dir / name).is_file()
    ]
    if failures:
        return failures
    try:
        summary = json.loads(
            (output_dir / 'summary.json').read_text(encoding='utf-8'))
        run_manifest = json.loads(
            (output_dir / 'run_manifest.json').read_text(encoding='utf-8'))
        experiment_manifest = json.loads(
            (output_dir / 'replay_experiment_manifest.json').read_text(
                encoding='utf-8'))
    except json.JSONDecodeError as exc:
        return [f'规范化 JSON 损坏：{exc}']

    frame_lines = [
        line for line in (output_dir / 'frames.jsonl').read_text(
            encoding='utf-8').splitlines() if line.strip()
    ]
    if not frame_lines:
        failures.append('frames.jsonl 没有感知输出')
        records = []
    else:
        try:
            records = [json.loads(line) for line in frame_lines]
        except json.JSONDecodeError as exc:
            failures.append(f'frames.jsonl 损坏：{exc}')
            records = []
    if int(summary.get('frame_count', 0)) != len(frame_lines):
        failures.append('summary.frame_count 与 frames.jsonl 行数不一致')
    if run_manifest.get('capture_status') != 'closed':
        failures.append('证据记录器未正常关闭')
    contract = run_manifest.get('evidence_contract', {})
    if contract.get('truth_inputs_used') is not False:
        failures.append('证据清单未证明运行期禁用真值')
    if contract.get('contract_violations'):
        failures.append('证据合同存在违规项')
    if int(summary.get('trajectory_sample_count', 0)) <= 0:
        failures.append('没有合法 SLAM 轨迹样本')
    trajectory_name = str(summary.get('trajectory_file', '')).strip()
    trajectory_path = output_dir / trajectory_name if trajectory_name else None
    if trajectory_path is None or not trajectory_path.is_file():
        failures.append('缺少合法 SLAM 轨迹文件')
    else:
        trajectory_lines = [
            line for line in trajectory_path.read_text(
                encoding='utf-8').splitlines() if line.strip()
        ]
        if len(trajectory_lines) != int(
                summary.get('trajectory_sample_count', 0)):
            failures.append('SLAM 轨迹文件与 summary 样本数不一致')
    if not any(record.get('localization_ready') is True for record in records):
        failures.append('没有一帧完成内参、深度和 TF 三维定位同步')

    sync_contract = run_manifest.get('evidence_synchronization', {})
    try:
        max_rgb_delta = float(
            sync_contract['max_detection_rgb_delta_sec'])
        max_depth_delta = float(sync_contract['max_rgb_depth_delta_sec'])
    except (KeyError, TypeError, ValueError):
        max_rgb_delta = max_depth_delta = float('nan')
        failures.append('证据清单缺少 RGB-D 时间同步阈值')
    evidence_records = [
        record for record in records if record.get('evidence_image')]
    if not evidence_records:
        failures.append('逐帧记录没有链接代表性标注图')
    for record in evidence_records:
        linked_paths = {
            '标注图': str(record.get('evidence_image', '')).strip(),
            '原始图': str(record.get('evidence_raw_image', '')).strip(),
            '深度': str(record.get('evidence_depth', '')).strip(),
        }
        for label, relative_path in linked_paths.items():
            if not relative_path:
                failures.append(f'代表证据缺少{label}链接')
                continue
            linked_path = (output_dir / relative_path).resolve()
            try:
                linked_path.relative_to(output_dir.resolve())
            except ValueError:
                failures.append(f'代表{label}链接越出实验目录')
                continue
            if not linked_path.is_file():
                failures.append(f'代表{label}链接的文件不存在')
        try:
            rgb_delta = float(record['evidence_detection_rgb_delta_sec'])
        except (KeyError, TypeError, ValueError):
            failures.append('代表图缺少检测到 RGB 的时间差')
            break
        if (not math.isfinite(rgb_delta) or rgb_delta < 0.0
                or (math.isfinite(max_rgb_delta)
                    and rgb_delta > max_rgb_delta)):
            failures.append('代表图检测与 RGB 时间不同步')
            break
        if record.get('evidence_depth'):
            try:
                depth_delta = float(record['evidence_rgb_depth_delta_sec'])
            except (KeyError, TypeError, ValueError):
                failures.append('代表深度缺少 RGB 到深度的时间差')
                break
            if (not math.isfinite(depth_delta) or depth_delta < 0.0
                    or (math.isfinite(max_depth_delta)
                        and depth_delta > max_depth_delta)):
                failures.append('代表 RGB 与深度时间不同步')
                break

    raw_dir = output_dir / 'selected_images' / 'raw'
    annotated_dir = output_dir / 'selected_images' / 'annotated'
    raw_keys = {
        path.name.removesuffix('_raw.png')
        for path in raw_dir.glob('*_raw.png')
    } if raw_dir.is_dir() else set()
    annotated_keys = {
        path.name.removesuffix('_annotated.png')
        for path in annotated_dir.glob('*_annotated.png')
    } if annotated_dir.is_dir() else set()
    if not raw_keys or raw_keys != annotated_keys:
        failures.append('代表性原始图与标注图缺失或未一一配对')
    depth_dir = output_dir / 'selected_depth'
    if not depth_dir.is_dir() or not any(depth_dir.glob('*.npy')):
        failures.append('缺少与代表 RGB 配对的米制深度证据')
    map_snapshot = str(summary.get('map_snapshot_file', '')).strip()
    if not map_snapshot or not (output_dir / map_snapshot).is_file():
        failures.append('缺少本轮合法 SLAM 地图快照')
    if not (output_dir / 'cartographer_map.pgm').is_file():
        failures.append('缺少地图图像 cartographer_map.pgm')
    if not (output_dir / 'README.md').read_text(
            encoding='utf-8').strip():
        failures.append('README.md 为空')
    parameter_snapshot = output_dir / 'perception_config.yaml'
    expected_parameter_hash = str(
        experiment_manifest.get('parameter_sha256', '')).strip()
    if (len(expected_parameter_hash) != 64
            or _sha256(parameter_snapshot) != expected_parameter_hash):
        failures.append('感知参数快照与本轮实际参数哈希不一致')
    annotation_snapshot_name = str(
        experiment_manifest.get('annotation_snapshot', '')).strip()
    if annotation_snapshot_name:
        annotation_snapshot = output_dir / annotation_snapshot_name
        expected_annotation_hash = str(
            experiment_manifest.get('annotation_sha256', '')).strip()
        if (not annotation_snapshot.is_file()
                or len(expected_annotation_hash) != 64
                or _sha256(annotation_snapshot) != expected_annotation_hash):
            failures.append('人工标注快照与本轮评估哈希不一致')
    return failures


def validate_formal_output_dir(output_dir: Path) -> None:
    """正式回放成果必须落在仓库 reports/perception 的子目录。"""

    resolved = Path(output_dir).expanduser().resolve()
    try:
        relative = resolved.relative_to(FORMAL_EVIDENCE_ROOT)
    except ValueError as exc:
        raise ValueError(
            '正式回放输出必须位于仓库 reports/perception/ 下；'
            '临时调试可显式使用 --allow-external-output') from exc
    if not relative.parts:
        raise ValueError('不能直接把成果写到 reports/perception 根目录')


def validate_labeled_outputs(output_dir: Path) -> list[str]:
    """验证人工标注评估真正生成了可比较指标。"""

    output_dir = Path(output_dir)
    failures = [
        f'缺少标注评估输出：{name}'
        for name in REQUIRED_LABELED_OUTPUTS
        if not (output_dir / name).is_file()
    ]
    if failures:
        return failures
    try:
        evaluation = json.loads(
            (output_dir / 'evaluation.json').read_text(encoding='utf-8'))
        replay_manifest = json.loads(
            (output_dir / 'replay_experiment_manifest.json').read_text(
                encoding='utf-8'))
    except (json.JSONDecodeError, OSError) as exc:
        return [f'标注评估 JSON 损坏：{exc}']
    if int(evaluation.get('labeled_frame_count', 0)) <= 0:
        failures.append('人工标注评估没有有效帧')
    inputs = evaluation.get('evaluation_inputs', {})
    if inputs.get('replay_control_contract_verified') is not True:
        failures.append('标注评估未通过回放参数与数据集合同')
    evaluated_annotation_hash = str(
        inputs.get('annotation_file', {}).get('sha256', '')).strip()
    if evaluated_annotation_hash != str(
            replay_manifest.get('annotation_sha256', '')).strip():
        failures.append('标注评估使用的文件与固化快照不一致')
    for metric_name in (
            'candidate_metrics', 'confirmed_output_metrics',
            'candidate_to_confirmation_latency_sec',
            'localization_error_m', 'processing_time_ms'):
        if not isinstance(evaluation.get(metric_name), dict):
            failures.append(f'标注评估缺少指标：{metric_name}')
    return failures


def _git_commit() -> str:
    try:
        return subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'], cwd=REPO_ROOT,
            text=True, stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return ''


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_git_state_for_run(
        git_state: dict, *, allow_external_output: bool,
        allow_dirty_worktree: bool) -> None:
    """正式成果只接受已提交代码；脏代码只能写入外部临时目录。"""

    if git_state.get('dirty') is not True:
        return
    if allow_dirty_worktree and allow_external_output:
        return
    entries = git_state.get('dirty_entries', [])
    changed = '、'.join(str(item) for item in entries[:5]) or 'Git 来源不可解析'
    raise ValueError(
        '正式回放拒绝未提交的代码或配置；请先提交，或仅在外部临时目录'
        '同时使用 --allow-external-output --allow-dirty-worktree。'
        f' 当前改动：{changed}')


def run(args) -> int:
    """执行一次实验；任何失败均写入独立结果目录，不伪造成功。"""

    if int(args.domain_id) == 42:
        raise SystemExit('拒绝在在线 ROS_DOMAIN_ID=42 中执行离线回放')
    if shutil.which('ros2') is None:
        raise SystemExit('未找到 ros2；请先 source ROS2 和本工作区 setup.bash')

    session_dir = Path(args.session).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    parameter_file = Path(args.parameter_file).expanduser().resolve()
    git_state = read_git_state(REPO_ROOT)
    if not args.allow_external_output:
        try:
            validate_formal_output_dir(output_dir)
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc
    try:
        validate_git_state_for_run(
            git_state,
            allow_external_output=bool(args.allow_external_output),
            allow_dirty_worktree=bool(args.allow_dirty_worktree),
        )
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    if output_dir.exists():
        raise SystemExit(f'输出目录已存在，拒绝覆盖：{output_dir}')
    if args.allow_invalid_source and not args.allow_external_output:
        raise SystemExit(
            '--allow-invalid-source 只允许与 --allow-external-output 同时使用')
    if not parameter_file.is_file():
        raise SystemExit(f'感知参数文件不存在：{parameter_file}')
    annotation_file = None
    if args.annotations:
        annotation_file = Path(args.annotations).expanduser().resolve()
        if not annotation_file.is_file():
            raise SystemExit(f'人工标注文件不存在：{annotation_file}')
    try:
        if args.allow_invalid_source:
            source_manifest, source_contract_errors = (
                load_focus_diagnostic_session(session_dir))
        else:
            source_manifest = load_valid_session(session_dir)
            source_contract_errors = []
    except (ValueError, json.JSONDecodeError) as exc:
        raise SystemExit(str(exc)) from exc

    seed = str(source_manifest.get('scenario_seed', '')).strip()
    if not seed:
        raise SystemExit('数据集清单缺少 scenario_seed')
    try:
        localization_provenance = resolve_localization_provenance(
            source_manifest,
            recompute_localization=bool(args.recompute_localization),
            requested_provenance=str(args.localization_provenance),
        )
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    output_dir.mkdir(parents=True)
    # 固定文件名便于测试组复核；业务 launch 也直接使用该快照，
    # 确保清单中的哈希就是实际运行参数，而不是事后复制的近似文件。
    parameter_snapshot = output_dir / 'perception_config.yaml'
    shutil.copy2(parameter_file, parameter_snapshot)
    annotation_snapshot = None
    if annotation_file is not None:
        annotation_snapshot = output_dir / 'evaluation_annotations.json'
        shutil.copy2(annotation_file, annotation_snapshot)
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(args.domain_id)
    code_version = _git_commit()
    launch_command = build_launch_command(
        output_dir=output_dir,
        parameter_file=parameter_snapshot,
        seed=seed,
        code_version=code_version,
        recompute_localization=bool(args.recompute_localization),
        localization_provenance=localization_provenance,
    )
    bag_dir = session_dir / str(source_manifest.get('bag_relative_path', 'bag'))
    replay_command = build_replay_command(
        bag_dir,
        rate=float(args.rate),
        start_offset_sec=float(args.start_offset_sec),
        playback_duration_sec=float(args.playback_duration_sec),
        recompute_localization=bool(args.recompute_localization),
    )
    segment_preroll_command = (
        build_segment_preroll_command(bag_dir)
        if float(args.start_offset_sec) > 0.0 else []
    )
    experiment_manifest = {
        'schema_version': 1,
        'status': 'starting',
        'started_at_utc': datetime.now(timezone.utc).isoformat(),
        'finished_at_utc': None,
        'source_session': str(session_dir),
        'source_manifest_sha256': _sha256(session_dir / 'run_manifest.json'),
        'source_live_chain_preflight_sha256': str(
            source_manifest.get('live_chain_preflight', {}).get('sha256', '')),
        'source_bag_fingerprint_sha256': str(
            source_manifest.get('bag_validation', {}).get(
                'content_fingerprint_sha256', '')),
        'source_contract_mode': (
            'focus_diagnostic_override'
            if source_contract_errors else 'formal_validated'
        ),
        'source_contract_errors': source_contract_errors,
        'formal_evidence_eligible': not source_contract_errors,
        'scenario_seed': seed,
        'git': git_state,
        'ros_domain_id': str(args.domain_id),
        'algorithm_label': args.algorithm_label,
        'replay_start_offset_sec': float(args.start_offset_sec),
        'replay_duration_sec': float(args.playback_duration_sec),
        'parameter_file': str(parameter_file),
        'parameter_snapshot': parameter_snapshot.name,
        'parameter_sha256': _sha256(parameter_snapshot),
        'annotation_snapshot': (
            annotation_snapshot.name if annotation_snapshot is not None else ''),
        'annotation_sha256': (
            _sha256(annotation_snapshot)
            if annotation_snapshot is not None else ''),
        'recompute_localization': bool(args.recompute_localization),
        'localization_provenance': localization_provenance,
        'launch_command': shlex.join(launch_command),
        'segment_preroll_command': (
            shlex.join(segment_preroll_command)
            if segment_preroll_command else ''
        ),
        'segment_preroll_exit_code': None,
        'replay_command': shlex.join(replay_command),
        'launch_exit_code': None,
        'replay_exit_code': None,
        'evaluation_exit_code': None,
        'failure_reason': '',
    }
    manifest_path = output_dir / 'replay_experiment_manifest.json'
    _write_manifest(manifest_path, experiment_manifest)

    launch_process = subprocess.Popen(
        launch_command, cwd=REPO_ROOT, env=env, start_new_session=True)
    replay_exit_code = 1
    evaluation_exit_code = None
    try:
        available_nodes = wait_for_nodes(
            REQUIRED_REPLAY_NODES,
            env=env,
            timeout_sec=float(args.startup_timeout_sec),
        )
        missing_nodes = sorted(REQUIRED_REPLAY_NODES - available_nodes)
        if launch_process.poll() is not None:
            raise RuntimeError(
                f'业务 launch 提前退出：{launch_process.returncode}')
        if missing_nodes:
            raise RuntimeError('回放消费者未就绪：' + ', '.join(missing_nodes))
        experiment_manifest['status'] = 'replaying'
        _write_manifest(manifest_path, experiment_manifest)
        if segment_preroll_command:
            preroll_exit_code = subprocess.call(
                segment_preroll_command, cwd=REPO_ROOT, env=env,
            )
            experiment_manifest['segment_preroll_exit_code'] = (
                preroll_exit_code
            )
            if preroll_exit_code != 0:
                raise RuntimeError(
                    f'片段静态合同预滚失败：{preroll_exit_code}'
                )
        replay_exit_code = subprocess.call(
            replay_command, cwd=REPO_ROOT, env=env)
        if replay_exit_code != 0:
            raise RuntimeError(f'rosbag 回放失败：{replay_exit_code}')
        # 给最后一帧回调和证据落盘留出一个调度周期。
        time.sleep(1.0)
    except (KeyboardInterrupt, RuntimeError) as exc:
        experiment_manifest['failure_reason'] = str(exc)
    finally:
        launch_exit_code = stop_process_group(launch_process)
        experiment_manifest['launch_exit_code'] = launch_exit_code
        experiment_manifest['replay_exit_code'] = replay_exit_code

    output_failures = validate_normalized_outputs(output_dir)
    experiment_manifest['normalized_output_validation'] = {
        'passed': not output_failures,
        'failures': output_failures,
    }
    if output_failures and not experiment_manifest['failure_reason']:
        experiment_manifest['failure_reason'] = '；'.join(output_failures)
    if (experiment_manifest['launch_exit_code'] not in _CLEAN_LAUNCH_EXIT_CODES
            and not experiment_manifest['failure_reason']):
        experiment_manifest['failure_reason'] = (
            f"业务 launch 非正常退出：{experiment_manifest['launch_exit_code']}")

    if args.annotations and not experiment_manifest['failure_reason']:
        evaluate_command = [
            sys.executable,
            str(REPO_ROOT / 'scripts' / 'evaluate_perception_replay.py'),
            '--frames', str(output_dir / 'frames.jsonl'),
            '--annotations', str(annotation_snapshot),
            '--output-dir', str(output_dir),
            '--scenario', args.scenario or output_dir.name,
            '--algorithm-label', args.algorithm_label,
            '--parameter-file', str(parameter_snapshot),
        ]
        evaluation_exit_code = subprocess.call(
            evaluate_command, cwd=REPO_ROOT, env=env)
        experiment_manifest['evaluation_command'] = shlex.join(evaluate_command)
        experiment_manifest['evaluation_exit_code'] = evaluation_exit_code
        if evaluation_exit_code != 0:
            experiment_manifest['failure_reason'] = (
                f'人工标注评估失败：{evaluation_exit_code}')
        else:
            labeled_failures = validate_labeled_outputs(output_dir)
            experiment_manifest['labeled_output_validation'] = {
                'passed': not labeled_failures,
                'failures': labeled_failures,
            }
            if labeled_failures:
                experiment_manifest['failure_reason'] = '；'.join(
                    labeled_failures)

    experiment_manifest['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
    if experiment_manifest['failure_reason']:
        experiment_manifest['status'] = 'failed'
    elif source_contract_errors and args.annotations:
        experiment_manifest['status'] = 'diagnostic_complete'
    elif source_contract_errors:
        experiment_manifest['status'] = 'diagnostic_captured_unlabeled'
    elif args.annotations:
        experiment_manifest['status'] = 'complete'
    else:
        experiment_manifest['status'] = 'captured_unlabeled'
    _write_manifest(manifest_path, experiment_manifest)
    return 0 if experiment_manifest['status'] in (
        'complete', 'captured_unlabeled', 'diagnostic_complete',
        'diagnostic_captured_unlabeled') else 2


def _write_manifest(path: Path, payload: dict) -> None:
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    temporary.replace(path)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument(
        '--allow-external-output', action='store_true',
        help='仅用于诊断：允许把非正式结果写到 reports/perception 之外',
    )
    parser.add_argument(
        '--allow-dirty-worktree', action='store_true',
        help='仅与 --allow-external-output 同用：允许未提交代码做临时诊断',
    )
    parser.add_argument(
        '--allow-invalid-source', action='store_true',
        help=(
            '仅与 --allow-external-output 同用：允许只因巡检路程/跨度不足而'
            '失效的完整录包做专项诊断回放；输出永远标记为非正式证据'),
    )
    parser.add_argument('--parameter-file', default='config/perception.yaml')
    parser.add_argument('--algorithm-label', default='hsv_depth_tf')
    parser.add_argument('--annotations', default='')
    parser.add_argument('--scenario', default='')
    parser.add_argument('--domain-id', type=int, default=142)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument(
        '--start-offset-sec', type=float, default=0.0,
        help='从数据集起点偏移指定秒数后开始，用于复跑已定位的困难片段',
    )
    parser.add_argument(
        '--playback-duration-sec', type=float, default=0.0,
        help='仅回放指定秒数；0 表示回放到数据集末尾',
    )
    parser.add_argument('--startup-timeout-sec', type=float, default=30.0)
    parser.add_argument('--recompute-localization', action='store_true')
    parser.add_argument(
        '--localization-provenance', default='lidar_imu_slam',
        choices=sorted(ALLOWED_LOCALIZATION_PROVENANCE),
        help='仅在 --recompute-localization 时声明当前合法定位实现',
    )
    return parser


def main() -> int:
    return run(_parser().parse_args())


if __name__ == '__main__':
    raise SystemExit(main())
