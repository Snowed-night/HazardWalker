#!/usr/bin/env python3
"""正式人工巡检的实时预检与 rosbag 录制统一入口。

所属组：感知定位组。负责人：姜晨。
文件作用：用同一份固定 SEED、控制源和定位来源先执行只读实时预检；通过后
直接把当前进程替换为正式录包器。脚本不启动平台、不发布速度，也不代替人工
巡检，避免分步复制命令时产生参数漂移。
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / 'scripts'
OFFICIAL_RANDOM_ROOT = (
    REPO_ROOT / 'reports' / 'perception' / 'official_random').resolve()
SAFE_NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]*$')
LOCALIZATION_PROVENANCE = (
    'lidar_imu_slam',
    'lidar_imu_slam+public_floor_action',
    'visual_inertial_slam',
)


def _safe_text(value: str, *, field: str) -> str:
    text = str(value or '').strip()
    if not SAFE_NAME.fullmatch(text):
        raise ValueError(
            f'{field} 只能包含字母、数字、点、下划线和连字符：{text!r}')
    return text


def validate_output_paths(report_dir: Path, session_dir: Path) -> tuple[Path, Path]:
    """保证正式报告进入仓库规范目录，而大体积 rosbag 留在仓库外。"""

    report_dir = report_dir.expanduser().resolve()
    session_dir = session_dir.expanduser().resolve()
    try:
        relative_report = report_dir.relative_to(OFFICIAL_RANDOM_ROOT)
    except ValueError as exc:
        raise ValueError(
            f'正式报告目录必须位于 {OFFICIAL_RANDOM_ROOT} 下') from exc
    if not relative_report.parts:
        raise ValueError('正式报告必须使用 official_random 下的独立运行目录')
    try:
        session_dir.relative_to(REPO_ROOT)
    except ValueError:
        pass
    else:
        raise ValueError('rosbag 输出必须位于仓库外的数据盘目录')
    if report_dir.exists():
        raise ValueError(f'报告目录已存在，拒绝覆盖：{report_dir}')
    if session_dir.exists():
        raise ValueError(f'录包目录已存在，拒绝覆盖：{session_dir}')
    return report_dir, session_dir


def build_commands(
        *, seed: str, run_id: str, operator: str,
        localization_provenance: str, control_source: str,
        report_dir: Path, session_dir: Path, sample_sec: float,
        first_person_health_url: str, topic_wait_timeout_sec: float,
        preflight_max_age_sec: float) -> tuple[list[str], list[str]]:
    """生成共享同一正式合同的预检与录包命令。"""

    seed = _safe_text(seed, field='seed')
    run_id = _safe_text(run_id, field='run-id')
    operator = str(operator or '').strip()
    if not operator:
        raise ValueError('operator 不能为空')
    if control_source not in ('keyboard', 'navigation'):
        raise ValueError(f'不支持的控制源：{control_source}')
    if localization_provenance not in LOCALIZATION_PROVENANCE:
        raise ValueError(f'不支持的定位来源：{localization_provenance}')
    if sample_sec <= 0.0 or topic_wait_timeout_sec <= 0.0:
        raise ValueError('采样和话题等待时间必须为正数')
    if preflight_max_age_sec <= 0.0:
        raise ValueError('预检最大有效期必须为正数')
    preflight_report = report_dir / 'live_chain_preflight.json'
    preflight = [
        sys.executable,
        str(SCRIPTS_DIR / 'verify_perception_live_chain.py'),
        '--control-source', control_source,
        '--localization-provenance', localization_provenance,
        '--scenario-seed', seed,
        '--sample-sec', str(float(sample_sec)),
        '--first-person-health-url', first_person_health_url,
        '--output', str(preflight_report),
    ]
    record = [
        sys.executable,
        str(SCRIPTS_DIR / 'official_perception_bag.py'),
        'record',
        '--output', str(session_dir),
        '--seed', seed,
        '--run-id', run_id,
        '--operator', operator,
        '--localization-provenance', localization_provenance,
        '--preflight-report', str(preflight_report),
        '--preflight-max-age-sec', str(float(preflight_max_age_sec)),
        '--topic-wait-timeout-sec', str(float(topic_wait_timeout_sec)),
    ]
    return preflight, record


def validate_preflight_report(path: Path, *, seed: str,
                              control_source: str,
                              localization_provenance: str) -> dict:
    """在进入录包器前再次核对预检输出，拒绝弱校验或参数漂移。"""

    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f'无法读取实时预检报告：{path}') from exc
    if not isinstance(payload, dict) or payload.get('passed') is not True:
        raise ValueError('实时预检未通过，拒绝开始录包')
    if str(payload.get('expected_scenario_seed', '')).strip() != str(seed):
        raise ValueError('预检报告 SEED 与本轮参数不一致')
    if payload.get('control_source') != control_source:
        raise ValueError('预检报告控制源与本轮参数不一致')
    if payload.get('expected_localization_provenance') != localization_provenance:
        raise ValueError('预检报告定位来源与本轮参数不一致')
    if payload.get('traffic_checked') is not True:
        raise ValueError('预检报告没有完成实时消息采样')
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--operator', required=True)
    parser.add_argument(
        '--localization-provenance', choices=LOCALIZATION_PROVENANCE,
        default='lidar_imu_slam')
    parser.add_argument(
        '--control-source', choices=('keyboard', 'navigation'),
        default='keyboard')
    parser.add_argument('--report-dir', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--sample-sec', type=float, default=3.0)
    parser.add_argument('--topic-wait-timeout-sec', type=float, default=20.0)
    parser.add_argument('--preflight-max-age-sec', type=float, default=300.0)
    parser.add_argument(
        '--first-person-health-url',
        default='http://127.0.0.1:6082/healthz')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='只显示两条确定命令，不访问 ROS 图，也不创建目录')
    args = parser.parse_args()

    try:
        report_dir, session_dir = validate_output_paths(
            Path(args.report_dir), Path(args.output))
        preflight, record = build_commands(
            seed=args.seed,
            run_id=args.run_id,
            operator=args.operator,
            localization_provenance=args.localization_provenance,
            control_source=args.control_source,
            report_dir=report_dir,
            session_dir=session_dir,
            sample_sec=args.sample_sec,
            first_person_health_url=args.first_person_health_url,
            topic_wait_timeout_sec=args.topic_wait_timeout_sec,
            preflight_max_age_sec=args.preflight_max_age_sec,
        )
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    if args.dry_run:
        print(json.dumps({
            'schema': 'hazardwalker_official_perception_patrol_plan_v1',
            'preflight_command': shlex.join(preflight),
            'record_command': shlex.join(record),
        }, ensure_ascii=False, indent=2))
        return 0

    completed = subprocess.run(preflight, cwd=REPO_ROOT, check=False)
    if completed.returncode != 0:
        raise SystemExit(completed.returncode)
    try:
        validate_preflight_report(
            report_dir / 'live_chain_preflight.json',
            seed=str(args.seed).strip(),
            control_source=args.control_source,
            localization_provenance=args.localization_provenance)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    # 录包阶段用 exec 替换当前进程，让 Ctrl+C 只由录包器及 rosbag 处理，
    # 确保停止后仍能完成 SQLite 统计和 run_manifest.json 收尾。
    os.execv(sys.executable, record)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
