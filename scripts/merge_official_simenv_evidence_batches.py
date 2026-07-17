#!/usr/bin/env python3
"""合并同一官方 SimEnv 套件的分批隔离实测结果。

Gazebo Classic 长时间连续 spawn/delete 可能让服务退化。调用方可以每 6--8 个案例
重建隔离容器，再用本脚本把独立批次合并为一个正式目录。脚本只复制已经生成的证据，
不重新检测、不修改案例结论，也不把缺失案例写成通过。
"""

from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path


def _read_batch(path: Path) -> tuple[dict, list[dict]]:
    summary_path = path / 'summary.json'
    cases_path = path / 'cases.json'
    if not summary_path.is_file() or not cases_path.is_file():
        raise ValueError(f'批次缺少 summary.json 或 cases.json：{path}')
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    cases = json.loads(cases_path.read_text(encoding='utf-8'))
    if not isinstance(cases, list):
        raise ValueError(f'cases.json 必须为列表：{cases_path}')
    return summary, cases


def merge_batches(suite: str, batch_dirs: list[Path], output_dir: Path) -> dict:
    """合并去重后的批次，并保留每个批次的原始文件夹以便审计。"""

    if not batch_dirs:
        raise ValueError('至少需要一个批次目录。')
    records = []
    seen_ids = set()
    schemas = set()
    for batch_dir in batch_dirs:
        summary, cases = _read_batch(batch_dir)
        if str(summary.get('suite')) != suite:
            raise ValueError(f'批次 suite 不一致：{batch_dir}')
        schemas.add(str(summary.get('schema', '')))
        for case in cases:
            case_id = str(case.get('case_id', ''))
            if not case_id or case_id in seen_ids:
                raise ValueError(f'重复或空 case_id：{case_id}')
            seen_ids.add(case_id)
            copied = dict(case)
            copied['source_batch'] = batch_dir.name
            records.append(copied)

    if len(schemas) != 1:
        raise ValueError(f'批次 schema 不一致：{sorted(schemas)}')
    records.sort(key=lambda item: str(item['case_id']))
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True)
    for batch_dir in batch_dirs:
        shutil.copytree(batch_dir, output_dir / 'batches' / batch_dir.name)

    summary = {
        'schema': schemas.pop(),
        'suite': suite,
        'case_count': len(records),
        'pass_count': sum(item.get('result') == 'pass' for item in records),
        'fail_count': sum(item.get('result') != 'pass' for item in records),
        'batch_count': len(batch_dirs),
        'truth_usage': '仅在快照保存后离线匹配；运行期检测器、运动策略不读取真值。',
        'note': '分批结果因 Gazebo 清理隔离而合并；每个原始批次完整保留在 batches/。',
    }
    (output_dir / 'cases.json').write_text(
        json.dumps(records, ensure_ascii=False, indent=2) + '\n', encoding='utf-8',
    )
    (output_dir / 'summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8',
    )
    fields = sorted({key for item in records for key in item})
    with (output_dir / 'cases.csv').open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(records)
    (output_dir / 'README.md').write_text(
        '# 官方 SimEnv 分批实测合并记录\n\n'
        f'- 套件：`{suite}`\n'
        f'- 案例数：{summary["case_count"]}\n'
        f'- 通过/失败：{summary["pass_count"]}/{summary["fail_count"]}\n'
        f'- 批次数：{summary["batch_count"]}\n\n'
        '由于每批均在独立 Gazebo 容器中完成，`batches/` 下保留了原始截图、JSON、日志和汇总。'
        '缺失案例不会被补写为通过。\n', encoding='utf-8',
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--suite', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('batch_dirs', nargs='+', type=Path)
    args = parser.parse_args()
    print(json.dumps(merge_batches(args.suite, args.batch_dirs, args.output_dir), ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
