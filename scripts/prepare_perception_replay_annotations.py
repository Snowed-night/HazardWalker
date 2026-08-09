#!/usr/bin/env python3
"""从感知回放证据生成待人工审核的标注草稿。

所属组：感知定位组。负责人：姜晨。
文件作用：读取 ``frames.jsonl``，仅登记已经归档且仍存在的原始 RGB 帧，
保留精确 ``record_index``，生成不会被正式评估器误接受的人工标注草稿。
算法输出只用于定位原始图片，不会预填目标框，避免把预测结果当作人工真值。
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _resolve_evidence_path(root: Path, relative_value: object) -> Path:
    """只允许引用本轮结果目录内的证据，防止草稿串到其他实验。"""

    relative = Path(str(relative_value or '').strip())
    if not str(relative):
        raise ValueError('原始图路径为空')
    path = (root / relative).resolve()
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f'原始图越出回放结果目录：{relative}') from exc
    if not path.is_file():
        raise ValueError(f'原始图不存在：{path}')
    return path


def build_annotation_draft(frames_path: Path) -> dict:
    """为所有已保存原始帧生成显式未审核条目。"""

    frames_path = frames_path.expanduser().resolve()
    if not frames_path.is_file():
        raise ValueError(f'frames.jsonl 不存在：{frames_path}')
    result_root = frames_path.parent.resolve()
    selected = []
    seen_images = set()
    record_index = -1
    for line_number, line in enumerate(
            frames_path.read_text(encoding='utf-8').splitlines(), start=1):
        if not line.strip():
            continue
        # 评估器会跳过空行后建立紧凑列表，因此索引必须按有效 JSON 记录递增。
        record_index += 1
        record = json.loads(line)
        if not isinstance(record, dict):
            raise ValueError(f'第 {line_number} 行不是 JSON 对象')
        raw_value = str(record.get('evidence_raw_image', '')).strip()
        if not raw_value:
            continue
        raw_path = _resolve_evidence_path(result_root, raw_value)
        normalized_relative = raw_path.relative_to(result_root).as_posix()
        if normalized_relative in seen_images:
            raise ValueError(f'原始图被多个记录重复引用：{normalized_relative}')
        seen_images.add(normalized_relative)
        annotated_value = str(record.get('evidence_image', '')).strip()
        annotated_relative = ''
        if annotated_value:
            annotated_path = _resolve_evidence_path(result_root, annotated_value)
            annotated_relative = annotated_path.relative_to(result_root).as_posix()
        selected.append({
            'record_index': record_index,
            'timestamp_sec': record.get('timestamp_sec'),
            'raw_image': normalized_relative,
            # 检测标注图仅方便复核算法输出，人工框必须依据 raw_image 绘制。
            'algorithm_overlay_image': annotated_relative,
            'reviewed': False,
            # ``null`` 明确表示尚未审核；审核过的负样本必须改成空列表 []。
            'objects': None,
        })
    if not selected:
        raise ValueError('frames.jsonl 中没有可供人工标注的已归档原始帧')
    return {
        'schema_version': 1,
        'annotation_provenance': 'manual_image_annotation',
        'draft_incomplete': True,
        'source_frames': {
            'path': str(frames_path),
            'sha256': _sha256(frames_path),
        },
        'instructions': [
            '逐帧只查看 raw_image 并填写 objects；algorithm_overlay_image 仅用于事后核对。',
            '审核过的负样本填写 objects=[]；红球填写稳定 id、bbox，三维参考可选。',
            '全部条目 reviewed=true 后，将 draft_incomplete 改为 false 才能正式评估。',
        ],
        'frames': selected,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', required=True, help='回放结果中的 frames.jsonl')
    parser.add_argument('--output', required=True, help='待人工填写的 JSON 草稿')
    args = parser.parse_args()
    output = Path(args.output).expanduser().resolve()
    if output.exists():
        raise SystemExit(f'拒绝覆盖已有标注：{output}')
    try:
        draft = build_annotation_draft(Path(args.frames))
    except (ValueError, json.JSONDecodeError) as exc:
        raise SystemExit(str(exc)) from exc
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(draft, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    print(f'已生成 {len(draft["frames"])} 帧人工标注草稿：{output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
