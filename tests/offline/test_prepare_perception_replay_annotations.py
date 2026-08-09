"""感知回放人工标注草稿入口离线测试。"""

import importlib.util
import json
from pathlib import Path
import sys
import tempfile

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'prepare_perception_replay_annotations.py'
spec = importlib.util.spec_from_file_location('prepare_annotations', SCRIPT_PATH)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


def _write_png(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b'not-a-real-png-but-stable-test-evidence')


def test_draft_uses_exact_record_indices_and_only_archived_raw_frames():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        _write_png(root / 'selected_images/raw/frame_1.png')
        _write_png(root / 'selected_images/annotated/frame_1.png')
        _write_png(root / 'selected_images/raw/frame_3.png')
        records = [
            {'timestamp_sec': 1.0, 'evidence_raw_image': ''},
            {
                'timestamp_sec': 2.0,
                'evidence_raw_image': 'selected_images/raw/frame_1.png',
                'evidence_image': 'selected_images/annotated/frame_1.png',
            },
            {'timestamp_sec': 3.0, 'evidence_raw_image': ''},
            {
                'timestamp_sec': 4.0,
                'evidence_raw_image': 'selected_images/raw/frame_3.png',
            },
        ]
        frames = root / 'frames.jsonl'
        frames.write_text(
            json.dumps(records[0]) + '\n\n' +
            ''.join(json.dumps(item) + '\n' for item in records[1:]),
            encoding='utf-8')

        draft = module.build_annotation_draft(frames)

        assert draft['draft_incomplete'] is True
        assert [item['record_index'] for item in draft['frames']] == [1, 3]
        assert all(item['reviewed'] is False for item in draft['frames'])
        assert all(item['objects'] is None for item in draft['frames'])
        assert draft['source_frames']['sha256']


def test_draft_rejects_missing_or_external_evidence_images():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        frames = root / 'frames.jsonl'
        frames.write_text(json.dumps({
            'evidence_raw_image': 'missing.png',
        }) + '\n', encoding='utf-8')
        with pytest.raises(ValueError, match='原始图不存在'):
            module.build_annotation_draft(frames)

        outside = root.parent / f'{root.name}-outside.png'
        _write_png(outside)
        frames.write_text(json.dumps({
            'evidence_raw_image': f'../{outside.name}',
        }) + '\n', encoding='utf-8')
        try:
            with pytest.raises(ValueError, match='越出回放结果目录'):
                module.build_annotation_draft(frames)
        finally:
            outside.unlink(missing_ok=True)
