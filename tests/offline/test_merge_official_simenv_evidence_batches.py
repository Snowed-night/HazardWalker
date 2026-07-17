"""分批实测证据合并器测试。"""

import json
import os
import sys
import tempfile
from pathlib import Path


REPO_ROOT = Path(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
sys.path.insert(0, str(REPO_ROOT / 'scripts'))

from merge_official_simenv_evidence_batches import merge_batches


def _write_batch(root, name, case_id, result='pass'):
    path = root / name
    path.mkdir()
    (path / 'summary.json').write_text(json.dumps({
        'schema': 'test_schema', 'suite': 'red_objects',
    }), encoding='utf-8')
    (path / 'cases.json').write_text(json.dumps([{
        'case_id': case_id, 'result': result,
    }]), encoding='utf-8')
    return path


def test_merge_keeps_all_case_results_and_batch_provenance():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        first = _write_batch(root, 'batch_01', 'object_01', 'pass')
        second = _write_batch(root, 'batch_02', 'object_02', 'fail')
        summary = merge_batches('red_objects', [first, second], root / 'merged')

        assert summary['case_count'] == 2
        assert summary['pass_count'] == 1
        assert summary['fail_count'] == 1
        records = json.loads((root / 'merged' / 'cases.json').read_text(encoding='utf-8'))
        assert [item['source_batch'] for item in records] == ['batch_01', 'batch_02']
