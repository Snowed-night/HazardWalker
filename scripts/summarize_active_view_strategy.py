"""比较单视角候选判定与真实运动多视角确认，生成同目录策略证据。"""

import argparse
import json
from pathlib import Path


def _metrics(rows, field):
    tp = fp = tn = fn = 0
    for row in rows:
        truth = bool(json.loads(row['metadata']).get('is_sphere'))
        predicted = int(row.get(field, 0)) > 0
        if truth and predicted:
            tp += 1
        elif truth:
            fn += 1
        elif predicted:
            fp += 1
        else:
            tn += 1
    precision = tp / max(tp + fp, 1)
    recall = tp / max(tp + fn, 1)
    return {
        'tp': tp, 'fp': fp, 'tn': tn, 'fn': fn,
        'precision': round(precision, 4),
        'recall': round(recall, 4),
    }


def summarize(suite_dir: Path):
    rows = json.loads((suite_dir / 'cases.json').read_text(encoding='utf-8'))
    motion_distances = []
    for row in rows:
        motion_distances.append(sum(
            float(item.get('translation_m', 0.0))
            for item in json.loads(row.get('motions', '[]'))
        ))
    result = {
        'comparison': 'single_view_candidate_vs_active_multiview_confirmation',
        'case_count': len(rows),
        'single_view_candidate': _metrics(rows, 'initial_strict_count'),
        'active_multiview_confirmation': _metrics(rows, 'best_target_confirmed_count'),
        'mean_actual_view_count': round(
            sum(float(row['actual_robot_view_count']) for row in rows) / max(len(rows), 1), 3,
        ),
        'mean_robot_translation_m': round(sum(motion_distances) / max(len(rows), 1), 4),
        'note': (
            '该对比证明主动侧视对确认精度的收益；尚不等同于整栋房间探索速度对比。'
        ),
    }
    (suite_dir / 'strategy_comparison.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8',
    )
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('suite_dir', type=Path)
    args = parser.parse_args()
    print(json.dumps(summarize(args.suite_dir), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
