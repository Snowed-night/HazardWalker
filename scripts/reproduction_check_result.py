#!/usr/bin/env python3
"""实验结束后核验房间、实际返回一层、SLAM和官方危险源匹配；不会启动仿真。

最终状态FINISHED必须同时具有返回一层和起点区域的轨迹证据。
没有正式评估时保留空值，不把缺失数据记作零漏报或零虚警。
"""
from pathlib import Path
import argparse,json,math

def read(path):
    try:return json.loads(path.read_text(encoding='utf-8'))
    except (OSError,ValueError):return {}

def rows(path):
    if not path.is_file():return
    with path.open(encoding='utf-8') as f:
        for line in f:
            try:yield json.loads(line)
            except ValueError:continue

def evaluate(output):
    manifest=read(output/'run_manifest.json');evaluation=read(output/'evaluation_result.json')
    metrics=evaluation.get('metrics') or {};slam=manifest.get('slam_physical_alignment') or {}
    rooms={str(f):sorted({r['sector'] for r in rows(output/'navigation/room_coverage.jsonl') if r.get('floor')==f and r.get('phase')=='completed'}) for f in range(3)}
    last=None
    for record in rows(output/'navigation/trajectory.jsonl'):last=record
    start=manifest.get('preflight',{}).get('world_from_map',{})
    distance=None
    if last and all(isinstance(v,(int,float)) for v in [last.get('official_x'),last.get('official_y'),start.get('x'),start.get('y')]):
        distance=math.hypot(last['official_x']-start['x'],last['official_y']-start['y'])
    checks={'runner_complete':manifest.get('status')=='complete','finished_state':manifest.get('final_nav_state')=='FINISHED',
        'twelve_rooms':all(set(v)=={'far_left','far_right','near_left','near_right'} for v in rooms.values()),'navigation_accepted':(manifest.get('navigation_acceptance') or {}).get('passed') is True,
        'returned_ground_floor':bool(last and last.get('floor_index')==0),'returned_start_region':distance is not None and distance<=1.0,
        'slam_three_floors':slam.get('accepted') is True and slam.get('floor_count')==3,
        'all_hazards_correct':bool(metrics.get('truth_count',0)>0 and metrics.get('correct')==metrics.get('truth_count') and metrics.get('missed')==0 and metrics.get('false_alarms')==0)}
    return {'passed':all(checks.values()),'checks':checks,'source_result':str(output),'seed':manifest.get('scenario_seed'),
        'official_metrics':metrics,'completed_rooms':rooms,'last_floor_index':last.get('floor_index') if last else None,
        'distance_from_start_m':distance,'slam_p95_m':slam.get('p95_error_m'),'slam_max_m':slam.get('max_error_m'),
        'runner_failure_reason':manifest.get('failure_reason'),'localization_sources':manifest.get('preflight',{}).get('adapter_status',{}).get('sources',{})}

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('result_dir',type=Path)
    args=parser.parse_args();output=args.result_dir.resolve()
    if not (output/'run_manifest.json').is_file():raise SystemExit('缺少run_manifest.json；尚无完整运行记录：'+str(output))
    result=evaluate(output)
    (output/'reproduction_acceptance.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    print('完整三层复现通过。' if result['passed'] else '本轮完整验收未通过，详见checks与原始记录。')
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
