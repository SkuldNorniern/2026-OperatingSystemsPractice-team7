"""Compare latest V1/V2 runs, or two explicitly supplied run directories."""
import argparse,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def summarize(folder):
    state=json.loads((folder/'status.json').read_text())
    markers=json.loads((folder/'markers.json').read_text())
    events=[json.loads(l) for l in (folder/'events.jsonl').read_text().splitlines()]
    info=json.loads((folder/'run_info.json').read_text()) if (folder/'run_info.json').exists() else {}
    return {'world_sha256':info.get('world_sha256'),'version':state.get('version','unknown'),'run':str(folder),'status':state['status'],
        'elapsed_sim_seconds':state.get('elapsed_sim_seconds'),'distance_m':state.get('distance_m'),
        'known_map_cells':state.get('known_map_cells'),
        'detected_ids':sorted(int(i) for i in markers['markers']),
        'visited_ids':sorted(int(i) for i in markers['visited']),
        'visit_order':[e['id'] for e in events if e['event']=='marker_visited'],
        'corner_waypoints':min(4,sum(e['event']=='waypoint_reached' for e in events)),
        'scan_rotations':sum(e['event']=='scan_finished' for e in events),
        'frontier_goals':sum(e['event']=='v2_goal_selected' and e['kind']=='frontier' for e in events),
        'coverage_goals':sum(e['event']=='v2_goal_selected' and e['kind']=='coverage' for e in events)}

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('runs',nargs='*',type=Path);args=ap.parse_args()
    folders=args.runs
    if not folders:
        for version in ['v1','v2']:
            matches=sorted((ROOT/'runs').glob('*-'+version))
            if matches:folders.append(matches[-1])
    if not folders:ap.error('No runs found; execute --version v1 and --version v2 first.')
    result=[summarize(f) for f in folders]
    print(json.dumps(result,indent=2,ensure_ascii=False))
    (ROOT/'runs/comparison.json').write_text(json.dumps(result,indent=2,ensure_ascii=False))
