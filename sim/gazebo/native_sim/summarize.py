"""Summarize one completed run and draw saved lidar map / observed markers."""
import json,sys
from pathlib import Path
import cv2,numpy as np
folder=Path(sys.argv[1]) if len(sys.argv)>1 else sorted((Path(__file__).parent/'runs').glob('*'))[-1]
events=[json.loads(l) for l in (folder/'events.jsonl').read_text().splitlines()]
markers=json.loads((folder/'markers.json').read_text());status=json.loads((folder/'status.json').read_text())
summary={'version':status.get('version','legacy'),'status':status['status'],'distance_m':status.get('distance_m'),'elapsed_sim_seconds':status.get('elapsed_sim_seconds'),'detected_ids':sorted(int(k) for k in markers['markers']),
 'visited_ids':sorted(int(k) for k in markers['visited']),
 'waypoints_reached':sum(e['event']=='waypoint_reached' for e in events),
 'scan_rotations_completed':sum(e['event']=='scan_finished' for e in events),
 'five_meter_scan_distances':[e['distance_m'] for e in events if e['event']=='scan_begin' and e['distance_m']>=5],
 'visit_order':[e['id'] for e in events if e['event']=='marker_visited']}
(folder/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
image=cv2.imread(str(folder/'map.pgm'));n=image.shape[0]
def pixel(x,y):return(int((x+25)/.1),n-1-int((y+25)/.1))
for m in markers['markers'].values():
    p=pixel(m['x'],m['y']);cv2.circle(image,p,4,(0,0,255),-1)
    cv2.arrowedLine(image,p,pixel(m['x']+.7*m['normal_x'],m['y']+.7*m['normal_y']),(0,0,255),1)
    cv2.putText(image,str(m['id']),(p[0]+5,p[1]-5),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,255),1)
x0,y0=pixel(-11,7);x1,y1=pixel(11,-7)
image=image[y0:y1,x0:x1];cv2.imwrite(str(folder/'map_with_markers.png'),cv2.resize(image,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
