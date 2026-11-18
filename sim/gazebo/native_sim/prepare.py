"""Create a runnable copy of track1 without overwriting the user's saved layout."""
import json, math
from pathlib import Path
import xml.etree.ElementTree as E
import cv2
ROOT=Path(__file__).resolve().parent.parent

def put(e,path,value):
    child=e.find(path)
    if child is None:
        child=e
        for part in path.split('/'):
            found=child.find(part)
            child=found if found is not None else E.SubElement(child,part)
    child.text=str(value)

def geom(parent,kind,values):
    g=E.SubElement(parent,'geometry'); k=E.SubElement(g,kind)
    for key,val in values.items(): put(k,key,val)

def visual(link,name,pose,size,color):
    v=E.SubElement(link,'visual',name=name);put(v,'pose',pose)
    geom(v,'box',{'size':size});put(v,'material/ambient',color);put(v,'material/diffuse',color)

root=E.parse(ROOT/'track1.sdf').getroot();w=root.find('world'); w.set('name','team7_track1')
put(w,'physics/real_time_factor',4)
r=w.find("model[@name='turtlebot4']")
# Correct the original wheel spacing and add frictionless caster supports.
p=r.find("plugin[@name='gz::sim::systems::DiffDrive']")
put(p,'wheel_separation',.34);put(p,'odom_publish_frequency',30)
put(p,'odom_topic','/model/turtlebot4/odometry')
for j in r.findall('joint'):
    put(j,'axis/xyz','0 1 0');j.find('axis/xyz').set('expressed_in','__model__') # local wheel z after roll +pi/2 = world +y
od=E.SubElement(r,'plugin',name='gz::sim::systems::OdometryPublisher',filename='gz-sim-odometry-publisher-system')
put(od,'odom_topic','/model/turtlebot4/ideal_odometry');put(od,'odom_publish_frequency',30)
base=r.find("link[@name='base_link']")
for name,x in [('front',.12),('back',-.12)]:
    c=E.SubElement(base,'collision',name=name+'_caster');put(c,'pose',f'{x} 0 -0.071 0 0 0')
    geom(c,'sphere',{'radius':.02});put(c,'surface/friction/ode/mu',0);put(c,'surface/friction/ode/mu2',0)
# Floor upper surface z=.05. Wheels bottom=.029 at original spawn; lift to .141.
pose=list(map(float,r.findtext('pose').split()));pose[2]=.142;put(r,'pose',' '.join(map(str,pose)))
for sensor in base.findall('sensor'):
    put(sensor,'always_on','true')
    if sensor.get('type')=='camera':
        put(sensor,'camera/image/format','R8G8B8')
        put(sensor,'camera/image/width',1280);put(sensor,'camera/image/height',960)
        put(sensor,'camera/lens/type','gnomonical')
        put(sensor,'camera/camera_info_topic','/turtlebot4/camera_info')
        put(sensor,'update_rate',15)
# Keep saved marker centers. Use upright static panels: they stay put but can be moved with GUI transform.
# Each marker retains its saved facing (project normal onto XY), black outer square size is known to PnP.
dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
markers={}
for m in w.findall('model'):
    if not m.get('name','').startswith('aruco_marker_'):continue
    mid=int(m.get('name').rsplit('_',1)[1]);link=m.find('link')
    old=list(map(float,m.findtext('pose').split()));x,y=old[:2]
    roll,pitch,yaw=old[3:]
    # Normal of original panel (+Z) under Rz Ry Rx.
    nx=math.cos(yaw)*math.sin(pitch)*math.cos(roll)+math.sin(yaw)*math.sin(roll)
    ny=math.sin(yaw)*math.sin(pitch)*math.cos(roll)-math.cos(yaw)*math.sin(roll)
    if math.hypot(nx,ny)<.5: nx,ny=-x,-y
    # Choose inward normal so camera travelling around room can see each face.
    if nx*(-x)+ny*(-y)<0:nx,ny=-nx,-ny
    facing=math.atan2(ny,nx)+math.pi/2
    put(m,'pose',f'{x} {y} 0.38 {math.pi/2} 0 {facing}')
    put(m,'static','true')
    oldsize=float(link.findtext('collision/geometry/box/size').split()[0]);size=oldsize
    markers[str(mid)]=size
    for v in link.findall('visual'):link.remove(v)
    put(link,'collision/geometry/box/size',f'{size*1.3} {size*1.3} .02')
    visual(link,'white_margin','0 0 0 0 0 0',f'{size*1.3} {size*1.3} .02','1 1 1 1')
    bits=cv2.aruco.generateImageMarker(dictionary,mid,7)
    for row in range(7):
      for col in range(7):
        if bits[row,col]==0:
          cell=size/7
          visual(link,f'cell_{row}_{col}',f'{(col-3)*cell} {(3-row)*cell} .011 0 0 0',f'{cell*1.002} {cell*1.002} .002','0 0 0 1')
# Close doorway for the requested rectangular-room patrol, only in runnable copy.
m=E.SubElement(w,'model',name='patrol_door_fill');put(m,'static','true');put(m,'pose','0 -5.6 1.3 0 0 0')
l=E.SubElement(m,'link',name='link');c=E.SubElement(l,'collision',name='collision');geom(c,'box',{'size':'1.36 .15 2.5'})
visual(l,'wall','0 0 0 0 0 0','1.36 .15 2.5','0.80 0.82 0.84 1')
# Algorithm validation uses a kinematic base, avoiding wheel-slip/axis errors of the hand-made mesh.
for child in list(r):
    if child.tag=='joint' or (child.tag=='link' and child.get('name')!='base_link') or (child.tag=='plugin' and child.get('name')=='gz::sim::systems::DiffDrive'):
        r.remove(child)
vc=E.SubElement(r,'plugin',name='gz::sim::systems::VelocityControl',filename='gz-sim-velocity-control-system')
put(vc,'topic','/model/turtlebot4/cmd_vel')
for col in base.findall('collision'):
    put(col,'surface/friction/ode/mu',0);put(col,'surface/friction/ode/mu2',0)
E.indent(root);E.ElementTree(root).write(ROOT/'native_sim/track1_native.sdf',encoding='unicode')
config={'dictionary':'DICT_5X5_50','marker_sizes':markers,'room_width':19.85,'room_height':11.05,'wall_standoff':.8,'scan_every_m':5.,'marker_standoff':.8}
(ROOT/'native_sim/config.json').write_text(json.dumps(config,indent=2))
print('Prepared track1_native.sdf; saved layout preserved in track1.sdf')
