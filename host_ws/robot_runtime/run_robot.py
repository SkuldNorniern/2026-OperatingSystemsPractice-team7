"""Real ROS 2 TurtleBot adapter entry point. Default mode validates inputs without motion."""
import argparse,json,os,signal,sys,time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'sim/gazebo/native_sim'))

def parser():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--version',choices=['v1'],default='v1')
    mode=ap.add_mutually_exclusive_group();mode.add_argument('--execute',action='store_true');mode.add_argument('--check',action='store_true')
    ap.add_argument('--config',type=Path,default=Path(__file__).with_name('robot_config.json'))
    ap.add_argument('--marker-size',type=float,help='Actual outer black marker side length in metres (same size for all unspecified IDs)')
    ap.add_argument('--marker-ids', type=int, nargs=3, metavar='ID', help='Only accept these three physical ArUco IDs')
    ap.add_argument('--dictionary',default=None)
    ap.add_argument('--room-width',type=float);ap.add_argument('--room-height',type=float)
    ap.add_argument('--scan-topic',default='/scan');ap.add_argument('--odom-topic',default='/odom');ap.add_argument('--map-topic',default='/map')
    ap.add_argument('--image-topic',default='/oakd/rgb/preview/image_raw/compressed')
    ap.add_argument('--camera-info-topic',default='/oakd/rgb/preview/camera_info')
    ap.add_argument('--depth-topic', default=os.environ.get('TEAM7_DEPTH_TOPIC', 'auto'),
                    help='RGB-aligned depth Image topic, auto (default), or off')
    ap.add_argument('--depth-camera-info-topic', default=os.environ.get('TEAM7_DEPTH_CAMERA_INFO_TOPIC'),
                    help='Override the camera_info topic paired with the depth image')
    ap.add_argument('--optical-frame',default='')
    ap.add_argument('--map-frame',default='map');ap.add_argument('--base-frame',default='base_link')
    ap.add_argument('--nav2',action='store_true',help='Use the existing Nav2 actions for navigation and rotation')
    ap.add_argument('--marker-slowdown', action='store_true', help=argparse.SUPPRESS)
    ap.add_argument('--interior-explorer', type=Path, help=argparse.SUPPRESS)
    ap.add_argument('--interior-params', type=Path, help=argparse.SUPPRESS)
    ap.add_argument('--cmd-topic',default='/cmd_vel')
    ap.add_argument('--cmd-type',choices=['stamped','unstamped'],default='stamped' if os.environ.get('ROS_DISTRO','jazzy')=='jazzy' else 'unstamped')
    ap.add_argument('--timeout',type=float,default=30);ap.add_argument('--time-limit',type=float,default=600)
    ap.add_argument('--output-dir',type=Path,default=Path.home()/'team7_robot_runs')
    return ap

def load_config(ap, args, *, require_room_size=True):
    cfg=json.loads(args.config.read_text())
    if args.marker_size is not None:cfg['default_marker_size']=args.marker_size
    if args.marker_ids is not None:
        cfg['marker_ids'] = args.marker_ids
    if args.dictionary:cfg['dictionary']=args.dictionary
    ids = cfg.get('marker_ids') or [int(key) for key in cfg.get('marker_sizes', {})]
    if ids and (len(ids) != 3 or len(set(ids)) != 3 or any(not 0 <= i <= 32767 for i in ids)):
        ap.error('Provide three distinct nonnegative marker IDs')
    if args.execute and ids and not cfg.get('default_marker_size') and any(str(i) not in cfg['marker_sizes'] for i in ids):
        ap.error('Each target ID needs a marker size, or set a common --marker-size')
    for name in ['room_width','room_height']:
        if getattr(args,name) is not None:cfg[name]=getattr(args,name)
    if args.execute:
        if not cfg.get('default_marker_size') and not cfg.get('marker_sizes'):ap.error('Set actual --marker-size or per-ID marker_sizes before execution')
        if require_room_size and not all(cfg.get(k,0)>0 for k in ['room_width','room_height']):ap.error('V1 requires actual --room-width and --room-height, measured between inner walls')
    if any(not(0<float(s)<2) for s in [*cfg.get('marker_sizes',{}).values(),*([cfg['default_marker_size']] if cfg.get('default_marker_size') else [])]):ap.error('Invalid physical marker size')
    return cfg

def wait_for_inputs(adapter, args, mission=None):
    deadline = time.monotonic() + args.timeout
    while (mission is None or mission.running) and time.monotonic() < deadline:
        if adapter.ready():
            inputs = ['scan', 'odom', 'camera', 'camera_info', 'map', 'TF']
            if args.nav2:
                inputs += ['Nav2', 'velocity_guard']
            print(json.dumps({'ready': True, 'motion_enabled': mission is not None,
                              'inputs': inputs, 'version': args.version}), flush=True)
            return True
        time.sleep(.1)
    print(json.dumps({'ready': False, 'motion_enabled': False,
                      'errors': adapter.errors.copy(), 'received': list(adapter.gate.fresh)}), flush=True)
    return False

def main():
    ap=parser();args=ap.parse_args();cfg=load_config(ap,args,require_room_size=not args.interior_explorer)
    try:
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from ros_transport import ROSAdapter
    except ImportError as e:ap.exit(2,f'ROS 2 Python unavailable: {e}\nRun on a ROS-enabled Ubuntu host or TurtleBot, not the Gazebo-only Mac.\n')
    from mission import Mission
    args.config=cfg;args.external_map=True;args.max_seconds=args.time_limit;args.max_sim_seconds=0
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    adapter=ROSAdapter(args)
    try:
        if not args.execute:
            adapter.start(None)
            return 0 if wait_for_inputs(adapter, args) else 2
        mission=Mission(args,transport=adapter)
        signal.signal(signal.SIGINT,lambda *_:setattr(mission,'running',False))
        signal.signal(signal.SIGTERM,lambda *_:setattr(mission,'running',False))
        if wait_for_inputs(adapter, args, mission):
            mission.start_wall = time.monotonic()
        elif mission.running:
            mission.status = 'startup_timeout'
            mission.running = False
            mission.event('startup_timeout')
        mission.run()
        print(f'V1 mission status: {mission.status}. Results: {mission.out}', flush=True)
        if mission.status == 'interrupted':
            return 130
        return 0 if mission.status=='completed' else 2
    finally:
        adapter.close()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':raise SystemExit(main())
