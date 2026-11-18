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
    ap.add_argument('--dictionary',default=None)
    ap.add_argument('--room-width',type=float);ap.add_argument('--room-height',type=float)
    ap.add_argument('--scan-topic',default='/scan');ap.add_argument('--odom-topic',default='/odom');ap.add_argument('--map-topic',default='/map')
    ap.add_argument('--image-topic',default='/oakd/rgb/preview/image_raw/compressed')
    ap.add_argument('--camera-info-topic',default='/oakd/rgb/preview/camera_info')
    ap.add_argument('--optical-frame',default='')
    ap.add_argument('--map-frame',default='map');ap.add_argument('--base-frame',default='base_link')
    ap.add_argument('--cmd-topic',default='/cmd_vel')
    ap.add_argument('--cmd-type',choices=['stamped','unstamped'],default='stamped' if os.environ.get('ROS_DISTRO','jazzy')=='jazzy' else 'unstamped')
    ap.add_argument('--timeout',type=float,default=30);ap.add_argument('--time-limit',type=float,default=600)
    ap.add_argument('--output-dir',type=Path,default=Path.home()/'team7_robot_runs')
    return ap

def main():
    ap=parser();args=ap.parse_args();cfg=json.loads(args.config.read_text())
    if args.marker_size is not None:cfg['default_marker_size']=args.marker_size
    if args.dictionary:cfg['dictionary']=args.dictionary
    for name in ['room_width','room_height']:
        if getattr(args,name) is not None:cfg[name]=getattr(args,name)
    if args.execute:
        if not cfg.get('default_marker_size') and not cfg.get('marker_sizes'):ap.error('Set actual --marker-size or per-ID marker_sizes before execution')
        if not all(cfg.get(k,0)>0 for k in ['room_width','room_height']):ap.error('V1 requires actual --room-width and --room-height, measured between inner walls')
    if any(not(0<float(s)<2) for s in [*cfg.get('marker_sizes',{}).values(),*([cfg['default_marker_size']] if cfg.get('default_marker_size') else [])]):ap.error('Invalid physical marker size')
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
            adapter.start(None);deadline=time.monotonic()+args.timeout
            while time.monotonic()<deadline:
                if adapter.ready():
                    print(json.dumps({'ready':True,'motion_enabled':False,'inputs':['scan','odom','camera','camera_info','map','TF'],'version':args.version}));return 0
                time.sleep(.1)
            print(json.dumps({'ready':False,'motion_enabled':False,'errors':adapter.errors,'received':list(adapter.gate.fresh)}));return 2
        mission=Mission(args,transport=adapter)
        signal.signal(signal.SIGINT,lambda *_:setattr(mission,'running',False))
        signal.signal(signal.SIGTERM,lambda *_:setattr(mission,'running',False))
        mission.run()
        return 0 if mission.status=='completed' else 2
    finally:
        adapter.close()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':raise SystemExit(main())
