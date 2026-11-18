"""Launch isolated native Gazebo server, optional GUI, and mission; Ctrl-C stops all."""
import argparse,fcntl,hashlib,os,shutil,signal,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
ap=argparse.ArgumentParser();ap.add_argument('--version',choices=['v1'],default='v1');ap.add_argument('--max-sim-seconds',type=float,default=600);ap.add_argument('--headless',action='store_true');ap.add_argument('--max-seconds',type=float,default=0);args=ap.parse_args()
lock=(ROOT/'.run.lock').open('w')
try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:sys.exit('A native mission is already running.')
env=os.environ.copy();env.setdefault('GZ_IP','127.0.0.1')
partition='team7_native_'+hashlib.sha256(str(ROOT).encode()).hexdigest()[:8]
env.setdefault('GZ_PARTITION',partition)
gz=shutil.which('gz')
if not gz:sys.exit('Gazebo not found. Install Homebrew Gazebo Harmonic first.')
brew=shutil.which('brew')
prefix=Path(subprocess.check_output([brew,'--prefix'],text=True).strip()) if brew else None
# Homebrew may retain the previous simdjson ABI needed by Arrow / Gazebo.
libs=sorted({str(p.parent) for p in (prefix/'Cellar/simdjson').glob('*/lib/libsimdjson.36.dylib')}) if prefix else []
if libs:env['DYLD_FALLBACK_LIBRARY_PATH']=':'.join(libs+[env.get('DYLD_FALLBACK_LIBRARY_PATH','')])
ruby=prefix/'opt/ruby/bin/ruby' if prefix else None
command=[str(ruby),gz,'sim'] if ruby and ruby.exists() else [gz,'sim']
processes=[]
log=(ROOT/'server.log').open('w')
def start(cmd,output=None):
    p=subprocess.Popen(cmd,cwd=ROOT.parent,env=env,stdout=output,stderr=subprocess.STDOUT if output else None,start_new_session=True);processes.append(p);return p
try:
    subprocess.run([sys.executable,str(ROOT/'prepare.py')],check=True)
    server=start(command+['-s','-r',str(ROOT/'track1_native.sdf')],log)
    time.sleep(3)
    if server.poll() is not None:raise RuntimeError('Gazebo failed: see native_sim/server.log')
    if not args.headless:start(command+['-g'],log)
    mission=start([sys.executable,'-u',str(ROOT/'mission.py'),'--version',args.version,'--max-sim-seconds',str(args.max_sim_seconds),'--max-seconds',str(args.max_seconds)])
    print(f'ROS-free track1 {args.version} mission running (GZ_PARTITION={env["GZ_PARTITION"]}). Ctrl-C stops all. Logs: native_sim/runs/',flush=True)
    while mission.poll() is None:
        if server.poll() is not None:raise RuntimeError('Gazebo exited. See server.log')
        time.sleep(.3)
    if mission.returncode:raise RuntimeError(f'Mission exited {mission.returncode}')
except KeyboardInterrupt:pass
finally:
    for p in reversed(processes):
        if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
    for p in reversed(processes):
        try:p.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM)
    log.close()
