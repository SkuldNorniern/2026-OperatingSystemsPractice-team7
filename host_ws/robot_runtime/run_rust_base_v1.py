"""Run the upstream rust_base demo with the V1 mission as its Nav2 client."""
import argparse
import fcntl
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time

from run_robot import ROOT, load_config, parser as mission_parser

DEMO = Path('host_ws/scripts/run_demo.sh')
WORKSPACE = Path('host_ws/ros2_ws')
RUST_BASE_REF = 'refs/remotes/origin/rust_base'


def has_rust_base_source(checkout):
    launch = checkout / WORKSPACE / 'src/team7_bringup/launch/team7.launch.py'
    return (checkout / DEMO).is_file() and launch.is_file()


def installed_node(checkout, name):
    install = checkout / WORKSPACE / 'install'
    for prefix in (install / 'team7_nodes', install):
        executable = prefix / 'lib/team7_nodes' / name
        if executable.is_file():
            return executable
    return None


def has_rust_base_build(checkout):
    return ((checkout / WORKSPACE / 'install/setup.bash').is_file()
            and all(installed_node(checkout, node) is not None for node in ('velocity_guard', 'explorer')))


def prepare_rust_base(root, override):
    if override is not None:
        checkout = override.expanduser().resolve()
    else:
        default = root.with_name(root.name + '-rust_base')
        worktrees = subprocess.check_output(
            ['git', 'worktree', 'list', '--porcelain', '-z'], cwd=root, text=True,
        )
        candidates = [default, root.parent / 'rust_base']
        candidates.extend(Path(field.removeprefix('worktree '))
                          for field in worktrees.split('\0') if field.startswith('worktree '))
        for prefix in filter(None, os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)):
            path = Path(prefix)
            install = path if path.name == 'install' else path.parent
            if install.parts[-3:] == ('host_ws', 'ros2_ws', 'install'):
                candidates.append(install.parents[2])
        candidates.extend(path for path in root.parent.iterdir()
                          if root.name in path.name or 'rust_base' in path.name)
        sources = [path for path in dict.fromkeys(candidates) if has_rust_base_source(path)]
        fallback = sources[0] if sources else default
        checkout = next((path for path in sources if has_rust_base_build(path)), fallback)

    has_source = has_rust_base_source(checkout)
    if not has_source and (override is not None or checkout.exists() or checkout.is_symlink()):
        raise ValueError(f'{checkout} does not contain the rust_base demo. Set TEAM7_RUST_BASE to its checkout path.')
    if has_source and has_rust_base_build(checkout):
        return checkout
    if shutil.which('colcon') is None:
        raise ValueError('colcon is needed to build rust_base. See the Ubuntu setup in host_ws/robot_runtime/README.md.')

    if not has_source:
        local_ref = subprocess.run(['git', 'show-ref', '--verify', '--quiet', RUST_BASE_REF], cwd=root)
        if local_ref.returncode != 0:
            subprocess.run(['git', 'fetch', 'origin', f'refs/heads/rust_base:{RUST_BASE_REF}'], cwd=root, check=True)
        subprocess.run(['git', 'worktree', 'add', '--detach', str(checkout), RUST_BASE_REF], cwd=root, check=True)

    print(f'Building the rust_base ROS workspace at {checkout / WORKSPACE}', flush=True)
    subprocess.run(['colcon', 'build', '--symlink-install', '--packages-select', 'team7_nodes', 'team7_bringup'],
                   cwd=checkout / WORKSPACE, check=True)
    if not has_rust_base_build(checkout):
        raise ValueError(f'Build at {checkout / WORKSPACE} did not produce the ROS setup, velocity_guard and explorer.')
    return checkout


def stop(process):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue


def start_logged(command, cwd, log_path):
    log = log_path.open('w', encoding='utf-8')
    try:
        process = subprocess.Popen(command, cwd=cwd, start_new_session=True,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, errors='replace')
    except OSError:
        log.close()
        raise

    def relay():
        with log, process.stdout:
            terminal = True
            for line in process.stdout:
                log.write(line)
                log.flush()
                if terminal:
                    try:
                        print(line, end='', flush=True)
                    except OSError:
                        terminal = False

    thread = threading.Thread(target=relay, daemon=True)
    thread.start()
    return process, thread


def main():
    ap = argparse.ArgumentParser(description=__doc__, add_help=False)
    ap.add_argument('--rust-base', type=Path, help='Use this rust_base checkout (overrides TEAM7_RUST_BASE)')
    ap.add_argument('--no-prepare', action='store_true', help='Compatibility option. V1 always skips upstream robot preparation.')
    ap.add_argument('--no-record', action='store_true', help='Skip upstream rosbag recording')
    ap.add_argument('--no-rviz', action='store_true', help='Run without RViz')
    if '-h' in sys.argv[1:] or '--help' in sys.argv[1:]:
        ap.print_help()
        mission_parser().print_help()
        return 0
    stack_args, mission_args = ap.parse_known_args()
    # Nav2's first transform is allowed up to 60 seconds during bringup.
    mission_args = ['--timeout', '75', *mission_args]
    robot_parser = mission_parser()
    args = robot_parser.parse_args(mission_args)
    cfg = load_config(robot_parser, args, require_room_size=False)
    checkout_env = os.environ.get('TEAM7_RUST_BASE')
    override = stack_args.rust_base or (Path(checkout_env) if checkout_env else None)
    lock_dir = Path(os.environ.get('XDG_RUNTIME_DIR') or tempfile.gettempdir())
    try:
        with (lock_dir / f'team7_v1_build_{os.getuid()}.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            checkout = prepare_rust_base(ROOT, override)
            if args.execute and not (ROOT / 'target/release/libtb4_aruco_gazebo.so').is_file():
                subprocess.run(['bash', str(ROOT / 'host_ws/scripts/build_robot.sh')], cwd=ROOT, check=True)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        ap.exit(2, f'Could not prepare the robot workspace: {exc}\nSee host_ws/robot_runtime/README.md.\n')
    demo = checkout / DEMO
    install = checkout / WORKSPACE / 'install/setup.bash'
    # Validate dependencies before starting the upstream demo.
    import action_msgs.msg, cv2, nav2_msgs.action, nav2_msgs.msg, numpy, rclpy, tf2_ros, visualization_msgs.msg
    if args.execute:
        from rust_detector import RustDetector
        RustDetector(cfg['dictionary']).close()

    folder = args.output_dir.expanduser().resolve() / time.strftime('%Y%m%d-%H%M%S-rust-base-v1')
    folder.mkdir(parents=True)
    # Dock actions can move the base independently of the forward-only velocity guard.
    command = ['bash', str(demo), '--no-prepare']
    stack_output = folder / 'rust_base'
    command += ['explorer:=none', 'mission:=false', 'nav:=nav2',
                f'output_dir:={stack_output}',
                f'record:={str(args.execute and not stack_args.no_record).lower()}',
                f'rviz:={str(args.execute and not stack_args.no_rviz).lower()}']
    command.append(f'camera_topic:={args.image_topic.removesuffix("/compressed")}')
    if args.marker_size is not None:
        command.append(f'marker_size:={args.marker_size}')
    v1 = [sys.executable, '-u', str(Path(__file__).with_name('run_robot.py')),
          *mission_args, '--nav2', '--marker-slowdown', '--output-dir', str(folder / 'mission')]
    # The original script configures wired DDS itself. Both processes must use it.
    addresses = subprocess.run(['ip', '-4', '-o', 'addr', 'show', 'scope', 'global'],
                               capture_output=True, text=True, check=True).stdout
    if any('inet 192.168.185.10/' in line for line in addresses.splitlines()):
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(
            checkout / 'host_ws/ros2_ws/src/team7_bringup/config/fastdds_wired.xml')

    from bringup_config import prepare_bringup
    overlay = folder / 'bringup_overlay'
    share = prepare_bringup(
        checkout / 'host_ws/ros2_ws/src/team7_bringup', overlay,
        map_frame=args.map_frame, map_topic=args.map_topic,
    )
    if args.execute:
        v1 += ['--interior-explorer', str(installed_node(checkout, 'explorer')),
               '--interior-params', str(share / 'config/team7.yaml')]
        v1 = ['bash', '-ec', 'source "$1"; shift; exec "$@"', 'team7-mission', str(install), *v1]
    # Source the original install before prepending the generated package share.
    # Check precedence after a second source, as run_demo.sh sources it again.
    bootstrap = """
source "$1"
export AMENT_PREFIX_PATH="$2${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
source "$1"
if [[ $(ros2 pkg prefix --share team7_bringup) != "$3" ]]; then
    echo 'Could not load the generated team7_bringup configuration.' >&2
    exit 2
fi
shift 3
exec "$@"
"""
    command = ['bash', '-ec', bootstrap, 'team7-bringup', str(install), str(overlay), str(share), *command]
    stack = mission = None
    log_threads = []
    # Stop the mission (and cancel its goal) before stopping the safety stack.
    def interrupted(_sig, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        print(f'Run outputs: {folder}', flush=True)
        stack, stack_log = start_logged(command, checkout, folder / 'stack.log')
        log_threads.append(stack_log)
        # run_demo.sh opens console.log in the launch pipeline after preparation.
        deadline = time.monotonic() + 120
        while not (stack_output / 'console.log').is_file():
            if stack.poll() is not None:
                print(f'rust_base demo failed before launch (status {stack.returncode})', file=sys.stderr)
                return 1
            if time.monotonic() >= deadline:
                print('Timed out waiting for rust_base to start its ROS launch', file=sys.stderr)
                return 1
            time.sleep(.2)
        mission, mission_log = start_logged(v1, ROOT, folder / 'mission.log')
        log_threads.append(mission_log)
        while mission.poll() is None:
            if stack.poll() is not None:
                print(f'rust_base demo stopped with status {stack.returncode}', file=sys.stderr)
                return 1
            time.sleep(.2)
        return mission.returncode
    except KeyboardInterrupt:
        return 130
    finally:
        stop(mission)
        stop(stack)
        for thread in log_threads:
            thread.join(timeout=2)
        if mission is not None:
            print(f'V1 exited with status {mission.returncode}. Mission log: {folder / "mission.log"}', flush=True)
        print(f'Stack log: {folder / "stack.log"}', flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
