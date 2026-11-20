"""Run the upstream rust_base demo with the V1 mission as its Nav2 client."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from run_robot import ROOT, load_config, parser as mission_parser


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


def main():
    ap = argparse.ArgumentParser(description=__doc__, add_help=False)
    ap.add_argument('--rust-base', type=Path, default=ROOT.with_name(ROOT.name + '-rust_base'))
    ap.add_argument('--no-prepare', action='store_true', help='Compatibility option. V1 always skips upstream robot preparation.')
    ap.add_argument('--no-record', action='store_true', help='Skip upstream rosbag recording')
    ap.add_argument('--no-rviz', action='store_true', help='Run without RViz')
    if '-h' in sys.argv[1:] or '--help' in sys.argv[1:]:
        ap.print_help()
        mission_parser().print_help()
        return 0
    stack_args, mission_args = ap.parse_known_args()
    robot_parser = mission_parser()
    args = robot_parser.parse_args(mission_args)
    cfg = load_config(robot_parser, args)
    checkout = stack_args.rust_base.expanduser().resolve()
    demo = checkout / 'host_ws/scripts/run_demo.sh'
    install = checkout / 'host_ws/ros2_ws/install/setup.bash'
    if not demo.is_file() or not install.is_file():
        ap.error(f'Expected a built rust_base checkout at {checkout}. See host_ws/robot_runtime/README.md or pass --rust-base.')
    if not (ROOT / 'target/release/libtb4_aruco_gazebo.so').is_file() and args.execute:
        ap.error('Build V1 on this host first with host_ws/scripts/build_robot.sh.')
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
    # Stop the mission (and cancel its goal) before stopping the safety stack.
    def interrupted(_sig, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        print(f'Run outputs: {folder}', flush=True)
        stack = subprocess.Popen(command, cwd=checkout, start_new_session=True)
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
        mission = subprocess.Popen(v1, cwd=ROOT, start_new_session=True)
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


if __name__ == '__main__':
    raise SystemExit(main())
