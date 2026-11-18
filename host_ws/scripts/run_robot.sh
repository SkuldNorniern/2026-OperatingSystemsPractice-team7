#!/usr/bin/env bash
# Run on Ubuntu with the robot's ROS environment; no Gazebo is launched.
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd -- "$HERE/../.." && pwd)
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  exec "${TEAM7_PYTHON:-python3}" "$REPO/host_ws/robot_runtime/run_robot.py" --help
fi
if [[ "$(uname -s)" != Linux ]]; then
  echo 'Run this script on the TurtleBot/ROS 2 Ubuntu host. The Mac Gazebo launcher is sim/gazebo/run_native.sh.' >&2
  exit 2
fi
if ! command -v ros2 >/dev/null; then
  ros_setup="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
  [[ -f "$ros_setup" ]] || { echo "ROS 2 unavailable: $ros_setup" >&2; exit 2; }
  set +u
  source "$ros_setup"
  set -u
fi
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/team7_robot_${UID}.lock"
flock -n 9 || { echo 'Another Team7 real-robot run is active.' >&2; exit 2; }
with_slam=false
args=()
for arg in "$@"; do
  if [[ "$arg" == --with-slam ]]; then with_slam=true; else args+=("$arg"); fi
done
PYTHON_BIN="${TEAM7_PYTHON:-/usr/bin/python3}"
"$PYTHON_BIN" -c 'import rclpy, tf2_ros, numpy, cv2' || { echo 'Install dependencies described in host_ws/robot_runtime/README.md.' >&2; exit 2; }
[[ -f "$REPO/target/release/libtb4_aruco_gazebo.so" ]] || { echo 'Build for this Linux machine first: host_ws/scripts/build_robot.sh' >&2; exit 2; }
if [[ "$with_slam" == false ]]; then
  exec "$PYTHON_BIN" -u "$REPO/host_ws/robot_runtime/run_robot.py" "${args[@]}"
fi
# This mode owns a new SLAM process. Use only when another SLAM/localization stack is not running.
ros2 pkg prefix slam_toolbox >/dev/null
slam_pid=
mission_pid=
cleanup() {
  if [[ -n "$mission_pid" ]] && kill -0 "$mission_pid" 2>/dev/null; then kill -INT "$mission_pid"; wait "$mission_pid" || true; fi
  if [[ -n "$slam_pid" ]] && kill -0 "$slam_pid" 2>/dev/null; then kill -INT "$slam_pid"; wait "$slam_pid" || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false slam_params_file:="$REPO/host_ws/robot_runtime/slam.yaml" &
slam_pid=$!
"$PYTHON_BIN" -u "$REPO/host_ws/robot_runtime/run_robot.py" "${args[@]}" &
mission_pid=$!
wait "$mission_pid"
