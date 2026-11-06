#!/usr/bin/env bash
# Start a demo run on the laptop.
#
#   1. reset Create 3 odometry so the map origin is the start pose
#   2. launch SLAM + Nav2 + ArUco + explorer + mission + RViz
#
# Usage: host_ws/scripts/run_demo.sh [launch args...]
#   e.g. host_ws/scripts/run_demo.sh explorer:=explore_lite output_dir:=~/runs/try3
#
# Place the robot at the start pose BEFORE running this.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HERE/../ros2_ws"

set +u
source /opt/ros/jazzy/setup.bash
[ -f "$WS/install/setup.bash" ] || { echo "build first: cd $WS && colcon build" >&2; exit 1; }
source "$WS/install/setup.bash"
set -u

echo "[*] checking robot topics"
if ! timeout 10 ros2 topic list | grep -qx "/scan"; then
  echo "error: /scan not visible. Is the robot on, and are ROS_DOMAIN_ID / RMW_IMPLEMENTATION the same as on the Pi?" >&2
  exit 1
fi

echo "[*] resetting odometry (map origin = current pose)"
if ! timeout 10 ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose "{}" >/dev/null; then
  echo "warning: /reset_pose failed; map origin will be wherever odom currently is" >&2
fi
sleep 1

exec ros2 launch team7_bringup team7.launch.py "$@"
