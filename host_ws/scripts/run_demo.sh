#!/usr/bin/env bash
# usage: run_demo.sh [launch args]. put robot on start pose first
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

if timeout 5 ros2 topic echo --once /dock_status 2>/dev/null | grep -q "is_docked: true"; then
  echo "[*] robot is docked: undocking"
  timeout 60 ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" >/dev/null \
    || echo "warning: undock failed" >&2
  sleep 3
fi

echo "[*] resetting odometry (map origin = current pose)"
if ! timeout 10 ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose "{}" >/dev/null; then
  echo "warning: /reset_pose failed; map origin will be wherever odom currently is" >&2
fi
sleep 1

exec ros2 launch team7_bringup team7.launch.py "$@"
