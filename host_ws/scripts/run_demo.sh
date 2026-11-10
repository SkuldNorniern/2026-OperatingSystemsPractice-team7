#!/usr/bin/env bash
# usage: run_demo.sh [--no-prepare] [launch args]. put robot on start pose first
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HERE/../ros2_ws"
PREPARE_ROBOT=true
LAUNCH_ARGS=()
for argument in "$@"; do
  case "$argument" in
    --no-prepare) PREPARE_ROBOT=false ;;
    --help|-h)
      echo 'usage: run_demo.sh [--no-prepare] [name:=value ...]'
      echo '--no-prepare skips undock/reset; use explorer:=none mission:=false for stationary diagnostics.'
      exit 0 ;;
    *) LAUNCH_ARGS+=("$argument") ;;
  esac
done
set -- "${LAUNCH_ARGS[@]}"

# Two launches would publish competing TF and velocity commands to one robot.
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/team7_demo_${UID}.lock"
flock -n 9 || { echo 'error: a Team7 run is already active; stop it before starting another' >&2; exit 1; }

set +u
source /opt/ros/jazzy/setup.bash
[ -f "$WS/install/setup.bash" ] || { echo "build first: cd $WS && colcon build" >&2; exit 1; }
source "$WS/install/setup.bash"
set -u

# A partial colcon install can have setup.bash without the runnable packages.
# Check these before undocking or resetting odometry.
for package in team7_nodes team7_bringup turtlebot4_navigation aruco_opencv rosbag2_transport rosbag2_storage_mcap; do
  ros2 pkg prefix "$package" >/dev/null || {
    echo "error: $package missing; install dependencies and rebuild $WS" >&2
    exit 1
  }
done
ros2 interface show irobot_create_msgs/srv/ResetPose >/dev/null
ros2 interface show irobot_create_msgs/action/Undock >/dev/null

# Prefer the configured robot cable without taking down Wi-Fi internet.
if ip -4 -o addr show scope global | grep -q '192\.168\.185\.10/'; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/src/team7_bringup/config/fastdds_wired.xml"
  echo '[*] ROS transport: wired 192.168.185.10 (Wi-Fi excluded from DDS)'
fi

RUN_DIR="$HOME/team7_runs/$(date +%Y%m%d_%H%M%S_%N)"
RECORD=true
SIM_TIME=false
for argument in "$@"; do
  case "$argument" in
    output_dir:=*) RUN_DIR=${argument#output_dir:=} ;;
    record:=*) RECORD=${argument#record:=} ;;
    use_sim_time:=*) SIM_TIME=${argument#use_sim_time:=} ;;
  esac
done
RUN_DIR=${RUN_DIR/#\~\//$HOME/}
mkdir -p "$RUN_DIR/config"
RUN_DIR=$(cd "$RUN_DIR" && pwd)
cp "$WS/src/team7_bringup/config/"* "$RUN_DIR/config/"
export ROS_LOG_DIR="$RUN_DIR/ros_logs"
{
  date -u
  uname -a
  git -C "$HERE/../.." rev-parse HEAD
  git -C "$HERE/../.." status --short
  printf 'launch argument: %s\n' "$@"
  printf 'ROS_DOMAIN_ID=%s\nRMW_IMPLEMENTATION=%s\nROS_DISCOVERY_SERVER=%s\nFASTRTPS_DEFAULT_PROFILES_FILE=%s\n' \
    "${ROS_DOMAIN_ID:-0}" "${RMW_IMPLEMENTATION:-}" "${ROS_DISCOVERY_SERVER:-}" "${FASTRTPS_DEFAULT_PROFILES_FILE:-}"
  ip -4 -br address
} > "$RUN_DIR/run_info.txt"
git -C "$HERE/../.." diff -- host_ws crates > "$RUN_DIR/source_changes.patch"
echo "[*] run artifacts: $RUN_DIR"
RECORDER_PID=
cleanup() {
  if [ -n "$RECORDER_PID" ] && kill -0 "$RECORDER_PID" 2>/dev/null; then
    # Background jobs inherit ignored SIGINT. ros2 bag's SIGTERM handler
    # cancels recording cleanly and writes metadata before exiting.
    kill -TERM "$RECORDER_PID" 2>/dev/null || true
    wait "$RECORDER_PID" || true
    /usr/bin/python3 "$HERE/summarize_run.py" "$RUN_DIR" > "$RUN_DIR/summary.log" 2>&1 || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if [ "$RECORD" = true ]; then
  bash "$HERE/record_run.sh" "$RUN_DIR" "$SIM_TIME" > "$RUN_DIR/recorder.log" 2>&1 &
  RECORDER_PID=$!
  sleep 2
  kill -0 "$RECORDER_PID" 2>/dev/null || {
    cat "$RUN_DIR/recorder.log" >&2
    echo 'error: recording failed; robot preparation cancelled' >&2
    exit 1
  }
fi

echo "[*] checking robot topics"
TOPICS=$(timeout 15 ros2 topic list --no-daemon --spin-time 3 || true)
if ! grep -qx "/scan" <<<"$TOPICS"; then
  echo "error: /scan not visible. Is the robot on, and are ROS_DOMAIN_ID / RMW_IMPLEMENTATION the same as on the Pi?" >&2
  exit 1
fi
# run still works without these, but no markers / no odom means wasted 10 min, so say it now
for topic in /odom /oakd/rgb/preview/image_raw/compressed /oakd/rgb/preview/camera_info; do
  grep -qx "$topic" <<<"$TOPICS" || echo "warning: $topic not visible; check the robot camera / create3 bridge" >&2
done

if [ "$PREPARE_ROBOT" = true ]; then
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
fi

# The recorder above is already running, including during undock/reset.
ros2 launch team7_bringup team7.launch.py "$@" "output_dir:=$RUN_DIR" record:=false \
  2>&1 | tee "$RUN_DIR/console.log"
