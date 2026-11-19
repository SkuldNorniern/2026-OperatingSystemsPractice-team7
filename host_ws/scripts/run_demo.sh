#!/usr/bin/env bash
# usage: run_demo.sh [--stock] [--replay BAG_DIR [--start-offset SEC]] [--no-prepare] [--no-record]
#                        [name:=value ...]
#   The core run script. launch_team7.sh and launch_stock.sh call this.
#   default: live robot, team7 navigation (Lazy Theta* + MPPI) with team7 SLAM.
#            Before launching: records the run, checks the robot topics, undocks if docked and
#            resets odometry (map origin = current pose). The robot must be on its start pose.
#   --stock: the committed HEAD of this repo (rust_base), built into a cache, with the stock
#            launch defaults (Nav2 navigation, slam_toolbox). set TEAM7_REV to pin another commit.
#            Same live preparation as the default.
#   --replay BAG_DIR: play a recorded run instead of the robot (bag clock, TF relayed). No preparation,
#            never records a bag; map/targets go to the replay log folder, not ~/team7_run.
#   --start-offset SEC: with --replay, start the bag SEC seconds in. Default: just after the odometry
#            reset that the live run recorded during its preparation (0 = from the very start).
#   --graph: also open rqt_graph on the run's ROS domain (save the node graph from its window)
#   --viz: open team7_viz (read-only map viewer) instead of RViz, on the run's ROS domain, stopped with
#            the run. Built from the main workspace (colcon build --packages-select team7_viz), also for
#            --stock. Add rviz:=true to get both.
#   --no-prepare: live only; skip undock and odometry reset (stationary diagnostics).
#   --no-record / record:=: live only. A replay never records (a bag of a bag only fills the disk).
# extra name:=value args go to team7.launch.py, e.g. nav:=lazy_nav2, explorer:=none, output_dir:=DIR
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
WS="$HERE/../ros2_ws"
BAG=""
OFFSET=""
STOCK=false
PREPARE=true
GRAPH=false
VIZ=false
RECORD=true
RECORD_SET=false
RUN_DIR=""
EXTRA=()
LAUNCH_ARGS=(use_sim_time:=false nav:=team7 slam_backend:=team7 aruco:=team7)

while [ $# -gt 0 ]; do
  case "$1" in
    --stock) STOCK=true ;;
    --replay) BAG="${2:?--replay needs a bag directory}"; shift ;;
    --start-offset) OFFSET="${2:?--start-offset needs seconds}"; shift ;;
    --no-prepare) PREPARE=false ;;
    --graph) GRAPH=true ;;
    --viz) VIZ=true ;;
    --no-record) RECORD=false; RECORD_SET=true ;;
    --help|-h) sed -n "2,/^set -euo/{/^#/p}" "$0"; exit 0 ;;
    output_dir:=*) RUN_DIR="${1#output_dir:=}" ;;
    record:=*) RECORD="${1#record:=}"; RECORD_SET=true ;;
    *) EXTRA+=("$1") ;;
  esac
  shift
done

if [ "$STOCK" = true ]; then
  # build the pinned commit from git into a cache, so the working tree stays untouched
  REV="${TEAM7_REV:-HEAD}"
  SHA="$(git -C "$REPO" rev-parse --verify "$REV^{commit}")"
  CACHE="${XDG_CACHE_HOME:-$HOME/.cache}/team7_stock"
  if [ "$(cat "$CACHE/.sha" 2>/dev/null || true)" != "$SHA" ]; then
    echo "stock: building $SHA into $CACHE (first run takes a few minutes)"
    rm -rf "$CACHE" && mkdir -p "$CACHE"
    git -C "$REPO" archive "$SHA" | tar -x -C "$CACHE"
    (cd "$CACHE/host_ws/ros2_ws" && set +u && source /opt/ros/jazzy/setup.bash && set -u \
      && colcon build --packages-select team7_nodes team7_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release \
      >"$CACHE/build.log" 2>&1) \
      || { echo "stock build failed, see $CACHE/build.log" >&2; exit 1; }
    echo "$SHA" > "$CACHE/.sha"
  fi
  WS="$CACHE/host_ws/ros2_ws"
  # stock launch defaults: nav:=nav2 and slam_toolbox, not our team7 backends
  LAUNCH_ARGS=(use_sim_time:=false)
fi
LAUNCH_ARGS+=(${EXTRA[@]+"${EXTRA[@]}"})
# --viz replaces RViz unless rviz:= was given
if [ "$VIZ" = true ] && ! printf '%s\n' ${EXTRA[@]+"${EXTRA[@]}"} | grep -q '^rviz:='; then
  LAUNCH_ARGS+=(rviz:=false)
fi

# one run at a time: replay, live and stock share this lock
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/team7_demo_${UID}.lock"
flock -n 9 || { echo 'error: a Team7 run is already active; stop it before starting another' >&2; exit 1; }

set +u
source /opt/ros/jazzy/setup.bash
[ -f "$WS/install/setup.bash" ] || { echo "build first: cd $WS && colcon build" >&2; exit 1; }
source "$WS/install/setup.bash"
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

PIDS=()
RECORDER_PID=""
cleanup() {
  for p in "${PIDS[@]:-}"; do kill -INT "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${PIDS[@]:-}"; do kill -9 "$p" 2>/dev/null || true; done
  if [ -n "$RECORDER_PID" ] && kill -0 "$RECORDER_PID" 2>/dev/null; then
    # background jobs inherit ignored SIGINT; ros2 bag closes cleanly on SIGTERM and writes metadata
    kill -TERM "$RECORDER_PID" 2>/dev/null || true
    wait "$RECORDER_PID" || true
    if [ -n "$RUN_DIR" ]; then
      /usr/bin/python3 "$HERE/summarize_run.py" "$RUN_DIR" > "$RUN_DIR/summary.log" 2>&1 || true
    fi
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# ---------------------------------------------------------------- live preparation
prepare_live() {
  # Prefer the configured robot cable without taking down Wi-Fi internet.
  if ip -4 -o addr show scope global | grep -q '192\.168\.185\.10/'; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/src/team7_bringup/config/fastdds_wired.xml"
    echo '[*] ROS transport: wired 192.168.185.10 (Wi-Fi excluded from DDS)'
  fi

  # A partial colcon install can have setup.bash without the runnable packages.
  local package
  for package in team7_nodes team7_bringup turtlebot4_navigation aruco_opencv rosbag2_transport rosbag2_storage_mcap; do
    ros2 pkg prefix "$package" >/dev/null || {
      echo "error: $package missing; install dependencies and rebuild $WS" >&2
      exit 1
    }
  done
  ros2 interface show irobot_create_msgs/srv/ResetPose >/dev/null
  ros2 interface show irobot_create_msgs/action/Undock >/dev/null

  [ -n "$RUN_DIR" ] || RUN_DIR="$HOME/team7_runs/$(date +%Y%m%d_%H%M%S_%N)"
  RUN_DIR=${RUN_DIR/#\~\//$HOME/}
  mkdir -p "$RUN_DIR/config"
  RUN_DIR=$(cd "$RUN_DIR" && pwd)
  cp "$WS/src/team7_bringup/config/"* "$RUN_DIR/config/"
  export ROS_LOG_DIR="$RUN_DIR/ros_logs"
  {
    date -u
    uname -a
    git -C "$REPO" rev-parse HEAD
    git -C "$REPO" status --short
    printf 'launch argument: %s\n' ${LAUNCH_ARGS[@]+"${LAUNCH_ARGS[@]}"}
    printf 'ROS_DOMAIN_ID=%s\nRMW_IMPLEMENTATION=%s\nFASTRTPS_DEFAULT_PROFILES_FILE=%s\n' \
      "${ROS_DOMAIN_ID:-0}" "${RMW_IMPLEMENTATION:-}" "${FASTRTPS_DEFAULT_PROFILES_FILE:-}"
    ip -4 -br address
  } > "$RUN_DIR/run_info.txt"
  git -C "$REPO" diff -- host_ws crates > "$RUN_DIR/source_changes.patch"
  echo "[*] run artifacts: $RUN_DIR"

  if [ "$RECORD" = true ]; then
    # recording starts before any robot preparation, so the undock and reset are in the bag too
    bash "$HERE/record_run.sh" "$RUN_DIR" false > "$RUN_DIR/recorder.log" 2>&1 &
    RECORDER_PID=$!
    sleep 2
    kill -0 "$RECORDER_PID" 2>/dev/null || {
      cat "$RUN_DIR/recorder.log" >&2
      echo 'error: recording failed; robot preparation cancelled' >&2
      exit 1
    }
  fi

  echo "[*] checking robot topics"
  local topics
  topics=$(timeout 15 ros2 topic list --no-daemon --spin-time 3 || true)
  if ! grep -qx "/scan" <<<"$topics"; then
    echo "error: /scan not visible. Is the robot on, and are ROS_DOMAIN_ID / RMW_IMPLEMENTATION the same as on the Pi?" >&2
    exit 1
  fi
  # run still works without these, but no markers / no odom means wasted 10 min, so say it now
  for topic in /odom /oakd/rgb/preview/image_raw/compressed /oakd/rgb/preview/camera_info; do
    grep -qx "$topic" <<<"$topics" || echo "warning: $topic not visible; check the robot camera / create3 bridge" >&2
  done

  # Create 3 clock: /odom stamps must be close to this laptop's time (case 25). Warning only,
  # because the laptop CLI can miss messages; the Pi check (tb4_rpi_timesync.sh check) is the reference.
  local odom_sec clock_off
  odom_sec=$(timeout 8 ros2 topic echo /odom --once --field header.stamp 2>/dev/null | awk '/^sec:/{print $2; exit}' || true)
  if [[ "$odom_sec" =~ ^[0-9]+$ ]]; then
    clock_off=$(( $(date +%s) - odom_sec ))
    if (( clock_off > 5 || clock_off < -5 )); then
      echo "warning: Create 3 /odom stamp is ${clock_off}s from laptop time. Check the clock first (docs/troubleshooting case 25, reboot the Create 3 if still off)." >&2
    else
      echo "[*] Create 3 clock ok (${clock_off}s from laptop time)"
    fi
  else
    echo "warning: no /odom stamp read; Create 3 clock not checked" >&2
  fi

  if [ "$PREPARE" = true ]; then
    if timeout 5 ros2 topic echo --once /dock_status 2>/dev/null | grep -q "is_docked: true"; then
      echo "[*] robot is docked: undocking"
      timeout 60 ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" >/dev/null \
        || echo "warning: undock failed" >&2
      sleep 3
    fi
    echo "[*] resetting odometry (map origin = current pose)"
    # ResetPose takes a pose. "{}" also works (the message defaults w to 1, checked against a mock service),
    # but the explicit identity pose does not depend on defaults.
    if ! timeout 10 ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose \
        '{pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' >/dev/null; then
      echo "warning: /reset_pose failed; map origin will be wherever odom currently is" >&2
    fi
    sleep 1
  fi

  # the recorder is already running, so the launch itself must not record a second bag
  LAUNCH_ARGS+=("output_dir:=$RUN_DIR" record:=false)
}

# ---------------------------------------------------------------- replay
replay_relays() {
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-79}"
  RECORD=false
  [ "$RECORD_SET" = true ] && echo '[*] replay: recording option ignored, a replay never records' >&2
  # record:=false last, so it wins over anything passed through
  LAUNCH_ARGS=("${LAUNCH_ARGS[@]/use_sim_time:=false/use_sim_time:=true}" record:=false)
  TMP="$(mktemp -d)"
  # A live bag starts before the robot preparation, so it holds the /reset_pose jump (in run
  # 20261118_185946 from (4.69, 2.18) to (0, 0)). Start after it unless an offset was given.
  if [ -z "$OFFSET" ]; then
    OFFSET=$(python3 - "$BAG" <<'PY' 2>/dev/null || true
import math, sys, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/odom']))
t0 = prev = reset = None
while r.has_next():
    _, data, ts = r.read_next()
    t = ts / 1e9
    p = deserialize_message(data, Odometry).pose.pose.position
    t0 = t0 or t
    if t - t0 > 60:
        break
    # a jump onto the origin: the reset to the identity pose
    if prev and math.hypot(p.x - prev[0], p.y - prev[1]) > 0.02 and math.hypot(p.x, p.y) < 0.01:
        reset = t - t0
    prev = (p.x, p.y)
if reset is not None:
    print(f'{reset + 0.5:.1f}')
PY
)
    [ -n "$OFFSET" ] && echo "[*] replay: starting at ${OFFSET}s, after the recorded odometry reset (--start-offset 0 replays from the start)"
  fi
  # the recorded TF has map->odom from an old SLAM; relay it without the map frame
  # and forward the static TF, so the live SLAM owns map->odom.
  cat > "$TMP/tf_relay.py" <<'PY'
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
class Relay(Node):
    def __init__(self):
        super().__init__('bag_tf_relay')
        self.pub = self.create_publisher(TFMessage, '/tf', 100)
        self.create_subscription(TFMessage, '/old_tf', self.cb, 100)
    def cb(self, msg):
        keep = [t for t in msg.transforms if t.header.frame_id != 'map']
        if keep:
            out = TFMessage(); out.transforms = keep
            self.pub.publish(out)
rclpy.init(); rclpy.spin(Relay())
PY
  # the bag records static TF only at its start, so read it from the bag and keep it latched
  cat > "$TMP/static_relay.py" <<'PY'
import sys
import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
def read_static(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    out = []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == '/tf_static':
            out.append(deserialize_message(data, TFMessage))
    return out
class R(Node):
    def __init__(self, msgs):
        super().__init__('bag_static_relay')
        self.msgs = msgs
        self.pub = self.create_publisher(TFMessage, '/tf_static', latched)
        self.create_timer(1.0, self.publish_all)
    def publish_all(self):
        for m in self.msgs:
            self.pub.publish(m)
rclpy.init()
rclpy.spin(R(read_static(sys.argv[1])))
PY
  ros2 bag play "$BAG" --clock ${OFFSET:+--start-offset "$OFFSET"} \
    --exclude-topics /map /map_updates /map_metadata /global_costmap/costmap /global_costmap/costmap_updates \
      /global_costmap/costmap_raw /local_costmap/costmap /local_costmap/costmap_updates /local_costmap/costmap_raw \
      /plan /received_global_plan /cmd_vel /cmd_vel_nav /cmd_vel_smoothed /cmd_vel_filtered /cmd_vel_unstamped \
      /team7/frontiers /team7/goal /team7/targets /team7/mission_state /aruco_detections \
      /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback /collision_monitor_state \
      --remap /tf:=/old_tf /tf_static:=/old_tf_static > "$TMP/bag.log" 2>&1 &
  PIDS+=($!)
  python3 "$TMP/tf_relay.py" > "$TMP/tf_relay.log" 2>&1 & PIDS+=($!)
  python3 "$TMP/static_relay.py" "$BAG" > "$TMP/static_relay.log" 2>&1 & PIDS+=($!)
  echo "replay: $BAG on ROS_DOMAIN_ID=$ROS_DOMAIN_ID (logs in $TMP)"
  sleep 3
}

# ---------------------------------------------------------------- run
if [ -n "$BAG" ]; then
  replay_relays
else
  prepare_live
fi

# every run keeps its logs: live runs in their run folder (next to the bag), replays in a replay_* folder.
# console.log is the launch output; ros_logs/ holds each node's own log. tee ignores Ctrl-C, so the
# file still gets the last lines when the run is stopped.
LOG_DIR="${RUN_DIR:-}"
if [ -n "$BAG" ]; then
  [ -n "$LOG_DIR" ] || LOG_DIR="$HOME/team7_runs/replay_$(basename "${BAG%/}")_$(date +%Y%m%d_%H%M%S_%N)"
  mkdir -p "$LOG_DIR"
  LOG_DIR=$(cd "$LOG_DIR" && pwd)
  export ROS_LOG_DIR="$LOG_DIR/ros_logs"
  # map, targets and autosaves of the replay stay with its logs instead of overwriting ~/team7_run
  LAUNCH_ARGS+=("output_dir:=$LOG_DIR")
fi
echo "[*] run logs: $LOG_DIR"

if [ "$GRAPH" = true ]; then
  # same ROS_DOMAIN_ID as the run (79 on replays); give the nodes time to come up, then refresh in the window.
  # A Nix glibc on LD_LIBRARY_PATH (desktop Mesa setup) breaks PySide2, so leave /nix paths out for it.
  (sleep 15; LD_LIBRARY_PATH="$(tr ':' '\n' <<<"${LD_LIBRARY_PATH:-}" | grep -v '^/nix/' | paste -sd: -)" \
    exec ros2 run rqt_graph rqt_graph) > "$LOG_DIR/rqt_graph.log" 2>&1 &
  PIDS+=($!)
fi
if [ "$VIZ" = true ]; then
  # always the main workspace: the --stock cache builds only team7_nodes and team7_bringup
  VIZ_BIN="$HERE/../ros2_ws/install/team7_viz/lib/team7_viz/team7_viz"
  if [ -x "$VIZ_BIN" ]; then
    (set +u; source "$HERE/../ros2_ws/install/setup.bash"; exec "$VIZ_BIN") > "$LOG_DIR/team7_viz.log" 2>&1 &
    PIDS+=($!)
  else
    echo "warning: --viz: $VIZ_BIN not built (cd host_ws/ros2_ws && colcon build --packages-select team7_viz)" >&2
  fi
fi
echo "launch: team7.launch.py ${LAUNCH_ARGS[*]}"
# Started from a script or in the background, bash hands SIGINT down as ignored, and the documented
# stop (kill -INT on the launch) would do nothing. Reset it to the default for the launch and its nodes.
python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
  ros2 launch team7_bringup team7.launch.py "${LAUNCH_ARGS[@]}" 2>&1 | (trap '' INT; exec tee -a "$LOG_DIR/console.log")
