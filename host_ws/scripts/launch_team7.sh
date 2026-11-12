#!/usr/bin/env bash
# usage: launch_team7.sh [--stock] [--replay BAG_DIR] [name:=value ...]
#   default: live robot, team7 navigation (our Lazy Theta* + MPPI, no Nav2) with team7 SLAM
#   --stock: the committed HEAD of this repo (rust_base), built into a cache, with the stock
#            launch defaults (Nav2 navigation, slam_toolbox). set TEAM7_REV to pin another commit.
#   --replay BAG_DIR: test on a recorded run instead of the robot (bag clock, TF relayed)
#   --start-offset SEC: with --replay, start the bag SEC seconds in (e.g. where /odom starts)
# extra name:=value args go to team7.launch.py, e.g. nav:=lazy_nav2 or explorer:=none
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
WS="$HERE/../ros2_ws"
BAG=""
OFFSET=""
STOCK=false
EXTRA=()
LAUNCH_ARGS=(use_sim_time:=false nav:=team7 slam_backend:=team7)

while [ $# -gt 0 ]; do
  case "$1" in
    --stock) STOCK=true ;;
    --replay) BAG="${2:?--replay needs a bag directory}"; shift ;;
    --start-offset) OFFSET="${2:?--start-offset needs seconds}"; shift ;;
    --help|-h) sed -n 2,8p "$0"; exit 0 ;;
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

# one team7 run at a time (same lock as run_demo.sh)
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/team7_demo_${UID}.lock"
flock -n 9 || { echo 'error: a Team7 run is already active; stop it before starting another' >&2; exit 1; }

set +u
source /opt/ros/jazzy/setup.bash
[ -f "$WS/install/setup.bash" ] || { echo "build first: cd $WS && colcon build" >&2; exit 1; }
source "$WS/install/setup.bash"
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

PIDS=()
cleanup() {
  for p in "${PIDS[@]:-}"; do kill -INT "$p" 2>/dev/null || true; done
  sleep 2
  for p in "${PIDS[@]:-}"; do kill -9 "$p" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

if [ -n "$BAG" ]; then
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-79}"
  LAUNCH_ARGS=("${LAUNCH_ARGS[@]/use_sim_time:=false/use_sim_time:=true}" record:=false)
  TMP="$(mktemp -d)"
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
fi

echo "launch: team7.launch.py ${LAUNCH_ARGS[*]}"
ros2 launch team7_bringup team7.launch.py "${LAUNCH_ARGS[@]}"
