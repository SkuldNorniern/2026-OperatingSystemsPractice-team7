#!/usr/bin/env bash
# usage: record_start.sh [raw|full|all] [name]
# raw = sensors only, record with SLAM/Nav2 off for replay tuning. full = sensors + our outputs
set -euo pipefail

MODE=${1:-full}
NAME=${2:-${MODE}_$(date +%m%d_%H%M%S)}
BAG_DIR=${BAG_DIR:-$HOME/bags}
PID_FILE=$BAG_DIR/.recording.pid

RAW=(/scan /odom /imu /tf /tf_static /robot_description /cmd_vel
     /oakd/rgb/preview/image_raw /oakd/rgb/preview/camera_info)
FULL=("${RAW[@]}" /map /global_costmap/costmap /plan /aruco_detections
      /team7/targets /team7/frontiers /team7/mission_state)

case "$MODE" in
  raw)  TOPICS=("${RAW[@]}") ;;
  full) TOPICS=("${FULL[@]}") ;;
  all)  TOPICS=(-a) ;;
  *) echo "usage: $0 [raw|full|all] [name]" >&2; exit 1 ;;
esac

if ! command -v ros2 >/dev/null; then
  set +u; source /opt/ros/jazzy/setup.bash; set -u
fi

mkdir -p "$BAG_DIR"
if [ -f "$PID_FILE" ] && kill -0 "$(sed -n 1p "$PID_FILE")" 2>/dev/null; then
  echo "already recording (pid $(sed -n 1p "$PID_FILE")), run record_stop.sh first" >&2
  exit 1
fi
OUT=$BAG_DIR/$NAME
[ -e "$OUT" ] && { echo "$OUT already exists" >&2; exit 1; }

# job control on, so recorder gets its own process group and SIGINT is not ignored
set -m
nohup ros2 bag record -o "$OUT" --storage-preset-profile zstd_fast "${TOPICS[@]}" \
  > "$OUT.log" 2>&1 < /dev/null &
PID=$!
set +m
echo "$PID" > "$PID_FILE"
echo "$OUT" >> "$PID_FILE"

sleep 2
if ! kill -0 "$PID" 2>/dev/null; then
  echo "recorder died, see $OUT.log" >&2
  rm -f "$PID_FILE"
  exit 1
fi
echo "recording $MODE -> $OUT (pid $PID, log $OUT.log)"
