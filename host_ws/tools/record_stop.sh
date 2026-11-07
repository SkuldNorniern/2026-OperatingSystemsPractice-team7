#!/usr/bin/env bash
# usage: record_stop.sh. stops recorder from record_start.sh and prints bag info
set -euo pipefail

BAG_DIR=${BAG_DIR:-$HOME/bags}
PID_FILE=$BAG_DIR/.recording.pid

[ -f "$PID_FILE" ] || { echo "not recording" >&2; exit 1; }
PID=$(sed -n 1p "$PID_FILE")
OUT=$(sed -n 2p "$PID_FILE")

if kill -0 "$PID" 2>/dev/null; then
  # SIGINT to whole group so mcap gets closed properly
  kill -INT -- "-$PID" 2>/dev/null || kill -INT "$PID"
  for _ in $(seq 1 30); do
    kill -0 "$PID" 2>/dev/null || break
    sleep 0.5
  done
  if kill -0 "$PID" 2>/dev/null; then
    echo "recorder still running after 15 s, sending SIGTERM" >&2
    kill -TERM -- "-$PID" 2>/dev/null || kill -TERM "$PID"
    sleep 2
  fi
fi
rm -f "$PID_FILE"

if ! command -v ros2 >/dev/null; then
  set +u; source /opt/ros/jazzy/setup.bash; set -u
fi
echo "stopped -> $OUT"
ros2 bag info "$OUT" 2>/dev/null || echo "no bag info, check $OUT.log" >&2
