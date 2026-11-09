#!/usr/bin/env bash
# Record before robot preparation; also usable independently for diagnostics.
set -euo pipefail
RUN_DIR=${1:?usage: record_run.sh OUTPUT_DIR [true for simulation]}
SHARE=$(ros2 pkg prefix --share team7_bringup)
mkdir -p "$RUN_DIR"
BAG_DIR="$RUN_DIR/rosbag_$(date +%Y%m%d_%H%M%S_%N)"
SIM_ARGS=()
if [ "${2:-false}" = true ]; then SIM_ARGS+=(--use-sim-time); fi
exec ros2 bag record -o "$BAG_DIR" -s mcap \
  --include-hidden-topics --regex "$(cat "$SHARE/config/recording_topics.regex")" \
  --qos-profile-overrides-path "$SHARE/config/recording_qos.yaml" \
  --max-bag-size 1073741824 --max-cache-size 67108864 \
  --disable-keyboard-controls "${SIM_ARGS[@]}"
