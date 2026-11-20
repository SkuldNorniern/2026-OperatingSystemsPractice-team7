#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  printf '%s\n\n' 'Run with no arguments to start V1 using the room and marker settings in this script.'
  exec "${TEAM7_PYTHON:-python3}" "$HERE/../robot_runtime/run_rust_base_v1.py" --help
fi

if (( $# == 0 )); then
  # Room dimensions and the marker side length are in metres.
  set -- --execute \
    --room-width 19.85 --room-height 11.05 \
    --marker-size 0.10 --marker-ids 8 19 29
fi

if [[ "$(uname -s)" != Linux ]]; then
  echo 'Run this launcher on the Ubuntu ROS 2 Jazzy host connected to the TurtleBot4.' >&2
  exit 2
fi
ros_setup=/opt/ros/jazzy/setup.bash
[[ -f "$ros_setup" ]] || { echo "Missing $ros_setup" >&2; exit 2; }
set +u
source "$ros_setup"
set -u
exec "${TEAM7_PYTHON:-/usr/bin/python3}" -u "$HERE/../robot_runtime/run_rust_base_v1.py" "$@"
