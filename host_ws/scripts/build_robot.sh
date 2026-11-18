#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd -- "$HERE/../.." && pwd)
[[ "$(uname -s)" == Linux ]] || { echo 'Build on the Linux host that will execute the robot runtime.' >&2; exit 2; }
command -v cargo >/dev/null || { echo 'Install Rust 1.89+ first (see host_ws/robot_runtime/README.md).' >&2; exit 2; }
cargo build --manifest-path "$REPO/Cargo.toml" --release -p tb4_aruco_gazebo --locked --target-dir "$REPO/target"
echo 'Built real-robot Rust detector/explorer. No Gazebo or colcon is required for this adapter.'
