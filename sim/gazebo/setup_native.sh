#!/bin/zsh
set -eu
cd "${0:A:h}"

if ! command -v brew >/dev/null || ! command -v gz >/dev/null; then
  print -u2 "Install Homebrew Gazebo Harmonic first: brew install osrf/simulation/gz-harmonic"
  exit 1
fi

if ! command -v cargo >/dev/null; then
  print -u2 "Install Rust 1.89 or newer first: brew install rust"
  exit 1
fi
repo_root="${PWD:h:h}"
cargo build --manifest-path "$repo_root/Cargo.toml" --release -p tb4_aruco_gazebo --locked

# Gazebo's native extension must match the Python interpreter ABI.
python_bin="${GAZEBO_PYTHON:-$(brew --prefix)/bin/python3.14}"
if [[ ! -x "$python_bin" ]]; then
  print -u2 "Python 3.14 not found; install it or set GAZEBO_PYTHON to a compatible interpreter."
  exit 1
fi
"$python_bin" -c 'from gz.transport13 import Node' || {
  print -u2 "This Python cannot import Homebrew gz.transport13. Check the Python ABI and Gazebo installation."
  exit 1
}
"$python_bin" -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r native_sim/requirements.txt
.venv/bin/python -c 'import cv2, numpy; from gz.transport13 import Node; from gz.msgs10.twist_pb2 import Twist; print("Native Gazebo Python dependencies ready.")'
