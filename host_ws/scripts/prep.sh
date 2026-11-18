#!/usr/bin/env bash
# usage: prep.sh [--deps] [--clean-bindings] [--no-build]
#   Builds the team7 ROS workspace (host_ws/ros2_ws), the same way host_ws/README.md describes.
#   Run it once after a fresh checkout, and again after changing code.
#
#   default:          check tools, source ROS, colcon build (skips multirobot_map_merge)
#   --deps:           also vcs import team7.repos and rosdep install (needs network, may ask for sudo)
#   --clean-bindings: clear the cached r2r bindings first (after installing new message packages)
#   --no-build:       only run the checks and the optional steps above
#
#   Holds the same run lock as run_demo.sh, so a build never changes the install under a live run.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../ros2_ws" && pwd)"
ROS_SETUP=/opt/ros/jazzy/setup.bash
MIN_RUST="1.85"

DEPS=false; CLEAN=false; BUILD=true
for arg in "$@"; do
  case "$arg" in
    --deps) DEPS=true ;;
    --clean-bindings) CLEAN=true ;;
    --no-build) BUILD=false ;;
    -h|--help) sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $arg (see --help)" >&2; exit 2 ;;
  esac
done

die() { echo "error: $*" >&2; exit 1; }

# one run at a time: a build must not change the install while a run uses it (same lock as run_demo.sh)
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/team7_demo_${UID}.lock"
flock -n 9 || die "a Team7 run is active; stop it before building (the build would change the install it uses)"

# tools
[ -f "$ROS_SETUP" ] || die "ROS 2 Jazzy not found at $ROS_SETUP (install ros-jazzy-desktop first)"
command -v cargo >/dev/null || die "cargo not found. Install rust with rustup (apt rust is too old)"
command -v colcon >/dev/null || die "colcon not found: sudo apt install python3-colcon-common-extensions"
command -v vcs >/dev/null || [ "$DEPS" = false ] || die "vcs not found: sudo apt install python3-vcstool"
rust_ver="$(rustc --version | awk '{print $2}')"
if [ "$(printf '%s\n%s\n' "$MIN_RUST" "$rust_ver" | sort -V | head -1)" != "$MIN_RUST" ]; then
  die "rustc $rust_ver is older than $MIN_RUST (edition 2024). Update with rustup update"
fi
[ -x /usr/bin/clang ] || ls /usr/lib/llvm-*/lib/libclang*.so* >/dev/null 2>&1 \
  || echo "warning: libclang not found; r2r needs it (sudo apt install libclang-dev)" >&2
echo "[*] rustc $rust_ver, workspace $WS"

# ROS environment (bash; zsh cannot source setup.bash, see CLAUDE.md)
set +u; source "$ROS_SETUP"; set -u

cd "$WS"

if [ "$DEPS" = true ]; then
  echo "[*] importing sources from team7.repos"
  vcs import src < team7.repos
  echo "[*] installing ROS dependencies with rosdep"
  rosdep install --from-paths src --ignore-src -y -r
fi

if [ "$CLEAN" = true ]; then
  echo "[*] clearing cached r2r bindings"
  ( cd src/team7_nodes && CARGO_TARGET_DIR=../../build/team7_nodes/cargo \
      cargo clean --profile colcon -p r2r -p r2r_msg_gen -p r2r_rcl )
fi

if [ "$BUILD" = true ]; then
  echo "[*] colcon build (first r2r build takes about 30 s)"
  colcon build --symlink-install --packages-skip multirobot_map_merge \
    || die "colcon build failed; read the first error above"
  echo "[*] build ok. Source the workspace with: source $WS/install/setup.bash"
  echo "    (zsh: source $WS/install/setup.zsh)"
fi
