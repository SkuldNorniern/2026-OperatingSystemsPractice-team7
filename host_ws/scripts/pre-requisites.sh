#!/usr/bin/env bash
# usage: pre-requisites.sh [--yes] [--skip-apt] [--skip-rust] [--skip-sources] [--skip-rosdep]
#   One-time setup for a new laptop, before prep.sh. Covers everything in host_ws/README.md "setup (once)":
#     1. apt packages (ROS 2 Jazzy desktop extras, colcon, vcstool, rosdep, libclang, Nav2 and TB4 packages)
#     2. rust from rustup (apt rust is too old; we need 1.85 or newer)
#     3. workspace sources: vcs import team7.repos into host_ws/ros2_ws/src
#     4. ROS dependencies: rosdep install
#
#   It asks before each step that changes the system. --yes answers yes to all (you still type the sudo password).
#   Safe to re-run: each step checks what is already installed.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../ros2_ws" && pwd)"
ROS_SETUP=/opt/ros/jazzy/setup.bash
MIN_RUST="1.85"

YES=false; SKIP_APT=false; SKIP_RUST=false; SKIP_SRC=false; SKIP_ROSDEP=false
for arg in "$@"; do
  case "$arg" in
    --yes) YES=true ;;
    --skip-apt) SKIP_APT=true ;;
    --skip-rust) SKIP_RUST=true ;;
    --skip-sources) SKIP_SRC=true ;;
    --skip-rosdep) SKIP_ROSDEP=true ;;
    -h|--help) sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $arg (see --help)" >&2; exit 2 ;;
  esac
done

APT_PKGS=(
  python3-colcon-common-extensions python3-vcstool python3-rosdep libclang-dev
  ros-jazzy-turtlebot4-desktop ros-jazzy-turtlebot4-navigation ros-jazzy-slam-toolbox
  ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server ros-jazzy-aruco-opencv ros-jazzy-irobot-create-msgs
)

die() { echo "error: $*" >&2; exit 1; }

ask() {  # ask "question": returns 0 for yes
  $YES && return 0
  read -r -p "$1 [y/N] " reply || reply=""
  [[ "$reply" =~ ^[Yy]$ ]]
}

[ -f "$ROS_SETUP" ] || die "ROS 2 Jazzy is not installed at $ROS_SETUP. Install ros-jazzy-desktop from the ROS 2 apt repository first."

# 1. apt packages
if [ "$SKIP_APT" = false ]; then
  missing=()
  for p in "${APT_PKGS[@]}"; do
    dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p")
  done
  if [ "${#missing[@]}" -eq 0 ]; then
    echo "[1] apt packages: all present"
  else
    echo "[1] apt packages missing: ${missing[*]}"
    if ask "install them with apt (needs sudo)?"; then
      sudo apt-get update
      sudo apt-get install -y "${missing[@]}"
    else
      echo "    skipped; prep.sh will fail until they are installed"
    fi
  fi
fi

# 2. rust
if [ "$SKIP_RUST" = false ]; then
  if command -v cargo >/dev/null && command -v rustc >/dev/null; then
    rust_ver="$(rustc --version | awk '{print $2}')"
    if [ "$(printf '%s\n%s\n' "$MIN_RUST" "$rust_ver" | sort -V | head -1)" = "$MIN_RUST" ]; then
      echo "[2] rust $rust_ver: ok"
    else
      echo "[2] rust $rust_ver is older than $MIN_RUST"
      ask "run rustup update?" && rustup update stable
    fi
  else
    echo "[2] rust not found"
    if ask "install rust with rustup (official installer, interactive)?"; then
      curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
      echo "    open a new shell or run: source \$HOME/.cargo/env"
    fi
  fi
fi

# 3. sources (vcs import)
if [ "$SKIP_SRC" = false ]; then
  # every repo listed in team7.repos must exist under src/
  missing_src=()
  while read -r repo; do
    [ -d "$WS/src/$repo" ] || missing_src+=("$repo")
  done < <(awk '/^  [^ ].*:$/{sub(/^  /,""); sub(/:$/,""); print}' "$WS/team7.repos")
  if [ "${#missing_src[@]}" -eq 0 ]; then
    echo "[3] workspace sources: all present"
  else
    echo "[3] workspace sources missing: ${missing_src[*]}"
    echo "[3] workspace sources missing"
    if ask "vcs import team7.repos into $WS/src (needs network)?"; then
      ( cd "$WS" && vcs import src < team7.repos )
    fi
  fi
fi

# 4. rosdep
if [ "$SKIP_ROSDEP" = false ]; then
  if ask "run rosdep install for the workspace (may ask for sudo)?"; then
    [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || { sudo rosdep init; }
    rosdep update
    ( cd "$WS" && rosdep install --from-paths src --ignore-src -y -r )
  fi
fi

echo "[*] prerequisites done. Next: host_ws/scripts/prep.sh (build)"
