#!/usr/bin/env bash
# usage: export_bags.sh [--src DIR]... [--dest DIR] [--name NAME] [--dry-run]
#   Zips every rosbag under the run folders and moves the zip to the Downloads folder.
#   Default sources: ~/team7_run and ~/team7_runs (live runs use the second).
#   Default destination: ~/Downloads. The zip keeps the bag folder names, so each bag is unpacked as
#   rosbag_<time>/ with metadata.yaml and the .mcap file.
#
#   Refuses to run while a Team7 run is active (same lock as run_demo.sh), so no bag is zipped while it
#   is still being written. Checks the zip with `unzip -t` before the copy is moved to Downloads.
set -euo pipefail

SRCS=()
DEST="$HOME/Downloads"
NAME="team7_bags_$(date +%Y%m%d_%H%M%S)"
DRY=false
while [ $# -gt 0 ]; do
  case "$1" in
    --src) SRCS+=("$2"); shift 2 ;;
    --dest) DEST="$2"; shift 2 ;;
    --name) NAME="$2"; shift 2 ;;
    --dry-run) DRY=true; shift ;;
    -h|--help) sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $1 (see --help)" >&2; exit 2 ;;
  esac
done
[ "${#SRCS[@]}" -gt 0 ] || SRCS=("$HOME/team7_run" "$HOME/team7_runs")

die() { echo "error: $*" >&2; exit 1; }

# same run lock as run_demo.sh: do not zip a bag while a run writes it
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/team7_demo_${UID}.lock"
flock -n 9 || die "a Team7 run is active; stop it first so the bags are complete"

command -v zip >/dev/null || die "zip not found: sudo apt install zip"

# collect bag folders (any folder named rosbag_* that contains metadata.yaml or an .mcap file)
bags=()
for src in "${SRCS[@]}"; do
  [ -d "$src" ] || continue
  while IFS= read -r -d '' b; do
    if [ -f "$b/metadata.yaml" ] || compgen -G "$b/*.mcap" >/dev/null; then
      bags+=("$b")
    else
      echo "skip (not a bag folder): $b" >&2
    fi
  done < <(find "$src" -type d -name 'rosbag_*' -print0)
done
# run folders with logs (live runs and replay_* runs, with or without a bag). Each is archived whole,
# so its console.log, ros_logs/, recorder.log, summary and config go into the zip with it.
logruns=()
for src in "${SRCS[@]}"; do
  [ -d "$src" ] || continue
  while IFS= read -r -d '' d; do
    if [ -f "$d/console.log" ] || [ -f "$d/run_info.txt" ] || [ -d "$d/ros_logs" ]; then
      logruns+=("$d")
    fi
  done < <(find "$src" -mindepth 1 -maxdepth 1 -type d -print0)
done
[ "${#bags[@]}" -gt 0 ] || [ "${#logruns[@]}" -gt 0 ] || die "no rosbag_* folders or run logs under: ${SRCS[*]}"

# a bag without metadata.yaml still plays after `ros2 bag reindex`; say so instead of hiding it
for b in "${bags[@]}"; do
  [ -f "$b/metadata.yaml" ] || echo "warning: no metadata.yaml in $b (run: ros2 bag reindex $b)" >&2
done

if [ "${#bags[@]}" -gt 0 ]; then
  total_kb=$(du -ck "${bags[@]}" | tail -1 | awk '{print $1}')
  echo "[*] ${#bags[@]} bags, about $((total_kb / 1024)) MB"
fi
for b in "${bags[@]}" "${logruns[@]}"; do
  case "$b" in "$HOME"/*) ;; *) die "folder outside \$HOME, the archive would store an absolute path: $b" ;; esac
done
echo "[*] run logs: ${#logruns[@]} run folder(s)"
for d in "${logruns[@]}"; do echo "    $d"; done

if $DRY; then echo "[dry-run] would write $DEST/$NAME.zip"; exit 0; fi

mkdir -p "$DEST"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
zipfile="$tmp/$NAME.zip"

# what goes into the zip: a bag's run folder when it has one (the run's console.log, ros_logs/,
# recorder.log, summary and config travel with the bag), otherwise the bag folder alone
items=()
for b in "${bags[@]}"; do
  run="$(dirname "$b")"
  if [ -f "$run/run_info.txt" ] || [ -f "$run/console.log" ]; then
    item="$run"
  else
    item="$b"
  fi
  case " ${items[*]} " in *" $item "*) ;; *) items+=("$item") ;; esac
done
for d in "${logruns[@]}"; do
  case " ${items[*]} " in *" $d "*) ;; *) items+=("$d") ;; esac
done
[ "${#items[@]}" -gt 0 ] || die "nothing to archive"
echo "[*] archiving ${#items[@]} folder(s); run folders include their logs"

# zip with the path relative to $HOME, so the archive has no absolute paths
( for it in "${items[@]}"; do
    printf '%s\0' "${it#"$HOME"/}"
  done ) | ( cd "$HOME" && xargs -0 zip -q -r "$zipfile" )

echo "[*] checking the archive"
unzip -tq "$zipfile" >/dev/null || die "archive check failed; nothing was moved"

final="$DEST/$NAME.zip"
mv "$zipfile" "$final"
echo "[ok] $final ($(du -h "$final" | awk '{print $1}'))"
