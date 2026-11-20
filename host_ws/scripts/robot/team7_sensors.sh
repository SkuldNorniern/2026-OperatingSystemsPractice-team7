#!/usr/bin/env bash
# Enable the Create 3 bump (hazard_detection) and IR (ir_intensity) relays in the TB4 republisher config.
# Runs on the Pi as root, at every boot, before turtlebot4.service (see team7-sensors.service).
# Idempotent: if the lines are already enabled it changes nothing. Safe after apt upgrades, which
# overwrite the /opt file and bring the lines back commented out.
#
# usage (on the Pi, as root): team7_sensors.sh [--file PATH] [--dry-run]
set -euo pipefail

FILE=/opt/ros/jazzy/share/turtlebot4_bringup/config/republisher.yaml
ETC=/etc/turtlebot4/republisher.yaml
DRY=false
while [ $# -gt 0 ]; do
  case "$1" in
    --file) FILE="$2"; ETC="$2.etc-copy"; shift 2 ;;
    --dry-run) DRY=true; shift ;;
    -h|--help) sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

log() { echo "team7_sensors: $*"; }
[ -f "$FILE" ] || { log "no $FILE (TurtleBot 4 software not installed?)"; exit 1; }

# the two lines, commented or not
commented=$(grep -cE '^\s*# "(hazard_detection|ir_intensity)",' "$FILE" || true)
enabled=$(grep -cE '^\s*"(hazard_detection|ir_intensity)",' "$FILE" || true)

if [ "$commented" -eq 0 ] && [ "$enabled" -ge 2 ]; then
  log "already enabled in $FILE"
else
  if [ "$enabled" -lt 2 ] && [ "$commented" -eq 0 ]; then
    log "neither line found in $FILE; the file differs from the jazzy repo, not changing it"
    exit 1
  fi
  if $DRY; then
    log "[dry-run] would uncomment $commented line(s) in $FILE"
  else
    [ -f "$FILE.orig" ] || cp -a "$FILE" "$FILE.orig"
    sed -i -E 's/^(\s*)# ("(hazard_detection|ir_intensity)",)/\1  \2/' "$FILE"
    log "uncommented $commented line(s) in $FILE (backup: $FILE.orig)"
  fi
fi

# the copy in /etc is what we re-apply after updates; keep it in step with the file
if $DRY; then
  log "[dry-run] would copy the file to the /etc copy"
else
  mkdir -p "$(dirname "$ETC")"
  if ! cmp -s "$FILE" "$ETC" 2>/dev/null; then
    cp -a "$FILE" "$ETC"
    log "copied to $ETC"
  fi
fi

# verify
grep -qE '^\s*"hazard_detection",' "$FILE" && grep -qE '^\s*"ir_intensity",' "$FILE" \
  || { log "verify failed: lines not enabled in $FILE"; exit 1; }
log "ok"
