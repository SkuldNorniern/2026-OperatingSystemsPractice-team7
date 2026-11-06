#!/usr/bin/env bash
# usage: sudo ./oakd_preview.sh set <size> [fps] | status | restore
# then turtlebot4-service-restart. apt upgrade overwrites it
set -euo pipefail

CONFIG_DIR=/opt/ros/jazzy/share/turtlebot4_bringup/config
MODEL=${TB4_CAMERA:-oakd_pro}
CONF=$CONFIG_DIR/$MODEL.yaml
BACKUP=$CONF.team7.bak

die() { echo "error: $*" >&2; exit 1; }
[ -f "$CONF" ] || die "$CONF not found (set TB4_CAMERA=oakd_lite on a Lite?)"

case "${1:-status}" in
  set)
    [ "$(id -u)" -eq 0 ] || die "run with sudo"
    size=${2:?size required, e.g. 480}
    fps=${3:-}
    [[ "$size" =~ ^[0-9]+$ ]] || die "size must be an integer"
    [ -f "$BACKUP" ] || cp -a "$CONF" "$BACKUP"
    sed -i -E "s/^(\s*i_preview_size:\s*).*/\1$size/" "$CONF"
    if [ -n "$fps" ]; then
      [[ "$fps" =~ ^[0-9]+(\.[0-9]+)?$ ]] || die "fps must be a number"
      [[ "$fps" == *.* ]] || fps="$fps.0"
      sed -i -E "s/^(\s*i_fps:\s*).*/\1$fps/" "$CONF"
    fi
    echo "[*] updated $CONF (backup: $BACKUP)"
    grep -E "i_preview_size|i_fps" "$CONF"
    echo "[*] restart the robot software to apply: turtlebot4-service-restart"
    ;;
  status)
    grep -E "i_preview_size|i_fps" "$CONF"
    [ -f "$BACKUP" ] && echo "(modified; backup at $BACKUP)" || echo "(factory config)"
    ;;
  restore)
    [ "$(id -u)" -eq 0 ] || die "run with sudo"
    [ -f "$BACKUP" ] || die "no backup; config is unmodified"
    mv "$BACKUP" "$CONF"
    echo "[*] restored $CONF; restart the robot software to apply"
    ;;
  *) die "usage: $0 set <size> [fps] | status | restore" ;;
esac
