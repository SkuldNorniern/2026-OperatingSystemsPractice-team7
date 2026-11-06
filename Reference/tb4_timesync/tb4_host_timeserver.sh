#!/usr/bin/env bash
# =====================================================================
#  tb4_host_timeserver.sh — run ON the host laptop (Ubuntu 24.04)
#
#  Makes the laptop the NTP time server for the TurtleBot4 RPi.
#  The laptop keeps its own internet NTP sources when online, and still
#  serves time (stratum 8) when offline, e.g. on a robot-only Wi-Fi.
#
#  Usage:
#     sudo ./tb4_host_timeserver.sh setup [ROBOT_SUBNET]   default 192.168.185.0/24
#          ./tb4_host_timeserver.sh status
#     sudo ./tb4_host_timeserver.sh restore
#
#  Note: the course script (tb4_timesync_install.txt) turns NTP off on the
#  laptop and stops chrony. `setup` turns chrony back on; don't mix the two.
#  Keep this file with LF line endings.
# =====================================================================
set -euo pipefail

CONF_DIR=/etc/chrony/conf.d
MAIN_CONF=/etc/chrony/chrony.conf
DROPIN=$CONF_DIR/50-tb4-server.conf

die()       { echo "error: $*" >&2; exit 1; }
need_root() { [ "$(id -u)" -eq 0 ] || die "run with sudo"; }

do_setup() {
  need_root
  local subnet=${1:-192.168.185.0/24}
  [[ $subnet =~ ^[0-9.]+/[0-9]{1,2}$ ]] || die "invalid subnet: $subnet"

  if ! command -v chronyd >/dev/null 2>&1; then
    echo "[*] installing chrony (replaces systemd-timesyncd)"
    apt-get update && apt-get install -y chrony
  fi
  systemctl disable --now systemd-timesyncd >/dev/null 2>&1 || true

  grep -qE "^confdir[[:space:]]+$CONF_DIR" "$MAIN_CONF" || echo "confdir $CONF_DIR" >> "$MAIN_CONF"
  mkdir -p "$CONF_DIR"
  cat > "$DROPIN" <<EOF
# Managed by tb4_host_timeserver.sh — remove with: sudo $0 restore
# Let the TurtleBot4 subnet query this laptop.
allow $subnet
# Serve time even without internet upstream.
local stratum 8
EOF

  systemctl enable chrony >/dev/null 2>&1 || true
  systemctl restart chrony

  if command -v ufw >/dev/null 2>&1 && ufw status | grep -q 'Status: active'; then
    echo "[*] opening UDP 123 for $subnet in ufw"
    ufw allow from "$subnet" to any port 123 proto udp >/dev/null
  fi

  echo "NTP server ready for $subnet. Use one of these as HOST_IP on the robot:"
  ip -4 -o addr show scope global | awk '{print "  " $2 ": " $4}'
  echo "Then on the RPi:  sudo ./tb4_rpi_timesync.sh setup <HOST_IP>"
}

do_status() {
  chronyc tracking
  echo
  if [ "$(id -u)" -eq 0 ]; then
    echo "== NTP clients (the RPi should appear here) =="
    chronyc clients
  else
    echo "(run with sudo to list NTP clients)"
  fi
}

do_restore() {
  need_root
  rm -f "$DROPIN"
  systemctl restart chrony
  echo "removed $DROPIN (ufw rule, if any, is left in place)"
}

case "${1:-}" in
  setup)   shift; do_setup "$@" ;;
  status)  do_status ;;
  restore) do_restore ;;
  *) sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
