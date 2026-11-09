#!/usr/bin/env bash
# =====================================================================
#  tb4_rpi_timesync.sh — run ON the TurtleBot4 Raspberry Pi
#  (Ubuntu 24.04 / ROS 2 Jazzy)
#
#  Time chain (all NTP via chrony, kept in sync continuously):
#
#     host laptop  ──NTP──▶  RPi (this script)  ──NTP──▶  Create3
#     (tb4_host_timeserver.sh)                  192.168.186.2 -> .3
#
#  Usage:
#     sudo ./tb4_rpi_timesync.sh setup <HOST_IP>   one-time: follow the host clock
#     sudo ./tb4_rpi_timesync.sh sync              step to the host clock right now
#          ./tb4_rpi_timesync.sh status            offset / sources / Create3 client
#     sudo ./tb4_rpi_timesync.sh restore           undo setup (stock chrony config)
#
#  After `setup`, nothing has to be run on each boot: chrony steps the clock
#  as soon as the host is reachable (the RPi has no RTC, so this matters).
#  Keep this file with LF line endings.
# =====================================================================
set -euo pipefail

MAIN_CONF=/etc/chrony/chrony.conf
CONF_DIR=/etc/chrony/conf.d
DROPIN=$CONF_DIR/50-tb4-host.conf
BACKUP=$MAIN_CONF.tb4.bak
CREATE3_NET=192.168.186.0/24
CREATE3_IP=192.168.186.2
MAX_OFFSET=0.05   # seconds; same tolerance as the course script

die()       { echo "error: $*" >&2; exit 1; }
need_root() { [ "$(id -u)" -eq 0 ] || die "run with sudo"; }

ensure_chrony() {
  command -v chronyd >/dev/null 2>&1 && return
  echo "[*] installing chrony"
  { apt-get update && apt-get install -y chrony; } \
    || die "chrony is missing and apt failed (no internet?). Install chrony first."
}

do_setup() {
  need_root
  local host=${1:-}
  [ -n "$host" ] || die "usage: sudo $0 setup <HOST_IP>"
  [[ $host =~ ^[A-Za-z0-9.-]+$ ]] || die "invalid host: $host"
  ensure_chrony

  ping -c1 -W2 "$host" >/dev/null 2>&1 \
    || echo "warning: $host does not answer ping; continuing anyway"

  echo "[1] disabling competing time services"
  systemctl disable --now systemd-timesyncd >/dev/null 2>&1 || true

  echo "[2] neutralizing stock sources in $MAIN_CONF (backup: $BACKUP)"
  [ -f "$BACKUP" ] || cp -a "$MAIN_CONF" "$BACKUP"
  # Comment out internet pools / DHCP servers / boot-only makestep so the
  # host is the single time authority. Reversible with `restore`.
  sed -i -E 's/^(pool|server|peer|makestep|sourcedir[[:space:]]+\/run\/chrony-dhcp)([[:space:]]|$)/#tb4# &/' "$MAIN_CONF"
  grep -qE "^confdir[[:space:]]+$CONF_DIR" "$MAIN_CONF" || echo "confdir $CONF_DIR" >> "$MAIN_CONF"
  mkdir -p "$CONF_DIR"

  echo "[3] writing $DROPIN"
  cat > "$DROPIN" <<EOF
# Managed by tb4_rpi_timesync.sh — remove with: sudo $0 restore
# Follow the host laptop; poll fast (4-16 s) since it is on the LAN.
server $host iburst prefer minpoll 2 maxpoll 4
# No RTC on the RPi and the clock can be far off: step whenever the
# offset exceeds 0.1 s, at any time (not only during the first updates).
makestep 0.1 -1
# Serve time to the Create3 over the USB-C link (Create3 uses 192.168.186.3).
allow $CREATE3_NET
# Keep serving the Create3 even while the host is unreachable.
local stratum 10
EOF

  echo "[4] restarting chrony"
  systemctl enable chrony >/dev/null 2>&1 || true
  systemctl restart chrony
  do_sync
}

do_sync() {
  need_root
  systemctl is-active --quiet chrony || systemctl restart chrony
  echo "[*] syncing to host"
  chronyc burst 4/4 >/dev/null 2>&1 || true
  sleep 3
  chronyc makestep >/dev/null 2>&1 || true
  if chronyc waitsync 20 "$MAX_OFFSET" 0 1 >/dev/null 2>&1; then
    echo "  synced:"
    chronyc tracking | grep -E 'Reference ID|System time|Last offset|Leap status' | sed 's/^/    /'
    echo "  Restart SLAM / Nav2 / RViz if they were running during a clock step."
  else
    echo "  NOT synced within 20 s. Current sources:"
    chronyc sources | sed 's/^/    /'
    cat <<'EOF'
  Check: host runs `tb4_host_timeserver.sh setup` with the robot subnet allowed,
         UDP 123 is open on the host firewall, and HOST_IP is correct
         (cat /etc/chrony/conf.d/50-tb4-host.conf).
EOF
    exit 1
  fi
}

do_status() {
  echo "== tracking =="
  chronyc tracking
  echo
  echo "== sources =="
  chronyc sources
  echo
  echo "== Create3 ($CREATE3_IP) polling this RPi =="
  if [ "$(id -u)" -eq 0 ]; then
    chronyc clients | grep -E "Hostname|$CREATE3_IP" \
      || echo "  Create3 not seen yet (it should use server 192.168.186.3 in its ntp.conf)"
  else
    echo "  (run with sudo to list NTP clients)"
  fi
}

do_restore() {
  need_root
  rm -f "$DROPIN"
  if [ -f "$BACKUP" ]; then
    cp -a "$BACKUP" "$MAIN_CONF" && rm -f "$BACKUP"
  fi
  systemctl restart chrony
  echo "restored stock chrony configuration"
}

# Measure (RPi clock - /odom header stamp). /odom is stamped by the Create3,
# so a large value means the Create3 clock is not following the RPi.
do_check_create3() {
  if [ -z "${ROS_DISTRO:-}" ]; then
    set +u; source /opt/ros/jazzy/setup.bash; set -u
  fi
  local off
  off=$(/usr/bin/python3 - <<'PY' 2>/dev/null
import time, rclpy
from nav_msgs.msg import Odometry
rclpy.init(); n = rclpy.create_node('tb4_create3_clock')
d = []
def cb(m):
    s = m.header.stamp
    d.append(time.time() - (s.sec + s.nanosec * 1e-9))
n.create_subscription(Odometry, '/odom', cb, 10)
end = time.time() + 5
while time.time() < end and len(d) < 10:
    rclpy.spin_once(n, timeout_sec=0.1)
print(f"{sorted(d)[len(d)//2]:.3f}" if d else "nan")
PY
)
  if [ -z "$off" ] || [ "$off" = "nan" ]; then
    echo "Create3 clock: no /odom received (Create3 application may be stopped)"
    return 1
  fi
  if awk -v d="$off" -v m="$MAX_OFFSET" 'BEGIN{exit !(d>=-m && d<=m)}'; then
    echo "Create3 clock: OK (offset $off s)"
  else
    echo "Create3 clock: OFF by $off s. Restart the Create3 application (web UI http://192.168.186.2)."
    return 1
  fi
}

case "${1:-}" in
  setup)   shift; do_setup "$@" ;;
  sync)    do_sync ;;
  status)  do_status; echo; do_check_create3 || true ;;
  check)   do_check_create3 ;;
  restore) do_restore ;;
  *) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
