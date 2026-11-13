#!/usr/bin/env bash
# =====================================================================
#  tb4_push_timesync.sh — run on the host laptop before a robot run
#
#  One command that gets the TurtleBot4 clock chain ready:
#    1. makes sure this laptop serves NTP to the robot subnet
#       (tb4_host_timeserver.sh setup, skipped if already set up)
#    2. finds the laptop IP on the route to the robot
#    3. copies tb4_rpi_timesync.sh to the RPi (scp)
#    4. runs `setup` on the RPi with that IP (ssh, sudo)
#    5. checks the RPi sync and the Create3 /odom clock
#
#  Usage:
#     ./tb4_push_timesync.sh                      default robot ubuntu@192.168.185.3
#     TB4_HOST=192.168.185.3 TB4_USER=ubuntu ./tb4_push_timesync.sh
#     ./tb4_push_timesync.sh status               only the RPi/Create3 check
#
#  Passwords are not stored here. ssh/scp/sudo ask for them in the terminal
#  (run this from an interactive shell). Set up SSH keys to skip the prompt.
# =====================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TB4_USER="${TB4_USER:-ubuntu}"
TB4_HOST="${TB4_HOST:-192.168.185.3}"
SUBNET="${TB4_SUBNET:-192.168.185.0/24}"
REMOTE_DIR="/home/$TB4_USER"
SSH_OPTS=(-o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new)
MODE="${1:-push}"

die() { echo "error: $*" >&2; exit 1; }

host_side() {
  local conf=/etc/chrony/conf.d/50-tb4-server.conf
  if [ ! -f "$conf" ]; then
    echo "[1] laptop NTP server not set up yet"
    sudo "$HERE/tb4_host_timeserver.sh" setup "$SUBNET"
  else
    echo "[1] laptop NTP server already set up ($conf)"
  fi
}

host_ip() {
  # source address this laptop uses to reach the robot
  ip -4 route get "$TB4_HOST" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}'
}

push() {
  ping -c1 -W2 "$TB4_HOST" >/dev/null 2>&1 \
    || die "robot $TB4_HOST does not answer ping. Power the TB4 and join its LAN first."

  host_side

  local ip
  ip="$(host_ip)"
  [ -n "$ip" ] || die "could not find this laptop's address toward $TB4_HOST"
  echo "[2] laptop address on the robot network: $ip"

  echo "[3] sending tb4_rpi_timesync.sh to $TB4_USER@$TB4_HOST:$REMOTE_DIR"
  scp "${SSH_OPTS[@]}" "$HERE/tb4_rpi_timesync.sh" "$TB4_USER@$TB4_HOST:$REMOTE_DIR/"

  echo "[4] setting up the RPi to follow $ip"
  ssh -t "${SSH_OPTS[@]}" "$TB4_USER@$TB4_HOST" \
    "sed -i 's/\r\$//' $REMOTE_DIR/tb4_rpi_timesync.sh && chmod +x $REMOTE_DIR/tb4_rpi_timesync.sh && sudo $REMOTE_DIR/tb4_rpi_timesync.sh setup $ip"

  echo "[5] checking"
  check
}

check() {
  ssh -t "${SSH_OPTS[@]}" "$TB4_USER@$TB4_HOST" \
    "sudo $REMOTE_DIR/tb4_rpi_timesync.sh status" || echo "warning: status check failed"
}

case "$MODE" in
  push)   push ;;
  status) check ;;
  *) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
