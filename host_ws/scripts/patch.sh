#!/usr/bin/env bash
# usage: patch.sh find | laptop [--dry-run] | robot [--dry-run] | verify
#   Makes the robot and this laptop use the addresses our scripts assume:
#     robot (Pi eth0):   192.168.185.3/24, static, no gateway      (run_demo, tb4_ops, timesync)
#     laptop (wired):    192.168.185.10/24, static, never default route (fastdds_wired.xml, timesync)
#
#   find     scan the robot subnet and list hosts that answer on SSH (read only)
#   laptop   create or update the NetworkManager profile "team7-tb4" on the wired port (sudo)
#   robot    write a netplan file that sets the Pi's eth0 to 192.168.185.3 (ssh + sudo on the Pi)
#   verify   check that the robot answers at 192.168.185.3 and shows its eth0 address (read only)
#
#   Environment:
#     TB4_CUR     the Pi's current address, for `robot` (from `find`). Required for robot mode.
#     TB4_USER    default ubuntu
#     IFACE       laptop wired port, default: first ethernet device NetworkManager knows
#     TB4_ETH     the Pi's ethernet port, default eth0
#     SUBNET      subnet for find, default 192.168.185.0/24
#
#   Passwords are typed in the terminal (ssh/sudo prompt). Nothing is stored.
#   `robot` changes the robot's network config. Keep the robot powered and in view, and read the
#   rollback lines printed at the end before running it.
set -euo pipefail

TB4_IP=192.168.185.3
LAPTOP_IP=192.168.185.10
TB4_USER="${TB4_USER:-ubuntu}"
TB4_ETH="${TB4_ETH:-eth0}"
SUBNET="${SUBNET:-192.168.185.0/24}"
CON_NAME=team7-tb4
MODE="${1:-}"; shift || true
DRY=false
for a in "$@"; do case "$a" in --dry-run) DRY=true ;; *) echo "unknown option: $a" >&2; exit 2 ;; esac; done

die() { echo "error: $*" >&2; exit 1; }
run() { if $DRY; then echo "[dry-run] $*"; else "$@"; fi; }

find_robot() {
  local base="${SUBNET%.*}"
  echo "[*] scanning $base.1-254 for hosts (ping, then SSH port 22)"
  local tmp; tmp="$(mktemp -d)"
  for i in $(seq 1 254); do
    ( ping -c1 -W1 "$base.$i" >/dev/null 2>&1 && echo "$base.$i" > "$tmp/$i" ) &
  done
  wait
  local found=0
  for f in "$tmp"/*; do
    [ -e "$f" ] || continue
    local ip; ip="$(cat "$f")"
    if timeout 2 bash -c "echo > /dev/tcp/$ip/22" 2>/dev/null; then
      echo "  ssh open: $ip   (try: ssh $TB4_USER@$ip)"
      found=$((found+1))
    else
      echo "  answers ping: $ip"
    fi
  done
  rm -rf "$tmp"
  [ "$found" -gt 0 ] || echo "  no SSH host found on $SUBNET. Check the cable, the laptop profile (patch.sh laptop) and the robot power."
}

laptop_iface() {
  if [ -n "${IFACE:-}" ]; then echo "$IFACE"; return; fi
  nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="ethernet"{print $1; exit}'
}

do_laptop() {
  local ifc; ifc="$(laptop_iface)"
  [ -n "$ifc" ] || die "no ethernet device found; set IFACE=<wired port>"
  echo "[*] laptop wired port: $ifc, profile $CON_NAME -> $LAPTOP_IP/24 (never the default route)"
  if nmcli -t -f NAME connection show | grep -qx "$CON_NAME"; then
    run sudo nmcli connection modify "$CON_NAME" connection.interface-name "$ifc" \
      ipv4.method manual ipv4.addresses "$LAPTOP_IP/24" ipv4.gateway "" ipv4.never-default yes \
      ipv6.method ignore connection.autoconnect yes
  else
    run sudo nmcli connection add type ethernet ifname "$ifc" con-name "$CON_NAME" \
      ipv4.method manual ipv4.addresses "$LAPTOP_IP/24" ipv4.never-default yes \
      ipv6.method ignore connection.autoconnect yes
  fi
  if ! run sudo nmcli connection up "$CON_NAME"; then
    echo "warning: profile is set but the link is not up. Plug the robot cable into $ifc and run again." >&2
  fi
  $DRY || ip -br addr show "$ifc" || true
}

robot_script() {  # runs on the Pi, as root
  cat <<EOF
set -eu
ETH="$TB4_ETH"; IP="$TB4_IP"
BK=/root/netplan-backup-\$(date +%s)
mkdir -p "\$BK"; cp -a /etc/netplan/. "\$BK/"
echo "[pi] backup of /etc/netplan in \$BK"
# other netplan files that define the same port would merge with ours: move them out (kept in the backup)
for f in /etc/netplan/*.yaml; do
  [ "\$(basename "\$f")" = "60-team7-eth.yaml" ] && continue
  if grep -q "\$ETH" "\$f" 2>/dev/null; then
    echo "[pi] moving \$f aside (it also configures \$ETH)"; mv "\$f" "\$BK/moved-\$(basename "\$f")"
  fi
done
cat > /etc/netplan/60-team7-eth.yaml <<YAML
network:
  version: 2
  ethernets:
    \$ETH:
      dhcp4: false
      addresses: [\$IP/24]
YAML
chmod 600 /etc/netplan/60-team7-eth.yaml
netplan generate && echo "[pi] netplan config valid"
echo "[pi] applying; the SSH session will drop if the address changes"
netplan apply
echo "[pi] rollback (from a console, if needed): rm /etc/netplan/60-team7-eth.yaml && cp -a \$BK/. /etc/netplan/ && netplan apply"
EOF
}

do_robot() {
  [ -n "${TB4_CUR:-}" ] || die "set TB4_CUR to the Pi's current address (run: patch.sh find)"
  local tmp; tmp="$(mktemp)"
  robot_script > "$tmp"
  echo "[*] Pi $TB4_USER@$TB4_CUR: set $TB4_ETH to $TB4_IP/24 (static)"
  if $DRY; then
    echo "[dry-run] would run on the Pi:"; sed 's/^/    /' "$tmp"; rm -f "$tmp"; return
  fi
  echo "    type 'yes' to continue, anything else aborts"
  read -r ans; [ "$ans" = yes ] || { rm -f "$tmp"; die "aborted"; }
  scp -o StrictHostKeyChecking=accept-new "$tmp" "$TB4_USER@$TB4_CUR:/tmp/team7-netpatch.sh"
  rm -f "$tmp"
  ssh -t -o StrictHostKeyChecking=accept-new "$TB4_USER@$TB4_CUR" "sudo bash /tmp/team7-netpatch.sh; rm -f /tmp/team7-netpatch.sh" || true
  echo "[*] next: check from the laptop with: patch.sh verify   (the Pi may take 30 s to come back)"
}

do_verify() {
  if ping -c2 -W2 "$TB4_IP" >/dev/null 2>&1; then
    echo "[ok] robot answers at $TB4_IP"
    ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new "$TB4_USER@$TB4_IP" \
      "ip -br addr show $TB4_ETH; chronyc tracking 2>/dev/null | head -2" || echo "[warn] ping ok but ssh failed"
  else
    echo "[fail] no answer at $TB4_IP. Run: patch.sh find, then check the laptop profile (patch.sh laptop)."
    exit 1
  fi
}

case "$MODE" in
  find)   find_robot ;;
  laptop) do_laptop ;;
  robot)  do_robot ;;
  verify) do_verify ;;
  *) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
