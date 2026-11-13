#!/usr/bin/env bash
# =====================================================================
#  tb4_ops.sh — day-to-day TurtleBot4 checks and recovery steps, run on the laptop
#
#  Usage:
#    tb4_ops.sh status              clocks (laptop, Pi, Create3 stamps), key topic rates, dock state
#    tb4_ops.sh topics              list ROS topics on the robot (domain 0)
#    tb4_ops.sh ssh [cmd ...]       ssh to the Pi (no cmd = interactive shell)
#    tb4_ops.sh restart-bringup     restart the Pi's turtlebot4.service (ROS bringup)
#    tb4_ops.sh logs [N]            last N bringup log lines from the Pi (default 60)
#    tb4_ops.sh tunnel [PORT]       open the Create3 web UI at http://localhost:PORT (default 8081)
#    tb4_ops.sh tunnel-stop         close that tunnel
#    tb4_ops.sh create3-off         ask the Create3 to power off (/robot_power). Needs a button press to boot again.
#
#  Environment (all optional):
#    TB4_HOST   default 192.168.185.3          TB4_USER  default ubuntu
#    TB4_PW     robot password. If unset, you are asked once per command.
#               The password is never written to disk; it is passed to ssh
#               through a temporary askpass helper that is deleted on exit.
#
#  The Create3 web UI lives on the Pi's private USB network (192.168.186.2),
#  so it is reached through an SSH tunnel, not the robot LAN address.
# =====================================================================
set -euo pipefail

TB4_HOST="${TB4_HOST:-192.168.185.3}"
TB4_USER="${TB4_USER:-ubuntu}"
CREATE3_IP="192.168.186.2"
SSH_OPTS=(-o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new -o ServerAliveInterval=15)
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

die() { echo "error: $*" >&2; exit 1; }

# --- password handling -------------------------------------------------
PW="${TB4_PW:-}"
if [ -z "$PW" ] && [ -t 0 ]; then
  read -r -s -p "robot password for $TB4_USER@$TB4_HOST: " PW; echo
fi
cat > "$TMP/askpass.sh" <<'EOF'
#!/bin/sh
printf '%s\n' "$TB4_PW_INTERNAL"
EOF
chmod 700 "$TMP/askpass.sh"

# run ssh with the password supplied through askpass
rssh() {
  [ -n "$PW" ] || die "no password (set TB4_PW or run in a terminal)"
  TB4_PW_INTERNAL="$PW" SSH_ASKPASS="$TMP/askpass.sh" SSH_ASKPASS_REQUIRE=force \
    setsid ssh "${SSH_OPTS[@]}" "$TB4_USER@$TB4_HOST" "$@" </dev/null 2>/dev/null
}
# same, but the Pi is interactive-free: keep stdin for piped input
rssh_in() {
  [ -n "$PW" ] || die "no password (set TB4_PW or run in a terminal)"
  TB4_PW_INTERNAL="$PW" SSH_ASKPASS="$TMP/askpass.sh" SSH_ASKPASS_REQUIRE=force \
    setsid ssh "${SSH_OPTS[@]}" "$TB4_USER@$TB4_HOST" "$@" 2>/dev/null
}
# run a script (read from stdin) on the Pi with its ROS 2 environment and domain 0
ros_remote() {
  rssh_in "bash -lc 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; bash -s'"
}
# sudo on the Pi: the password goes on stdin, not on the command line
sudo_remote() {
  printf '%s\n' "$PW" | rssh_in "sudo -S -p '' bash -c '$*'"
}

ros_local() {
  bash -c "source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp && $*"
}

# --- commands ----------------------------------------------------------
cmd_status() {
  echo "laptop  $(date +%s)"
  ros_remote <<'EOS' || echo "robot not reachable at $TB4_HOST"
echo "pi      $(date +%s)"
chronyc tracking | grep -E 'Reference|System time' | sed 's/^/  /'
timeout 12 ros2 topic echo /odom --once --field header.stamp 2>/dev/null | grep -m1 sec | sed 's/^/odom stamp /'
timeout 12 ros2 topic echo /scan --once --field header.stamp 2>/dev/null | grep -m1 sec | sed 's/^/scan stamp /'
timeout 8 ros2 topic echo /dock_status --once 2>/dev/null | grep -m1 is_docked | sed 's/^/dock /'
if timeout 3 bash -c 'echo > /dev/tcp/192.168.186.2/80' 2>/dev/null; then echo 'create3 web: up'; else echo 'create3 web: down'; fi
exit 0
EOS
  echo "laptop topic rates (robot on domain 0):"
  for t in /odom /scan /team7/aruco/debug /global_costmap/costmap; do
    printf '  %-28s ' "$t"; ros_local "timeout 12 ros2 topic hz $t --window 5 2>&1 | head -1" || true
    echo
  done
}

cmd_topics() {
  ros_local "timeout 20 ros2 topic list"
}

cmd_ssh() {
  if [ "$#" -eq 0 ]; then
    TB4_PW_INTERNAL="$PW" SSH_ASKPASS="$TMP/askpass.sh" SSH_ASKPASS_REQUIRE=force \
      setsid ssh "${SSH_OPTS[@]}" "$TB4_USER@$TB4_HOST"
  else
    rssh "$@"
  fi
}

cmd_restart_bringup() {
  echo "restarting turtlebot4.service on the Pi (ROS bringup; the Create3 is not restarted)"
  sudo_remote "systemctl restart turtlebot4.service"
  echo "waiting 40 s for the bringup to come up"
  sleep 40
  cmd_status
}

cmd_logs() {
  local n="${1:-60}"
  sudo_remote "journalctl -u turtlebot4 -n $n --no-pager | cut -c1-200"
}

cmd_tunnel() {
  local port="${1:-8081}"
  if curl -s -o /dev/null -m 3 "http://localhost:$port/"; then
    echo "already open: http://localhost:$port"
    return
  fi
  TB4_PW_INTERNAL="$PW" SSH_ASKPASS="$TMP/askpass.sh" SSH_ASKPASS_REQUIRE=force \
    setsid ssh -f -N "${SSH_OPTS[@]}" -o ExitOnForwardFailure=yes \
      -L "$port:$CREATE3_IP:80" "$TB4_USER@$TB4_HOST" 2>/dev/null \
    || die "tunnel failed (check the password and that the robot answers at $TB4_HOST)"
  sleep 2
  echo "Create3 web UI: http://localhost:$port  (log in in your browser)"
  echo "close it later with: $0 tunnel-stop"
}

cmd_tunnel_stop() {
  # close only the -L tunnels this script opened (matched by their forward spec)
  pkill -f -- "-L [0-9]+:$CREATE3_IP:80" 2>/dev/null || true
  echo "tunnel stopped (if one was running)"
}

cmd_create3_off() {
  echo "This cuts battery power to the Create3 (/robot_power). It only comes back"
  echo "when someone presses its power button."
  read -r -p "type yes to continue: " ans
  [ "$ans" = yes ] || die "cancelled"
  ros_remote <<'EOS'
timeout 20 ros2 service call /robot_power irobot_create_msgs/srv/RobotPower '{}'
EOS
}

case "${1:-}" in
  status)           cmd_status ;;
  topics)           cmd_topics ;;
  ssh)              shift; cmd_ssh "$@" ;;
  restart-bringup)  cmd_restart_bringup ;;
  logs)             shift; cmd_logs "$@" ;;
  tunnel)           shift; cmd_tunnel "$@" ;;
  tunnel-stop)      cmd_tunnel_stop ;;
  create3-off)      cmd_create3_off ;;
  *) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
