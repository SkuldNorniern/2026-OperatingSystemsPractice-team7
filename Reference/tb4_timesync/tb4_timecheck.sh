#!/usr/bin/env bash
# =====================================================================
#  tb4_timecheck.sh — run ON the host laptop, in a terminal that sees
#  the robot's ROS 2 topics. Read-only: it changes no clocks.
#
#  Measures (laptop time − message header stamp) for
#     /odom  (stamped by the Create3)
#     /scan  (stamped by the RPi)
#  Each value should be close to 0 (a few ms of network latency is normal).
# =====================================================================
set -uo pipefail

MAX_OFFSET=${MAX_OFFSET:-0.05}
command -v ros2 >/dev/null 2>&1 || source /opt/ros/jazzy/setup.bash

measure() {   # median of (laptop time - header stamp), in seconds
/usr/bin/python3 - "$1" <<'PY' 2>/dev/null
import sys, time, rclpy
from rclpy.qos import qos_profile_sensor_data
topic = sys.argv[1]
if topic == '/odom':
    from nav_msgs.msg import Odometry as M
else:
    from sensor_msgs.msg import LaserScan as M
rclpy.init(); n = rclpy.create_node('tb4_timecheck')
d = []
def cb(m):
    s = m.header.stamp
    d.append(time.time() - (s.sec + s.nanosec * 1e-9))
n.create_subscription(M, topic, cb, qos_profile_sensor_data)
end = time.time() + 5
while time.time() < end and len(d) < 20:
    rclpy.spin_once(n, timeout_sec=0.1)
print(f"{sorted(d)[len(d)//2]:.3f}" if d else "nan")
PY
}

rc=0
for t in /odom /scan; do
  v=$(measure "$t")
  if [ -z "$v" ] || [ "$v" = "nan" ]; then
    echo "  $t: no messages (check ROS_DOMAIN_ID / discovery / robot state)"; rc=1; continue
  fi
  if awk -v d="$v" -v m="$MAX_OFFSET" 'BEGIN{exit !(d>=-m && d<=m)}'; then
    echo "  $t delay = $v s  OK"
  else
    echo "  $t delay = $v s  OFF (> ${MAX_OFFSET} s)"; rc=1
  fi
done

if [ $rc -ne 0 ]; then
  cat <<'EOF'
Hints:
  /scan off  -> RPi not following the host: on the RPi run `sudo ./tb4_rpi_timesync.sh status`
  /odom off  -> Create3 not following the RPi: check its ntp.conf (web UI at 192.168.186.2)
               uses `server 192.168.186.3 iburst`, then restart the Create3 application.
EOF
fi
exit $rc
