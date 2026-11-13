# tb4_ops.sh — TurtleBot4 checks and recovery from the laptop

One script for the routine checks and recovery steps used during robot runs:
clock and data checks, restarting the Pi's ROS bringup, the Create3 web UI,
and powering the Create3 off.

Run it from `host_ws/scripts/` on the laptop.

## Quick start

```bash
cd host_ws/scripts
./tb4_ops.sh status              # clocks, /odom /scan stamps, dock state, topic rates
./tb4_ops.sh tunnel              # Create3 web UI at http://localhost:8081
```

The script asks for the robot password once per command. To skip the prompt:

```bash
TB4_PW=... ./tb4_ops.sh status   # set in the command, never saved to a file
```

## Commands

| Command | What it does |
|---|---|
| `status` | Laptop and Pi clocks, chrony reference, `/odom` and `/scan` stamps, dock state, Create3 web UI up/down, and laptop-side topic rates for `/odom`, `/scan`, the ArUco debug view and the global costmap. |
| `topics` | Lists the robot's ROS topics (domain 0). |
| `ssh [cmd ...]` | Opens a shell on the Pi, or runs one command there. |
| `restart-bringup` | Restarts `turtlebot4.service` on the Pi (the ROS bringup). Does not restart the Create3. Waits 40 s, then prints `status`. |
| `logs [N]` | Last N lines of the Pi's bringup log (default 60). |
| `tunnel [PORT]` | Opens an SSH tunnel to the Create3 web UI (`192.168.186.2:80`) on `localhost:PORT` (default 8081). |
| `tunnel-stop` | Closes the tunnel opened by `tunnel`. |
| `create3-off` | Calls the Create3's `/robot_power` service to cut its battery power. It asks you to type `yes` first. The robot only comes back when someone presses its power button. |

## Environment

| Variable | Default | Meaning |
|---|---|---|
| `TB4_HOST` | `192.168.185.3` | Pi address on the robot LAN. |
| `TB4_USER` | `ubuntu` | Pi login. |
| `TB4_PW` | (asked) | Robot password. Passed to ssh through a temporary askpass helper that is deleted on exit. Never written to disk. |

## Why a tunnel for the web UI

The Create3's web UI sits on the Pi's private USB network (`192.168.186.2`).
Your laptop can't route there, and the Pi's own forward on port 8080 isn't
reachable from the laptop. `tunnel` forwards it over SSH instead.

## Reading the status

- **Clocks.** Laptop and Pi should match to well under a second, and the Pi
  should report the laptop as its reference (`192.168.185.10`).
- **`/odom` stamp far from the Pi clock** (about 1.29 years off, for example):
  the Create3 clock is wrong. Fix it by restarting the Create3 application
  from the web UI, or by power-cycling the Create3.
- **`/odom` and `/scan` "not published yet":** the Create3 or the lidar is not
  sending. Check the Create3's comm LED first. Off means no link to the Pi.
  Then run `restart-bringup`.
- **Create3 web UI down:** the Create3 is off or still booting.

## Known gaps

- Create3 LED meanings are not in this script. Check the iRobot Create 3
  documentation for the current meaning of each light.
- The script does not restart the Create3 application. That needs the web UI
  login or a power cycle.
